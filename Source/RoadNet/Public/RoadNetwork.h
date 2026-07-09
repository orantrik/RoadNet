#pragma once
#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "Curve/GeneralPolygon2.h"
#include "RoadNetTypes.h"
#include "RoadNetPerimeters.h"
#include "RoadNetwork.generated.h"

// ===========================================================================
// URoadNetwork — the orchestration container (§1.1 / §10).
//
// Holds the persistent FRoadDef source-of-truth and drives the staged rebuild
// pipeline (§2 / §10.18). Both authoring modes converge here: OSM import adds
// roads via AddRoad(); hand-draw tools mutate roads + call Rebuild().
//
// This class is INDEPENDENT of RoadBLD/WorldBLD/CityBLD and of OSMRoadCore —
// the OSM → FRoadDef mapping lives in a bridge inside OSMRoadCore, so the
// dependency only ever points OSMRoadCore -> RoadNet.
// ===========================================================================

// ---- Compute-only curve products (not reflected; regenerated each rebuild) --
struct FRoadCurves
{
	int32 RoadIndex = INDEX_NONE;
	TArray<FVector> Sampled;    // reference line resampled at PolylineDensity
	TArray<FVector> LeftEdge;   // +right-axis outer edge at +HalfWidth
	TArray<FVector> RightEdge;  // -right-axis outer edge at -HalfWidth
	double Length = 0.0;
};

// ---- Derived topology node (§10.7) -----------------------------------------
struct FRoadNetJoint
{
	int64 NodeId = -1;
	FVector2D Location = FVector2D::ZeroVector;
	// (RoadIndex, bAtStart) arms meeting at this node.
	TArray<TPair<int32, bool>> Arms;
	ERoadNetJointKind Kind = ERoadNetJointKind::Terminal;
	double Z = 0.0;
};

// ---- 2-D centerline crossing between two roads (§10.12 / §10.8) -------------
// Computed once per rebuild with a spatial-grid broadphase and shared by the
// zone partition (grade separation) and the surface union (junction discs).
struct FRoadNetCrossing
{
	int32 RoadA = INDEX_NONE;   // global road index
	int32 RoadB = INDEX_NONE;   // global road index
	FVector2D Point = FVector2D::ZeroVector;
	double Za = 0.0;            // Z on RoadA at the crossing
	double Zb = 0.0;            // Z on RoadB at the crossing
};

// ---- Compute bus for one rebuild (§2 FRebuildContext) -----------------------
struct FRoadNetRebuildContext
{
	TArray<int32> Modified;
	TArray<int32> Pending;
	TArray<int32> TestAgainst;
	TMap<int32, FRoadCurves> Curves;
	TArray<FRoadNetJoint> Joints;
	// §10.12/§10.8 all 2-D centerline crossings (computed once, shared).
	TArray<FRoadNetCrossing> Crossings;
	// §10.12 grade-separation zones: each group is a set of road indices that are
	// at-grade with one another and get unioned + meshed independently.
	TArray<TArray<int32>> Zones;
	// §10.9 boolean-union surface, per zone (parallel to Zones).
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneSurfacePolys;
	// §8.12 sidewalk bands, per zone (parallel to Zones).
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneSidewalkPolys;
	// §8.10 lane-marking ribbons, per zone (parallel to Zones), split by colour.
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneMarkingWhitePolys;
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneMarkingYellowPolys;
	// Flattened unions (for logging/QA only).
	TArray<UE::Geometry::FGeneralPolygon2d> SurfacePolys;
	TArray<UE::Geometry::FGeneralPolygon2d> SidewalkPolys;
	// §10.11 perimeter loops (network outlines + block holes) for PCG export (§8.4).
	TArray<FRoadNetLoop> PerimeterLoops;
	// Later phases populate: overlap masks, details.
};

class UMaterialInterface;

UCLASS(BlueprintType, EditInlineNew, DefaultToInstanced)
class ROADNET_API URoadNetwork : public UObject
{
	GENERATED_BODY()

public:
	// ---- optional per-layer materials (§10.16). If unset, the layer falls back
	// to its constant vertex-colour override so geometry is always visible. ----
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Materials")
	TObjectPtr<UMaterialInterface> RoadMaterial;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Materials")
	TObjectPtr<UMaterialInterface> SidewalkMaterial;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Materials")
	TObjectPtr<UMaterialInterface> MarkingWhiteMaterial;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Materials")
	TObjectPtr<UMaterialInterface> MarkingYellowMaterial;

	// Register a road; returns its stable index. Assigns a GUID if unset.
	int32 AddRoad(const FRoadDef& Road);

	// Clear all roads (e.g. before a fresh OSM import).
	void ResetRoads();

	// Remove all roads whose Source matches (re-import refresh — §9.4 keeps
	// hand-drawn roads by passing ERoadNetSource::OSM).
	int32 RemoveRoadsBySource(ERoadNetSource Source);

	const TArray<FRoadDef>& GetRoads() const { return Roads; }
	int32 NumRoads() const { return Roads.Num(); }

	// ---- interactive edit API (§9.3 edit/split controllers) ---------------
	// Move one reference point of a road to a new world position. Returns false
	// if indices are invalid. Caller triggers Rebuild().
	bool MoveRoadPoint(int32 RoadIdx, int32 PointIdx, const FVector& NewWorldPos);

	// Delete one reference point. If the road drops below 2 points it is removed
	// entirely. Returns true when the whole road was removed (so callers can drop
	// any cached selection/indices, which shift on removal).
	bool DeleteRoadPoint(int32 RoadIdx, int32 PointIdx, bool& bOutRoadRemoved);

	// Insert a point after AfterIdx at Pos (mid-span split). Returns false on bad
	// index. Caller triggers Rebuild().
	bool InsertRoadPoint(int32 RoadIdx, int32 AfterIdx, const FVector& Pos);

	// Remove an entire road by index. Returns false on bad index. Note this
	// shifts later indices, so callers must drop cached selections. Caller
	// triggers Rebuild().
	bool RemoveRoad(int32 RoadIdx);

	// Staged rebuild entry point (§10.18). Empty Modified = rebuild everything.
	void Rebuild(TArrayView<const int32> Modified = TArrayView<const int32>());

	// Bind a target world for the (future) commit stage that spawns geometry.
	void SetWorld(UWorld* InWorld) { WorldPtr = InWorld; }

private:
	UPROPERTY()
	TArray<FRoadDef> Roads;

	TWeakObjectPtr<UWorld> WorldPtr;

	// Spawned surface actors (reused across rebuilds of this network).
	TWeakObjectPtr<AActor> GeoActor;              // road carriageway
	TWeakObjectPtr<AActor> GeoSidewalkActor;      // sidewalk band
	TWeakObjectPtr<AActor> GeoMarkingWhiteActor;  // white markings (edge + lane dividers)
	TWeakObjectPtr<AActor> GeoMarkingYellowActor; // yellow markings (centre line)
	TWeakObjectPtr<AActor> GeoPerimeterActor;     // §8.4 PCG spline loops (road edges + blocks)

	// ---- pipeline stages (§10.18) --------------------------------------
	void DeterminePendingRoads(FRoadNetRebuildContext& Ctx) const;
	void BuildCurves(FRoadNetRebuildContext& Ctx) const;
	void BuildCrossings(FRoadNetRebuildContext& Ctx) const;      // §10.12 grid broadphase (shared)
	void BuildEndpointJoints(FRoadNetRebuildContext& Ctx) const;
	void BuildZones(FRoadNetRebuildContext& Ctx) const;          // §10.12 grade separation
	void BuildSurfaceUnion(FRoadNetRebuildContext& Ctx) const;   // §10.9 per-zone union + §8.12 sidewalks
	void BuildPerimeterLoops(FRoadNetRebuildContext& Ctx) const; // §10.11 loops for PCG export
	void CommitGeometry(FRoadNetRebuildContext& Ctx);            // §10.15 mesh + spawn
	void CommitPerimeters(FRoadNetRebuildContext& Ctx);          // §8.4 spline loops for PCG

	// Mesh a set of per-zone polygons and spawn/update a colored actor. If
	// Material is set it is applied to slot 0; otherwise the constant Color is
	// used as a vertex-colour override so the layer is always visible.
	int32 CommitLayer(TWeakObjectPtr<AActor>& ActorPtr, const TCHAR* Label,
		const TArray<TArray<UE::Geometry::FGeneralPolygon2d>>& ZonePolys,
		double ExtraLiftCm, FColor Color, UMaterialInterface* Material, FRoadNetRebuildContext& Ctx);
	// TODO: overlap masks (§10.10), per-road perimeter loops (§10.11), markings.
};
