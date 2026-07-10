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
	// Central median strips, per zone (parallel to Zones) — raised plantable /
	// walkable block in the carriageway gap of divided roads.
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneMedianPolys;
	// §8.10 lane-marking ribbons, per zone (parallel to Zones), split by colour.
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneMarkingWhitePolys;
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneMarkingYellowPolys;
	// §12.1 per-lane ribbons, per zone (parallel to Zones). Split into two banks
	// by alternating lane index so adjacent lanes render in contrasting shades
	// (an "additional" layer above the unified carriageway).
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneLaneEvenPolys;
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneLaneOddPolys;
	// Per-zone junction CLIP region (§2.1 "שטח הצומת"): the true junction area
	// bounded by the roads' edge lines and their imaginary extension — computed as
	// the mutual overlap of crossing carriageways, then dilated by the stop-line
	// setback. Markings and lane ribbons are subtracted against this so paint ends
	// at the junction edge (not on a lazy circular disc).
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneJunctionClip;
	// § junction traffic-signal placeholder placements (location, yawDeg) for
	// this rebuild — committed as a HISM.
	TArray<TPair<FVector, float>> Signals;
	// Flattened unions (for logging/QA only).
	TArray<UE::Geometry::FGeneralPolygon2d> SurfacePolys;
	TArray<UE::Geometry::FGeneralPolygon2d> SidewalkPolys;
	// §10.11 perimeter loops (network outlines + block holes) for PCG export (§8.4).
	TArray<FRoadNetLoop> PerimeterLoops;
	// §12.2 lane-connectivity graph (derived from joints + resolved lanes).
	TArray<FRoadNetLaneConnection> LaneConnections;
	// Later phases populate: overlap masks, details.
};

class UMaterialInterface;
class UStaticMesh;

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

	// ---- sampling (§2.6) --------------------------------------------------
	// Arc-length spacing (cm) used to resample every road's reference polyline
	// before offsetting/meshing. Lower = more points per segment = smoother
	// curves and tighter terrain conformance (at the cost of more geometry).
	// Curvature knots are always preserved on top of this uniform spacing.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Sampling",
		meta = (ClampMin = "25.0", UIMin = "50.0", UIMax = "500.0"))
	double PolylineDensityCm = 200.0;

	// ---- lanes (§12.1) ----------------------------------------------------
	// Render each resolved lane as its own ribbon strip (alternating shades)
	// layered above the carriageway. Reflects lane add/remove + authored widths.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Lanes")
	bool bShowLaneRibbons = true;

	// Optional material for the per-lane ribbon layer (else a flat shade).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Lanes")
	TObjectPtr<UMaterialInterface> LaneMaterial;

	// Build + export the lane-connectivity graph (§12.2) as tagged splines for
	// PCG / traffic. Disable to skip the graph stages on large imports.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Lanes")
	bool bBuildLaneGraph = true;

	// ---- kerbs (§8.12 companion) ------------------------------------------
	// Instance a kerb-segment mesh along the road/sidewalk boundary as a HISM.
	// The kerb line is derived from the merged carriageway + sidewalk polygons
	// each rebuild, so it tracks lane growth and curved junction corners and is
	// only emitted where a sidewalk actually exists.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Curbs")
	bool bBuildCurbs = true;

	// Kerb segment mesh (long axis = travel). Author its raised face toward −Y
	// (road side) and its pivot at the base. If unset a scaled engine cube is
	// used as a visible placeholder so you can see the kerb line.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Curbs")
	TObjectPtr<UStaticMesh> CurbMesh;

	// Two material overrides applied to the kerb HISM (slot 0 + slot 1), so a
	// two-slot kerb mesh (e.g. top vs. face) can be textured separately — like
	// the RoadPCG kerb kit.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Curbs")
	TObjectPtr<UMaterialInterface> CurbMaterial0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Curbs")
	TObjectPtr<UMaterialInterface> CurbMaterial1;

	// Along-line spacing (cm) of kerb pieces. Each piece is stretched to tile
	// its run gaplessly, so this is a target length per segment.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Curbs",
		meta = (ClampMin = "50.0", UIMin = "50.0", UIMax = "600.0"))
	double CurbSpacingCm = 100.0;

	// ---- junction markings (§2 junctions) ---------------------------------
	// Master toggle for junction paint (stop/give-way lines, crosswalks) and
	// the traffic-signal placeholders. Per-junction treatment is chosen
	// interactively in the RoadNet Draw mode (click a junction, cycle preset).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Junctions")
	bool bBuildJunctionMarkings = true;

	// Placeholder mesh for a signalized junction (one per approach). If unset a
	// scaled engine cylinder is instanced as a visible "pole" placeholder.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Junctions")
	TObjectPtr<UStaticMesh> SignalMesh;

	// ---- median (§ divided road) ------------------------------------------
	// Material for the raised median strip. If unset a flat green (plantable)
	// colour is used so the median is visible.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Median")
	TObjectPtr<UMaterialInterface> MedianMaterial;

	// ---- junctions (§10.8/§10.9) ------------------------------------------
	// Morphological "close" radius (cm) applied to the merged carriageway. Larger
	// = rounder junction corners / more gap bridging. Adjustable live with the
	// [ and ] hotkeys in the RoadNet Draw mode.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Junctions",
		meta = (ClampMin = "0.0", UIMin = "0.0", UIMax = "300.0"))
	double JunctionSmoothingCm = 20.0;

	// Stop-line setback (cm): how far markings and lane ribbons stop BACK from the
	// true junction area (§2.1 מבואות). Applied as a dilation of the edge-line
	// junction polygon before clipping paint. 0 = paint right up to the edge.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Junctions",
		meta = (ClampMin = "0.0", UIMin = "0.0", UIMax = "600.0"))
	double JunctionClearanceCm = 60.0;

	// Nudge JunctionSmoothingCm by DeltaCm (clamped ≥ 0). Returns the new value.
	// Caller triggers Rebuild(). Wired to the [ / ] hotkeys.
	double AdjustJunctionSmoothing(double DeltaCm);

	// Register a road; returns its stable index. Assigns a GUID if unset.
	int32 AddRoad(const FRoadDef& Road);

	// ---- lane editing (§12.1) ---------------------------------------------
	// Add one lane to a road on the given side (Left/Right). Grows the count
	// model (Total + Forward/Backward). Returns false on bad index. Caller
	// triggers Rebuild().
	bool AddLane(int32 RoadIdx, ERoadNetSide Side);

	// Remove the outermost lane on the given side. Keeps at least one lane on
	// the road. Returns false on bad index or if nothing could be removed.
	bool RemoveLane(int32 RoadIdx, ERoadNetSide Side);

	// Number of lanes on a road (for HUD/debug).
	int32 GetLaneCount(int32 RoadIdx) const;

	// ---- median editing (RoadNet Draw hotkeys) ----------------------------
	// Toggle the central median on a road. Returns the new bMedian state.
	// Caller triggers Rebuild().
	bool ToggleMedian(int32 RoadIdx);

	// Cycle the median edge treatment (Plantable → CurbOnly → SidewalkAndCurb).
	// Turns the median on if it was off. Returns the new edge treatment.
	ERoadNetMedianEdge CycleMedianEdge(int32 RoadIdx, int32 Dir);

	// Nudge the median width (cm, clamped). Returns the new width.
	float AdjustMedianWidth(int32 RoadIdx, float DeltaCm);

	// Median state accessors (for HUD/debug).
	bool IsMedian(int32 RoadIdx) const;
	float GetMedianWidth(int32 RoadIdx) const;

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

	// ---- junction marking authoring (§2 junctions) ------------------------
	// A junction as surfaced to the editor tool after a rebuild: its world
	// centre, arm count and the currently-resolved preset.
	struct FRoadNetJunctionView
	{
		FVector Location = FVector::ZeroVector;
		ERoadNetJunctionPreset Preset = ERoadNetJunctionPreset::None;
		int32 ArmCount = 0;
	};
	const TArray<FRoadNetJunctionView>& GetJunctionViews() const { return JunctionViews; }

	// Resolve the stored preset for the junction nearest Loc (within tolerance);
	// ERoadNetJunctionPreset::None if no override exists there.
	ERoadNetJunctionPreset ResolveJunctionPresetNear(const FVector2D& Loc) const;

	// Advance (Dir=+1) or reverse (Dir=-1) the preset for the junction nearest
	// Loc, creating an override entry if needed. Returns the new preset. Caller
	// triggers Rebuild().
	ERoadNetJunctionPreset CycleJunctionPresetNear(const FVector2D& Loc, int32 Dir);

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
	TWeakObjectPtr<AActor> GeoLaneGraphActor;     // §12.2 lane-connectivity splines for PCG/traffic
	TWeakObjectPtr<AActor> GeoLaneEvenActor;      // §12.1 per-lane ribbons (even bank)
	TWeakObjectPtr<AActor> GeoLaneOddActor;       // §12.1 per-lane ribbons (odd bank)
	TWeakObjectPtr<AActor> GeoCurbActor;          // §8.12 kerb-line HISM (road/sidewalk edge)
	TWeakObjectPtr<AActor> GeoSignalActor;        // § junction traffic-signal placeholder HISM
	TWeakObjectPtr<AActor> GeoMedianActor;        // § raised median strip mesh
	TWeakObjectPtr<AActor> GeoMedianSplineActor;  // § median centre splines (PCG tree scatter)

	// Persistent per-junction marking overrides (keyed by location).
	UPROPERTY()
	TArray<FRoadNetJunctionConfig> JunctionConfigs;

	// Transient snapshot of the last rebuild's junctions (>=3 arms) for the
	// editor tool to render + hit-test. Not serialized.
	TArray<FRoadNetJunctionView> JunctionViews;

	// ---- pipeline stages (§10.18) --------------------------------------
	void DeterminePendingRoads(FRoadNetRebuildContext& Ctx) const;
	void BuildCurves(FRoadNetRebuildContext& Ctx) const;
	void BuildCrossings(FRoadNetRebuildContext& Ctx) const;      // §10.12 grid broadphase (shared)
	void BuildEndpointJoints(FRoadNetRebuildContext& Ctx) const;
	void BuildZones(FRoadNetRebuildContext& Ctx) const;          // §10.12 grade separation
	void BuildSurfaceUnion(FRoadNetRebuildContext& Ctx) const;   // §10.9 per-zone union + §8.12 sidewalks
	void BuildPerimeterLoops(FRoadNetRebuildContext& Ctx) const; // §10.11 loops for PCG export
	void BuildLaneGraph(FRoadNetRebuildContext& Ctx) const;      // §12.2 lane connectivity
	void BuildLaneRibbons(FRoadNetRebuildContext& Ctx) const;    // §12.1 per-lane ribbon polys
	void BuildJunctionMarkings(FRoadNetRebuildContext& Ctx);     // §2 junction paint + signals
	void CommitGeometry(FRoadNetRebuildContext& Ctx);            // §10.15 mesh + spawn
	void CommitCurbs(FRoadNetRebuildContext& Ctx);               // §8.12 kerb-line HISM
	void CommitJunctionSignals(FRoadNetRebuildContext& Ctx);     // § signal placeholder HISM
	void CommitMedian(FRoadNetRebuildContext& Ctx);              // § raised median strip + centre splines
	void CommitPerimeters(FRoadNetRebuildContext& Ctx);          // §8.4 spline loops for PCG
	void CommitLaneGraph(FRoadNetRebuildContext& Ctx);           // §12.2 lane-graph splines for PCG

	// Mesh a set of per-zone polygons and spawn/update a colored actor. If
	// Material is set it is applied to slot 0; otherwise the constant Color is
	// used as a vertex-colour override so the layer is always visible.
	int32 CommitLayer(TWeakObjectPtr<AActor>& ActorPtr, const TCHAR* Label,
		const TArray<TArray<UE::Geometry::FGeneralPolygon2d>>& ZonePolys,
		double ExtraLiftCm, FColor Color, UMaterialInterface* Material, FRoadNetRebuildContext& Ctx,
		bool bBakeLaneColors = false);
	// TODO: overlap masks (§10.10), per-road perimeter loops (§10.11), markings.
};
