#pragma once
#include "CoreMinimal.h"
#include "RoadNetTypes.generated.h"

// ===========================================================================
// RoadNet core data model  (ROADBLD_REPLICATION_PLAN.md §5 / §10)
//
// These are the PERSISTENT source-of-truth types. Roads persist; geometry is
// disposable (regenerated every rebuild). Both authoring modes — OSM import
// and hand-draw (§9) — converge on FRoadDef and mutate a URoadNetwork.
//
// INDEPENDENCE: nothing here references RoadBLD / WorldBLD / CityBLD.
// ===========================================================================

// Road class — mirrors OSM highway= tiers but is engine-local (no OSM dependency
// at the type level; OSMRoadCore maps EOSMHighwayClass → this on import).
UENUM(BlueprintType)
enum class ERoadNetClass : uint8
{
	Motorway,
	Trunk,
	Primary,
	Secondary,
	Tertiary,
	Residential,
	Service,
	Pedestrian,
	Path,
	Unknown
};

// How a road was authored. Lets a re-import refresh OSM roads WITHOUT destroying
// hand-drawn edits (§9.4): re-import replaces only Source==OSM roads.
UENUM(BlueprintType)
enum class ERoadNetSource : uint8
{
	OSM,
	HandDrawn
};

// Classification of a topology node where road ends meet (§10.7).
UENUM(BlueprintType)
enum class ERoadNetJointKind : uint8
{
	Terminal,      // degree 1 — dead end
	Seam,          // degree 2, compatible — merged into a through-road
	Split,         // one road fanning into several (lane-disparity case)
	Intersection   // degree >= 3 — a real junction
};

// ---------------------------------------------------------------------------
// Lane specification (directional, OSM-aware). Widths in centimetres.
// ---------------------------------------------------------------------------
USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetLaneSpec
{
	GENERATED_BODY()

	// Lane counts. Forward/Backward take priority over Total when > 0 (§ bridge
	// logic in OSMRoadBuilderBridge). Total is the fallback.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	int32 Total = 2;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	int32 Forward = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	int32 Backward = 0;

	// Per-lane width (cm). If empty, LaneWidthDefault is used for every lane.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	TArray<float> LaneWidths;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	float LaneWidthDefault = 350.f;   // 3.5 m typical lane

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	bool bOneway = false;

	// Sidewalk presence (left = +right-axis side, right = -right-axis side).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	bool bSidewalkLeft = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	bool bSidewalkRight = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	float SidewalkWidth = 200.f;      // 2 m

	// Total carriageway half-width (cm) = sum(lane widths)/2. Computed helper.
	float HalfWidthCm() const
	{
		const int32 N = EffectiveLaneCount();
		float W = 0.f;
		for (int32 i = 0; i < N; ++i)
		{
			W += LaneWidths.IsValidIndex(i) ? LaneWidths[i] : LaneWidthDefault;
		}
		return 0.5f * W;
	}

	int32 EffectiveLaneCount() const
	{
		const int32 Dir = Forward + Backward;
		if (Dir > 0) { return Dir; }
		return FMath::Max(1, Total);
	}
};

// ---------------------------------------------------------------------------
// Endpoint link — authored/derived topology at a road end (§1.3 / §10.7).
// Persisted on the first (Start) or last (End) point of a road.
// ---------------------------------------------------------------------------
USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetEndpointLink
{
	GENERATED_BODY()

	// Index of the linked road within URoadNetwork::Roads. INDEX_NONE = unset.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Topology")
	int32 OtherRoad = INDEX_NONE;

	// Which end of the linked road: true = first point (Start), false = last (End).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Topology")
	bool bOtherFirst = true;

	// Signed lateral offset (cm) from the linked road's endpoint, measured along
	// its right axis (positive = right, negative = left). Mirrors RoadBLD's
	// FRoadEndpointLink.RightAxisOffset.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Topology")
	double RightAxisOffset = 0.0;
};

// ---------------------------------------------------------------------------
// FRoadDef — the persistent definition of a single road (§5).
// Geometry is the reference polyline in WORLD centimetres; per-point radius and
// elevation ride alongside it. Everything downstream is derived from this.
// ---------------------------------------------------------------------------
USTRUCT(BlueprintType)
struct ROADNET_API FRoadDef
{
	GENERATED_BODY()

	// Stable identity — survives incremental rebuilds & re-imports (§2.4).
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="RoadNet")
	FGuid Id;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet")
	ERoadNetSource Source = ERoadNetSource::OSM;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet")
	ERoadNetClass Class = ERoadNetClass::Residential;

	// Reference centerline, world-space cm. Drives all generation (§10.2).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Geometry")
	TArray<FVector> Ref;

	// Optional per-point elevation override (cm). If empty, Ref.Z is used.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Geometry")
	TArray<double> Elev;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	FRoadNetLaneSpec Lanes;

	// Grade separation (§10.12). Layer/bridge/tunnel decide which road is on top.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Grade")
	int32 Layer = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Grade")
	bool bBridge = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Grade")
	bool bTunnel = false;

	// OSM node id per Ref point (parallel array). Shared ids drive endpoint-joint
	// derivation (§10.7). -1 for synthetic/clip-inserted points. Empty for
	// hand-drawn roads (topology authored via links instead).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Topology")
	TArray<int64> NodeIds;

	// Authored/derived endpoint topology (§1.3). Multiple links per end support
	// T/Y-junctions, splits, merges.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Topology")
	TArray<FRoadNetEndpointLink> StartLinks;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Topology")
	TArray<FRoadNetEndpointLink> EndLinks;

	// Street name — used for continuity/seam merging (§10.7).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet")
	FString Name;

	bool IsValid() const { return Ref.Num() >= 2; }
};
