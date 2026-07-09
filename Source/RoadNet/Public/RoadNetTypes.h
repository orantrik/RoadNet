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

// Lane semantic type — mirrors RoadBLD ELaneType (minus deprecated None; our
// sidewalks are their own layer, not a lane type). See ROADBLD_FEATURES.md §4.
UENUM(BlueprintType)
enum class ERoadNetLaneType : uint8
{
	Normal,      // normal driving lane
	Parking,     // street-side parking area
	Border,      // border/edge lane
	Restricted,  // restricted (bus/bike/HOV)
	Shoulder,    // shoulder
	CenterTurn,  // center two-way turn lane
	Median       // median (non-drivable divider)
};

// Which side of the reference line a lane sits on. RoadBLD ERoadSide parity:
// direction is side-based (left travels one way, right the other), not a
// per-lane forward/back flag.
UENUM(BlueprintType)
enum class ERoadNetSide : uint8
{
	Left,    // −lateral offset off the road frame's right axis
	Right,   // +lateral offset
	Center   // straddles the centerline (center-turn / median)
};

// A single (distance-along-road, lateral-offset) knot of a lane boundary curve.
// RoadBLD UEdgeCurve OffsetPoints parity. Distances are arc length along the
// reference centerline (cm); Offset is signed lateral cm (+ = right).
USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetEdgeKnot
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	double Distance = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	double Offset = 0.0;
};

// A variable-width section along a lane (turn bay / ramp taper / add-drop).
// RoadBLD FLaneWidthSegment parity: the lane ramps to Width over TransitionIn at
// StartDistance and back over TransitionOut at EndDistance.
USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetLaneWidthSeg
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	double StartDistance = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	double EndDistance = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	double TransitionIn = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	double TransitionOut = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	float Width = 350.f;
};

// A first-class lane (RoadBLD UDynamicRoadLane parity). A lane is the ribbon
// between two boundary curves. For a plain uniform lane, CenterOffset + Width
// fully define it and the edge knot arrays are empty; authored variable lanes
// carry Left/RightEdge knots that override the uniform strip. LaneId is stable
// so the connectivity graph (§12.2) can reference it across rebuilds.
USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetLane
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="RoadNet|Lanes")
	FGuid LaneId;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	ERoadNetLaneType Type = ERoadNetLaneType::Normal;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	ERoadNetSide Side = ERoadNetSide::Right;

	// Signed lateral offset (cm, +right) of the lane centre from the reference.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	double CenterOffset = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	float Width = 350.f;

	// Optional boundary curves (dist→offset). Empty ⇒ derive a uniform strip
	// from CenterOffset ± Width/2.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	TArray<FRoadNetEdgeKnot> LeftEdge;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	TArray<FRoadNetEdgeKnot> RightEdge;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	TArray<FRoadNetLaneWidthSeg> WidthSegments;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	TObjectPtr<class UMaterialInterface> OverlayMaterial = nullptr;

	bool bDrivable() const
	{
		return Type == ERoadNetLaneType::Normal
		    || Type == ERoadNetLaneType::Restricted
		    || Type == ERoadNetLaneType::CenterTurn;
	}
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

	// Authored per-lane entities (RoadBLD-style). When non-empty these OVERRIDE
	// the count model above; when empty, ResolveLanes() synthesizes lanes from
	// the counts so downstream code always iterates real lanes. This is how the
	// flat OSM lane count and hand-authored variable lanes share one path.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	TArray<FRoadNetLane> DetailedLanes;

	bool HasDetailedLanes() const { return DetailedLanes.Num() > 0; }

	// Total carriageway half-width (cm) = sum(lane widths)/2. Computed helper.
	float HalfWidthCm() const
	{
		if (HasDetailedLanes())
		{
			double Lo = 0.0, Hi = 0.0;
			for (const FRoadNetLane& L : DetailedLanes)
			{
				Lo = FMath::Min(Lo, L.CenterOffset - 0.5 * L.Width);
				Hi = FMath::Max(Hi, L.CenterOffset + 0.5 * L.Width);
			}
			return 0.5f * (float)(Hi - Lo);
		}
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
		if (HasDetailedLanes()) { return DetailedLanes.Num(); }
		const int32 Dir = Forward + Backward;
		if (Dir > 0) { return Dir; }
		return FMath::Max(1, Total);
	}

	// Resolve to concrete lane entities: return authored lanes when present, else
	// synthesize a uniform lane set from the count model. Backward lanes land on
	// the LEFT (−offset), forward on the RIGHT (+offset), stacked outward from
	// the centerline (RoadBLD side-based direction; ROADBLD_FEATURES.md §4).
	TArray<FRoadNetLane> ResolveLanes() const
	{
		if (HasDetailedLanes()) { return DetailedLanes; }

		int32 Fwd = Forward, Bwd = Backward;
		if (Fwd + Bwd <= 0)
		{
			const int32 N = FMath::Max(1, Total);
			if (bOneway) { Fwd = N; Bwd = 0; }
			else         { Fwd = (N + 1) / 2; Bwd = N - Fwd; }
		}

		auto WidthAt = [this](int32 GlobalIdx) -> float
		{
			return LaneWidths.IsValidIndex(GlobalIdx) ? LaneWidths[GlobalIdx] : LaneWidthDefault;
		};

		TArray<FRoadNetLane> Out;
		Out.Reserve(Fwd + Bwd);
		int32 GlobalIdx = 0;

		// Forward (right) lanes stack from the centerline outward to +offset.
		double RightEdge = 0.0;
		for (int32 i = 0; i < Fwd; ++i)
		{
			const float LW = WidthAt(GlobalIdx++);
			FRoadNetLane L;
			L.LaneId = FGuid::NewGuid();
			L.Type = ERoadNetLaneType::Normal;
			L.Side = ERoadNetSide::Right;
			L.Width = LW;
			L.CenterOffset = RightEdge + 0.5 * LW;
			RightEdge += LW;
			Out.Add(L);
		}
		// Backward (left) lanes stack from the centerline outward to −offset.
		double LeftEdge = 0.0;
		for (int32 i = 0; i < Bwd; ++i)
		{
			const float LW = WidthAt(GlobalIdx++);
			FRoadNetLane L;
			L.LaneId = FGuid::NewGuid();
			L.Type = ERoadNetLaneType::Normal;
			L.Side = ERoadNetSide::Left;
			L.Width = LW;
			L.CenterOffset = -(LeftEdge + 0.5 * LW);
			LeftEdge += LW;
			Out.Add(L);
		}
		return Out;
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

// ---------------------------------------------------------------------------
// Lane-connectivity graph (§12.2). RoadBLD builds no routing graph, so this is
// net-new: connections are derived from the welded endpoint joints + resolved
// per-side lanes, then exported (spline components) for PCG / traffic. A
// connection is a directed movement from one lane (entering a joint) to another
// lane (leaving it).
// ---------------------------------------------------------------------------
USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetLaneRef
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Graph")
	int32 Road = INDEX_NONE;   // index into URoadNetwork::Roads

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Graph")
	int32 Lane = INDEX_NONE;   // index into that road's resolved lane set
};

USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetLaneConnection
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Graph")
	FRoadNetLaneRef From;      // lane entering the joint

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Graph")
	FRoadNetLaneRef To;        // lane leaving the joint

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Graph")
	int32 Joint = INDEX_NONE;  // index into the rebuild's joint list

	// World-space movement endpoints (cm): where From enters and To leaves.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Graph")
	FVector Entry = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Graph")
	FVector Exit = FVector::ZeroVector;

	// True when From and To are the same travel line through a 2-arm seam
	// (a straight-through movement, not a turn).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Graph")
	bool bThrough = false;
};
