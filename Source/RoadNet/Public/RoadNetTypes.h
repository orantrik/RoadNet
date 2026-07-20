#pragma once
#include "CoreMinimal.h"
#include "UObject/SoftObjectPtr.h"
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

// Junction marking treatment, cycled interactively per junction. Each step is a
// superset of the previous paint; Signalized also drops traffic-light meshes.
UENUM(BlueprintType)
enum class ERoadNetJunctionPreset : uint8
{
	None,              // no junction markings
	StopLine,          // stop bar across each entering approach
	StopAndCrosswalk,  // stop bar + zebra crosswalk
	Signalized,        // stop + crosswalk + traffic-light placeholder meshes
	GiveWay            // give-way (yield) dashed bar
};

// Edge treatment for a central median strip.
UENUM(BlueprintType)
enum class ERoadNetMedianEdge : uint8
{
	Plantable,          // raised soil strip + centre spline for PCG tree scatter
	CurbOnly,           // raised soil strip with a kerb on each edge
	SidewalkAndCurb,    // walkable (concrete) raised median with kerbs on each edge
	PlantableWalkCurb   // outer kerb + walkable concrete band + kerbed green planter centre
};

// Persistent per-junction override, keyed by world location and matched by
// proximity so it survives road-index shifts and minor re-smoothing between
// rebuilds (joints themselves are recomputed every rebuild).
USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetJunctionConfig
{
	GENERATED_BODY()

	UPROPERTY()
	FVector2D Location = FVector2D::ZeroVector;

	UPROPERTY()
	ERoadNetJunctionPreset Preset = ERoadNetJunctionPreset::None;

	// Raise curbed grass channelizing islands in this junction's unused corner
	// areas (the pavement wedges between adjacent arms). Toggled per junction.
	UPROPERTY()
	bool bCornerIslands = false;

	// Per-junction morphological-close (smoothing) radius in cm. Negative =
	// inherit the network default (URoadNetwork::JunctionSmoothingCm). Set with
	// the [ / ] hotkeys on the junction under the cursor so each junction can be
	// rounded independently without touching (or rebuilding) the others.
	UPROPERTY()
	float SmoothingCm = -1.f;
};

// Active RoadNet Draw sub-tool. Exactly ONE is active at a time, so a click /
// hotkey has a single unambiguous meaning (no Alt/Ctrl guessing). Driven by the
// roadnet.DrawTool CVar, toggled from the OSM Roads panel segmented control and
// the 1-5 number keys in the viewport. Kept in this shared header so both the
// RoadNet runtime (CVar) and the OSMRoadCore panel (labels) agree on the values.
UENUM(BlueprintType)
enum class ERoadNetDrawTool : uint8
{
	Draw      = 0,  // click drops road points; never selects
	Points    = 1,  // select/move/delete control points, marquee, split, merge
	Lanes     = 2,  // select a lane, insert lanes, cycle lane type
	Junctions = 3,  // junction presets, islands, smoothing, medians
	Edge      = 4   // drag the outer-edge vertices (sidewalk/curb/markings follow)
};

// Shape drawn by the Draw sub-tool. Freehand is the classic click-per-point
// polyline; Roundabout and Curve are two-click geometric primitives (see the
// roadnet.DrawShape CVar). Roundabout: click centre, click/hover for radius ->
// closed circular road. Curve: click start, click end -> a circular arc that
// bends by roadnet.DrawAngleDeg (90/45/25...). Shared header so the RoadNet
// runtime (CVar) and the OSMRoadCore panel (labels) agree on the values.
UENUM(BlueprintType)
enum class ERoadNetDrawShape : uint8
{
	Freehand   = 0,  // click-per-point polyline (original behaviour)
	Roundabout = 1,  // centre + radius -> closed perfect circle
	Curve      = 2,  // start + end -> circular arc bending by DrawAngleDeg
	FreeCurve  = 3   // origin + destination + apex click -> quadratic Bezier
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
	Median,      // median (non-drivable divider)
	Bicycle      // dedicated bicycle path
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

	// ---- central median (divided road) -----------------------------------
	// When bMedian, a raised central strip of MedianWidth splits the
	// carriageway: driving lanes are pushed outward by MedianWidth/2 (a central
	// gap), and the strip is meshed with the chosen edge treatment plus a centre
	// spline for PCG tree scatter.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Median")
	bool bMedian = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Median",
		meta=(ClampMin="30.0", UIMin="30.0", UIMax="1000.0"))
	float MedianWidth = 300.f;        // 3 m

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Median")
	ERoadNetMedianEdge MedianEdge = ERoadNetMedianEdge::Plantable;

	// Half the median gap (cm), or 0 when there is no median.
	float MedianHalfCm() const { return bMedian ? 0.5f * FMath::Max(30.f, MedianWidth) : 0.f; }

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
			return 0.5f * (float)(Hi - Lo) + MedianHalfCm();
		}
		const int32 N = EffectiveLaneCount();
		float W = 0.f;
		for (int32 i = 0; i < N; ++i)
		{
			W += LaneWidths.IsValidIndex(i) ? LaneWidths[i] : LaneWidthDefault;
		}
		return 0.5f * W + MedianHalfCm();
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

		// A central median opens a gap of MedianHalf on each side of the
		// reference line, so the innermost lanes start beyond the median.
		const double MedianHalf = (double)MedianHalfCm();

		// Forward (right) lanes stack from the centerline (or median edge) outward.
		double RightEdge = MedianHalf;
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
		// Backward (left) lanes stack from the centerline (or median edge) outward.
		double LeftEdge = MedianHalf;
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
// Street furniture (street features). A configurable prop type placed
// automatically along the road. Placeholder-first workflow: with no overrides a
// grey box is instanced (HISM) so the layout reads at a glance; assign a
// StaticMesh to swap the instanced mesh, or a Blueprint/actor class to spawn
// real actors instead. Commit resolution order: BlueprintClass > MeshOverride >
// engine cube placeholder.
// ---------------------------------------------------------------------------
UENUM(BlueprintType)
enum class ERoadNetFurniturePlacement : uint8
{
	SpacedPoints,   // one item every SpacingCm (bench, bus stop, kiosk…)
	Continuous      // tiled end-to-end segments along the run (guard rail…)
};

USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetFurnitureType
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Furniture")
	bool bEnabled = true;

	// Label only (helps identify rows in the array editor / instance tags).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Furniture")
	FName Name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Furniture")
	ERoadNetFurniturePlacement Placement = ERoadNetFurniturePlacement::SpacedPoints;

	// Spacing between items (SpacedPoints) or the tile length (Continuous), cm.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Furniture",
		meta=(ClampMin="20.0", UIMin="50.0", UIMax="10000.0"))
	float SpacingCm = 3000.f;

	// Lateral offset (cm) from the carriageway edge. + pushes outward (onto the
	// sidewalk / away from the road), − pulls back toward the centerline.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Furniture",
		meta=(ClampMin="-1000.0", UIMin="-500.0", UIMax="1000.0"))
	float SideOffsetCm = 120.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Furniture")
	bool bLeft = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Furniture")
	bool bRight = true;

	// Extra yaw (deg) on top of the road-facing orientation.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Furniture")
	float YawOffsetDeg = 0.f;

	// Box extent (cm) of the grey placeholder when no mesh/BP is assigned. For a
	// Continuous run the along-road extent is overridden to the tile length so
	// segments join up.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Furniture")
	FVector PlaceholderExtentCm = FVector(60.f, 40.f, 90.f);

	// Assign to swap the instanced placeholder for a real mesh (HISM).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Furniture")
	TSoftObjectPtr<class UStaticMesh> MeshOverride;

	// Assign to spawn actors of this class instead of instancing a mesh.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Furniture")
	TSoftClassPtr<class AActor> BlueprintClass;
};

// ---------------------------------------------------------------------------
// Standard parking bay (street features). A run of marked stalls along one side
// of a road, generated to standard dimensions — the counterpart to the
// free-form Edge-tool bulge. Stored on the road so it persists and reshapes
// with the centerline.
// ---------------------------------------------------------------------------
UENUM(BlueprintType)
enum class ERoadNetParkingLayout : uint8
{
	Parallel,        // stalls parallel to the kerb
	Perpendicular,   // 90° stalls
	Angled           // stalls at AngleDeg to the kerb
};

USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetParkingBay
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Parking")
	ERoadNetSide Side = ERoadNetSide::Right;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Parking")
	ERoadNetParkingLayout Layout = ERoadNetParkingLayout::Parallel;

	// Stall size (cm): Width runs ALONG the kerb, Depth runs OUT from the kerb.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Parking",
		meta=(ClampMin="150.0", UIMin="200.0", UIMax="800.0"))
	float StallWidthCm = 250.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Parking",
		meta=(ClampMin="150.0", UIMin="200.0", UIMax="800.0"))
	float StallDepthCm = 250.f;

	// Angled layout only: stall angle to the kerb (deg).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Parking",
		meta=(ClampMin="30.0", UIMin="30.0", UIMax="90.0"))
	float AngleDeg = 45.f;

	// Arc-length window along the road (cm). LengthCm<=0 ⇒ to the road end.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Parking")
	float StartArcCm = 0.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Parking")
	float LengthCm = 0.f;
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

	// Authored outer-edge profiles (Edge tool). Each knot is (Distance along the
	// reference arc length, signed lateral Offset in cm). When non-empty, the
	// carriageway outer edge on that side follows the profile instead of the
	// uniform ±HalfWidth — and because the sidewalk, curb and edge marking are
	// all derived from the carriageway boundary, they reshape with it (e.g. a
	// parking-bay bulge). OuterEdgeRight is the +offset side, OuterEdgeLeft the
	// −offset side. Empty ⇒ uniform edge.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Geometry")
	TArray<FRoadNetEdgeKnot> OuterEdgeRight;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Geometry")
	TArray<FRoadNetEdgeKnot> OuterEdgeLeft;

	// Standard parking bays authored on this road (street features). Persist with
	// the road and are regenerated to standard stall dimensions each rebuild,
	// alongside (not replacing) any Edge-tool outer-edge bulge.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Parking")
	TArray<FRoadNetParkingBay> ParkingBays;

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
