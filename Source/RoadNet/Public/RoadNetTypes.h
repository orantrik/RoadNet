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

// Persistent per-roundabout override, keyed by world location the same way as
// FRoadNetJunctionConfig. Matched by proximity (within max(kJunctionMatchCm,
// InscribedRadiusCm)) so a rebuild that re-smooths the ring does not orphan it.
USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetRoundaboutConfig
{
	GENERATED_BODY()

	UPROPERTY()
	FVector2D Location = FVector2D::ZeroVector;

	// Centreline radius of the circulating road (cm).
	UPROPERTY()
	float InscribedRadiusCm = 1500.f;

	// Circulating carriageway width (cm). Applied to the ring road on rebuild.
	UPROPERTY()
	float CirculatoryWidthCm = 700.f;

	// Truck apron inside the inner kerb (cm). 0 = island meets the kerb.
	UPROPERTY()
	float ApronWidthCm = 200.f;

	UPROPERTY()
	bool bSplitterIslands = true;

	UPROPERTY()
	bool EntryGiveWay = true;
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
	Lanes     = 2,  // select a lane, insert lanes, cycle lane type, central median
	Junctions = 3,  // junction presets, islands, smoothing
	Edge       = 4,  // drag the outer-edge vertices (sidewalk/curb/markings follow)
	Markings   = 5,  // lane-mark pick + turn-role cycle
	Crosswalk  = 6,  // draw a spline gesture; projects to a centreline zebra
	Island     = 7,  // draw a closed loop for a placed pedestrian island
	CurbBrush  = 8,  // paint existing kerb stones (no road-mesh rebuild)
	BikeCrossing = 9 // draw a path; paints an elephant's-footprint cycle crossing
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
	Bicycle,     // dedicated bicycle path
	Sidewalk     // footway outside the carriageway
};

// Authored turn intent on one lane. Auto = paint whatever the rebuild's
// lane-connectivity graph says that lane does at the next junction. The other
// values FILTER graph movements (a Right lane never gets a left arrow).
UENUM(BlueprintType)
enum class ERoadNetTurnRole : uint8
{
	Auto,
	Through,
	Left,
	Right,
	UTurn,
	None         // never paint a turn mark on this lane
};

// Curb-brush paint type. Unpainted stones keep the CurbA/CurbB zebra.
UENUM(BlueprintType)
enum class ERoadNetCurbPaintType : uint8
{
	Gray,
	WhiteBlack,
	WhiteRed,
	WhiteBlue
};

// Which side of the reference line a lane sits on. RoadBLD ERoadSide parity.
UENUM(BlueprintType)
enum class ERoadNetSide : uint8
{
	Left,    // −lateral offset off the road frame's right axis
	Right,   // +lateral offset
	Center   // straddles the centerline (center-turn / median)
};

// Which way traffic runs along a lane, relative to the road's own +arc
// direction (first reference point → last).
//
// FromSide is the default ON PURPOSE. Before this property existed direction
// was inferred from Side alone, so every authored lane already on disk
// deserialises as FromSide and keeps travelling exactly the way it did. Only
// lanes explicitly set to something else break that rule — which is what makes
// a contraflow bus lane, or a one-way pair on the "wrong" side, expressible.
UENUM(BlueprintType)
enum class ERoadNetLaneDirection : uint8
{
	FromSide,   // legacy rule: the traffic side travels +arc, the other −arc
	Forward,    // +arc (start → end) whatever side the lane sits on
	Backward,   // −arc
	Both,       // bidirectional — centre turn lanes
	None        // not driven at all — parking, median, shoulder
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

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	ERoadNetLaneDirection Direction = ERoadNetLaneDirection::FromSide;

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

	// Authored turn filter. Auto (default) trusts the junction graph.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	ERoadNetTurnRole TurnRole = ERoadNetTurnRole::Auto;

	bool bDrivable() const
	{
		return Type == ERoadNetLaneType::Normal
		    || Type == ERoadNetLaneType::Restricted
		    || Type == ERoadNetLaneType::CenterTurn;
	}

	bool bOutboard() const { return Type == ERoadNetLaneType::Sidewalk; }

	// Does traffic on this lane run start→end along the reference line?
	//
	// bDriveOnLeft only reaches the FromSide fallback: an explicit Direction
	// means what it says on either side of the road, so flipping the network's
	// handedness must not flip it a second time.
	bool bTravelsForward(bool bDriveOnLeft = false) const
	{
		switch (Direction)
		{
		case ERoadNetLaneDirection::Forward:  return true;
		case ERoadNetLaneDirection::Backward: return false;
		case ERoadNetLaneDirection::Both:     return true;
		case ERoadNetLaneDirection::None:     return false;
		default: break;
		}
		if (Side == ERoadNetSide::Center) { return true; }
		return (Side == ERoadNetSide::Right) != bDriveOnLeft;
	}

	bool bTravelsBackward(bool bDriveOnLeft = false) const
	{
		switch (Direction)
		{
		case ERoadNetLaneDirection::Forward:  return false;
		case ERoadNetLaneDirection::Backward: return true;
		case ERoadNetLaneDirection::Both:     return true;
		case ERoadNetLaneDirection::None:     return false;
		default: break;
		}
		if (Side == ERoadNetSide::Center) { return true; }
		return (Side == ERoadNetSide::Left) != bDriveOnLeft;
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

	// ---- outboard cycle track (separated cycling) -------------------------
	// A bike path built OUTSIDE the footway, so the sidewalk is what stands
	// between riders and moving traffic. This is a different thing from
	// ERoadNetLaneType::Bicycle, which is a bike LANE inside the carriageway
	// separated only by paint — a road can carry either, or both.
	// Sides follow the sidewalk convention: left = +right-axis.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	bool bBikePathLeft = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes")
	bool bBikePathRight = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Lanes",
		meta=(ClampMin="60.0", UIMin="60.0", UIMax="600.0"))
	float BikePathWidth = 200.f;      // 2 m

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

	// Half the median gap (cm), or 0 when there is no median. Prefers a Median
	// lane in the stack; falls back to the legacy bMedian/MedianWidth flags.
	float MedianHalfCm() const
	{
		for (const FRoadNetLane& Ln : DetailedLanes)
		{
			if (Ln.Type == ERoadNetLaneType::Median)
			{
				return 0.5f * FMath::Max(30.f, Ln.Width);
			}
		}
		return bMedian ? 0.5f * FMath::Max(30.f, MedianWidth) : 0.f;
	}

	// Stack lanes left→right and pin the carriageway (everything that is not a
	// sidewalk) on the reference line. Median and sidewalk are slots in this
	// stack, not a separate gap.
	static void RelayoutStack(TArray<FRoadNetLane>& Lanes)
	{
		auto WidthOf = [](const FRoadNetLane& Ln) { return (double)FMath::Max(1.f, Ln.Width); };
		double Cursor = 0.0;
		for (FRoadNetLane& Ln : Lanes)
		{
			const double W = WidthOf(Ln);
			Ln.CenterOffset = Cursor + 0.5 * W;
			Cursor += W;
		}
		double Lo = TNumericLimits<double>::Max(), Hi = TNumericLimits<double>::Lowest();
		bool bAny = false;
		for (const FRoadNetLane& Ln : Lanes)
		{
			if (Ln.bOutboard()) { continue; }
			bAny = true;
			Lo = FMath::Min(Lo, Ln.CenterOffset - 0.5 * WidthOf(Ln));
			Hi = FMath::Max(Hi, Ln.CenterOffset + 0.5 * WidthOf(Ln));
		}
		if (!bAny) { Lo = 0.0; Hi = Cursor; }
		const double Mid = 0.5 * (Lo + Hi);
		for (FRoadNetLane& Ln : Lanes)
		{
			Ln.CenterOffset -= Mid;
			Ln.Side = (Ln.Type == ERoadNetLaneType::Median) ? ERoadNetSide::Center
				: (Ln.CenterOffset < 0.0) ? ERoadNetSide::Left : ERoadNetSide::Right;
		}
	}

	void RelayoutDetailed() { RelayoutStack(DetailedLanes); }

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
				if (L.bOutboard()) { continue; }
				Lo = FMath::Min(Lo, L.CenterOffset - 0.5 * L.Width);
				Hi = FMath::Max(Hi, L.CenterOffset + 0.5 * L.Width);
			}
			// No MedianHalfCm() here: RelayoutLanes pins the gap to +/-MedianHalf
			// and stacks the lanes around it, so the span measured above already
			// spans the median. Adding it again counted the median twice, and the
			// two models then disagreed — the count-model branch below adds it
			// because ITS widths are lanes only. Both now report the same number
			// for the same road.
			return 0.5f * (float)(Hi - Lo);
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
	// synthesize a uniform lane set from the count model, stacked outward from
	// the centerline (RoadBLD side-based direction; ROADBLD_FEATURES.md §4).
	//
	// Drive-on-right puts forward traffic on the RIGHT (+offset) and backward on
	// the left; drive-on-left mirrors that. Synthesized lanes carry an explicit
	// Direction rather than leaning on FromSide, so the lane graph reads the same
	// answer under either handedness without inferring it twice.
	TArray<FRoadNetLane> ResolveLanes(bool bDriveOnLeft = false) const
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
		Out.Reserve(Fwd + Bwd + (bMedian ? 1 : 0) + (bSidewalkLeft ? 1 : 0) + (bSidewalkRight ? 1 : 0));
		int32 GlobalIdx = 0;

		auto Emit = [&](int32 Count, bool bLeftSide, ERoadNetLaneDirection Dir)
		{
			for (int32 i = 0; i < Count; ++i)
			{
				const float LW = WidthAt(GlobalIdx++);
				FRoadNetLane L;
				L.LaneId = FGuid::NewGuid();
				L.Type = ERoadNetLaneType::Normal;
				L.Side = bLeftSide ? ERoadNetSide::Left : ERoadNetSide::Right;
				L.Direction = Dir;
				L.Width = LW;
				Out.Add(L);
			}
		};

		auto MakeSlot = [](ERoadNetLaneType Type, float Width) -> FRoadNetLane
		{
			FRoadNetLane L;
			L.LaneId = FGuid::NewGuid();
			L.Type = Type;
			L.Direction = ERoadNetLaneDirection::None;
			L.Width = FMath::Max(30.f, Width);
			L.Side = (Type == ERoadNetLaneType::Median) ? ERoadNetSide::Center : ERoadNetSide::Right;
			return L;
		};

		if (bSidewalkLeft) { Out.Add(MakeSlot(ERoadNetLaneType::Sidewalk, SidewalkWidth)); }
		if (bDriveOnLeft)
		{
			Emit(Fwd, /*bLeftSide*/ true,  ERoadNetLaneDirection::Forward);
			if (bMedian) { Out.Add(MakeSlot(ERoadNetLaneType::Median, MedianWidth)); }
			Emit(Bwd, /*bLeftSide*/ false, ERoadNetLaneDirection::Backward);
		}
		else
		{
			Emit(Bwd, /*bLeftSide*/ true,  ERoadNetLaneDirection::Backward);
			if (bMedian) { Out.Add(MakeSlot(ERoadNetLaneType::Median, MedianWidth)); }
			Emit(Fwd, /*bLeftSide*/ false, ERoadNetLaneDirection::Forward);
		}
		if (bSidewalkRight) { Out.Add(MakeSlot(ERoadNetLaneType::Sidewalk, SidewalkWidth)); }
		RelayoutStack(Out);
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

// ---------------------------------------------------------------------------
// A pedestrian crossing painted somewhere ALONG a road, away from any junction.
// Junction zebras come from the junction preset instead; this is the mid-block
// crossing outside a school or a shop that no junction preset can express.
// ---------------------------------------------------------------------------
USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetCrossingMark
{
	GENERATED_BODY()

	// Arc distance along the reference centreline, cm.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Crossings")
	float DistanceCm = 0.f;

	// Depth of the zebra band along the direction of travel (cm).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Crossings",
		meta=(ClampMin="100.0", UIMin="200.0", UIMax="1000.0"))
	float DepthCm = 400.f;

	// Paint a stop bar on the approach half of each direction, just before the
	// band, the way a signalled or marked crossing is striped.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Crossings")
	bool bStopBar = true;
};

// A pedestrian cut-through on a placed island (crosswalk attaching to it).
USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetIslandPath
{
	GENERATED_BODY()

	UPROPERTY()
	FVector2D A = FVector2D::ZeroVector;

	UPROPERTY()
	FVector2D B = FVector2D::ZeroVector;

	UPROPERTY()
	float WidthCm = 250.f;
};

UENUM(BlueprintType)
enum class ERoadNetPlacedKind : uint8
{
	None,
	Mark,
	Island,
	BikeCrossing,
	CurbPaint
};

// Authored pedestrian island (closed ring in world XY cm). Rebuilt every pass as
// a real body — the ring folds into the median layer, so it gets that layer's
// grass surface and the kerb wrap that already runs around every median.
//
// There is no mesh override: an island is the shape that was drawn. A prefab
// seated at the ring's centroid is a different shape, in a different place, and
// leaves the drawn ring as a bare outline around it.
USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetIsland
{
	GENERATED_BODY()

	UPROPERTY()
	FGuid Id;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Islands")
	TArray<FVector> Ring;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Islands",
		meta=(ClampMin="0.0", UIMin="0.0", UIMax="800.0"))
	float SmoothCm = 150.f;

	UPROPERTY()
	TArray<FRoadNetIslandPath> Paths;
};

// Authored cycle crossing (an "elephant's footprints" crossing): a drawn path in
// world XY cm, painted each rebuild as two rows of square blocks flanking the
// path, optionally with a bicycle stencil at its centre.
//
// Deliberately NOT an ERoadNetMarkKind. A mark kind is one stamp placed at one
// point on one lane; a crossing is a run whose length the user draws, and whose
// blocks have to be laid out along that run.
USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetBikeCrossing
{
	GENERATED_BODY()

	UPROPERTY()
	FGuid Id;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Markings")
	TArray<FVector> Path;

	// Distance between the two rows of blocks (the cycleway's width).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Markings",
		meta=(ClampMin="100.0", UIMin="150.0", UIMax="500.0"))
	float WidthCm = 200.f;
};

// Which arrow a road mark paints. The order IS the mesh/material slot order on
// URoadNetwork (ArrowThrough..ArrowLeftRight) and the order of the panel's
// dropdown, so adding a kind means adding a slot pair and a stencil row.
UENUM(BlueprintType)
enum class ERoadNetMarkKind : uint8
{
	Through      UMETA(DisplayName = "Through"),
	Left         UMETA(DisplayName = "Left"),
	Right        UMETA(DisplayName = "Right"),
	UTurn        UMETA(DisplayName = "U-Turn"),
	ThroughLeft  UMETA(DisplayName = "Through + Left"),
	ThroughRight UMETA(DisplayName = "Through + Right"),
	LeftRight    UMETA(DisplayName = "Left + Right"),
};

// A hand-placed road mark. Unlike the automatic turn arrows (which are derived
// from the lane graph every rebuild and vanish when it changes), this is
// authored data: it stays exactly where it was clicked until it is deleted.
USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetPlacedMark
{
	GENERATED_BODY()

	UPROPERTY()
	FGuid Id;

	// World cm, where the mark was clicked. Kept as the fallback for a mark that
	// resolves to no road, and as the migration source for saves written before
	// the road binding below existed.
	UPROPERTY()
	FVector Location = FVector::ZeroVector;

	// Direction of TRAVEL the arrow points along, degrees. Fallback only once the
	// mark is bound — a bound mark yaws from LaneYawDeg + the road's tangent.
	UPROPERTY()
	float YawDeg = 0.f;

	UPROPERTY()
	ERoadNetMarkKind Kind = ERoadNetMarkKind::Through;

	// ---- road binding (the FRoadNetCurbPaint pattern) ----------------------
	// A world point cannot follow the road it was painted on. Opening a median
	// restacks every lane outward, and a world-anchored arrow stays put — which
	// leaves it under the new median instead of in its lane. Binding the mark to
	// road + arc + LANE means the restack moves it for free, and the same is true
	// of any other edit that reshapes the road.
	//
	// Road == INDEX_NONE means "not resolved yet" (an old save) or "resolved to
	// nothing" (clicked off-road); both fall back to Location.
	UPROPERTY()
	int32 Road = INDEX_NONE;

	// Arc length along that road's reference line (cm).
	UPROPERTY()
	float DistanceCm = 0.f;

	// The lane, left→right, the mark sits in, and its offset from that lane's
	// centre. Indexed rather than keyed by FGuid because a count-model road
	// synthesizes new LaneIds on every ResolveLanes, so a GUID would only be
	// stable on authored roads. Index survives a median change (which never adds
	// or removes lanes); inserting a lane shifts the mark by one, which is
	// visible and fixable, unlike an arrow silently left behind.
	UPROPERTY()
	int32 LaneIndex = INDEX_NONE;

	UPROPERTY()
	float LaneOffsetCm = 0.f;

	// Travel yaw relative to the road's tangent at DistanceCm (deg), so a mark
	// re-yaws when the road is reshaped instead of pointing at the old heading.
	UPROPERTY()
	float LaneYawDeg = 0.f;

	bool IsBoundToRoad() const { return Road != INDEX_NONE; }
};

// Persistent curb-brush sample. Keyed by road + side + arc distance so it
// survives HISM ClearInstances (instance indices do not).
USTRUCT(BlueprintType)
struct ROADNET_API FRoadNetCurbPaint
{
	GENERATED_BODY()

	UPROPERTY()
	FGuid Id;

	UPROPERTY()
	int32 Road = INDEX_NONE;

	UPROPERTY()
	ERoadNetSide Side = ERoadNetSide::Right;

	UPROPERTY()
	float DistanceCm = 0.f;

	UPROPERTY()
	ERoadNetCurbPaintType Type = ERoadNetCurbPaintType::Gray;

	// World XY of the stroke (cm). Rebuild uses Road+Side+Distance; the brush
	// also matches stones by proximity to this point so island kerbs paint.
	UPROPERTY()
	FVector World = FVector::ZeroVector;
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

	// Entry/exit taper (cm) OUTSIDE the window at each end. 0 (default) =
	// constant-depth rectangular pocket. Non-zero ramps the carriageway edge
	// out to the bay depth over that length (trapezoidal inclave).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Parking",
		meta=(ClampMin="0.0", UIMin="0.0", UIMax="2000.0"))
	float TaperCm = 0.f;

	// Resolved FLAT window [S0,S1] on a road of arc length Len (cm). The tapers
	// sit outside it, on [S0-Taper, S0] and [S1, S1+Taper]. False if degenerate.
	FORCEINLINE bool ResolveWindow(double Len, double& OutS0, double& OutS1) const
	{
		OutS0 = FMath::Clamp((double)StartArcCm, 0.0, Len);
		OutS1 = (LengthCm > 0.f) ? FMath::Min(Len, OutS0 + (double)LengthCm) : Len;
		return (OutS1 - OutS0) >= 1.0;
	}

	// Outward bulge (cm) the carriageway edge takes at arc S: the full stall
	// depth across the flat window, ramped linearly over each taper, 0 outside.
	// Shared by the edge geometry and the paint so they cannot disagree.
	FORCEINLINE double BulgeAt(double S, double Len) const
	{
		double S0, S1;
		if (!ResolveWindow(Len, S0, S1)) { return 0.0; }
		const double Depth = FMath::Max(0.0, (double)StallDepthCm);
		if (S >= S0 && S <= S1) { return Depth; }
		const double Taper = FMath::Max(0.0, (double)TaperCm);
		if (Taper <= 0.0) { return 0.0; }
		const double U = (S < S0) ? (S - (S0 - Taper)) / Taper : ((S1 + Taper) - S) / Taper;
		return (U > 0.0) ? Depth * U : 0.0;
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

	// Design speed (km/h) — the key every geometric-design table is looked up
	// by (כרך 1 Table 2.4). 0 means "not authored": RoadNetStandards derives it
	// from Class instead, so roads on disk from before this field keep working.
	// OSM fills it from maxspeed= where the tag exists.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet",
		meta=(ClampMin="0", UIMin="0", UIMax="130"))
	int32 DesignSpeedKph = 0;

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

	// Mid-block pedestrian crossings authored on this road, by arc distance.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Crossings")
	TArray<FRoadNetCrossingMark> Crossings;

	// Deepest bay pocket on this road (cm), 0 with no bays. Everything derived
	// from a constant half-width — the sidewalk masks, the terrain corridor —
	// must reach this much further out, or it clips the retreated walk away.
	FORCEINLINE double MaxBayDepthCm() const
	{
		double D = 0.0;
		for (const FRoadNetParkingBay& B : ParkingBays) { D = FMath::Max(D, (double)B.StallDepthCm); }
		return D;
	}

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

	// Emit a ZoneGraph spline for this segment (traffic AI groundwork).
	//
	// Opt-in PER ROAD, and stored on the road rather than derived from the
	// viewport selection: the selection is transient and long gone by the next
	// rebuild, so a selection-driven design would emit shapes once and lose them.
	// URoadNetwork::bBuildZoneGraph is the master gate over the top of this.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Graph")
	bool bZoneGraph = false;

	bool IsValid() const { return Ref.Num() >= 2; }
};

// One road's ZoneGraph spline, published by the rebuild for an external builder.
//
// RoadNet does NOT depend on the ZoneGraph plugin (its independence mandate
// forbids the coupling), so the rebuild — the only place that knows where the
// junctions are — publishes the trimmed centreline and the lane facts, and
// OSMRoadCore turns them into AZoneShape actors. Serialized, so a project that
// opens without rebuilding still has its shapes.
USTRUCT(BlueprintType)
struct FRoadNetZoneRun
{
	GENERATED_BODY()

	// Stable road id, so a run survives the index churn of a split or a merge.
	UPROPERTY(BlueprintReadOnly, Category="RoadNet|Graph")
	FGuid RoadId;

	// Centreline in world space, already trimmed short of the junction at any
	// end that has one. Empty when the road is entirely inside its junctions.
	UPROPERTY(BlueprintReadOnly, Category="RoadNet|Graph")
	TArray<FVector> Points;

	// Drivable lanes in each direction, and the widest lane width (cm), which is
	// all a ZoneGraph lane profile can express.
	UPROPERTY(BlueprintReadOnly, Category="RoadNet|Graph")
	int32 LanesForward = 1;

	UPROPERTY(BlueprintReadOnly, Category="RoadNet|Graph")
	int32 LanesBackward = 1;

	UPROPERTY(BlueprintReadOnly, Category="RoadNet|Graph")
	float LaneWidthCm = 350.f;

	// A bicycle lane on either side, which is a separate profile in ZoneGraph.
	UPROPERTY(BlueprintReadOnly, Category="RoadNet|Graph")
	bool bHasBikeLane = false;
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

	// Graph classification of this movement (Through/Left/Right/UTurn). Auto
	// is unused here — Emit always writes a concrete kind.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="RoadNet|Graph")
	ERoadNetTurnRole Kind = ERoadNetTurnRole::Through;
};
