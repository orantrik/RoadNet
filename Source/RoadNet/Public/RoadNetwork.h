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

// ---- Terrain-conform corridor (exposed to OSMRoadCore) ---------------------
// One per ground-level road after a rebuild: the SAME smoothed + densified
// centreline the mesh is built from (world cm, Z = draped road bed), plus the
// flat half-width of the paved+walk footprint. OSMRoadCore ramps the landscape
// continuously between these dense points (so terrain can never lap over the
// road between sparse source knots — that gap was the disc-per-point look).
struct FRoadNetDeformCorridor
{
	TArray<FVector> Points;    // densified, G2-smoothed centreline (world cm, Z = bed)
	double FlatHalfCm = 0.0;   // half of carriageway (+median gap) + sidewalk footprint
	bool   bBridge = false;
	bool   bTunnel = false;
	int32  Layer   = 0;        // grade-separation layer (only Layer 0 deforms terrain)
};

// ---- Per-cell terrain-conform cache (§ tiling) -----------------------------
// World-space ground triangle soup contributed by ONE grid cell. Cached so a
// windowed rebuild can reassemble the WHOLE-network conform arrays (clean cells
// from cache + dirty cells recomputed) — the landscape sculpt always sees the
// complete surface, never just the edited region.
struct FRoadNetTileConform
{
	TArray<FVector> Verts; // world cm, one triple per triangle
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

// ---- Street-furniture placements for one rebuild ----------------------------
// One bucket per URoadNetwork::FurnitureTypes entry (parallel by TypeIndex).
// Instances are world-space transforms; CommitFurniture turns each bucket into a
// HISM (or spawned actors) using that type's mesh / Blueprint / cube fallback.
struct FRoadNetFurnitureBucket
{
	int32 TypeIndex = INDEX_NONE;
	TArray<FTransform> Instances;
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
	// Central median strips, per zone (parallel to Zones) — raised block in the
	// carriageway gap of divided roads. Soil = Plantable/CurbOnly (green);
	// Walk = SidewalkAndCurb (concrete, walkable).
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneMedianPolys;
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneMedianWalkPolys;
	// §8.10 lane-marking ribbons, per zone (parallel to Zones), split by colour.
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneMarkingWhitePolys;
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneMarkingYellowPolys;
	// §12.1 per-lane ribbons, per zone (parallel to Zones). Split into two banks
	// by alternating lane index so adjacent lanes render in contrasting shades
	// (an "additional" layer above the unified carriageway).
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneLaneEvenPolys;
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneLaneOddPolys;
	// Typed-lane overlay banks, per zone: lanes cycled to Bicycle / Parking get
	// their own thin surface (their material or a green/amber tint) lifted just
	// above the carriageway, so those lanes read distinctly.
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneLaneBikePolys;
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneLaneParkPolys;
	// Per-zone junction CLIP region (§2.1 "שטח הצומת"): the true junction area
	// bounded by the roads' edge lines and their imaginary extension — computed as
	// the mutual overlap of crossing carriageways, then dilated by the stop-line
	// setback. Markings and lane ribbons are subtracted against this so paint ends
	// at the junction edge (not on a lazy circular disc).
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneJunctionClip;
	// Per-zone junction CARVE region: the clip above grown outward by the sidewalk
	// width (+margin). The raw clip only covers the carriageway overlap, so a
	// road's SIDEWALK runs past the junction uncut and the tile spans it. The
	// carve reaches across the sidewalk band so EVERY layer (surface + sidewalk)
	// breaks at the junction — this is the region tiles are cut/assigned by
	// (the junction tile), while the raw clip stays the paint-clip boundary.
	// Built by BuildTopoAccel.
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneJunctionCarve;
	// § junction traffic-signal placeholder placements (location, yawDeg) for
	// this rebuild — committed as a HISM.
	TArray<TPair<FVector, float>> Signals;
	// § street-furniture placements for this rebuild (one bucket per enabled
	// FurnitureTypes entry). Committed as HISMs / spawned actors.
	TArray<FRoadNetFurnitureBucket> FurnitureBuckets;
	// Flattened unions (for logging/QA only).
	TArray<UE::Geometry::FGeneralPolygon2d> SurfacePolys;
	TArray<UE::Geometry::FGeneralPolygon2d> SidewalkPolys;
	// §10.11 perimeter loops (network outlines + block holes) for PCG export (§8.4).
	TArray<FRoadNetLoop> PerimeterLoops;
	// §12.2 lane-connectivity graph (derived from joints + resolved lanes).
	TArray<FRoadNetLaneConnection> LaneConnections;

	// ---- spatial-commit control (§ tiling) --------------------------------
	// The set of grid cells being (re)committed this pass. When bFullCommit is
	// true every cell is dirty (a full rebuild) and DirtyTiles is ignored;
	// otherwise only cells in DirtyTiles are cleared + repopulated and all other
	// tiles are left untouched (windowed edit). Populated by DeterminePendingRoads.
	TSet<FIntPoint> DirtyTiles;
	bool bFullCommit = true;
	// Optional explicit dirty region (world XY). When valid it OVERRIDES the
	// modified-road corridors when choosing DirtyTiles, so a junction edit
	// (markings / islands / smoothing) re-commits only the junction's own tiles
	// instead of the full length of its (possibly long) arm roads. Invalid box
	// (the default) = derive dirty tiles from the modified roads' corridors.
	FBox2D ExplicitDirtyBox = FBox2D(ForceInit);

	// ---- topological tile accel (§ topo tiles) — transient, built per commit -
	// Flat list of this pass's junction clip regions (one per clip poly) with a
	// cached XY bbox, its owning zone/index (to re-test containment) and its
	// stable junction tile key, so a world point can be mapped to its junction
	// tile with a cheap bbox pre-filter. Built by CommitGeometry (BuildTopoAccel).
	struct FTopoJunctionRegion
	{
		FBox2D Box = FBox2D(ForceInit);
		FIntPoint Key = FIntPoint(0, 0);
		int32 Zone = INDEX_NONE;
		int32 Index = INDEX_NONE;
	};
	TArray<FTopoJunctionRegion> TopoJunctions;
	// Coarse XY grid (cell ~20 m): grid cell -> road indices whose sampled
	// centreline passes through it, to accelerate point->segment assignment in
	// the point-based commits (curbs / furniture / median / perimeters / lanes).
	TMap<FIntPoint, TArray<int32>> RoadSampleGrid;
	// Per-road arm index of each sampled centreline point: which inter-junction
	// stretch the point is on (-1 = the point sits inside a junction clip). A
	// road that passes through N junctions has arms 0..N, so it becomes a
	// SEPARATE segment tile on each side of every junction. Filled by
	// BuildTopoAccel; read by TopoKeyOf. Keyed by road index (== Curves key).
	TMap<int32, TArray<int32>> RoadSampleArm;
	// Per-road contiguous arm RUNS derived from RoadSampleArm: the array index is
	// the arm value (0,1,2,... between successive junctions) and the value is the
	// inclusive [loSample, hiSample] range of that stretch on the sampled
	// centreline. Junction samples (arm -1) are the gaps between runs. Built by
	// BuildTopoAccel; used to slice a segment's own cross-section + splines and to
	// pair divided carriageways. Keyed by road index (== Curves key).
	TMap<int32, TArray<TPair<int32, int32>>> RoadArmRuns;
	// § tiling v2 (ownership by construction): per-layer, per-zone, per-tile
	// polygon buckets built by BuildTilePartition. Each polygon is generated from
	// its own road's arm-run cross-section and tagged with its tile key AT
	// CREATION — CommitLayer consumes these directly and never re-assigns
	// geometry spatially (the v1 routing-heuristic bug class: sidewalk theft).
	// Keyed by CommitLayer's LayerName ("Surface", "Sidewalks").
	TMap<FName, TArray<TMap<FIntPoint, TArray<UE::Geometry::FGeneralPolygon2d>>>> ZoneTileLayers;
	// Later phases populate: overlap masks, details.
};

class UMaterialInterface;
class UStaticMesh;
class ARoadNetTileActor;

UCLASS(BlueprintType, EditInlineNew, DefaultToInstanced)
class ROADNET_API URoadNetwork : public UObject
{
	GENERATED_BODY()

public:
	URoadNetwork();

	// ---- spatial tiling (§ tiling) ----------------------------------------
	// Committed geometry is partitioned into a world-aligned grid of square
	// cells this many cm on a side. Each populated cell is its own
	// ARoadNetTileActor so (a) an edit only re-commits the cells it touches and
	// (b) World Partition can stream the network by location. Default 256 m.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Tiling",
		meta = (ClampMin = "2000.0", UIMin = "6400.0", UIMax = "102400.0"))
	double TileSizeCm = 25600.0;

	// § divided-road tiling: pair the two one-way carriageways of a divided road
	// (opposite direction, parallel, within the gap below) into ONE segment tile
	// so a carriageway + median + partner ride together. Off = each carriageway is
	// its own tile. Every pair it forms is logged so a mis-pair is visible.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Tiling")
	bool bPairDividedRoads = true;

	// Max centre-to-centre lateral gap (cm) between two carriageways still treated
	// as one divided road. Larger pairs wider medians but risks false positives.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Tiling",
		meta = (ClampMin = "200.0", UIMin = "400.0", UIMax = "6000.0"))
	double DividedRoadMaxGapCm = 2500.0;

	// Stable identity for this network, stamped onto every tile actor it spawns
	// so the tile registry can be rebuilt from the level (and multiple networks
	// in one level never claim each other's tiles). Assigned lazily.
	UPROPERTY()
	FGuid NetworkId;

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

	// ---- sidewalk (§8.12) -------------------------------------------------
	// Default sidewalk width (cm) used for newly-drawn roads and by the panel's
	// "Apply Sidewalk Width" action (which also pushes it onto existing roads).
	// Per-road width lives on FRoadNetLaneSpec::SidewalkWidth and can be nudged
	// live with the Edge tool ',' / '.' hotkeys.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Sidewalk",
		meta = (ClampMin = "50.0", UIMin = "100.0", UIMax = "600.0"))
	float DefaultSidewalkWidthCm = 200.f;

	// ---- sampling (§2.6) --------------------------------------------------
	// Arc-length spacing (cm) used to resample every road's reference polyline
	// before offsetting/meshing. Lower = more points per segment = smoother
	// curves and tighter terrain conformance (at the cost of more geometry).
	// Curvature knots are always preserved on top of this uniform spacing.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Sampling",
		meta = (ClampMin = "25.0", UIMin = "50.0", UIMax = "500.0"))
	double PolylineDensityCm = 200.0;

	// Longitudinal GRADE smoothing half-window (metres). The road bed's height
	// profile (Z vs arc length) is fit to a local straight line within ±this
	// distance, turning the draped/cubic-overshoot centreline into a clean ramp
	// so the terrain conform stops stepping/washboarding under the road. On a
	// constant slope the fit is exact (grade preserved); larger = smoother ramp
	// but longer cut/fill approach to real grade changes. 0 disables.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Sampling",
		meta = (ClampMin = "0.0", UIMin = "0.0", UIMax = "60.0"))
	double GradeSmoothingM = 20.0;

	// ---- lanes (§12.1) ----------------------------------------------------
	// Render each resolved lane as its own ribbon strip (alternating shades)
	// layered above the carriageway. Reflects lane add/remove + authored widths.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Lanes")
	bool bShowLaneRibbons = true;

	// Optional material for the per-lane ribbon layer (else a flat shade).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Lanes")
	TObjectPtr<UMaterialInterface> LaneMaterial;

	// Materials for the typed-lane overlay (drawn just above the carriageway for
	// Bicycle / Parking lanes so they read distinctly). If unset, a flat tint is
	// used (green for bike, amber for parking).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Lanes")
	TObjectPtr<UMaterialInterface> BikeLaneMaterial;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Lanes")
	TObjectPtr<UMaterialInterface> ParkingMaterial;

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

	// Standard kerb-piece length (cm). Pieces are tiled at EXACTLY this length on
	// straights (uniform, no stretch) and only compress SHORTER at corners/curves
	// so the stone follows the arc. Lower = finer corners + more instances.
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

	// Corner-island setback (cm): how far a channelizing island is eroded inward
	// from the junction pavement edges, so turning traffic passes around it. The
	// island is the corner pavement between adjacent arms; enable it per junction
	// (RoadNet Draw: click a junction, press K).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Junctions",
		meta = (ClampMin = "0.0", UIMin = "0.0", UIMax = "400.0"))
	double JunctionIslandInsetCm = 90.0;

	// ---- street furniture (street features) -------------------------------
	// Master toggle for automatic street-furniture placement along roads.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Furniture")
	bool bBuildFurniture = true;

	// Furniture types placed each rebuild. Placeholder-first: a null MeshOverride
	// and BlueprintClass instances a grey cube so the layout is visible; assign a
	// Static Mesh to swap the HISM mesh, or a Blueprint/actor class to spawn
	// actors instead. Seeded with Bench / GuardRail / BusStop / Kiosk defaults in
	// the constructor.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Furniture")
	TArray<FRoadNetFurnitureType> FurnitureTypes;

	// ---- standard parking bays (street features) --------------------------
	// Default stall dimensions used by AddStandardParkingBay for each layout
	// (the per-bay FRoadNetParkingBay copies then stores its own values so bays
	// stay stable if the defaults change).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Parking",
		meta = (ClampMin = "150.0", UIMin = "200.0", UIMax = "400.0"))
	float ParkingStallWidthCm = 250.f;      // along-kerb width (perp/angled)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Parking",
		meta = (ClampMin = "150.0", UIMin = "200.0", UIMax = "700.0"))
	float ParkingStallDepthCm = 500.f;      // out-from-kerb depth (perp/angled)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Parking",
		meta = (ClampMin = "300.0", UIMin = "400.0", UIMax = "900.0"))
	float ParkingParallelLengthCm = 600.f;  // along-kerb stall length (parallel)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Parking",
		meta = (ClampMin = "180.0", UIMin = "200.0", UIMax = "350.0"))
	float ParkingParallelDepthCm = 250.f;   // out-from-kerb depth (parallel)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Parking",
		meta = (ClampMin = "30.0", UIMin = "30.0", UIMax = "90.0"))
	float ParkingAngleDeg = 45.f;           // angled layout stall angle

	// Default arc-length window (cm) of a new bay, centred on the road. 0 = as
	// long as the road allows after the junction setback. The bay is an INCLAVE
	// — it bulges the carriageway edge out and the sidewalk retreats around it —
	// so it must stop short of the junctions.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Parking",
		meta = (ClampMin = "0.0", UIMin = "0.0", UIMax = "10000.0"))
	float ParkingBayLengthCm = 3000.f;

	// Clearance (cm) kept between each end of a new bay's taper and the road end,
	// so a pocket never eats into an intersection.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Parking",
		meta = (ClampMin = "0.0", UIMin = "0.0", UIMax = "5000.0"))
	float ParkingBayJunctionSetbackCm = 1200.f;

	// Entry/exit taper (cm) of a new bay — the slanted throat the pocket opens
	// through. ~800 is a 1:3 taper for a 2.5 m deep bay.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Parking",
		meta = (ClampMin = "0.0", UIMin = "0.0", UIMax = "2000.0"))
	float ParkingBayTaperCm = 0.f; // 0 = constant-depth rectangular pocket

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

	// Resolved lanes of a road ordered LEFT→RIGHT (ascending lateral offset).
	// The interactive lane editor selects/indexes lanes in this order, and the
	// per-lane edit ops below take that same left→right index. Empty on bad idx.
	TArray<FRoadNetLane> GetLanesLeftToRight(int32 RoadIdx) const;

	// Materialise a road's lanes as authored DetailedLanes (each with a stable
	// LaneId) so the interactive editor can track a selection across rebuilds.
	// A no-op if already authored; geometry is unchanged. Returns false on bad
	// index. Caller should Modify()/persist as needed (no Rebuild required).
	bool MaterializeLanes(int32 RoadIdx);

	// Insert a new normal lane immediately to the RIGHT (bRightSide) or LEFT of
	// the lane at left→right index LaneLtoR. Converts the road to authored
	// DetailedLanes on first use, then relays out all offsets. Returns the new
	// left→right index of the ORIGINALLY selected lane (so the caller can keep
	// its highlight), or INDEX_NONE on failure. Caller triggers Rebuild().
	int32 InsertLaneRelative(int32 RoadIdx, int32 LaneLtoR, bool bRightSide);

	// Cycle the type of the lane at left→right index LaneLtoR
	// (Normal → Bicycle → Parking → Normal; Dir=-1 reverses), snapping its width
	// to that type's default. Converts to authored DetailedLanes on first use.
	// Returns the new type. Caller triggers Rebuild().
	ERoadNetLaneType CycleLaneType(int32 RoadIdx, int32 LaneLtoR, int32 Dir);

	// ---- outer-edge authoring (Edge tool, §Phase 4) -----------------------
	// Side Right = +offset outer edge, Left = −offset. Distances are arc length
	// along the reference polyline (cm); Offset is signed lateral (cm, +right).

	// Fill Out with the road's authored profile for the side, or — if none is
	// authored yet — a synthesized FLAT profile (evenly spaced knots at the
	// current uniform ±HalfWidth) so the Edge tool always has handles to show.
	// The synthesized layout matches EnsureOuterEdgeProfile, so knot indices line
	// up. Out is emptied on a bad index.
	void GetOuterEdgeForDisplay(int32 RoadIdx, ERoadNetSide Side, TArray<FRoadNetEdgeKnot>& Out) const;

	// Materialise the synthesized flat profile onto the road if the side is empty
	// (no geometry change — it reproduces the uniform edge). Returns true if the
	// side now has a profile. Caller wraps in a transaction / Modify.
	bool EnsureOuterEdgeProfile(int32 RoadIdx, ERoadNetSide Side);

	// Set the lateral offset of knot KnotIdx on the side (materialises the
	// profile first). Clamped to keep the edge on its own side (≥/≤ ±50 cm).
	// Caller triggers Rebuild().
	void SetOuterEdgeKnotOffset(int32 RoadIdx, ERoadNetSide Side, int32 KnotIdx, double Offset);

	// Insert a knot at arc length Distance with lateral Offset (materialises the
	// profile first), kept sorted by Distance. Returns the new knot index.
	int32 AddOuterEdgeKnot(int32 RoadIdx, ERoadNetSide Side, double Distance, double Offset);

	// Remove knot KnotIdx, keeping at least two knots. Returns true if removed.
	bool RemoveOuterEdgeKnot(int32 RoadIdx, ERoadNetSide Side, int32 KnotIdx);

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

	// ---- sidewalk width (street features) ---------------------------------
	// Set every road's sidewalk width (cm) and enable both sides where a width
	// is present. Also updates DefaultSidewalkWidthCm. Caller triggers Rebuild().
	void SetAllSidewalkWidth(float WidthCm);

	// ---- smooth roads (street features) ------------------------------------
	// Simplify EVERY road's reference polyline (Ramer–Douglas–Peucker, max
	// deviation SimplifyTolCm) and round any remaining sharp corner (turn >
	// CornerAngleDeg) by corner-cutting, so the G2 rebuild spline yields clean
	// bezier-quality curves — roundabouts read round, import kinks disappear.
	// Junction topology is PRESERVED: endpoints and any point whose OSM node is
	// shared with another road are never moved or removed. Elev/NodeIds stay
	// parallel (new cut points get node id -1). Pressing again smooths further
	// (each pass is one corner-cut iteration). Returns the number of roads
	// changed. Caller triggers Rebuild().
	int32 SmoothAllRoads(float SimplifyTolCm = 150.f, float CornerAngleDeg = 18.f,
		float CornerMaxCutCm = 600.f);

	// Nudge one road's sidewalk width by DeltaCm (clamped ≥ 0). Enables both
	// sides on first widening. Returns the new width. Caller triggers Rebuild().
	float AdjustSidewalkWidth(int32 RoadIdx, float DeltaCm);

	// ---- standard parking bays (street features) --------------------------
	// Append a standard parking bay to a road on the given side + layout, using
	// the network's default stall dimensions. CenterArcCm is the arc-length (cm)
	// along the road where the bay is centred; < 0 centres it on the road mid.
	// Returns the new bay index in ParkingBays, or INDEX_NONE on a bad road.
	// Caller triggers Rebuild().
	int32 AddStandardParkingBay(int32 RoadIdx, ERoadNetSide Side, ERoadNetParkingLayout Layout,
		double CenterArcCm = -1.0);

	// Remove all standard parking bays from a road. Returns the count removed.
	// Caller triggers Rebuild().
	int32 ClearParkingBays(int32 RoadIdx);

	// Clear all roads (e.g. before a fresh OSM import).
	void ResetRoads();

	// Remove all roads whose Source matches (re-import refresh — §9.4 keeps
	// hand-drawn roads by passing ERoadNetSource::OSM).
	int32 RemoveRoadsBySource(ERoadNetSource Source);

	const TArray<FRoadDef>& GetRoads() const { return Roads; }
	int32 NumRoads() const { return Roads.Num(); }

	// Find a road by its stable FGuid (survives index shifts from delete/insert/
	// merge). Returns INDEX_NONE if not present.
	int32 FindRoadById(const FGuid& Id) const;

	// Ensure NetworkId is valid (lazily assigns one). Returns it.
	const FGuid& EnsureNetworkId();

	// Destroy every tile actor belonging to this network (used before a full
	// re-import / reset so no stale geometry survives). Clears the registry.
	void RetireAllTiles();

	// ---- terrain conform (§ landscape deform) -----------------------------
	// Per-road smoothed+densified centrelines + flat half-widths from the LAST
	// rebuild, so OSMRoadCore can ramp the landscape continuously under the road
	// (see FRoadNetDeformCorridor). Regenerated every Rebuild; empty until the
	// first rebuild. RoadNet itself never touches the landscape (dependency only
	// points OSMRoadCore -> RoadNet).
	const TArray<FRoadNetDeformCorridor>& GetDeformCorridors() const { return DeformCorridors; }

	// World-space triangle soup of the LAST rebuild's GROUND-level driving
	// surface (carriageway + sidewalk + median tops; bridges/tunnels/elevated
	// zones excluded). This is the EXACT built mesh, so OSMRoadCore can conform
	// the landscape to the real surface instead of the centreline+falloff
	// approximation (no drift → terrain can never lap over the ribbon). Verts
	// are absolute world cm; Tris are flat index triples. Regenerated every
	// Rebuild; empty until the first rebuild. Not serialized.
	const TArray<FVector>& GetConformVerts() const { return ConformVerts; }
	const TArray<int32>&   GetConformTris()  const { return ConformTris;  }

	// Bumped at the end of every Rebuild (full or windowed). The editor mode
	// watches this so the terrain conform runs after ANY authoring edit --
	// drawing, dragging a point, widening lanes/sidewalks, deleting, merging --
	// instead of only after the Draw tool commits. Transient, so it starts at 0
	// on load and the first rebuild of a session always registers as a change.
	uint32 GetRebuildSerial() const { return RebuildSerial; }

	// ---- interactive edit API (§9.3 edit/split controllers) ---------------
	// Move one reference point of a road to a new world position. Returns false
	// if indices are invalid. Caller triggers Rebuild().
	bool MoveRoadPoint(int32 RoadIdx, int32 PointIdx, const FVector& NewWorldPos);

	// Delete one reference point. If the road drops below 2 points it is removed
	// entirely. Returns true when the whole road was removed (so callers can drop
	// any cached selection/indices, which shift on removal).
	bool DeleteRoadPoint(int32 RoadIdx, int32 PointIdx, bool& bOutRoadRemoved);

	// Delete a set of reference points from ONE road, SPLITTING it into a new
	// road wherever the removed points leave a gap (deleting a contiguous run of
	// interior points breaks the polyline in two — a real hole — instead of
	// bridging across it). Surviving runs of <2 points are dropped. The first
	// surviving run reuses this road's slot (and Id); extra runs are appended via
	// AddRoad (indices only grow, never shift). Returns the number of resulting
	// roads (0 = whole road removed, 1 = trimmed, 2+ = split). Caller rebuilds.
	int32 DeleteRoadPointsSplitting(int32 RoadIdx, const TArray<int32>& PointIdxToRemove);

	// Insert a point after AfterIdx at Pos (mid-span split). Returns false on bad
	// index. Caller triggers Rebuild().
	bool InsertRoadPoint(int32 RoadIdx, int32 AfterIdx, const FVector& Pos);

	// Remove an entire road by index. Returns false on bad index. Note this
	// shifts later indices, so callers must drop cached selections. Caller
	// triggers Rebuild().
	bool RemoveRoad(int32 RoadIdx);

	// Force-merge two or more roads (by index) into ONE multi-lane road,
	// regardless of the automatic import proximity test. The longest member is
	// the primary (keeps its class/name/grade/source); the merged centreline is
	// the cluster MIDLINE (primary resampled, averaged with the nearest point on
	// each other member) and the lane count is the SUM of the members' lanes
	// (→ wider carriageway). Member roads are removed and the merged road added,
	// so cached selections/indices are invalid afterwards. Returns false if
	// fewer than two valid distinct roads were given. Caller triggers Rebuild().
	bool MergeRoads(TArrayView<const int32> RoadIndices);

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

	// Are corner (channelizing) islands enabled on the junction nearest Loc?
	bool ResolveJunctionIslandsNear(const FVector2D& Loc) const;

	// Toggle corner islands on the junction nearest Loc (creating an override
	// entry if needed). Returns the new state. Caller triggers Rebuild().
	bool ToggleJunctionIslandsNear(const FVector2D& Loc);

	// Resolve the effective morphological-close (smoothing) radius for the
	// junction nearest Loc: its per-junction override if set, else the network
	// default (JunctionSmoothingCm).
	double ResolveJunctionSmoothingNear(const FVector2D& Loc) const;

	// Nudge the PER-JUNCTION smoothing of the junction nearest Loc by DeltaCm
	// (clamped 0..300), creating/seeding the override from the current effective
	// value. Returns the new value; OutJunctionLoc is the matched junction's
	// location (for a disc-scoped rebuild). Caller triggers Rebuild().
	double AdjustJunctionSmoothingNear(const FVector2D& Loc, double DeltaCm, FVector2D& OutJunctionLoc);

	// Staged rebuild entry point (§10.18). Empty Modified = rebuild everything.
	// DirtyRegionWorld (when valid) scopes the COMMIT to the grid cells over that
	// world-XY box instead of the modified roads' full corridors — used by
	// junction edits so a change re-commits only the junction's tiles.
	void Rebuild(TArrayView<const int32> Modified = TArrayView<const int32>(),
		const FBox2D& DirtyRegionWorld = FBox2D(ForceInit));

	// Bind a target world for the (future) commit stage that spawns geometry.
	void SetWorld(UWorld* InWorld) { WorldPtr = InWorld; }

#if WITH_EDITOR
	// Undo/redo restores the reflected Roads / JunctionConfigs on this object,
	// but the generated geometry (dynamic-mesh + HISM actors) is not part of the
	// transaction — so regenerate it here to match the restored authoring state.
	virtual void PostEditUndo() override;

	// Called from ARoadNetActor::PostEditUndo (instanced sub-objects don't always
	// receive PostEditUndo themselves). Rebuilds only roads marked by the last
	// edit when possible, else the whole network.
	void NotifyAuthoringUndoRedo();

	// Remember which road(s) an edit touched so undo/redo can window the rebuild
	// instead of re-meshing the whole city (Ctrl+Z must stay interactive).
	void MarkRoadForUndoRebuild(int32 RoadIdx);

	// Call at the start of a new editor transaction so the undo window is fresh.
	void BeginAuthoringEdit();
#endif

private:
	UPROPERTY()
	TArray<FRoadDef> Roads;

	TWeakObjectPtr<UWorld> WorldPtr;

	// ---- spatial tile registry (§ tiling) ---------------------------------
	// Live map of grid cell -> tile actor for this network. Transient: rebuilt
	// from the level (scanning ARoadNetTileActor with our NetworkId) on demand
	// via EnsureTileRegistry(), so it survives editor reloads without being
	// serialized. GetOrCreateTile spawns a tile the first time a cell is used.
	TMap<FIntPoint, TWeakObjectPtr<ARoadNetTileActor>> TileActors;
	bool bTileRegistryLoaded = false;

	// ---- incremental caches (§ conform-cache) -----------------------------
	// A windowed rebuild recomputes only the roads in the edit window, so the
	// WHOLE-network terrain-conform outputs (GetDeformCorridors / GetConformVerts
	// / GetConformTris) are reassembled from clean cache entries (untouched
	// roads / cells) + freshly recomputed ones after every rebuild. Transient:
	// empty on load, so the first rebuild is a full rebuild that fills them.
	mutable TMap<FGuid, FRoadNetDeformCorridor> DeformCache;
	TMap<FIntPoint, FRoadNetTileConform> ConformCache;

	// Last-known tile CORRIDOR (the grid cells the road's polyline passes through,
	// plus a reach ring) per road GUID, so a windowed rebuild can dirty a MOVED
	// road's OLD cells as well as its new ones (its geometry must be cleared from
	// where it used to be). Stored as a corridor — NOT an AABB — so a long or
	// diagonal road only marks the thin band of cells it actually occupies, not
	// the whole bounding rectangle. Refreshed every rebuild.
	mutable TMap<FGuid, TArray<FIntPoint>> LastRoadCells;

	// ---- topological tile keying (§ topo tiles) ---------------------------
	// Tiles are cut at the junction boundary, so a tile is a SEGMENT (a road
	// between junctions) or a JUNCTION, not a grid cell. We reuse FIntPoint as an
	// identity key: .X = stable id, .Y = kind (0 = segment, 1 = junction). These
	// maps assign stable ids so windowed rebuilds hit the same tile actor; they
	// are reset on every full rebuild (all tiles are recreated) and reused across
	// windowed edits within a session.
	TMap<TPair<FGuid, int32>, int32> SegKeyOf;  // (road GUID, arm)          -> segment id
	// Divided-road pairing: a member carriageway's (GUID, arm) key redirected to
	// the pair's CANONICAL (GUID, arm), so both one-way carriageways of a divided
	// road (and the median between them) resolve to ONE segment tile. Built by
	// BuildDividedPairs; reset with SegKeyOf on every full rebuild. Resolved at
	// the top of SegTileKey so ALL routing (surface / sidewalk / median / splines)
	// honours it uniformly.
	TMap<TPair<FGuid, int32>, TPair<FGuid, int32>> SegAlias;
	TMap<FIntPoint, int32> JunKeyOf;   // quantised junction centre cell -> junction id
	int32 NextSegId = 0;
	int32 NextJunId = 0;
	// Junction tiles each road contributed to in the LAST rebuild, so a windowed
	// edit can dirty (and, if a junction dissolved, retire) them. Parallels
	// LastRoadCells; refreshed for pending roads every rebuild.
	mutable TMap<FGuid, TArray<FIntPoint>> LastRoadJunctions;
	static constexpr int32 kSegKind = 0;
	static constexpr int32 kJunKind = 1;

	// Rebuild TileActors from the level if not already loaded this session.
	void EnsureTileRegistry();
	// Get (spawning on first use) the tile actor for topological key Coord
	// (.X = id, .Y = kind).
	ARoadNetTileActor* GetOrCreateTile(const FIntPoint& Coord);
	// Stable segment tile key for a road GUID + arm index (assigns an id on
	// first use). Arm = which inter-junction stretch of the road; a road that
	// crosses a junction gets a distinct tile per arm.
	FIntPoint SegTileKey(const FGuid& RoadId, int32 Arm = 0);
	// Stable junction tile key for a junction centre (quantised, neighbour-tolerant).
	FIntPoint JunTileKey(const FVector2D& CentreCm);
	// Resolve a world point to its topological tile key for the point-based
	// commits: inside a junction clip region -> that junction; else the nearest
	// road's segment. Returns (INDEX_NONE, kSegKind) when no road is in range.
	FIntPoint TopoKeyOf(const FVector& WorldPos, const FRoadNetRebuildContext& Ctx);
	// Split a world-space polyline at its topological tile boundaries (where
	// TopoKeyOf changes — i.e. at junctions/merges) and add each arc as its own
	// spline to the tile it runs through. Neighbouring arcs share the boundary
	// point so they meet with no gap; the union of arcs equals the input, so no
	// seam coverage is lost — the spline is just re-homed per segment/junction
	// tile so a tile's bounds collapse to its own road. Returns arcs emitted.
	int32 AddSplineSplitByTile(const TArray<FVector>& Points, bool bClosed,
		bool bCurved, const TArray<FName>& Tags, FRoadNetRebuildContext& Ctx);
	// Build the per-commit topological accelerators (Ctx.TopoJunctions with keys
	// + Ctx.RoadSampleGrid) and assign all junction ids for this pass.
	void BuildTopoAccel(FRoadNetRebuildContext& Ctx);
	// Detect divided-road carriageway pairs (opposite one-way, parallel, within
	// DividedRoadMaxGapCm) and fill SegAlias so both arms + their median share ONE
	// segment tile. Logs every pair. No-op when bPairDividedRoads is false.
	void BuildDividedPairs(FRoadNetRebuildContext& Ctx);
	// § tiling v2 — build Ctx.ZoneTileLayers ("Surface" + "Sidewalks") by
	// GENERATING each segment tile's cross-section from its own arm run:
	// carriageway outline slice and per-side sidewalk ribbons (∩ zone band),
	// both minus the junction carve; junction tiles get merged-surface ∩ carve
	// and band ∩ carve. Ownership is fixed at creation — no spatial re-assignment
	// can steal a sidewalk. Also runs the both-sides self-check ([TILECHK]).
	void BuildTilePartition(FRoadNetRebuildContext& Ctx);

	// (Former network-wide Geo* actors removed — all committed geometry now lives
	// in per-cell ARoadNetTileActor components; see the tile registry above.)

	// Persistent per-junction marking overrides (keyed by location).
	UPROPERTY()
	TArray<FRoadNetJunctionConfig> JunctionConfigs;

	// Transient snapshot of the last rebuild's junctions (>=3 arms) for the
	// editor tool to render + hit-test. Not serialized.
	TArray<FRoadNetJunctionView> JunctionViews;

	// Transient per-road terrain-conform corridors from the last rebuild (see
	// GetDeformCorridors / FRoadNetDeformCorridor). Not serialized.
	TArray<FRoadNetDeformCorridor> DeformCorridors;

	// Transient world-space triangle soup of the last rebuild's ground driving
	// surface (see GetConformVerts/GetConformTris). Accumulated in CommitLayer
	// for the layers flagged bConformSurface, skipping elevated zones. Not
	// serialized.
	TArray<FVector> ConformVerts;
	TArray<int32>   ConformTris;

	// See GetRebuildSerial. Not serialized.
	uint32 RebuildSerial = 0;

#if WITH_EDITOR
	// Not serialized / not in the transaction: which roads the last authoring
	// edit touched, so PostEditUndo/NotifyAuthoringUndoRedo can window Rebuild.
	TArray<int32> UndoRebuildRoads;
#endif

	// ---- pipeline stages (§10.18) --------------------------------------
	void DeterminePendingRoads(FRoadNetRebuildContext& Ctx) const;
	void BuildCurves(FRoadNetRebuildContext& Ctx) const;
	void BuildCrossings(FRoadNetRebuildContext& Ctx) const;      // §10.12 grid broadphase (shared)
	void BuildEndpointJoints(FRoadNetRebuildContext& Ctx) const;
	// Vertical alignment: junctions become the vertical points of intersection,
	// the road runs a straight tangent grade between them (with a span-scaled
	// deviation budget so long links may still follow the ground), and each
	// junction gets a flat plate plus a smooth grade transition. Rewrites
	// Ctx.Curves[..].Sampled/LeftEdge/RightEdge Z in place. See RoadNetGrade.cpp.
	void BuildVerticalAlignment(FRoadNetRebuildContext& Ctx) const;
	void BuildZones(FRoadNetRebuildContext& Ctx) const;          // §10.12 grade separation
	void BuildSurfaceUnion(FRoadNetRebuildContext& Ctx) const;   // §10.9 per-zone union + §8.12 sidewalks
	void BuildPerimeterLoops(FRoadNetRebuildContext& Ctx) const; // §10.11 loops for PCG export
	void BuildLaneGraph(FRoadNetRebuildContext& Ctx) const;      // §12.2 lane connectivity
	void BuildLaneRibbons(FRoadNetRebuildContext& Ctx) const;    // §12.1 per-lane ribbon polys
	void BuildJunctionMarkings(FRoadNetRebuildContext& Ctx);     // §2 junction paint + signals
	void BuildJunctionIslands(FRoadNetRebuildContext& Ctx) const;// § corner channelizing grass islands
	void BuildStandardParkingBays(FRoadNetRebuildContext& Ctx) const; // § standard stalls → park overlay + white lines
	void BuildFurniture(FRoadNetRebuildContext& Ctx) const;      // § street-furniture placement sampling
	void CommitGeometry(FRoadNetRebuildContext& Ctx);            // §10.15 mesh + spawn
	void CommitCurbs(FRoadNetRebuildContext& Ctx);               // §8.12 kerb-line HISM
	void CommitFurniture(FRoadNetRebuildContext& Ctx);           // § street-furniture HISM / actors
	void CommitJunctionSignals(FRoadNetRebuildContext& Ctx);     // § signal placeholder HISM
	void CommitMedian(FRoadNetRebuildContext& Ctx);              // § raised median strip + centre splines
	void CommitPerimeters(FRoadNetRebuildContext& Ctx);          // §8.4 spline loops for PCG
	void CommitLaneGraph(FRoadNetRebuildContext& Ctx);           // §12.2 lane-graph splines for PCG
	// § per-segment editable centre + edge splines (one set per segment tile) so
	// each road stretch exposes an editable centreline and its two outer edges.
	void CommitSegmentSplines(FRoadNetRebuildContext& Ctx);

	// Mesh a set of per-zone polygons and route the result into the spatial tile
	// actors (§ tiling): each zone's polygons are clipped to every grid cell they
	// overlap and appended to that cell's named UDynamicMeshComponent (LayerName).
	// If Material is set it is applied to slot 0; otherwise the constant Color is
	// used as a vertex-colour override so the layer is always visible. Only cells
	// allowed by the commit scope (Ctx.bFullCommit / Ctx.DirtyTiles) are written.
	int32 CommitLayer(FName LayerName,
		const TArray<TArray<UE::Geometry::FGeneralPolygon2d>>& ZonePolys,
		double ExtraLiftCm, FColor Color, UMaterialInterface* Material, FRoadNetRebuildContext& Ctx,
		bool bBakeLaneColors = false, bool bWorldUVs = false, bool bConformSurface = false);

	// True if grid cell Coord may be written this commit pass (full rebuild, or
	// Coord is in the dirty set).
	bool IsTileInCommitScope(const FIntPoint& Coord, const FRoadNetRebuildContext& Ctx) const;

	// Clear (or retire) tile actors before repopulating: full rebuild clears ALL
	// registered tiles; a windowed pass clears only the dirty cells. Called once
	// at the top of CommitGeometry.
	void PrepareTilesForCommit(FRoadNetRebuildContext& Ctx);

	// Destroy any tile actor that ended a commit empty (retires cells an edit
	// emptied out). Called at the end of CommitGeometry.
	void RetireEmptyTiles(FRoadNetRebuildContext& Ctx);
	// TODO: overlap masks (§10.10), per-road perimeter loops (§10.11), markings.
};
