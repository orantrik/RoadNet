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

// ---- Street plan API (the latent-space seam) --------------------------------
// Queryable facts of the LAST rebuild, refreshed by CaptureStreetPlan. This is
// what "elements are aware of each other" means in practice: parcels, fences
// and the landscape read these instead of re-deriving (or guessing) road
// geometry. Everything is world cm; Z is already the reconciled alignment.

// One sidewalk band ring with Z = the TOP of the pavement (bed + slab lift +
// kerb lift), i.e. the height a fence base or parcel plate should meet.
struct FRoadNetPlanEdge
{
	int32 Zone  = 0;
	bool  bHole = false;         // inner ring of the band (faces the kerb)
	TArray<FVector> Points;      // closed ring
};

// Where one parcel meets the street: which road fronts it, where along that
// road, and the sidewalk-top height there. Mirrored onto the parcel actor as
// the osm:frontage tag for modules that must not depend on RoadNet.
struct FRoadNetParcelFrontage
{
	TWeakObjectPtr<AActor> Parcel;
	FGuid  RoadId;               // FRoadDef::Id — survives index shifts
	double StationCm   = 0.0;    // arc along the road's sampled centreline
	double ArcCm       = 0.0;    // arc along the parcel ring (same as osm:access)
	double FrontZTopCm = 0.0;    // sidewalk TOP at the landing point
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

// ---- One road end arriving at a topology node (§10.7) ----------------------
// Everything junction design needs to know about an arm without going back to
// the road: which way it points, how wide it is, how many lanes it carries.
// כרך 2 §3.1.4 gives the main road natural continuity through the junction, so
// one arm pair is elected main and the rest are minor.
struct FRoadNetJointArm
{
	int32 Road = INDEX_NONE;
	bool  bAtStart = true;      // true = the road's first point sits on the node

	// Bearing of the arm as seen FROM the node looking outward along the road,
	// radians in (-pi, pi] from +X. Outward (not inward) so two opposite arms of
	// a straight through-road differ by pi, which is what the main-axis pairing
	// and the through/left/right classification both test.
	double BearingRad = 0.0;

	int32  DrivableLanes = 0;   // lanes that carry traffic, both directions
	double HalfWidthCm = 0.0;   // carriageway half-width incl. any median gap
	ERoadNetClass Class = ERoadNetClass::Unknown;
	int32  DesignSpeedKph = 0;  // resolved, never 0

	// Elected main axis (§3.1.4). At most two arms per joint carry this.
	bool   bMain = false;
};

// ---- Derived topology node (§10.7) -----------------------------------------
struct FRoadNetJoint
{
	int64 NodeId = -1;
	FVector2D Location = FVector2D::ZeroVector;
	// Arms meeting at this node, sorted by outward bearing (§3.1.4), so
	// neighbouring entries are neighbouring approaches going anticlockwise.
	TArray<FRoadNetJointArm> Arms;
	ERoadNetJointKind Kind = ERoadNetJointKind::Terminal;
	double Z = 0.0;

	// Index into Arms of the two elected main-axis arms, or INDEX_NONE. A T
	// junction elects both ends of the through road; a Y with no clear main
	// road elects only one (or none), which is the plane-method fallback signal
	// for the elevation stage (§8.5.4).
	int32 MainA = INDEX_NONE;
	int32 MainB = INDEX_NONE;
};

// ---- Junction-driven widening of one road end (כרך 2 §5.2.4, §6, §7) -------
// The standard never narrows an arm at a junction. Where arms disagree, or a
// turn needs its own lane, the answer is to WIDEN — a surplus through lane is
// carried out of the junction and tapered away past it, and a turn bay is
// added on the approach. Both are the same shape: full extra width at the
// junction end, ramping linearly to nothing over TaperCm.
//
// This rides the outer-edge bulge path BuildCurves already runs for parking
// bays, so no second width pipeline exists to disagree with the first.
struct FRoadNetArmWidening
{
	int32  Road = INDEX_NONE;
	bool   bAtStart = true;                     // which end of the road sits on the junction
	ERoadNetSide Side = ERoadNetSide::Right;    // Right = +offset side, Left = −offset side
	double WidthCm = 0.0;                       // extra carriageway width AT the junction
	double TaperCm = 0.0;                       // length over which it ramps to zero

	// Extra width at arc position S on a road of arc length Len.
	FORCEINLINE double BulgeAt(double S, double Len) const
	{
		if (WidthCm <= 0.0 || TaperCm <= 0.0) { return 0.0; }
		const double D = bAtStart ? S : (Len - S);   // distance from the junction end
		if (D >= TaperCm) { return 0.0; }
		return WidthCm * (1.0 - FMath::Max(0.0, D) / TaperCm);
	}
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
	// כרך 2 §5.2.4 / §6 / §7 — junction lane drops and turn bays, expressed as
	// outer-edge widenings on the arms. Built between the joints and the curves
	// so BuildCurves can fold them into the same variable offset as parking.
	TArray<FRoadNetArmWidening> ArmWidenings;
	// §10.12/§10.8 all 2-D centerline crossings (computed once, shared).
	TArray<FRoadNetCrossing> Crossings;
	// §10.12 grade-separation zones: each group is a set of road indices that are
	// at-grade with one another and get unioned + meshed independently.
	TArray<TArray<int32>> Zones;
	// §10.9 boolean-union surface, per zone (parallel to Zones).
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneSurfacePolys;
	// §8.12 sidewalk bands, per zone (parallel to Zones).
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneSidewalkPolys;
	// Outboard cycle tracks, per zone (parallel to Zones) — the band beyond the
	// sidewalk, so the footway separates riders from traffic. Built after (and
	// subtracted against) ZoneSidewalkPolys, which is what guarantees the two
	// never claim the same ground.
	TArray<TArray<UE::Geometry::FGeneralPolygon2d>> ZoneBikePathPolys;
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
	// § parked cars sitting in standard parking bays. Key = index into
	// URoadNetwork::ParkingCarMeshes, Value = world transform of the stall.
	// Committed as HISMs alongside the furniture.
	TArray<TPair<int32, FTransform>> ParkedCars;
	// § bicycle glyphs at the middle of each drawn cycle crossing (location, yawDeg).
	TArray<TPair<FVector, float>> BikeStencils;
	// Flattened unions (for logging/QA only).
	TArray<UE::Geometry::FGeneralPolygon2d> SurfacePolys;
	TArray<UE::Geometry::FGeneralPolygon2d> SidewalkPolys;
	// Parcel access spurs, kept in their OWN array and never merged into
	// ZoneSidewalkPolys — see URoadNetwork::bBuildParcelAccessPaths for why that
	// separation is the whole point. Flat, for logging/QA; the geometry that gets
	// meshed goes straight into the "Sidewalks" tile buckets.
	TArray<UE::Geometry::FGeneralPolygon2d> ParcelAccessPolys;
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

	// Draw every painted mark as a projected decal instead of a meshed ribbon.
	// Decals conform to whatever they land on — a resurfaced or resculpted road
	// keeps its paint glued down, where a ribbon meshed at a fixed lift can float
	// or sink. Requires the two decal materials below; without them the meshed
	// ribbons are used, because a marking layer with no material is invisible.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Materials")
	bool bMarkingsAsDecals = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Materials",
		meta = (EditCondition = "bMarkingsAsDecals"))
	TSoftObjectPtr<UMaterialInterface> MarkingWhiteDecal;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Materials",
		meta = (EditCondition = "bMarkingsAsDecals"))
	TSoftObjectPtr<UMaterialInterface> MarkingYellowDecal;

	// ---- sidewalk (§8.12) -------------------------------------------------
	// Default sidewalk width (cm) used for newly-drawn roads and by the panel's
	// "Apply Sidewalk Width" action (which also pushes it onto existing roads).
	// Per-road width lives on FRoadNetLaneSpec::SidewalkWidth and can be nudged
	// live with the Edge tool ',' / '.' hotkeys.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Sidewalk",
		meta = (ClampMin = "50.0", UIMin = "100.0", UIMax = "600.0"))
	float DefaultSidewalkWidthCm = 200.f;

	// ---- parcel access paths ----------------------------------------------
	// Branch the sidewalk off the band and land it on each parcel's road-facing
	// edge, so a plot's frontage is reachable on foot instead of being separated
	// from the pavement by bare terrain.
	//
	// The spur is sidewalk MESH — same layer, material, kerb lift, terrain conform
	// and ground skirt — but it is deliberately NOT added to the sidewalk band
	// itself. The band drives three other things besides the mesh (it is carved out
	// of outboard cycle tracks, it generates the kerb line, and it is the placement
	// guard for street furniture), and an access path wants none of them: a garden
	// path with a kerb down both sides and a bench parked on it is not what this is
	// for. So the spurs go straight into the Sidewalks TILE bucket after the
	// partition, which is the one consumer that means "mesh this".
	//
	// Best effort per parcel, never a guarantee: a plot with no road within reach,
	// or whose frontage is already inside the band, gets no path and is counted in
	// the log rather than given an invented route.
	//
	// Costs nothing on a level with no parcels, which is why it is on by default.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Parcel Access")
	bool bBuildParcelAccessPaths = true;

	// Width (cm) of the spur and of the apron along the frontage. Its own setting
	// rather than the sidewalk width: a footpath to a plot reads narrower than the
	// pavement it leaves, and matching them makes the spur look like a mistake in
	// the band rather than a path.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Parcel Access",
		meta = (EditCondition = "bBuildParcelAccessPaths",
			ClampMin = "50.0", UIMin = "80.0", UIMax = "600.0"))
	float ParcelAccessWidthCm = 180.f;

	// How far (m) a parcel's frontage may be from the kerb and still get a path.
	// This is the "which parcels qualify" rule: everything within reach, nothing
	// beyond it. Set it long and a back-lot plot grows an absurd corridor across
	// its neighbours; set it short and genuine frontages are missed.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Parcel Access",
		meta = (EditCondition = "bBuildParcelAccessPaths",
			ClampMin = "1.0", UIMin = "5.0", UIMax = "100.0"))
	float ParcelAccessMaxReachM = 25.f;

	// Length (cm) of the apron laid along the frontage edge where the spur lands.
	// This is the part of the drawing that was a long horizontal stroke rather than
	// a stub: the path arrives and opens out along the plot boundary instead of
	// stopping at a point. 0 gives a bare spur.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Parcel Access",
		meta = (EditCondition = "bBuildParcelAccessPaths",
			ClampMin = "0.0", UIMin = "0.0", UIMax = "3000.0"))
	float ParcelAccessApronCm = 500.f;

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

	// ---- traffic handedness -----------------------------------------------
	// Which side of the road traffic drives on, for the WHOLE network — a city
	// does not change handedness street by street.
	//
	// This decides three things that must agree or the street reads wrong:
	// which side forward lanes stack on, which half of the carriageway a stop
	// bar and its zebra sit on, and whether the centre line is painted yellow
	// (right-hand convention) or white (UK).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Lanes")
	bool bDriveOnLeft = false;

	// Flip the whole network's handedness. Assigning bDriveOnLeft directly is NOT
	// enough on a road that carries authored DetailedLanes: ResolveLanes returns
	// those verbatim (it only mirrors the count model), so the stored per-lane
	// Direction keeps the old country's traffic until it is rewritten here.
	// Returns true when the value actually changed, so callers can skip a rebuild.
	UFUNCTION(BlueprintCallable, Category = "RoadNet|Lanes")
	bool SetDriveOnLeft(bool bNewDriveOnLeft);

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

	// MASTER GATE for the ZoneGraph export. Off publishes nothing regardless of
	// the per-road FRoadDef::bZoneGraph flags, so the whole feature can be parked
	// without clearing the per-road choices you made in the viewport.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Lanes")
	bool bBuildZoneGraph = false;

	// The ZoneGraph splines this network published on its last rebuild, one per
	// road with bZoneGraph, trimmed short of its junctions. RoadNet does not
	// depend on the ZoneGraph plugin; OSMRoadCore reads this and spawns the
	// AZoneShape actors. See FRoadNetZoneRun.
	UPROPERTY()
	TArray<FRoadNetZoneRun> ZoneGraphRuns;

	// Which ends of a road meet another road, by the same spatial endpoint weld
	// BuildEndpointJoints uses. For callers that run OUTSIDE a rebuild, where the
	// joint topology (FRoadNetRebuildContext::Joints) does not exist yet.
	void GetJunctionEnds(int32 RoadIdx, bool& bOutAtStart, bool& bOutAtEnd) const;

	// Set FRoadDef::bZoneGraph on a set of roads. Returns how many changed, so a
	// caller can tell "already on" from "did something". Does NOT rebuild.
	int32 SetZoneGraphOnRoads(TArrayView<const int32> RoadIndices, bool bOn);

	// Refill ZoneGraphRuns for the roads in this commit window. Per-road, keyed on
	// the road's stable id, so a windowed rebuild cannot drop the rest.
	void PublishZoneGraphRuns(FRoadNetRebuildContext& Ctx);

	// Channelize junctions to the Israeli standard (כרך 2): carry surplus
	// through lanes out of the junction and taper them away past it (§5.2.4,
	// Table 5.1), and widen single-lane approaches into turn bays (§6, §7).
	// Disable to get the plain constant-width arms.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Lanes")
	bool bChannelizeJunctions = true;

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

	// Four curb-brush materials (Assets strip). Unpainted stones keep CurbA/B.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Curbs")
	TObjectPtr<UMaterialInterface> CurbPaintGray;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Curbs")
	TObjectPtr<UMaterialInterface> CurbPaintWhiteBlack;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Curbs")
	TObjectPtr<UMaterialInterface> CurbPaintWhiteRed;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Curbs")
	TObjectPtr<UMaterialInterface> CurbPaintWhiteBlue;

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

	// Signal placed at a signalized junction (one per approach). Resolution
	// order mirrors furniture: SignalBlueprint > SignalMesh > engine cylinder
	// scaled into a visible "pole" placeholder.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Junctions")
	TObjectPtr<UStaticMesh> SignalMesh;

	// Assign to spawn real signal actors instead of instancing SignalMesh. The
	// Blueprint's own pivot and scale are used as authored; only the approach
	// position and heading are applied.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Junctions")
	TSoftClassPtr<AActor> SignalBlueprint;

	// Turn-arrow meshes/materials (lane marks). Unset mesh = scaled cube.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TObjectPtr<UStaticMesh> ArrowThroughMesh;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TObjectPtr<UMaterialInterface> ArrowThroughMaterial;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TObjectPtr<UStaticMesh> ArrowLeftMesh;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TObjectPtr<UMaterialInterface> ArrowLeftMaterial;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TObjectPtr<UStaticMesh> ArrowRightMesh;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TObjectPtr<UMaterialInterface> ArrowRightMaterial;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TObjectPtr<UStaticMesh> ArrowUTurnMesh;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TObjectPtr<UMaterialInterface> ArrowUTurnMaterial;

	// COMBINED arrows, for a lane that carries more than one movement. A shared
	// left+through lane is one lane with one arrow showing both heads, not a left
	// arrow and a through arrow stacked on the same tarmac — which is what you got
	// before, because CommitLaneMarks placed one instance per CONNECTION and every
	// movement out of that lane landed on the same setback point. Leave a slot
	// empty and that combination falls back to its dominant single arrow.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TObjectPtr<UStaticMesh> ArrowThroughLeftMesh;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TObjectPtr<UMaterialInterface> ArrowThroughLeftMaterial;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TObjectPtr<UStaticMesh> ArrowThroughRightMesh;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TObjectPtr<UMaterialInterface> ArrowThroughRightMaterial;
	// The שיטה ד case in §7.3.6: one shared lane serving both turns, no widening.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TObjectPtr<UStaticMesh> ArrowLeftRightMesh;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TObjectPtr<UMaterialInterface> ArrowLeftRightMaterial;

	// Ignore the meshes assigned above and use the stencil set shipped in
	// /Game/OSM/Stencils instead. A switch, not a fallback: it overrides even a
	// filled slot, so one tick swaps the whole arrow family without clearing
	// anything. Missing stencils fall back to the assigned mesh, so a partial
	// stencil folder degrades to a mix rather than to nothing.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	bool bUseOSMStencils = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings",
		meta = (ClampMin = "400.0", UIMin = "800.0", UIMax = "4000.0"))
	float ArrowSetbackCm = 1800.f;

	// (There is deliberately no IslandMesh slot. An island is the shape the user
	// drew, built as kerb + grass on the median layer; a prefab dropped at the
	// ring's centroid is a different shape in a different place.)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Islands")
	TArray<FRoadNetIsland> PlacedIslands;

	// Authored cycle crossings (elephant's footprints). Painted into the white
	// marking bank each rebuild, so they reshape with the road like any paint.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TArray<FRoadNetBikeCrossing> BikeCrossings;

	// Optional bicycle glyph stamped at each crossing's midpoint. Without it the
	// crossing is just the two rows of blocks, which is already a legal marking.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TSoftObjectPtr<UStaticMesh> BicycleStencilMesh;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TSoftObjectPtr<UMaterialInterface> BicycleStencilMaterial;

	// Width of a newly drawn cycle crossing (the gap between the two block rows).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings",
		meta = (ClampMin = "100.0", UIMin = "150.0", UIMax = "500.0"))
	float BikeCrossingWidthCm = 200.f;

	// Add a drawn cycle crossing. Returns its index in BikeCrossings, or
	// INDEX_NONE when the path is too short to paint.
	int32 AddBikeCrossing(const TArray<FVector>& Path, float WidthCm);

	// Hand-placed road marks. Authored, so unlike the automatic arrows these are
	// not re-derived on rebuild and survive any change to the lane graph.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Markings")
	TArray<FRoadNetPlacedMark> PlacedMarks;

	UPROPERTY()
	TArray<FRoadNetCurbPaint> CurbPaints;

	// ---- median (§ divided road) ------------------------------------------
	// Material for the raised median strip. If unset a flat green (plantable)
	// colour is used so the median is visible.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Median")
	TObjectPtr<UMaterialInterface> MedianMaterial;

	// ---- junctions (§10.8/§10.9) ------------------------------------------
	// Morphological "close" radius (cm) applied to the merged carriageway. Larger
	// = rounder junction corners / more gap bridging. Adjustable live with the
	// [ and ] hotkeys in the RoadNet Draw mode, which step by 10 cm (50 with
	// Shift) — so the 150 default is the 15 steps a freshly spawned junction
	// used to need by hand before it read as a real corner rather than a
	// mitre. Junction paint and signals set themselves back by this much on top
	// of JunctionClearanceCm: the close fillets the wedge between adjacent
	// arms, which carries the pavement that much further up each approach.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Junctions",
		meta = (ClampMin = "0.0", UIMin = "0.0", UIMax = "300.0"))
	double JunctionSmoothingCm = 150.0;

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

	// Cars parked in the bays. Empty = none, which is why there is no separate
	// on/off flag: clearing the array turns the feature off.
	//
	// One mesh per stall, chosen by hashing the stall's position so the same bay
	// keeps the same cars across rebuilds — a random pick would reshuffle the whole
	// street every time any road changed. Instanced with no collision: they are set
	// dressing, and giving a few hundred of them collision is a traffic hazard for
	// anything pathing down the road.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Parking")
	TArray<TSoftObjectPtr<UStaticMesh>> ParkingCarMeshes;

	// Fraction of stalls that get a car. 1 = a full bay, 0.6 = a realistic street.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RoadNet|Parking",
		meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float ParkingCarFill = 0.7f;

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

	// Backs the RoadNet.LaneSelfCheck console command. A member so it can drive
	// the joint and channelization stages directly — they need no world, and
	// asserting their output is the whole point of the check.
	static void RunSelfCheck();

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

	// ---- cross-section authoring (the Streetmix-style 2D editor) ----------
	// All four take the same left→right index as CycleLaneType, convert the
	// road to authored DetailedLanes on first use, and relayout so the stack
	// stays centred on the reference line. Caller triggers Rebuild().

	// Set one lane's width in cm, clamped to the range a lane can usefully be.
	bool SetLaneWidth(int32 RoadIdx, int32 LaneLtoR, double WidthCm);

	// Set one lane's type WITHOUT snapping its width, unlike CycleLaneType.
	// The 2D editor sets width and type independently, and a palette click that
	// silently discarded a width just dragged is what makes an editor feel
	// broken. Use CycleLaneType for the viewport hotkey, this for the panel.
	bool SetLaneType(int32 RoadIdx, int32 LaneLtoR, ERoadNetLaneType Type);

	// Authored turn-role filter. Converts to DetailedLanes on first use.
	bool SetLaneTurnRole(int32 RoadIdx, int32 LaneLtoR, ERoadNetTurnRole Role);
	ERoadNetTurnRole CycleLaneTurnRole(int32 RoadIdx, int32 LaneLtoR, int32 Dir);

	// Add a placed island from a closed world-XY ring. Returns its index.
	int32 AddPlacedIsland(const TArray<FVector>& Ring);

	// ---- hand-placed road marks -------------------------------------------
	// Add one mark at a world point. Returns its index in PlacedMarks.
	int32 AddPlacedMark(const FVector& WorldLoc, float YawDeg, ERoadNetMarkKind Kind);

	// Delete the nearest mark within RadiusCm of a world point (XY). Returns the
	// index that was removed, or INDEX_NONE if nothing was close enough.
	int32 RemovePlacedMarkNear(const FVector& WorldLoc, double RadiusCm);

	// Delete a placed element by kind + id. Returns true when something went.
	bool RemovePlacedById(ERoadNetPlacedKind Kind, FGuid Id);

	// Heading of travel (degrees) of the road nearest WorldLoc, so a placed arrow
	// lines up with the lane instead of the camera. Returns false when no road is
	// within RadiusCm and the caller should pick its own yaw.
	bool HeadingOfNearestRoad(const FVector& WorldLoc, double RadiusCm, float& OutYawDeg) const;

	// Bind a mark to the road under it (road + arc + lane + relative yaw), reading
	// Mk.Location / Mk.YawDeg. Returns false and leaves the mark world-anchored when
	// no road is within RadiusCm. See FRoadNetPlacedMark for why the binding exists.
	bool BindPlacedMarkToRoad(FRoadNetPlacedMark& Mk, double RadiusCm) const;

	// Where a mark actually goes this rebuild. Bound marks are derived from the
	// road, so a median restack or a reshaped centreline carries them along; an
	// unbound mark falls back to the world point it was clicked at. OutZ is the
	// slab top, resolved from the road model rather than a collision trace.
	bool ResolvePlacedMark(const FRoadNetPlacedMark& Mk, FVector& OutLoc, float& OutYawDeg) const;

	// Cut a pedestrian path through any island the gesture crosses.
	void TryCutIslandPaths(const TArray<FVector>& Gesture);

	// [ / ] on the Island tool: round the nearest island. Returns its index.
	int32 AdjustIslandSmoothNear(const FVector& WorldHit, float DeltaCm);

	// Curb-brush sample at a world hit. Returns stones rebucketed (0 = miss).
	int32 AddCurbPaintNear(const FVector& WorldHit, ERoadNetCurbPaintType Type);

	// Rebucket existing curb HISMs from CurbPaints — no road-mesh rebuild.
	// Returns the number of instances moved onto a paint HISM.
	int32 RebucketCurbPaint();

	// Set which way traffic runs on one lane.
	bool SetLaneDirection(int32 RoadIdx, int32 LaneLtoR, ERoadNetLaneDirection Dir);

	// Delete one lane. Refuses to remove the last, so a road always keeps a
	// carriageway. False on a bad index or that refusal.
	bool RemoveLaneAt(int32 RoadIdx, int32 LaneLtoR);

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

	// Set the median width outright (cm, clamped 30–2000). Returns the new width.
	// The cross-section editor drives this; the hotkeys go through AdjustMedianWidth.
	float SetMedianWidth(int32 RoadIdx, float WidthCm);

	// Set the edge treatment directly. Turns the median on, as cycling does.
	void SetMedianEdge(int32 RoadIdx, ERoadNetMedianEdge Edge);

	// Median state accessors (for HUD/debug and the cross-section view).
	bool IsMedian(int32 RoadIdx) const;
	float GetMedianWidth(int32 RoadIdx) const;
	ERoadNetMedianEdge GetMedianEdge(int32 RoadIdx) const;

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

	// Toggle the outboard cycle track on one road (both sides). Turning it on
	// also turns a sidewalk on if the road has none: the track is DEFINED as the
	// band beyond the footway, so without one there is nothing for it to be
	// outboard of and it would just be a bike lane with extra steps. Returns the
	// new state. Caller triggers Rebuild().
	bool ToggleBikePath(int32 RoadIdx);

	// ---- junction approach conditioning -----------------------------------
	// Control points bunch up around intersections for two reasons, both in the
	// import path: densification subdivides every segment including the short
	// ones at a junction, and arc-length resampling adds a knot wherever the turn
	// is tight — which a junction approach always is. The result is a knot every
	// few centimetres right where the spline most needs room to behave, so the
	// curve kinks and the terrain conform steps.
	//
	// Both passes leave PROTECTED points exactly where they are: an endpoint, or
	// a node id shared with another road, is a weld and moving it would tear the
	// junction apart. This is the same rule SmoothAllRoads uses.

	// Within RadiusCm of a protected point, drop unprotected points that sit
	// closer than MinSpacingCm to the previous kept point. Returns roads changed.
	int32 DeclusterNearJunctions(double RadiusCm = 1500.0, double MinSpacingCm = 400.0);

	// Whole-road packed-knot collapse (XY and Z), keeping junction welds.
	// SmoothG2Spline passes through every Ref point, so a 2 cm cluster with a
	// leftover drape spike becomes a facet across the lane. Returns roads changed.
	int32 DeclusterPackedAlongRoads();

	// Cosine-blend the first LengthCm of each protected point's approach onto the
	// straight tangent leaving it, so a road enters a junction square instead of
	// wandering into it. Full correction at the junction, none at LengthCm out —
	// this is what stops two roads meeting at a visible angle. Returns roads
	// changed.
	int32 StraightenJunctionApproaches(double LengthCm = 2000.0);

	// ---- turning crossings into junctions ---------------------------------
	// Split a road in two at arc distance ArcCm along its reference line. Both
	// halves keep the cut point as an endpoint, and that shared coordinate is
	// exactly what the spatial weld in BuildJoints collapses into one node.
	// Returns the new (far half) road index, or INDEX_NONE when ArcCm sits within
	// the weld radius of an end — there is already an endpoint there to weld
	// against, so cutting would only make a stub. Caller triggers Rebuild().
	int32 SplitRoadAt(int32 RoadIdx, double ArcCm);

	// Turn every at-grade centreline crossing into a real junction by splitting
	// both roads at the crossing point.
	//
	// This exists because a CROSSING and a JUNCTION are different things here:
	// BuildCrossings finds centrelines overlapping in plan, while BuildJoints only
	// ever grows arms from road ENDS. Draw one road over another and the surfaces
	// merge into something that looks like a crossroads, but the arm count never
	// passes 2 — so the lane graph produces no movements and turn arrows, stop
	// lines, corner islands and signals all stay silently empty.
	//
	// Grade-separated crossings are left alone: a bridge crosses on purpose.
	// Returns the number of roads created. Caller triggers Rebuild().
	int32 SplitRoadsAtCrossings(double MaxZGapCm = 350.0);

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

	// ---- mid-block pedestrian crossings -----------------------------------
	// Add a crossing to the road nearest WorldXY, at the arc distance of the
	// closest point on its centreline. A crossing already within MergeCm of that
	// spot is REMOVED instead, so the same click toggles one off. Returns the
	// road index touched (INDEX_NONE if nothing was near enough), and reports
	// through bOutAdded whether paint went on or came off. Caller rebuilds.
	int32 ToggleCrossingNear(const FVector2D& WorldXY, double PickRadiusCm,
		bool& bOutAdded, double MergeCm = 800.0);

	// Add (or update) a mid-block crossing at the nearest centreline. DepthCm
	// is the zebra band along travel. Returns the road index, or INDEX_NONE.
	int32 AddCrossingAt(const FVector2D& WorldXY, float DepthCm, double PickRadiusCm,
		double MergeCm = 800.0);

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

	// LATENT-ONLY rebuild — the street-plan order. Runs the pipeline up to and
	// including the plan (curves, vertical alignment, corridors, surface plan,
	// sidewalk edges), snaps the level's parcel splines onto the sidewalk outer
	// edges and cleans their rings — and STOPS. No triangle is committed and the
	// stale conform soup is dropped, so a landscape conform that follows reads
	// the latent spline plan, never yesterday's mesh. Import ends here; the
	// Build Street orchestrator conforms the landscape and only then runs the
	// full Rebuild to mesh onto the clean bed.
	void RebuildLatent();

	// ---- Street plan API (latent-space seam) -------------------------------
	// Refreshed on every rebuild by CaptureStreetPlan / BuildParcelAccessPaths.
	// Transient: derived facts, never saved — a rebuild is the source of truth.
	TArray<FRoadNetPlanEdge>       PlanSidewalkEdges;
	TArray<FRoadNetParcelFrontage> PlanParcelFrontages;

	// Sidewalk TOP above the height-field bed: the slab lift the road mesh gets
	// plus the kerb lift the sidewalk layer gets. Kept here so the plan API,
	// the frontage stamp and CommitLayer can never disagree about it.
	static constexpr double SidewalkTopLiftCm = 12.0 + 15.0;

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

	// § street validation — walk the COMMITTED street (conform triangle soup,
	// junction plates, plan sidewalk edges) and report every place the built
	// geometry violates the plan's slope contract: lateral slope past
	// roadnet.MaxSideSlopeDeg, tilted junction plates, and vertical steps along
	// the sidewalk outer edge (kerb/skirt discontinuities). With the latent
	// solver upstream this reports zero; it exists to catch regressions.
	// Returns the number of violations. Console: RoadNet.StreetValidate.
	int32 ValidateStreet() const;

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

	// Replace the selected roads with one closed circular ring (Kåsa fit of
	// their points), delete the sources, and retrim nearby approach endpoints
	// onto the ring along each approach's own bearing. Upserts a
	// FRoadNetRoundaboutConfig at the fitted centre. Caller triggers Rebuild().
	bool CleanRoundabout(TArrayView<const int32> RoadIndices);

	// Find the roundabout override nearest Loc, or nullptr.
	const FRoadNetRoundaboutConfig* FindRoundaboutNear(const FVector2D& Loc) const;

	// Create or re-anchor a roundabout override at Loc. Returns its index.
	int32 UpsertRoundaboutAt(const FVector2D& Loc, float InscribedRadiusCm, float CirculatoryWidthCm);

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
	const TArray<FRoadNetRoundaboutConfig>& GetRoundaboutConfigs() const { return RoundaboutConfigs; }

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

	// Branch the sidewalk to each reachable parcel's frontage (RoadNetParcelAccess.cpp).
	//
	// Must run AFTER BuildTopoAccel (it resolves each spur to a tile through TopoKeyOf)
	// and AFTER BuildTilePartition (it appends into the "Sidewalks" buckets that stage
	// creates, and subtracts what is already in them so nothing overlaps).
	void BuildParcelAccessPaths(FRoadNetRebuildContext& Ctx);

	// (Former network-wide Geo* actors removed — all committed geometry now lives
	// in per-cell ARoadNetTileActor components; see the tile registry above.)

	// Persistent per-junction marking overrides (keyed by location).
	UPROPERTY()
	TArray<FRoadNetJunctionConfig> JunctionConfigs;

	UPROPERTY()
	TArray<FRoadNetRoundaboutConfig> RoundaboutConfigs;

	// Transient snapshot of the last rebuild's junctions (>=3 arms) for the
	// editor tool to render + hit-test. Not serialized.
	TArray<FRoadNetJunctionView> JunctionViews;

	// Transient per-road terrain-conform corridors from the last rebuild (see
	// GetDeformCorridors / FRoadNetDeformCorridor). Not serialized.
	TArray<FRoadNetDeformCorridor> DeformCorridors;

	// True only while RebuildLatent() drives Rebuild(): stop after the plan.
	bool bLatentRebuild = false;

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
	// כרך 2 §5.2.4 / §6 / §7 — resolve arm mismatches and turn demand by
	// WIDENING the arms (lane drops carried outside the junction, turn bays
	// added on the approach), never by narrowing anything at the node. Emits
	// Ctx.ArmWidenings, which BuildCurves folds into the outer-edge offset, so
	// this must run after the joints and before the curves. See
	// RoadNetJunctions.cpp.
	void BuildJunctionChannelization(FRoadNetRebuildContext& Ctx) const;
	// Vertical alignment: junctions become the vertical points of intersection,
	// the road runs a straight tangent grade between them (with a span-scaled
	// deviation budget so long links may still follow the ground), and each
	// junction gets a flat plate plus a smooth grade transition. Rewrites
	// Ctx.Curves[..].Sampled/LeftEdge/RightEdge Z in place. See RoadNetGrade.cpp.
	void BuildVerticalAlignment(FRoadNetRebuildContext& Ctx) const;
	void BuildZones(FRoadNetRebuildContext& Ctx) const;          // §10.12 grade separation
	void BuildSurfaceUnion(FRoadNetRebuildContext& Ctx) const;   // §10.9 per-zone union + §8.12 sidewalks
	void CaptureStreetPlan(FRoadNetRebuildContext& Ctx);         // § street plan API (sidewalk edges w/ Z)
	void BindParcelsToStreet(FRoadNetRebuildContext& Ctx);       // § snap/weld/reconcile ALL splines + ring hygiene
	void RefreshPlanSplines(FRoadNetRebuildContext& Ctx);        // § mirror reconciled centrelines as visible spline components
	void BuildPerimeterLoops(FRoadNetRebuildContext& Ctx) const; // §10.11 loops for PCG export
	void BuildLaneGraph(FRoadNetRebuildContext& Ctx) const;      // §12.2 lane connectivity
	void BuildLaneRibbons(FRoadNetRebuildContext& Ctx) const;    // §12.1 per-lane ribbon polys
	void BuildJunctionMarkings(FRoadNetRebuildContext& Ctx);     // §2 junction paint + signals
	void BuildJunctionIslands(FRoadNetRebuildContext& Ctx) const;// § corner channelizing grass islands
	void BuildStandardParkingBays(FRoadNetRebuildContext& Ctx) const; // § standard stalls → park overlay + white lines
	void BuildBikeCrossings(FRoadNetRebuildContext& Ctx) const;       // § elephant's-footprint cycle crossings
	void BuildFurniture(FRoadNetRebuildContext& Ctx) const;      // § street-furniture placement sampling
	void CommitGeometry(FRoadNetRebuildContext& Ctx);            // §10.15 mesh + spawn
	void CommitCurbs(FRoadNetRebuildContext& Ctx);               // §8.12 kerb-line HISM
	// Same polygon banks as CommitLayer, drawn as projected decals instead of
	// meshed ribbons. Returns how many decals were placed.
	int32 CommitMarkingDecals(FName LayerName,
		const TArray<TArray<UE::Geometry::FGeneralPolygon2d>>& ZonePolys,
		UMaterialInterface* DecalMaterial, FRoadNetRebuildContext& Ctx);
	void CommitFurniture(FRoadNetRebuildContext& Ctx);           // § street-furniture HISM / actors
	void CommitJunctionSignals(FRoadNetRebuildContext& Ctx);     // § signal placeholder HISM
	void CommitLaneMarks(FRoadNetRebuildContext& Ctx);           // turn arrows from the lane graph
	void CommitPlacedMarks(FRoadNetRebuildContext& Ctx);         // hand-placed road marks
	// Mesh + material for one arrow kind, honouring bUseOSMStencils. OutKey is
	// the HISM bucket name. Cache spans one commit so a missing stencil warns
	// once instead of once per arrow. Returns null when the slot is empty.
	UStaticMesh* ResolveMarkMesh(ERoadNetMarkKind Kind, TMap<FName, UStaticMesh*>& Cache,
		UMaterialInterface*& OutMat, FName& OutKey) const;
	bool LookupCurbPaint(const FVector& WorldLoc, ERoadNetCurbPaintType& OutType) const;
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
	// bSkirtToGround drops a wall from every polygon boundary to the ground, so a
	// layer raised clear of the road (the sidewalk band) is a slab rather than a
	// floating sheet. The walls are excluded from the terrain conform.
	int32 CommitLayer(FName LayerName,
		const TArray<TArray<UE::Geometry::FGeneralPolygon2d>>& ZonePolys,
		double ExtraLiftCm, FColor Color, UMaterialInterface* Material, FRoadNetRebuildContext& Ctx,
		bool bBakeLaneColors = false, bool bWorldUVs = false, bool bConformSurface = false,
		bool bSkirtToGround = false);

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
