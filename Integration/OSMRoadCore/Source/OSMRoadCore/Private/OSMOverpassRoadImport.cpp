// OSMOverpassRoadImport.cpp
// Fetches OSM highway ways via Overpass, projects them with the shared
// GeoReferencingSystem, drapes + grade-smooths them onto the Landscape, then
// sculpts the road corridors DIRECTLY into the landscape heightmap via
// FHeightmapAccessor (the same operation "Apply Splines to Landscape" performs
// internally). This replaces both the RoadBuilder meshing path and the native
// ULandscapeSplinesComponent graph: the spline graph held thousands of
// interconnected control points/segments that recursed deep enough on GC /
// serialization to throw EXCEPTION_STACK_OVERFLOW, and fought itself at
// junctions. Direct sculpting keeps nothing persistent → no giant object graph.
#include "OSMOverpassRoadImport.h"
#include "OSMRoadDataSubsystem.h"

#if WITH_EDITOR

#include "OSMRoadContract.h"
#include "OSMRoadMeshBuilder.h"
#include "OSMRoadSplineDeform.h"
#include "OSMRoadSplineSource.h"
#include "OSMRoadDressing.h"
#include "OSMRoadDecals.h"
#include "OSMRoadBuilderBridge.h"
#include "OSMRoadNetBridge.h"
#include "OSMRoadLaneMerge.h"
#include "PCGComponent.h"

#include "HAL/IConsoleManager.h"
#include "GeoReferencingSystem.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "HttpModule.h"
#include "HttpManager.h"
#include "Interfaces/IHttpRequest.h"
#include "Interfaces/IHttpResponse.h"
#include "GenericPlatform/GenericPlatformHttp.h"
#include "Misc/FileHelper.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"
#include "Engine/HitResult.h"
#include "CollisionQueryParams.h"

// Direct-heightmap road backend (replaces RoadBuilder meshing AND the native
// landscape-spline graph — see SculptRoadCorridors).
#include "EngineUtils.h"
#include "Landscape.h"
#include "LandscapeProxy.h"
#include "LandscapeInfo.h"
#include "LandscapeEdit.h"        // FHeightmapAccessor — direct heightmap read/write
#include "LandscapeDataAccess.h"  // LANDSCAPE_ZSCALE
#include "LandscapeEditLayer.h"   // ULandscapeEditLayerBase::GetGuid (edit-layer aware write)
#include "DrawDebugHelpers.h"     // persistent debug lines for the spline-preview debug mode

// Node-feature props (Phase 5): instanced meshes at tagged road nodes.
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"

#include <initializer_list>
#include <cfloat>

DEFINE_LOG_CATEGORY_STATIC(LogOSMRoad, Log, All);

// Road terrain-deform backend selector.
//   0 = RoadBuilder model: build the merged road mesh on the draped terrain
//       FIRST, then conform the landscape to that finished mesh (deform AFTER
//       the mesh, no pre-carve). [default]
//   1 = native landscape-spline bake-then-discard (carves via ApplySplines —
//       trench-prone on coarse terrain; kept for A/B only).
static TAutoConsoleVariable<int32> CVarRoadDeformMode(
	TEXT("osm.RoadDeformMode"),
	0,
	TEXT("Road terrain deform backend:\n")
	TEXT("  0 = build merged road mesh first, THEN conform landscape to it (deform\n")
	TEXT("      after the mesh, no pre-carve) [default]\n")
	TEXT("  1 = native landscape-spline bake-then-discard (carves the heightmap;\n")
	TEXT("      trench-prone on coarse terrain — A/B only)\n")
	TEXT("  2 = legacy: Pass-1 corridor pre-carve + mesh + Pass-2 conform (A/B only)"),
	ECVF_Default);

// Road build PIPELINE selector (independent of the deform backend above).
//   0 = C++ mesh pipeline: OSMRoadMeshBuilder surface + sidewalks + dressing
//       scatter (self-contained, no PCG graph). [default]
//   1 = landscape-spline + PCG graph pipeline (Yazan-style): build a PERSISTENT
//       ALandscapeSplineActor (width + per-class LayerName, shared nodes) and
//       (re)generate the PCG_RoadGenerator graph that meshes/dresses it. Requires
//       the PCG_RoadGenerator graph (+ its content) present in /Game/OSM.
static TAutoConsoleVariable<int32> CVarRoadPipeline(
	TEXT("osm.RoadPipeline"),
	1,
	TEXT("Road build pipeline: 0 = C++ mesh + dressing; 1 = landscape-spline source + PCG_RoadGenerator graph (PCG renders road/sidewalks/curbs/decals) [default]; 2 = RoadBuilder plugin engine (per-lane carriageway + curbs/sidewalks + markings + CDT junctions), fed by OSM ways; 4 = RoadNet engine (independent clean-room pipeline: staged rebuild + Clipper2 boolean-union junctions, no RoadBLD dependency)."),
	ECVF_Default);

// Dead-flat buffer (metres) sculpted into the terrain BEYOND the road edge
// (carriageway + sidewalk) on EACH side. The whole strip is hard-pinned to the
// road bed height; the cosine grade back to natural terrain starts only after
// it. 2 m default per road-side clearance requirement.
static TAutoConsoleVariable<float> CVarRoadDeformBufferM(
	TEXT("osm.RoadDeformBufferM"),
	2.0f,
	TEXT("Flat terrain buffer (m) sculpted beyond each road edge before the falloff starts. Applies to corridor sculpting in all pipelines."),
	ECVF_Default);

// Lane-logic pre-pass (Pipeline 1): merge adjacent, near-parallel one-way
// carriageways (divided highways) into a single multi-lane way before building
// the landscape splines. OFF by default — the midline-averaging approach warps
// the geometry at junctions and made things worse (2026-07-06). Superseded by
// polyline-graph merging in BuildRoadSplineSource; kept as an opt-in fallback.
static TAutoConsoleVariable<int32> CVarRoadMergeParallel(
	TEXT("osm.RoadMergeParallel"),
	0,
	TEXT("Pipeline 1: 1 = merge adjacent parallel carriageways into one multi-lane road (lanes summed, midline centre); 0 = off [default]."),
	ECVF_Default);

// In pipeline 1, the PCG_RoadGenerator graph builds curbs/sidewalks/poles/decals
// but NEVER a drivable road surface (Yazan paints it onto the landscape). This
// adds the missing carriageway back as a C++ ProceduralMesh (Section 0 only, no
// sidewalks — PCG owns those), sitting on the spline-deformed road bed.
static TAutoConsoleVariable<int32> CVarRoadPCGCarriageway(
	TEXT("osm.RoadPCGCarriageway"),
	0,
	TEXT("Pipeline 1 only: 0 = the road surface is filled inside the PCG graph by the 'OSM Road Fill' node from the curb boundary vertices [default]; 1 = fall back to the C++ analytic carriageway mesh (centre +/- Scale.Y)."),
	ECVF_Default);

// Debug: draw the road centerline network as persistent debug lines (at the
// final draped + continuity-adjusted heights) and STOP before any mesh build or
// landscape deform — so the "splines" can be inspected first. Colour-coded:
//   ground = green, bridge = orange, tunnel = cyan, layer>0 = yellow.
// Set osm.RoadDebugSplines 0 to do the normal full build.
static TAutoConsoleVariable<int32> CVarRoadDebugSplines(
	TEXT("osm.RoadDebugSplines"),
	0,
	TEXT("1 = draw the road centerlines as persistent debug lines and skip mesh+deform\n")
	TEXT("    (inspect the spline network first); 0 = normal full build [default]."),
	ECVF_Default);

// Junction vertex-snap: reconcile the height of every shared OSM node so all
// roads meeting at an intersection agree on ONE Z (kills the aggressive spikes /
// radical per-approach Z differences at junctions), plus a graded flat landing
// on each approach so it's realistic. 1 = on [default], 0 = off.
static TAutoConsoleVariable<int32> CVarRoadJunctionSnap(
	TEXT("osm.RoadJunctionSnap"),
	1,
	TEXT("1 = snap shared-node heights + grade a flat landing at junctions (no Z\n")
	TEXT("    spikes at intersections) [default]; 0 = off."),
	ECVF_Default);

// Landing radius (cm) for the junction flat-grade (osm.RoadJunctionSnap).
static TAutoConsoleVariable<float> CVarRoadJunctionLandingCm(
	TEXT("osm.RoadJunctionLandingCm"),
	800.f,
	TEXT("Arclength (cm) each road is graded toward the junction height for a flat landing."),
	ECVF_Default);

// XY extent (cm) that groups junction vertices into ONE intersection: any
// junction nodes whose XY fall within this of each other merge into a single
// intersection cluster (catches dual-carriageway / staggered / near-coincident
// nodes). "Highest Z is king" is then applied per cluster.
static TAutoConsoleVariable<float> CVarRoadIntersectionExtentCm(
	TEXT("osm.RoadIntersectionExtentCm"),
	1200.f,
	TEXT("XY extent (cm) within which junction vertices are treated as one intersection."),
	ECVF_Default);

// Phase 1b: topologically fold every multi-node intersection cluster into one
// shared node (id → king, XY → centroid) so downstream sees a single junction.
static TAutoConsoleVariable<int32> CVarRoadIntersectionMerge(
	TEXT("osm.RoadIntersectionMerge"),
	1,
	TEXT("1 = collapse each intersection cluster to a single shared node (topological merge); 0 = keep separate nodes."),
	ECVF_Default);

// CityGML-3.0 inspired: an Intersection is a coherent AREA, not just a point.
// Hold the road flat at the junction Z within this radius of the junction node
// (the "intersection plate"), then grade the approach out to the landing radius.
// 0 = pure point-snap + landing (pre-CityGML behaviour).
static TAutoConsoleVariable<float> CVarRoadJunctionFlatCm(
	TEXT("osm.RoadJunctionFlatCm"),
	300.f,
	TEXT("Radius (cm) of the flat intersection plate held at the junction Z before the graded landing begins (CityGML Intersection-as-area)."),
	ECVF_Default);

// RoadBLD-parity: intersection height matching blends along the NARROWER road
// over (narrow road full width x this multiplier). The plate/landing radii are
// per-junction: the plate grows to cover the widest incident arm (+ margin) and
// the landing grows with this blend distance, so big arterial junctions get a
// proportionally larger flat area than alley crossings. FlatCm/LandingCm above
// act as the minimums.
static TAutoConsoleVariable<float> CVarRoadJunctionBlendWidthMult(
	TEXT("osm.RoadJunctionBlendWidthMult"),
	3.0f,
	TEXT("Junction height-match blend distance = narrowest arm full width x this (RoadBLD IntersectionHeightMatchBlendDistanceMultiplier)."),
	ECVF_Default);

// Extra flat apron past the widest arm's edge on the junction plate/disc
// (RoadBLD IntersectionPatchMarginCm).
static constexpr double kJunctionPlateMarginCm = 75.0;

// CityGML function/priority: the through (higher-class) road should define the
// junction plane. 0 = highest-Z-is-king (operator directive, never lowers a
// vertex) [default]; 1 = major-class-wins (minor roads bend to the arterial's Z,
// which MAY lower a minor approach — more realistic for highway/street crossings).
static TAutoConsoleVariable<int32> CVarRoadJunctionKingMode(
	TEXT("osm.RoadJunctionKingMode"),
	0,
	TEXT("Junction Z authority: 0 = highest Z wins (never lowers); 1 = highest road class wins (through road defines the plane)."),
	ECVF_Default);

// Phase 2: grade-separation correctness. Real tunnels are HIDDEN on the surface
// (no slab, no terrain deform) — the road correctly disappears into the portal
// and reappears at the far end. Culverts stay at grade. Bridge deck clearance
// and per-layer vertical separation are tunable knobs (were hardcoded).
static TAutoConsoleVariable<int32> CVarRoadHideTunnels(
	TEXT("osm.RoadHideTunnels"),
	1,
	TEXT("1 = exclude real tunnels (tunnel=yes/…) from the surface mesh + deform; 0 = drape+render them like ground roads. Culverts are always kept."),
	ECVF_Default);
static TAutoConsoleVariable<float> CVarRoadBridgeClearanceCm(
	TEXT("osm.RoadBridgeClearanceCm"),
	400.f,
	TEXT("Extra deck clearance (cm) added to bridges above terrain at mid-span (sin-tapered to 0 at the ground approaches)."),
	ECVF_Default);
static TAutoConsoleVariable<float> CVarRoadLayerHeightCm(
	TEXT("osm.RoadLayerHeightCm"),
	500.f,
	TEXT("Vertical separation (cm) per OSM layer= level for grade-separated ways."),
	ECVF_Default);

// Phase 3: alignment-feature recognition (slip lanes / links, hairpins, gores).
// Master toggle plus per-feature knobs; recognisers that only change Z are ON,
// mesh-geometry ones (gore/taper) are census-only until a builder pass lands.
static TAutoConsoleVariable<int32> CVarFeatureRecognize(
	TEXT("osm.FeatureRecognize"),
	1,
	TEXT("1 = run the alignment-feature pass (link grade continuity + hairpin/gore census); 0 = skip it."),
	ECVF_Default);
static TAutoConsoleVariable<int32> CVarFeatLinkGrade(
	TEXT("osm.FeatLinkGrade"),
	1,
	TEXT("1 = give highway=*_link ramps/slip lanes a clean monotonic grade between their junction-anchored ends (per span); 0 = leave terrain-draped."),
	ECVF_Default);
static TAutoConsoleVariable<float> CVarFeatHairpinDeg(
	TEXT("osm.FeatHairpinDeg"),
	120.f,
	TEXT("Deflection angle (deg) at an interior vertex above which it is counted as a hairpin/switchback bend (census only). <=0 disables the count."),
	ECVF_Default);

// Phase 5: local traffic calming. Speed humps/tables tagged on nodes raise the
// road Z locally (a small bump the deform then conforms terrain to). Layered on
// top of the reconciled base per the king priority (plan §3 rank 5).
static TAutoConsoleVariable<int32> CVarFeatCalming(
	TEXT("osm.FeatCalming"),
	1,
	TEXT("1 = raise road Z at traffic_calming=hump/bump/table/cushion nodes into a local speed bump; 0 = ignore (census only)."),
	ECVF_Default);
static TAutoConsoleVariable<float> CVarFeatHumpCm(
	TEXT("osm.FeatHumpCm"),
	12.f,
	TEXT("Peak height (cm) of a traffic-calming hump/table bump at its node (tapered to 0 over the hump radius)."),
	ECVF_Default);
static TAutoConsoleVariable<float> CVarFeatHumpRadiusCm(
	TEXT("osm.FeatHumpRadiusCm"),
	300.f,
	TEXT("Along-road radius (cm) over which a calming hump tapers from peak back to the road profile."),
	ECVF_Default);
static TAutoConsoleVariable<int32> CVarFeatProps(
	TEXT("osm.FeatProps"),
	1,
	TEXT("1 = spawn instanced props (bollards, toll booths) at tagged road nodes; 0 = skip."),
	ECVF_Default);

// Debug: build the NATIVE landscape splines (the same network the spline-bake
// path makes, control points shared at OSM nodes) and LEAVE THEM PERSISTENT in
// the level — no heightmap bake, no road mesh — so they can be inspected in
// Landscape mode → Manage → Splines. WARNING: keeping a city-scale graph is the
// known stack-overflow risk; preview only, re-import with 0 to clear.
static TAutoConsoleVariable<int32> CVarRoadSplineDebug(
	TEXT("osm.RoadSplineDebug"),
	0,
	TEXT("1 = create the native landscape splines and keep them persistent (no bake,\n")
	TEXT("    no mesh) for inspection; 0 = normal full build [default]."),
	ECVF_Default);

// Trace straight down at (X,Y) to find the Landscape surface height (engine cm).
// Returns 0 (and bHit=false) if nothing was hit (e.g. no terrain imported yet).
static double SampleTerrainZ(UWorld* World, double X, double Y, bool& bHit)
{
	bHit = false;
	FHitResult Hit;
	const FVector Start(X, Y,  1.0e7);
	const FVector End  (X, Y, -1.0e7);
	FCollisionQueryParams Params(TEXT("OSMRoadTerrainSample"), /*bTraceComplex*/ true);
	if (World->LineTraceSingleByChannel(Hit, Start, End, ECC_WorldStatic, Params))
	{
		bHit = true;
		return Hit.Location.Z;
	}
	return 0.0;
}

// ---------------------------------------------------------------------------
// Highway classification helpers
// ---------------------------------------------------------------------------

EOSMHighwayClass OSMRoadClassify::FromString(const FString& Tag)
{
	if (Tag == TEXT("motorway"))                            return EOSMHighwayClass::Motorway;
	if (Tag == TEXT("trunk"))                               return EOSMHighwayClass::Trunk;
	if (Tag == TEXT("primary"))                             return EOSMHighwayClass::Primary;
	if (Tag == TEXT("secondary"))                           return EOSMHighwayClass::Secondary;
	if (Tag == TEXT("tertiary"))                            return EOSMHighwayClass::Tertiary;
	if (Tag == TEXT("residential"))                         return EOSMHighwayClass::Residential;
	if (Tag == TEXT("unclassified"))                        return EOSMHighwayClass::Unclassified;
	if (Tag == TEXT("service"))                             return EOSMHighwayClass::Service;
	if (Tag == TEXT("living_street"))                       return EOSMHighwayClass::LivingStreet;
	if (Tag == TEXT("pedestrian"))                          return EOSMHighwayClass::Pedestrian;
	if (Tag == TEXT("footway"))                             return EOSMHighwayClass::Footway;
	if (Tag == TEXT("cycleway"))                            return EOSMHighwayClass::Cycleway;
	if (Tag == TEXT("path"))                                return EOSMHighwayClass::Path;
	if (Tag == TEXT("track"))                               return EOSMHighwayClass::Track;
	if (Tag == TEXT("steps"))                               return EOSMHighwayClass::Steps;
	return EOSMHighwayClass::Unknown;
}

bool OSMRoadClassify::IsDrivable(EOSMHighwayClass Class)
{
	switch (Class)
	{
	case EOSMHighwayClass::Motorway:
	case EOSMHighwayClass::Trunk:
	case EOSMHighwayClass::Primary:
	case EOSMHighwayClass::Secondary:
	case EOSMHighwayClass::Tertiary:
	case EOSMHighwayClass::Residential:
	case EOSMHighwayClass::Unclassified:
	case EOSMHighwayClass::Service:
	case EOSMHighwayClass::LivingStreet:
		return true;
	default:
		return false;
	}
}

// ---------------------------------------------------------------------------
// Liang–Barsky segment clip against an axis-aligned rect. On success returns
// the inside parameter range [t0,t1] along A→B (both in [0,1]).
// ---------------------------------------------------------------------------
static bool ClipSegmentToRect(
	const FVector2D& A, const FVector2D& B,
	double XMin, double YMin, double XMax, double YMax,
	double& OutT0, double& OutT1)
{
	double t0 = 0.0, t1 = 1.0;
	const double dx = B.X - A.X;
	const double dy = B.Y - A.Y;

	const double P[4] = { -dx,  dx, -dy,  dy };
	const double Q[4] = { A.X - XMin, XMax - A.X, A.Y - YMin, YMax - A.Y };

	for (int i = 0; i < 4; ++i)
	{
		if (FMath::IsNearlyZero(P[i]))
		{
			if (Q[i] < 0.0) return false;            // parallel & outside
		}
		else
		{
			const double r = Q[i] / P[i];
			if (P[i] < 0.0) { if (r > t1) return false; if (r > t0) t0 = r; }
			else            { if (r < t0) return false; if (r < t1) t1 = r; }
		}
	}
	OutT0 = t0;
	OutT1 = t1;
	return t0 <= t1;
}

// Clip a way's (lon,lat) polyline to a bbox, emitting one or more contiguous
// in-bounds runs (a pass-through way enters/exits → multiple runs).
static void ClipWayToBBox(
	const TArray<FVector>&  Pts,        // (lon,lat,0)
	double XMin, double YMin, double XMax, double YMax,
	TArray<TArray<FVector>>& OutRuns)
{
	TArray<FVector> Cur;
	auto FlushRun = [&]()
	{
		if (Cur.Num() >= 2) OutRuns.Add(MoveTemp(Cur));
		Cur.Reset();
	};

	for (int32 i = 0; i + 1 < Pts.Num(); ++i)
	{
		const FVector2D A(Pts[i]);
		const FVector2D B(Pts[i + 1]);
		double t0, t1;
		if (!ClipSegmentToRect(A, B, XMin, YMin, XMax, YMax, t0, t1))
		{
			FlushRun();  // this segment lies fully outside
			continue;
		}

		const FVector Ain = FMath::Lerp(Pts[i], Pts[i + 1], t0);
		const FVector Bin = FMath::Lerp(Pts[i], Pts[i + 1], t1);

		// t0>0 means we re-entered the box after a gap → start a fresh run.
		if (Cur.Num() == 0 || t0 > 0.0)
		{
			FlushRun();
			Cur.Add(Ain);
		}
		Cur.Add(Bin);

		if (t1 < 1.0)
			FlushRun();  // exited the box before reaching B
	}
	FlushRun();
}

// NodeId-aware clip. Same as ClipWayToBBox but carries OSM node ids alongside
// the points so shared-control-point junctions survive clipping. Interpolated
// boundary points (where t0>0 or t1<1) get node id -1 (no sharing); original
// vertices keep their node id. NodeIds may be empty → all emitted ids are -1.
static void ClipWayToBBoxN(
	const TArray<FVector>&  Pts,        // (lon,lat,0)
	const TArray<int64>&    NodeIds,    // parallel to Pts, or empty
	double XMin, double YMin, double XMax, double YMax,
	TArray<TArray<FVector>>& OutRuns,
	TArray<TArray<int64>>&   OutNodeRuns)
{
	const bool bHaveIds = (NodeIds.Num() == Pts.Num());
	auto IdAt = [&](int32 Index) -> int64 { return bHaveIds ? NodeIds[Index] : -1; };

	TArray<FVector> Cur;
	TArray<int64>   CurIds;
	auto FlushRun = [&]()
	{
		if (Cur.Num() >= 2)
		{
			OutRuns.Add(MoveTemp(Cur));
			OutNodeRuns.Add(MoveTemp(CurIds));
		}
		Cur.Reset();
		CurIds.Reset();
	};

	for (int32 i = 0; i + 1 < Pts.Num(); ++i)
	{
		const FVector2D A(Pts[i]);
		const FVector2D B(Pts[i + 1]);
		double t0, t1;
		if (!ClipSegmentToRect(A, B, XMin, YMin, XMax, YMax, t0, t1))
		{
			FlushRun();
			continue;
		}

		const FVector Ain = FMath::Lerp(Pts[i], Pts[i + 1], t0);
		const FVector Bin = FMath::Lerp(Pts[i], Pts[i + 1], t1);
		const int64   AinId = (t0 <= 0.0) ? IdAt(i)     : -1;
		const int64   BinId = (t1 >= 1.0) ? IdAt(i + 1) : -1;

		if (Cur.Num() == 0 || t0 > 0.0)
		{
			FlushRun();
			Cur.Add(Ain);     CurIds.Add(AinId);
		}
		Cur.Add(Bin);         CurIds.Add(BinId);

		if (t1 < 1.0)
			FlushRun();
	}
	FlushRun();
}

// ---------------------------------------------------------------------------
// Node-tagged point features (Phase 3.5 infra). OSM tags many road features on
// NODES, not ways: crossings, speed humps, turning circles, bollards, toll
// booths, level crossings, mini-roundabouts, stop/signal control. The way query
// alone can't see them; we fetch the ways' tagged nodes and classify by id so
// later phases (calming Z-bumps, crossings, level-crossing flatten, props) can
// look up node id → feature. For now: recognition + census only.
// ---------------------------------------------------------------------------
enum ENodeFeat : uint32
{
	NF_None          = 0,
	NF_Crossing      = 1u << 0,  // highway=crossing (pedestrian/zebra)
	NF_CalmingHump   = 1u << 1,  // traffic_calming=hump/bump/table/cushion (Z bump)
	NF_CalmingOther  = 1u << 2,  // traffic_calming=chicane/island/choker/rumble…
	NF_TurningCircle = 1u << 3,  // highway=turning_circle/turning_loop (cul-de-sac)
	NF_Bollard       = 1u << 4,  // barrier=bollard
	NF_BarrierOther  = 1u << 5,  // other barrier= (gate/kerb/lift_gate/…)
	NF_Toll          = 1u << 6,  // barrier=toll_booth / highway=toll_gantry
	NF_LevelCrossing = 1u << 7,  // railway=level_crossing / railway=crossing
	NF_MiniRoundabout= 1u << 8,  // highway=mini_roundabout
	NF_Stop          = 1u << 9,  // highway=stop
	NF_Signals       = 1u << 10, // highway=traffic_signals
};

// Classify one node's tag object into an ENodeFeat bitmask (0 = nothing we track).
static uint32 ClassifyNodeFeature(const TSharedPtr<FJsonObject>& Tags)
{
	uint32 F = NF_None;
	FString V;
	if (Tags->TryGetStringField(TEXT("highway"), V))
	{
		const FString H = V.ToLower();
		if      (H == TEXT("crossing"))         F |= NF_Crossing;
		else if (H == TEXT("mini_roundabout"))  F |= NF_MiniRoundabout;
		else if (H == TEXT("turning_circle") || H == TEXT("turning_loop")) F |= NF_TurningCircle;
		else if (H == TEXT("stop"))             F |= NF_Stop;
		else if (H == TEXT("traffic_signals"))  F |= NF_Signals;
		else if (H == TEXT("toll_gantry"))      F |= NF_Toll;
	}
	if (Tags->TryGetStringField(TEXT("traffic_calming"), V))
	{
		const FString C = V.ToLower();
		if (C == TEXT("hump") || C == TEXT("bump") || C == TEXT("table")
			|| C == TEXT("cushion") || C == TEXT("hump_table"))
			F |= NF_CalmingHump;
		else
			F |= NF_CalmingOther;
	}
	if (Tags->TryGetStringField(TEXT("barrier"), V))
	{
		const FString B = V.ToLower();
		if      (B == TEXT("bollard"))     F |= NF_Bollard;
		else if (B == TEXT("toll_booth"))  F |= NF_Toll;
		else                                F |= NF_BarrierOther;
	}
	if (Tags->TryGetStringField(TEXT("railway"), V))
	{
		const FString R = V.ToLower();
		if (R == TEXT("level_crossing") || R == TEXT("crossing")) F |= NF_LevelCrossing;
	}
	return F;
}

// ---------------------------------------------------------------------------
// Overpass JSON parser
// Overpass format: { "elements": [ { "type":"way", "id":…,
//   "tags": { "highway":"…", … }, "geometry": [ {"lat":…,"lon":…}, … ] } ] }
// Also parses standalone node elements that carry road-feature tags →
// OutNodeFeatures[nodeId] = ENodeFeat bitmask.
// ---------------------------------------------------------------------------
// incline= may be "10%", "-5%", "12", or the unsigned words "up"/"down".
// Returns the signed gradient in percent; unsigned/unknown forms → 0.
static float ParseInclinePct(const FString& In)
{
	FString S = In.TrimStartAndEnd();
	S.RemoveFromEnd(TEXT("%"));
	if (S.IsNumeric() || (S.StartsWith(TEXT("-")) && S.RightChop(1).IsNumeric()))
	{
		return FCString::Atof(*S);
	}
	return 0.f;
}

// maxspeed= is normally km/h ("50"); "30 mph" → km/h; locale codes ("RU:urban")
// or "none"/"walk" → 0 (unknown). Returns integer km/h.
static int32 ParseMaxSpeedKph(const FString& In)
{
	const FString S = In.TrimStartAndEnd();
	// Leading numeric run.
	FString Num;
	for (const TCHAR C : S)
	{
		if (FChar::IsDigit(C) || C == TEXT('.')) { Num.AppendChar(C); }
		else { break; }
	}
	if (Num.IsEmpty()) { return 0; }
	const float Val = FCString::Atof(*Num);
	if (S.Contains(TEXT("mph"))) { return FMath::RoundToInt(Val * 1.60934f); }
	return FMath::RoundToInt(Val);
}

static bool ParseOverpassJson(
	const FString&             Json,
	TArray<FOSMRoadWay>&       OutWays,
	TMap<int64, uint32>&       OutNodeFeatures,
	FString&                   OutError)
{
	TSharedPtr<FJsonObject> Root;
	TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
	if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
	{
		OutError = TEXT("JSON parse failed");
		return false;
	}

	const TArray<TSharedPtr<FJsonValue>>* Elements = nullptr;
	if (!Root->TryGetArrayField(TEXT("elements"), Elements) || !Elements)
	{
		OutError = TEXT("No 'elements' array in Overpass response");
		return false;
	}

	for (const TSharedPtr<FJsonValue>& El : *Elements)
	{
		const TSharedPtr<FJsonObject>* ObjPtr = nullptr;
		if (!El->TryGetObject(ObjPtr) || !ObjPtr) continue;
		const TSharedPtr<FJsonObject>& Obj = *ObjPtr;

		FString Type;
		if (!Obj->TryGetStringField(TEXT("type"), Type)) continue;

		// Node elements: classify road-feature tags by node id (census / future
		// feature lookup). Nodes without tracked tags are ignored.
		if (Type == TEXT("node"))
		{
			const TSharedPtr<FJsonObject>* NTagsPtr = nullptr;
			int64 NodeId = 0;
			if (Obj->TryGetObjectField(TEXT("tags"), NTagsPtr) && NTagsPtr
				&& Obj->TryGetNumberField(TEXT("id"), NodeId))
			{
				const uint32 F = ClassifyNodeFeature(*NTagsPtr);
				if (F != NF_None) OutNodeFeatures.Add(NodeId, F);
			}
			continue;
		}
		if (Type != TEXT("way")) continue;

		// Tags
		const TSharedPtr<FJsonObject>* TagsPtr = nullptr;
		if (!Obj->TryGetObjectField(TEXT("tags"), TagsPtr) || !TagsPtr) continue;
		const TSharedPtr<FJsonObject>& Tags = *TagsPtr;

		FString HighwayTag;
		if (!Tags->TryGetStringField(TEXT("highway"), HighwayTag)) continue;

		FOSMRoadWay Way;
		if (!Obj->TryGetNumberField(TEXT("id"), Way.WayId)) continue;

		Way.HighwayRaw = HighwayTag;
		Way.bLink      = HighwayTag.EndsWith(TEXT("_link"));
		FString BaseTag = Way.bLink ? HighwayTag.LeftChop(5) : HighwayTag;
		Way.Class      = OSMRoadClassify::FromString(BaseTag);

		FString StrVal;
		if (Tags->TryGetStringField(TEXT("lanes"), StrVal))
			Way.Lanes = FCString::Atoi(*StrVal);
		if (Tags->TryGetStringField(TEXT("lanes:forward"), StrVal))
			Way.LanesForward = FCString::Atoi(*StrVal);
		if (Tags->TryGetStringField(TEXT("lanes:backward"), StrVal))
			Way.LanesBackward = FCString::Atoi(*StrVal);
		if (Tags->TryGetStringField(TEXT("width"), StrVal))
			Way.WidthM = FCString::Atof(*StrVal);
		if (Tags->TryGetStringField(TEXT("oneway"), StrVal))
			Way.bOneway = (StrVal == TEXT("yes") || StrVal == TEXT("1"));
		if (Tags->TryGetStringField(TEXT("layer"), StrVal))
			Way.Layer = FCString::Atoi(*StrVal);
		if (Tags->TryGetStringField(TEXT("bridge"), StrVal))
			Way.bBridge = (StrVal == TEXT("yes") || StrVal == TEXT("1"));
		if (Tags->TryGetStringField(TEXT("tunnel"), StrVal))
		{
			// tunnel=culvert is a drainage pipe UNDER a road that stays at grade
			// → treat as a ground road (rendered + draped), NOT a hidden tunnel.
			// Any other non-"no" value (yes/1/building_passage/avalanche_protector)
			// is a real tunnel → hidden on the surface (Phase 2).
			const FString T = StrVal.ToLower();
			if      (T == TEXT("culvert"))            Way.bCulvert = true;
			else if (T != TEXT("no") && !T.IsEmpty()) Way.bTunnel  = true;
		}
		Tags->TryGetStringField(TEXT("junction"), Way.JunctionTag);

		// sidewalk= (both | left | right | no | none | separate). "separate"
		// means the sidewalk is mapped as its own footway, so this carriageway
		// shouldn't extrude one. Untagged → class default (resolved later).
		if (Tags->TryGetStringField(TEXT("sidewalk"), StrVal))
		{
			Way.bSidewalkTagged = true;
			const FString S = StrVal.ToLower();
			Way.bSidewalkLeft  = (S == TEXT("both") || S == TEXT("left"));
			Way.bSidewalkRight = (S == TEXT("both") || S == TEXT("right"));
		}

		// --- Extended attributes (Phase 2) ---------------------------------
		Tags->TryGetStringField(TEXT("surface"),         Way.Surface);
		Tags->TryGetStringField(TEXT("smoothness"),      Way.Smoothness);
		Tags->TryGetStringField(TEXT("access"),          Way.Access);
		Tags->TryGetStringField(TEXT("wheelchair"),      Way.Wheelchair);
		Tags->TryGetStringField(TEXT("crossing"),        Way.Crossing);
		Tags->TryGetStringField(TEXT("traffic_calming"), Way.TrafficCalming);
		Tags->TryGetStringField(TEXT("kerb"),            Way.Kerb);
		Tags->TryGetStringField(TEXT("footway"),         Way.FootwayType);
		Tags->TryGetStringField(TEXT("name"),            Way.Name);
		Tags->TryGetStringField(TEXT("name:he"),         Way.NameHe);
		Tags->TryGetStringField(TEXT("name:en"),         Way.NameEn);
		Tags->TryGetStringField(TEXT("ref"),             Way.Ref);

		if (Tags->TryGetStringField(TEXT("incline"), StrVal))
			Way.InclinePct = ParseInclinePct(StrVal);
		if (Tags->TryGetStringField(TEXT("maxspeed"), StrVal))
			Way.MaxSpeedKph = ParseMaxSpeedKph(StrVal);
		if (Tags->TryGetStringField(TEXT("step_count"), StrVal))
			Way.StepCount = FCString::Atoi(*StrVal);
		if (Tags->TryGetStringField(TEXT("tactile_paving"), StrVal))
			Way.bTactilePaving = (StrVal == TEXT("yes"));
		if (Tags->TryGetStringField(TEXT("lit"), StrVal))
			Way.bLit = (StrVal.ToLower() == TEXT("yes"));

		// oneway with explicit direction (-1 = digitised-reverse one-way).
		Way.OnewayDir = Way.bOneway ? 1 : 0;
		if (Tags->TryGetStringField(TEXT("oneway"), StrVal))
		{
			const FString Ow = StrVal.ToLower();
			if      (Ow == TEXT("-1") || Ow == TEXT("reverse")) { Way.OnewayDir = -1; Way.bOneway = true; }
			else if (Ow == TEXT("yes") || Ow == TEXT("1"))      { Way.OnewayDir =  1; Way.bOneway = true; }
			else                                                { Way.OnewayDir =  0; Way.bOneway = false; }
		}

		// Lifecycle (also handles highway=construction/proposed via BaseTag).
		Way.bConstruction = (BaseTag == TEXT("construction")) || Tags->HasField(TEXT("construction"));
		Way.bProposed     = (BaseTag == TEXT("proposed"))     || Tags->HasField(TEXT("proposed"));

		// Catch-all: copy EVERY raw OSM tag into the Way.Tags map so downstream
		// code / PCG graphs can filter on attributes not typed above.
		for (const TPair<FString, TSharedPtr<FJsonValue>>& Pair : Tags->Values)
		{
			FString V;
			if (Pair.Value.IsValid() && Pair.Value->TryGetString(V))
			{
				Way.Tags.Add(Pair.Key, V);
			}
		}

		// geometry array (Overpass "out geom" includes inline lat/lon per node)
		const TArray<TSharedPtr<FJsonValue>>* GeomArr = nullptr;
		if (!Obj->TryGetArrayField(TEXT("geometry"), GeomArr) || !GeomArr) continue;

		for (const TSharedPtr<FJsonValue>& GV : *GeomArr)
		{
			const TSharedPtr<FJsonObject>* GPtrPtr = nullptr;
			if (!GV->TryGetObject(GPtrPtr) || !GPtrPtr) continue;
			double Lat = 0.0, Lon = 0.0;
			(*GPtrPtr)->TryGetNumberField(TEXT("lat"), Lat);
			(*GPtrPtr)->TryGetNumberField(TEXT("lon"), Lon);
			Way.PointsCm.Add(FVector(Lon, Lat, 0.0)); // (lon, lat) — projected below
		}

		// OSM node ids (parallel to geometry in "out geom"). Used for shared
		// landscape-spline control points (junctions). Keep aligned 1:1 with
		// PointsCm; if counts disagree, drop them (sharing falls back to -1).
		const TArray<TSharedPtr<FJsonValue>>* NodesArr = nullptr;
		if (Obj->TryGetArrayField(TEXT("nodes"), NodesArr) && NodesArr)
		{
			Way.NodeIds.Reserve(NodesArr->Num());
			for (const TSharedPtr<FJsonValue>& NV : *NodesArr)
				Way.NodeIds.Add(static_cast<int64>(NV->AsNumber()));
		}
		if (Way.NodeIds.Num() != Way.PointsCm.Num())
			Way.NodeIds.Reset();

		if (Way.PointsCm.Num() >= 2)
			OutWays.Add(MoveTemp(Way));
	}

	return true;
}

// ---------------------------------------------------------------------------
// Landscape-spline road backend (replaces RoadBuilder meshing)
// ---------------------------------------------------------------------------

// Half-width (cm) of a road's drivable surface from OSM class/lanes/width.
float OSMRoadGeom::RoadHalfWidthCm(const FOSMRoadWay& Way)
{
	constexpr float LaneWidthCm = 350.f; // ~3.5 m per lane

	switch (Way.Class)
	{
	case EOSMHighwayClass::Footway:
	case EOSMHighwayClass::Path:
	case EOSMHighwayClass::Steps:
	case EOSMHighwayClass::Cycleway:
		return 90.f; // ~1.8 m total
	default:
		break;
	}

	if (Way.WidthM > 0.f)
		return FMath::Max(150.f, Way.WidthM * 100.f * 0.5f);

	int32 Lanes = Way.Lanes;
	if (Lanes <= 0)
		Lanes = Way.LanesForward + Way.LanesBackward; // directional tags only
	if (Lanes <= 0)
	{
		switch (Way.Class)
		{
		case EOSMHighwayClass::Motorway:
		case EOSMHighwayClass::Trunk:     Lanes = 4; break;
		case EOSMHighwayClass::Primary:   Lanes = 3; break;
		default:                          Lanes = 2; break;
		}
	}
	return Lanes * LaneWidthCm * 0.5f;
}

// Highways/expressways don't get curbs+sidewalks by default; urban streets do.
bool OSMRoadGeom::HasSidewalks(EOSMHighwayClass Class)
{
	switch (Class)
	{
	case EOSMHighwayClass::Motorway:
	case EOSMHighwayClass::Trunk:
	case EOSMHighwayClass::Service:        // alleys / driveways: usually none
		return false;
	default:
		return true;
	}
}

// Resolve sidewalk sides: explicit OSM sidewalk= tag wins, else class default.
void OSMRoadGeom::SidewalkSides(const FOSMRoadWay& Way, bool& bOutLeft, bool& bOutRight)
{
	if (Way.bSidewalkTagged)
	{
		bOutLeft  = Way.bSidewalkLeft;
		bOutRight = Way.bSidewalkRight;
		return;
	}
	const bool bDefault = HasSidewalks(Way.Class);
	bOutLeft = bOutRight = bDefault;
}

// Sidewalk depth (cm): wider walks along bigger arterials.
float OSMRoadGeom::SidewalkWidthCm(const FOSMRoadWay& Way)
{
	switch (Way.Class)
	{
	case EOSMHighwayClass::Primary:    return 300.f; // 3.0 m
	case EOSMHighwayClass::Secondary:  return 250.f; // 2.5 m
	case EOSMHighwayClass::Tertiary:   return 220.f; // 2.2 m
	default:                           return 200.f; // 2.0 m
	}
}

// Sculpts every drivable road corridor straight into the landscape heightmap
// via FHeightmapAccessor — no persistent spline graph is created (that graph
// was the EXCEPTION_STACK_OVERFLOW source). Each road flattens/grades the
// terrain to its (already drape+grade-smoothed) bed height, with a smooth
// shoulder falloff blending back to existing terrain. Bridges / overpasses
// (layer>0) / tunnels are skipped so grade-separated ways don't disturb the
// ground. Overlapping corridors resolve by max-weight (the surface a cell is
// closest to wins), which keeps junctions clean.
static bool SculptRoadCorridors(
	UWorld*                    World,
	ALandscape*                Landscape,
	const TArray<FOSMRoadWay>& Ways,
	int32&                     OutModifiedWays,
	int64&                     OutModifiedCells,
	FString&                   OutError)
{
	OutModifiedWays  = 0;
	OutModifiedCells = 0;

	if (!Landscape)
	{
		OutError = TEXT("SculptRoadCorridors: no target Landscape.");
		return false;
	}
	ULandscapeInfo* Info = Landscape->GetLandscapeInfo();
	if (!Info)
	{
		OutError = TEXT("SculptRoadCorridors: landscape has no LandscapeInfo.");
		return false;
	}

	const FTransform LtoW = Landscape->LandscapeActorToWorld();
	const FTransform WtoL = LtoW.Inverse();
	const double QuadSizeCm = FMath::Max(1.0, LtoW.GetScale3D().X); // cm per landscape quad

	int32 LMinX, LMinY, LMaxX, LMaxY;
	if (!Info->GetLandscapeExtent(LMinX, LMinY, LMaxX, LMaxY))
	{
		OutError = TEXT("SculptRoadCorridors: failed to query landscape extent.");
		return false;
	}

	auto WorldToQuad = [&](double WX, double WY) -> FVector2D
	{
		const FVector L = WtoL.TransformPosition(FVector(WX, WY, 0.0));
		return FVector2D(L.X, L.Y);
	};
	auto QuadToWorldXY = [&](int32 QX, int32 QY) -> FVector2D
	{
		const FVector Wv = LtoW.TransformPosition(FVector((double)QX, (double)QY, 0.0));
		return FVector2D(Wv.X, Wv.Y);
	};
	auto WorldZToHeight = [&](double WX, double WY, double WZ) -> double
	{
		const FVector L = WtoL.TransformPosition(FVector(WX, WY, WZ));
		return L.Z / LANDSCAPE_ZSCALE + 32768.0; // LANDSCAPE_ZSCALE = 1/128
	};

	// Per-way corridor params. Skip grade-separated ways (no ground deform).
	auto IsDeformable = [](const FOSMRoadWay& W)
	{
		return !(W.bBridge || W.bTunnel || W.Layer != 0);
	};
	// Native UE landscape-spline deform model (Yazan's approach), reproduced in
	// the heightmap so nothing persistent is kept (→ no stack overflow):
	//   * FLAT CORE (UE control-point "Width"): terrain pinned EXACTLY to the road
	//     bed. = carriageway + sidewalk + flat buffer (osm.RoadDeformBufferM,
	//     default 2 m per side) + 1 quad. The buffer guarantees a dead-flat apron
	//     beyond the road edge so terrain can never lap over the ribbon; the +1
	//     quad additionally pins the cells whose triangles touch the buffer edge.
	//   * COSINE GRADE (UE "Side Falloff"): short cosine blend back to existing
	//     terrain — road-like, NOT a wide shoulder (wide shoulders terraced slopes).
	const double BufferCm = FMath::Max(0.0, (double)CVarRoadDeformBufferM.GetValueOnGameThread() * 100.0);
	auto FlatHalfCm = [&](const FOSMRoadWay& W, double HalfWidth) -> double
	{
		return HalfWidth + (double)OSMRoadGeom::SidewalkWidthCm(W) + BufferCm + QuadSizeCm;
	};
	auto FalloffCm = [QuadSizeCm](double HalfWidth) -> double
	{
		// The cosine grade must span SEVERAL landscape quads or it collapses to a
		// single-vertex step on coarse terrain (600 cm/quad here) → the jagged
		// edges. Floor the grade at ~2.5 quads so it always has vertices to
		// interpolate across; widen with the road for big arterials.
		return FMath::Max(FMath::Clamp(HalfWidth, 150.0, 600.0), 2.5 * QuadSizeCm);
	};

	// 1. Union bbox (in landscape quads) of all deformable corridors.
	double UMinX = DBL_MAX, UMinY = DBL_MAX, UMaxX = -DBL_MAX, UMaxY = -DBL_MAX;
	for (const FOSMRoadWay& Way : Ways)
	{
		if (Way.PointsCm.Num() < 2 || !IsDeformable(Way)) continue;
		const double HalfWidth = OSMRoadGeom::RoadHalfWidthCm(Way);
		const double ReachQuads = (FlatHalfCm(Way, HalfWidth) + FalloffCm(HalfWidth)) / QuadSizeCm;
		for (const FVector& P : Way.PointsCm)
		{
			const FVector2D Q = WorldToQuad(P.X, P.Y);
			UMinX = FMath::Min(UMinX, Q.X - ReachQuads);
			UMinY = FMath::Min(UMinY, Q.Y - ReachQuads);
			UMaxX = FMath::Max(UMaxX, Q.X + ReachQuads);
			UMaxY = FMath::Max(UMaxY, Q.Y + ReachQuads);
		}
	}

	if (UMinX > UMaxX || UMinY > UMaxY)
	{
		UE_LOG(LogOSMRoad, Log, TEXT("SculptRoadCorridors: no deformable road corridors."));
		return true; // nothing to do (e.g. all bridges) — not an error
	}

	const int32 X1 = FMath::Clamp(FMath::FloorToInt(UMinX), LMinX, LMaxX);
	const int32 Y1 = FMath::Clamp(FMath::FloorToInt(UMinY), LMinY, LMaxY);
	const int32 X2 = FMath::Clamp(FMath::CeilToInt (UMaxX), LMinX, LMaxX);
	const int32 Y2 = FMath::Clamp(FMath::CeilToInt (UMaxY), LMinY, LMaxY);
	if (X2 < X1 || Y2 < Y1)
	{
		UE_LOG(LogOSMRoad, Log, TEXT("SculptRoadCorridors: corridor bounds outside landscape."));
		return true;
	}

	const int32 W = X2 - X1 + 1;
	const int32 H = Y2 - Y1 + 1;
	const int64 Count = (int64)W * (int64)H;
	// Safety clamp — sculpt holds a handful of float buffers/cell (carve + blur);
	// ~20 M cells keeps the transient working set a few hundred MB before refuse.
	if (Count > 20'000'000)
	{
		OutError = FString::Printf(
			TEXT("SculptRoadCorridors: corridor region too large (%lld cells). Use a smaller extent."), Count);
		return false;
	}

	// 2. Rasterize corridors: per cell store the bed Z and a blend weight (1 in
	//    the flat core, cosine in the grade). Max-weight wins so the surface a
	//    cell is closest to defines its bed (clean junctions, no self-fighting).
	TArray<float> BedZ;   BedZ.Init(0.f, (int32)Count);
	TArray<float> Weight; Weight.Init(0.f, (int32)Count);

	for (const FOSMRoadWay& Way : Ways)
	{
		const int32 N = Way.PointsCm.Num();
		if (N < 2 || !IsDeformable(Way)) continue;

		const double HalfWidth = OSMRoadGeom::RoadHalfWidthCm(Way);
		const double FlatHalf  = FlatHalfCm(Way, HalfWidth);
		const double Falloff   = FMath::Max(1.0, FalloffCm(HalfWidth));
		const double Reach     = FlatHalf + Falloff;
		const double ReachQuads = Reach / QuadSizeCm;
		bool bTouched = false;

		for (int32 s = 0; s + 1 < N; ++s)
		{
			const FVector& P0 = Way.PointsCm[s];
			const FVector& P1 = Way.PointsCm[s + 1];
			const FVector2D A(P0.X, P0.Y);
			const FVector2D B(P1.X, P1.Y);
			const FVector2D AB = B - A;
			const double    AB2 = FVector2D::DotProduct(AB, AB);

			const FVector2D qA = WorldToQuad(P0.X, P0.Y);
			const FVector2D qB = WorldToQuad(P1.X, P1.Y);
			const int32 sx1 = FMath::Clamp(FMath::FloorToInt(FMath::Min(qA.X, qB.X) - ReachQuads), X1, X2);
			const int32 sy1 = FMath::Clamp(FMath::FloorToInt(FMath::Min(qA.Y, qB.Y) - ReachQuads), Y1, Y2);
			const int32 sx2 = FMath::Clamp(FMath::CeilToInt (FMath::Max(qA.X, qB.X) + ReachQuads), X1, X2);
			const int32 sy2 = FMath::Clamp(FMath::CeilToInt (FMath::Max(qA.Y, qB.Y) + ReachQuads), Y1, Y2);

			for (int32 qy = sy1; qy <= sy2; ++qy)
			for (int32 qx = sx1; qx <= sx2; ++qx)
			{
				const FVector2D Pt = QuadToWorldXY(qx, qy);
				double t = 0.0;
				if (AB2 > 0.0)
					t = FMath::Clamp(FVector2D::DotProduct(Pt - A, AB) / AB2, 0.0, 1.0);
				const FVector2D Closest = A + AB * t;
				const double Dist = FVector2D::Distance(Pt, Closest);
				if (Dist > Reach) continue;

				double w;
				if (Dist <= FlatHalf)
				{
					w = 1.0; // flat core: terrain pinned exactly to the road bed
				}
				else
				{
					// Cosine grade back to existing terrain (UE "Side Falloff").
					const double p = (Reach - Dist) / Falloff;
					w = 0.5 * (1.0 - FMath::Cos(p * PI));
				}
				if (w <= 0.0) continue;

				const int64 Idx = (int64)(qy - Y1) * W + (qx - X1);
				if ((float)w > Weight[Idx])
				{
					Weight[Idx] = (float)w;
					BedZ[Idx]   = (float)FMath::Lerp((double)P0.Z, (double)P1.Z, t);
					bTouched = true;
				}
			}
		}
		if (bTouched) ++OutModifiedWays;
	}

	// 2b. Junction discs (RoadBLD bEnableIntersectionLandscapePatch parity).
	//     The per-way pass above only rasterizes the road RECTANGLES; the corner
	//     fillet wedges of a junction polygon lie outside every arm's corridor,
	//     so on slopes the terrain pokes through the junction mesh corners.
	//     Stamp a flat disc at every junction node (incidence >= 3): radius =
	//     widest arm's flat half-width + corner-fillet allowance, bed Z = the
	//     node's (already snapped/king) height. Max-weight-wins keeps this
	//     consistent with the arm corridors it overlaps.
	{
		// Fillet allowance past the widest arm edge. RoadBuilder pins unsolved
		// corners ~DefaultJunctionExtent (300 cm) past the crossing; solved
		// fillets curve within roughly the same reach on urban roads.
		constexpr double kFilletAllowanceCm = 300.0;

		TMap<int64, int32>  JIncidence;
		TMap<int64, double> JFlatR;   // per-node disc flat radius
		TMap<int64, double> JZ;       // per-node bed Z
		TMap<int64, FVector2D> JPos;
		for (const FOSMRoadWay& Way : Ways)
		{
			if (!IsDeformable(Way) || Way.NodeIds.Num() != Way.PointsCm.Num()) continue;
			const int32 N = Way.PointsCm.Num();
			const double HalfWidth = OSMRoadGeom::RoadHalfWidthCm(Way);
			const double ArmFlat   = FlatHalfCm(Way, HalfWidth);
			for (int32 i = 0; i < N; ++i)
			{
				const int64 Id = Way.NodeIds[i];
				if (Id < 0) continue;
				JIncidence.FindOrAdd(Id) += (i == 0 || i == N - 1) ? 1 : 2;
				double& R = JFlatR.FindOrAdd(Id);
				R = FMath::Max(R, ArmFlat);
				JZ.Add(Id, Way.PointsCm[i].Z); // copies agree post junction-snap
				JPos.Add(Id, FVector2D(Way.PointsCm[i]));
			}
		}

		int32 StampedDiscs = 0;
		for (const TPair<int64, int32>& KV : JIncidence)
		{
			if (KV.Value < 3) continue; // not a junction
			const double FlatR   = JFlatR[KV.Key] + kFilletAllowanceCm;
			const double Falloff = FMath::Max(1.0, FalloffCm(FlatR));
			const double Reach   = FlatR + Falloff;
			const FVector2D C    = JPos[KV.Key];
			const double BedWorldZ = JZ[KV.Key];

			const FVector2D qC = WorldToQuad(C.X, C.Y);
			const double ReachQuads = Reach / QuadSizeCm;
			const int32 sx1 = FMath::Clamp(FMath::FloorToInt(qC.X - ReachQuads), X1, X2);
			const int32 sy1 = FMath::Clamp(FMath::FloorToInt(qC.Y - ReachQuads), Y1, Y2);
			const int32 sx2 = FMath::Clamp(FMath::CeilToInt (qC.X + ReachQuads), X1, X2);
			const int32 sy2 = FMath::Clamp(FMath::CeilToInt (qC.Y + ReachQuads), Y1, Y2);
			bool bStamped = false;

			for (int32 qy = sy1; qy <= sy2; ++qy)
			for (int32 qx = sx1; qx <= sx2; ++qx)
			{
				const double Dist = FVector2D::Distance(QuadToWorldXY(qx, qy), C);
				if (Dist > Reach) continue;

				double w;
				if (Dist <= FlatR)
					w = 1.0;
				else
				{
					const double p = (Reach - Dist) / Falloff;
					w = 0.5 * (1.0 - FMath::Cos(p * PI));
				}
				if (w <= 0.0) continue;

				const int64 Idx = (int64)(qy - Y1) * W + (qx - X1);
				if ((float)w > Weight[Idx])
				{
					Weight[Idx] = (float)w;
					BedZ[Idx]   = (float)BedWorldZ;
					bStamped = true;
				}
			}
			if (bStamped) ++StampedDiscs;
		}
		if (StampedDiscs > 0)
			UE_LOG(LogOSMRoad, Log,
				TEXT("SculptRoadCorridors: stamped %d junction discs (corner-fillet terrain patches)."),
				StampedDiscs);
	}

	// 3. Read the heightmap region once, then for every footprint cell HARD-SET
	//    terrain to the road bed — this both CUTS terrain that was above the road
	//    ("nothing above the road") and FILLS holes that were below it ("no gaps
	//    under the road"), the up/down validation in one snap. Terrain lands at
	//    bedZ; the road surface mesh sits RoadZEpsilon above it (no Z-fight).
	//    Write back once. Honour landscape edit layers (write to layer 0 when
	//    present; otherwise the final heightmap).
	Landscape->Modify();
	int64 CutCells = 0, FillCells = 0;

	auto SculptHeightmap = [&]()
	{
		FHeightmapAccessor<false> HeightmapAccessor(Info);
		TArray<uint16> Heights; Heights.SetNumUninitialized((int32)Count);
		HeightmapAccessor.GetDataFast(X1, Y1, X2, Y2, Heights.GetData());

		// (a) Carved field: road cells pulled toward their bed by weight, every
		//     other cell = original terrain. Precompute each cell's bed target so
		//     the blur and the final core re-pin can both use it.
		TArray<float> Work;   Work.SetNumUninitialized((int32)Count);
		TArray<float> Target; Target.SetNumUninitialized((int32)Count);
		for (int32 qy = Y1; qy <= Y2; ++qy)
		for (int32 qx = X1; qx <= X2; ++qx)
		{
			const int64 Idx  = (int64)(qy - Y1) * W + (qx - X1);
			const double OldH = (double)Heights[Idx];
			const float  w    = Weight[Idx];
			if (w <= 0.f) { Work[Idx] = (float)OldH; Target[Idx] = (float)OldH; continue; }
			const FVector2D Pt = QuadToWorldXY(qx, qy);
			const double TgtH  = WorldZToHeight(Pt.X, Pt.Y, (double)BedZ[Idx]);
			Target[Idx] = (float)TgtH;
			Work[Idx]   = (float)FMath::Lerp(OldH, TgtH, (double)w);
		}

		// (b) Smooth the carved field (separable [1 2 1], a few passes) so the
		//     cut/fill embankment blends into the terrain instead of faceting.
		//     Reads neighbouring untouched terrain too, so corridor edges feather.
		constexpr int BlurPasses = 2;
		TArray<float> Tmp; Tmp.SetNumUninitialized((int32)Count);
		for (int32 Pass = 0; Pass < BlurPasses; ++Pass)
		{
			for (int32 y = 0; y < H; ++y)
			for (int32 x = 0; x < W; ++x)
			{
				const int64 i = (int64)y * W + x;
				const float c = Work[i];
				const float l = (x > 0)     ? Work[i - 1] : c;
				const float r = (x < W - 1) ? Work[i + 1] : c;
				Tmp[i] = 0.25f * l + 0.5f * c + 0.25f * r;
			}
			for (int32 y = 0; y < H; ++y)
			for (int32 x = 0; x < W; ++x)
			{
				const int64 i = (int64)y * W + x;
				const float c = Tmp[i];
				const float u = (y > 0)     ? Tmp[i - W] : c;
				const float d = (y < H - 1) ? Tmp[i + W] : c;
				Work[i] = 0.25f * u + 0.5f * c + 0.25f * d;
			}
		}

		// (c) Write corridor cells only: re-pin the flat core to the exact bed
		//     (w→1) while the smoothed embankment shows through (w→0). Untouched
		//     terrain (w=0) is left exactly as imported.
		for (int32 qy = Y1; qy <= Y2; ++qy)
		for (int32 qx = X1; qx <= X2; ++qx)
		{
			const int64 Idx = (int64)(qy - Y1) * W + (qx - X1);
			const float w = Weight[Idx];
			if (w <= 0.f) continue;

			const int32  OldH  = (int32)Heights[Idx];
			const double Final = FMath::Lerp((double)Work[Idx], (double)Target[Idx], (double)w);
			const int32  NewH  = FMath::Clamp(FMath::RoundToInt(Final), 0, 65535);
			if (NewH < OldH)      ++CutCells;
			else if (NewH > OldH) ++FillCells;
			Heights[Idx] = (uint16)NewH;
			++OutModifiedCells;
		}

		HeightmapAccessor.SetData(X1, Y1, X2, Y2, Heights.GetData());
		HeightmapAccessor.Flush();
	};

#if WITH_EDITOR
	// Force the landscape onto the edit-layer system (UE 5.8 deprecates non-edit-
	// layer landscapes; the SRTM importer makes a legacy one). Converting gives a
	// base edit layer we sculpt INTO, so the deform composites/recomputes normals
	// cleanly and the edit stays re-runnable instead of stamping raw heightmap.
	if (Landscape->GetEditLayersConst().Num() == 0)
	{
		UE_LOG(LogOSMRoad, Log, TEXT("SculptRoadCorridors: converting landscape to edit-layer system."));
		Landscape->ConvertNonEditLayerLandscape();
	}
	const ULandscapeEditLayerBase* BaseEditLayer =
		(Landscape->GetEditLayersConst().Num() > 0) ? Landscape->GetEditLayerConst(0) : nullptr;
	if (BaseEditLayer)
	{
		// Write into the base edit layer, then force a layer recompute so the
		// change shows up in the composited heightmap.
		FScopedSetLandscapeEditingLayer EditLayerScope(
			Landscape, BaseEditLayer->GetGuid(),
			[Landscape] { Landscape->RequestLayersContentUpdateForceAll(); });
		SculptHeightmap();
	}
	else
#endif
	{
		SculptHeightmap();
	}

	UE_LOG(LogOSMRoad, Log,
		TEXT("SculptRoadCorridors: %d corridors, %lld cells flattened (%lld cut down, %lld filled up)."),
		OutModifiedWays, OutModifiedCells, CutCells, FillCells);

	return true;
}

// ---------------------------------------------------------------------------
// PASS 2 — conform the landscape to the road network AS ONE MESH.
//
// Pass 1 (SculptRoadCorridors) brings the terrain down near each spline bed so
// the draped road mesh sits roughly right. This second pass takes the
// already-built road SURFACE (carriageway + sidewalk tops of ground-level ways,
// supplied as ONE world-space triangle soup) and rasterizes it into the
// heightmap: each landscape cell takes the **topmost** covering triangle's world
// Z — the single visible surface — so overlapping ways / junctions resolve to
// ONE height instead of competing per-spline flattens (those competing flattens
// were the bumps). Covered cells snap to that surface minus a small epsilon (the
// mesh stays a hair proud → "nothing covers the road"); a resolution-aware
// cosine grade in a margin ring + a short [1 2 1] blur feather the embankment
// into terrain; the flat core is re-pinned. One deform authority → clean roads.
// ---------------------------------------------------------------------------
static bool DeformLandscapeToRoadMesh(
	UWorld*                World,
	ALandscape*            Landscape,
	const TArray<FVector>& MeshVerts,
	const TArray<int32>&   MeshTris,
	int64&                 OutModifiedCells,
	FString&               OutError,
	int32                  InCoreDilateQuads = 2,
	int32                  InFalloffQuads    = 3,
	const TArray<FOSMRoadWay>* ProtectWays = nullptr)
{
	OutModifiedCells = 0;
	if (!Landscape) { OutError = TEXT("DeformLandscapeToRoadMesh: no target Landscape."); return false; }
	ULandscapeInfo* Info = Landscape->GetLandscapeInfo();
	if (!Info) { OutError = TEXT("DeformLandscapeToRoadMesh: landscape has no LandscapeInfo."); return false; }

	const int32 NumTris = MeshTris.Num() / 3;
	if (NumTris < 1 || MeshVerts.Num() < 3)
	{
		UE_LOG(LogOSMRoad, Log, TEXT("DeformLandscapeToRoadMesh: empty road surface — nothing to conform."));
		return true;
	}

	const FTransform LtoW = Landscape->LandscapeActorToWorld();
	const FTransform WtoL = LtoW.Inverse();
	const double QuadSizeCm = FMath::Max(1.0, LtoW.GetScale3D().X); // cm per landscape quad

	int32 LMinX, LMinY, LMaxX, LMaxY;
	if (!Info->GetLandscapeExtent(LMinX, LMinY, LMaxX, LMaxY))
	{
		OutError = TEXT("DeformLandscapeToRoadMesh: failed to query landscape extent.");
		return false;
	}

	auto WorldToQuad = [&](double WX, double WY) -> FVector2D
	{
		const FVector L = WtoL.TransformPosition(FVector(WX, WY, 0.0));
		return FVector2D(L.X, L.Y);
	};
	auto QuadToWorldXY = [&](int32 QX, int32 QY) -> FVector2D
	{
		const FVector Wv = LtoW.TransformPosition(FVector((double)QX, (double)QY, 0.0));
		return FVector2D(Wv.X, Wv.Y);
	};
	auto WorldZToHeight = [&](double WX, double WY, double WZ) -> double
	{
		const FVector L = WtoL.TransformPosition(FVector(WX, WY, WZ));
		return L.Z / LANDSCAPE_ZSCALE + 32768.0; // LANDSCAPE_ZSCALE = 1/128
	};

	// Road surface stays this many cm proud of the conformed ground (clears
	// heightmap rounding; matches the mesh's own RoadZEpsilon lift).
	constexpr double EpsilonCm = 10.0;
	// AGGRESSION knobs for the conform:
	//   CoreDilateQuads — quads BEYOND the rasterized road footprint that are
	//     still pinned DEAD-FLAT to the nearest road surface. This is the main
	//     "be more aggressive" lever: it cuts the terrain right at the road edge
	//     down to road level so nothing laps over the ribbon (and clears coarse-
	//     terrain triangles whose far corner would otherwise bulge over the edge).
	//     Raise for a wider, harder clear zone; lower for a tighter cut.
	//   FalloffQuads — cosine embankment width beyond the dilated core (~2.5 quads
	//     so the grade spans several vertices on coarse terrain, no single-step facet).
	const int32 CoreDilateQuads = InCoreDilateQuads;
	// Embankment grade width. NOTE: keep this SMALL. In a dense road grid the
	// reach (CoreDilate + Falloff) extends off BOTH sides of every road; if two
	// parallel roads are closer than ~2*Reach quads the ramps collide in the
	// block interior and tent/pit it (regular grid → grid of pyramids). The
	// big cliffs were the 20 m lift's fault (now capped at 4 m), NOT a narrow
	// falloff — so this stays at 3.
	const int32 FalloffQuads    = InFalloffQuads;
	const int32 ReachQuads      = CoreDilateQuads + FalloffQuads;

	// 1) Mesh XY bbox (quad space) + falloff margin, clamped to the landscape.
	double UMinX = DBL_MAX, UMinY = DBL_MAX, UMaxX = -DBL_MAX, UMaxY = -DBL_MAX;
	for (const FVector& V : MeshVerts)
	{
		const FVector2D Q = WorldToQuad(V.X, V.Y);
		UMinX = FMath::Min(UMinX, Q.X); UMinY = FMath::Min(UMinY, Q.Y);
		UMaxX = FMath::Max(UMaxX, Q.X); UMaxY = FMath::Max(UMaxY, Q.Y);
	}
	if (UMinX > UMaxX || UMinY > UMaxY)
	{
		UE_LOG(LogOSMRoad, Log, TEXT("DeformLandscapeToRoadMesh: degenerate mesh bounds."));
		return true;
	}
	const int32 X1 = FMath::Clamp(FMath::FloorToInt(UMinX) - ReachQuads, LMinX, LMaxX);
	const int32 Y1 = FMath::Clamp(FMath::FloorToInt(UMinY) - ReachQuads, LMinY, LMaxY);
	const int32 X2 = FMath::Clamp(FMath::CeilToInt (UMaxX) + ReachQuads, LMinX, LMaxX);
	const int32 Y2 = FMath::Clamp(FMath::CeilToInt (UMaxY) + ReachQuads, LMinY, LMaxY);
	if (X2 < X1 || Y2 < Y1)
	{
		UE_LOG(LogOSMRoad, Log, TEXT("DeformLandscapeToRoadMesh: mesh bounds outside landscape."));
		return true;
	}
	const int32 W = X2 - X1 + 1;
	const int32 H = Y2 - Y1 + 1;
	const int64 Count = (int64)W * (int64)H;
	if (Count <= 0 || Count > 20'000'000)
	{
		OutError = FString::Printf(TEXT("DeformLandscapeToRoadMesh: region too large (%lld cells)."), Count);
		return false;
	}

	auto CellIdx = [&](int32 qx, int32 qy) -> int64 { return (int64)(qy - Y1) * W + (qx - X1); };

	// 2) Rasterize every surface triangle → per-cell TOPMOST world Z (covered mask).
	//    Vertical curb faces project to zero XY area (degenerate denom) and are
	//    skipped, so only the horizontal road/sidewalk tops define the ground.
	const float UNSET = -FLT_MAX;
	TArray<float> SurfZ; SurfZ.Init(UNSET, (int32)Count);
	for (int32 t = 0; t < NumTris; ++t)
	{
		const FVector& A = MeshVerts[MeshTris[3 * t + 0]];
		const FVector& B = MeshVerts[MeshTris[3 * t + 1]];
		const FVector& C = MeshVerts[MeshTris[3 * t + 2]];
		const FVector2D qa = WorldToQuad(A.X, A.Y);
		const FVector2D qb = WorldToQuad(B.X, B.Y);
		const FVector2D qc = WorldToQuad(C.X, C.Y);

		const double Den = (qb.Y - qc.Y) * (qa.X - qc.X) + (qc.X - qb.X) * (qa.Y - qc.Y);
		if (FMath::IsNearlyZero(Den)) continue; // vertical/degenerate face → no XY footprint
		const double InvDen = 1.0 / Den;

		const int32 tx1 = FMath::Clamp(FMath::FloorToInt(FMath::Min3(qa.X, qb.X, qc.X)), X1, X2);
		const int32 ty1 = FMath::Clamp(FMath::FloorToInt(FMath::Min3(qa.Y, qb.Y, qc.Y)), Y1, Y2);
		const int32 tx2 = FMath::Clamp(FMath::CeilToInt (FMath::Max3(qa.X, qb.X, qc.X)), X1, X2);
		const int32 ty2 = FMath::Clamp(FMath::CeilToInt (FMath::Max3(qa.Y, qb.Y, qc.Y)), Y1, Y2);

		for (int32 qy = ty1; qy <= ty2; ++qy)
		for (int32 qx = tx1; qx <= tx2; ++qx)
		{
			const double Px = (double)qx; // landscape cells live at integer quad coords
			const double Py = (double)qy;
			const double L1 = ((qb.Y - qc.Y) * (Px - qc.X) + (qc.X - qb.X) * (Py - qc.Y)) * InvDen;
			const double L2 = ((qc.Y - qa.Y) * (Px - qc.X) + (qa.X - qc.X) * (Py - qc.Y)) * InvDen;
			const double L3 = 1.0 - L1 - L2;
			constexpr double Tol = -1.0e-4; // include shared edges so adjacent tris don't crack
			if (L1 < Tol || L2 < Tol || L3 < Tol) continue;
			const double Z = L1 * A.Z + L2 * B.Z + L3 * C.Z;
			const int64 Idx = CellIdx(qx, qy);
			if ((float)Z > SurfZ[Idx]) SurfZ[Idx] = (float)Z; // topmost surface wins
		}
	}

	// 3) Distance transform from covered cells (2-pass chamfer), propagating the
	//    nearest covered surface Z so the embankment grades from the real road edge.
	const float BIG = 1.0e9f;
	TArray<float> Dist;  Dist.Init(BIG, (int32)Count);
	TArray<float> NearZ; NearZ.Init(0.f, (int32)Count);
	for (int64 i = 0; i < Count; ++i)
	{
		if (SurfZ[i] > UNSET) { Dist[i] = 0.f; NearZ[i] = SurfZ[i]; }
	}
	auto Relax = [&](int64 i, int64 j, float cost)
	{
		if (Dist[j] + cost < Dist[i]) { Dist[i] = Dist[j] + cost; NearZ[i] = NearZ[j]; }
	};
	const float D1 = 1.0f, D2 = 1.41421356f;
	for (int32 y = 0; y < H; ++y)
	for (int32 x = 0; x < W; ++x)
	{
		const int64 i = (int64)y * W + x;
		if (x > 0)              Relax(i, i - 1,     D1);
		if (y > 0)              Relax(i, i - W,     D1);
		if (x > 0 && y > 0)     Relax(i, i - W - 1, D2);
		if (x < W - 1 && y > 0) Relax(i, i - W + 1, D2);
	}
	for (int32 y = H - 1; y >= 0; --y)
	for (int32 x = W - 1; x >= 0; --x)
	{
		const int64 i = (int64)y * W + x;
		if (x < W - 1)              Relax(i, i + 1,     D1);
		if (y < H - 1)              Relax(i, i + W,     D1);
		if (x < W - 1 && y < H - 1) Relax(i, i + W + 1, D2);
		if (x > 0 && y < H - 1)     Relax(i, i + W - 1, D2);
	}

	// 3b) Protected road corridors: cells the ROAD deform pass already owns
	//     (carriageway + sidewalk + 1 quad, same footprint SculptRoadCorridors
	//     pins) are excluded from this conform entirely — the block-plate
	//     terrace must not re-tilt roads that were already graded. Works for
	//     spline/decal road pipelines too, where no separate road mesh exists
	//     and the road surface IS the landscape.
	TBitArray<> ProtectedCells(false, (int32)Count);
	int64 ProtectedCount = 0;
	if (ProtectWays)
	{
		for (const FOSMRoadWay& Way : *ProtectWays)
		{
			const int32 N = Way.PointsCm.Num();
			if (N < 2 || Way.bBridge || Way.bTunnel || Way.Layer != 0) continue;

			const double HalfW = (double)OSMRoadGeom::RoadHalfWidthCm(Way)
				+ (double)OSMRoadGeom::SidewalkWidthCm(Way) + QuadSizeCm;
			const double ReachQ = HalfW / QuadSizeCm;

			for (int32 s = 0; s + 1 < N; ++s)
			{
				const FVector2D A(Way.PointsCm[s].X,     Way.PointsCm[s].Y);
				const FVector2D B(Way.PointsCm[s + 1].X, Way.PointsCm[s + 1].Y);
				const FVector2D AB = B - A;
				const double AB2 = FVector2D::DotProduct(AB, AB);

				const FVector2D qA = WorldToQuad(A.X, A.Y);
				const FVector2D qB = WorldToQuad(B.X, B.Y);
				const int32 sx1 = FMath::Clamp(FMath::FloorToInt(FMath::Min(qA.X, qB.X) - ReachQ), X1, X2);
				const int32 sy1 = FMath::Clamp(FMath::FloorToInt(FMath::Min(qA.Y, qB.Y) - ReachQ), Y1, Y2);
				const int32 sx2 = FMath::Clamp(FMath::CeilToInt (FMath::Max(qA.X, qB.X) + ReachQ), X1, X2);
				const int32 sy2 = FMath::Clamp(FMath::CeilToInt (FMath::Max(qA.Y, qB.Y) + ReachQ), Y1, Y2);

				for (int32 qy = sy1; qy <= sy2; ++qy)
				for (int32 qx = sx1; qx <= sx2; ++qx)
				{
					const int64 Idx = CellIdx(qx, qy);
					if (ProtectedCells[(int32)Idx]) continue;
					const FVector2D Pt = QuadToWorldXY(qx, qy);
					double t = 0.0;
					if (AB2 > 0.0)
						t = FMath::Clamp(FVector2D::DotProduct(Pt - A, AB) / AB2, 0.0, 1.0);
					if (FVector2D::Distance(Pt, A + AB * t) <= HalfW)
					{
						ProtectedCells[(int32)Idx] = true;
						++ProtectedCount;
					}
				}
			}
		}
	}

	// 4) Read region once, conform, blur the embankment, re-pin the core, write once.
	Landscape->Modify();
	int64 CutCells = 0, FillCells = 0;

	auto Sculpt = [&]()
	{
		FHeightmapAccessor<false> HeightmapAccessor(Info);
		TArray<uint16> Heights; Heights.SetNumUninitialized((int32)Count);
		HeightmapAccessor.GetDataFast(X1, Y1, X2, Y2, Heights.GetData());

		TArray<float> Work;   Work.SetNumUninitialized((int32)Count);
		TArray<float> Target; Target.SetNumUninitialized((int32)Count);
		TArray<float> Wgt;    Wgt.SetNumUninitialized((int32)Count);

		for (int32 qy = Y1; qy <= Y2; ++qy)
		for (int32 qx = X1; qx <= X2; ++qx)
		{
			const int64  Idx  = CellIdx(qx, qy);
			const double OldH = (double)Heights[Idx];

			float  w = 0.f;
			double TgtWorldZ = 0.0;
			if (ProtectedCells[(int32)Idx])
			{
				// Road-owned cell: leave exactly as the road deform graded it.
				Wgt[Idx] = 0.f; Work[Idx] = (float)OldH; Target[Idx] = (float)OldH;
				continue;
			}
			if (SurfZ[Idx] > UNSET || Dist[Idx] <= (float)CoreDilateQuads)
			{
				// Flat core = road footprint + dilated margin: HARD-pin to the road
				// surface so the edge ring is cut down to road level (no covering).
				w = 1.f;
				const double SZ = (SurfZ[Idx] > UNSET) ? (double)SurfZ[Idx] : (double)NearZ[Idx];
				TgtWorldZ = SZ - EpsilonCm;
			}
			else if (Dist[Idx] <= (float)ReachQuads)
			{
				// Cosine embankment from the dilated core edge back to terrain.
				const double p = (double)((float)ReachQuads - Dist[Idx]) / (double)FalloffQuads; // 1 at core edge → 0 outer
				w = (float)(0.5 * (1.0 - FMath::Cos(p * PI)));
				TgtWorldZ = (double)NearZ[Idx] - EpsilonCm;
			}

			Wgt[Idx] = w;
			if (w <= 0.f) { Work[Idx] = (float)OldH; Target[Idx] = (float)OldH; continue; }

			const FVector2D Wxy  = QuadToWorldXY(qx, qy);
			const double    TgtH = WorldZToHeight(Wxy.X, Wxy.Y, TgtWorldZ);
			Target[Idx] = (float)TgtH;
			Work[Idx]   = (float)FMath::Lerp(OldH, TgtH, (double)w);
		}

		// Separable [1 2 1] blur (2 passes) so embankments feather, not facet.
		constexpr int BlurPasses = 2;
		TArray<float> Tmp; Tmp.SetNumUninitialized((int32)Count);
		for (int32 Pass = 0; Pass < BlurPasses; ++Pass)
		{
			for (int32 y = 0; y < H; ++y)
			for (int32 x = 0; x < W; ++x)
			{
				const int64 i = (int64)y * W + x;
				const float c = Work[i];
				const float l = (x > 0)     ? Work[i - 1] : c;
				const float r = (x < W - 1) ? Work[i + 1] : c;
				Tmp[i] = 0.25f * l + 0.5f * c + 0.25f * r;
			}
			for (int32 y = 0; y < H; ++y)
			for (int32 x = 0; x < W; ++x)
			{
				const int64 i = (int64)y * W + x;
				const float c = Tmp[i];
				const float u = (y > 0)     ? Tmp[i - W] : c;
				const float d = (y < H - 1) ? Tmp[i + W] : c;
				Work[i] = 0.25f * u + 0.5f * c + 0.25f * d;
			}
		}

		// Re-pin the flat core to the exact surface (w→1) while the smoothed
		// embankment shows through (w→0). Untouched terrain (w=0) is left as-is.
		for (int32 qy = Y1; qy <= Y2; ++qy)
		for (int32 qx = X1; qx <= X2; ++qx)
		{
			const int64 Idx = CellIdx(qx, qy);
			const float w = Wgt[Idx];
			if (w <= 0.f) continue;

			const int32  OldH  = (int32)Heights[Idx];
			const double Final = FMath::Lerp((double)Work[Idx], (double)Target[Idx], (double)w);
			const int32  NewH  = FMath::Clamp(FMath::RoundToInt(Final), 0, 65535);
			if (NewH < OldH)      ++CutCells;
			else if (NewH > OldH) ++FillCells;
			Heights[Idx] = (uint16)NewH;
			++OutModifiedCells;
		}

		HeightmapAccessor.SetData(X1, Y1, X2, Y2, Heights.GetData());
		HeightmapAccessor.Flush();
	};

#if WITH_EDITOR
	if (Landscape->GetEditLayersConst().Num() == 0)
	{
		Landscape->ConvertNonEditLayerLandscape();
	}
	const ULandscapeEditLayerBase* BaseEditLayer =
		(Landscape->GetEditLayersConst().Num() > 0) ? Landscape->GetEditLayerConst(0) : nullptr;
	if (BaseEditLayer)
	{
		FScopedSetLandscapeEditingLayer EditLayerScope(
			Landscape, BaseEditLayer->GetGuid(),
			[Landscape] { Landscape->RequestLayersContentUpdateForceAll(); });
		Sculpt();
	}
	else
#endif
	{
		Sculpt();
	}

	UE_LOG(LogOSMRoad, Log,
		TEXT("DeformLandscapeToRoadMesh (PASS 2): %d tris → %lld cells conformed (%lld cut, %lld fill, %lld road cells protected)."),
		NumTris, OutModifiedCells, CutCells, FillCells, ProtectedCount);
	return true;
}

// ---------------------------------------------------------------------------
// DeformLandscapeToMesh — public wrapper around the road-mesh conform so other
// builders (block plates) can terrace the terrain to their surface too.
// ---------------------------------------------------------------------------
bool OSMOverpassRoadImport::DeformLandscapeToMesh(
	UWorld*                World,
	const TArray<FVector>& MeshVerts,
	const TArray<int32>&   MeshTris,
	int64&                 OutModifiedCells,
	FString&               OutError,
	int32                  CoreDilateQuads,
	int32                  FalloffQuads,
	const TArray<FOSMRoadWay>* ProtectRoadWays)
{
	OutModifiedCells = 0;
	if (!World) { OutError = TEXT("DeformLandscapeToMesh: no world."); return false; }

	ALandscape* Landscape = nullptr;
	for (TActorIterator<ALandscape> It(World); It; ++It) { Landscape = *It; break; }
	if (!Landscape) { OutError = TEXT("DeformLandscapeToMesh: no ALandscape in level."); return false; }

	return DeformLandscapeToRoadMesh(World, Landscape, MeshVerts, MeshTris, OutModifiedCells, OutError,
		FMath::Max(0, CoreDilateQuads), FMath::Max(1, FalloffQuads), ProtectRoadWays);
}

// ---------------------------------------------------------------------------
// SmoothLandscape — global separable [1 2 1] blur of the whole landscape
// heightmap. A smoother base terrain is the single biggest lever for natural
// road deformation (the coarse SRTM/quad faceting drives the jagged cuts).
// Writes through the base edit layer so it's non-destructive + recomputable.
// ---------------------------------------------------------------------------
bool OSMOverpassRoadImport::SmoothLandscape(UWorld* World, int32 Passes, float Strength, FString& OutError)
{
#if WITH_EDITOR
	if (!World) { OutError = TEXT("SmoothLandscape: no world."); return false; }
	Passes = FMath::Clamp(Passes, 0, 64);
	Strength = FMath::Clamp(Strength, 0.f, 1.f);
	if (Passes == 0 || Strength <= 0.f)
	{
		UE_LOG(LogOSMRoad, Log, TEXT("OSM.SmoothLandscape: nothing to do (passes=%d strength=%.2f)."), Passes, Strength);
		return true;
	}

	ALandscape* Landscape = nullptr;
	for (TActorIterator<ALandscape> It(World); It; ++It) { Landscape = *It; break; }
	if (!Landscape) { OutError = TEXT("SmoothLandscape: no ALandscape in level."); return false; }

	ULandscapeInfo* Info = Landscape->GetLandscapeInfo();
	if (!Info) { OutError = TEXT("SmoothLandscape: landscape has no LandscapeInfo."); return false; }

	int32 X1, Y1, X2, Y2;
	if (!Info->GetLandscapeExtent(X1, Y1, X2, Y2))
	{
		OutError = TEXT("SmoothLandscape: failed to query landscape extent.");
		return false;
	}
	const int32 W = X2 - X1 + 1;
	const int32 H = Y2 - Y1 + 1;
	const int64 Count = (int64)W * (int64)H;
	if (Count <= 0 || Count > 80'000'000)
	{
		OutError = FString::Printf(TEXT("SmoothLandscape: landscape too large (%lld cells)."), Count);
		return false;
	}

	Landscape->Modify();

	auto Run = [&]()
	{
		FHeightmapAccessor<false> HeightmapAccessor(Info);
		TArray<uint16> Heights; Heights.SetNumUninitialized((int32)Count);
		HeightmapAccessor.GetDataFast(X1, Y1, X2, Y2, Heights.GetData());

		TArray<float> A; A.SetNumUninitialized((int32)Count);
		for (int64 i = 0; i < Count; ++i) A[i] = (float)Heights[i];
		const TArray<float> Orig = A;
		TArray<float> B; B.SetNumUninitialized((int32)Count);

		for (int32 Pass = 0; Pass < Passes; ++Pass)
		{
			for (int32 y = 0; y < H; ++y)
			for (int32 x = 0; x < W; ++x)
			{
				const int64 i = (int64)y * W + x;
				const float c = A[i];
				const float l = (x > 0)     ? A[i - 1] : c;
				const float r = (x < W - 1) ? A[i + 1] : c;
				B[i] = 0.25f * l + 0.5f * c + 0.25f * r;
			}
			for (int32 y = 0; y < H; ++y)
			for (int32 x = 0; x < W; ++x)
			{
				const int64 i = (int64)y * W + x;
				const float c = B[i];
				const float u = (y > 0)     ? B[i - W] : c;
				const float d = (y < H - 1) ? B[i + W] : c;
				A[i] = 0.25f * u + 0.5f * c + 0.25f * d;
			}
		}

		for (int64 i = 0; i < Count; ++i)
		{
			const float Blended = FMath::Lerp(Orig[i], A[i], Strength);
			Heights[i] = (uint16)FMath::Clamp(FMath::RoundToInt(Blended), 0, 65535);
		}

		HeightmapAccessor.SetData(X1, Y1, X2, Y2, Heights.GetData());
		HeightmapAccessor.Flush();
	};

	if (Landscape->GetEditLayersConst().Num() == 0)
		Landscape->ConvertNonEditLayerLandscape();
	const ULandscapeEditLayerBase* BaseEditLayer =
		(Landscape->GetEditLayersConst().Num() > 0) ? Landscape->GetEditLayerConst(0) : nullptr;
	if (BaseEditLayer)
	{
		FScopedSetLandscapeEditingLayer EditLayerScope(
			Landscape, BaseEditLayer->GetGuid(),
			[Landscape] { Landscape->RequestLayersContentUpdateForceAll(); });
		Run();
	}
	else
	{
		Run();
	}

	UE_LOG(LogOSMRoad, Log,
		TEXT("OSM.SmoothLandscape: %dx%d verts smoothed (%d passes, strength %.2f)."),
		W, H, Passes, Strength);
	return true;
#else
	OutError = TEXT("SmoothLandscape is editor-only.");
	return false;
#endif
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Node-feature props (Phase 5). Spawn instanced meshes at tagged road nodes so
// the recognised point features are visible in-level. Additive — a single
// container actor (tagged, destroyed + rebuilt on re-import), no touch to the
// road mesh or terrain. Uses engine BasicShapes so no project assets required.
// ---------------------------------------------------------------------------
static const FName RoadPropsTag(TEXT("OSMRoadProps"));

static void SpawnNodeFeatureProps(
	UWorld* World, const TArray<FOSMRoadWay>& Ways, const TMap<int64, uint32>& NodeFeatures)
{
	if (!World || NodeFeatures.Num() == 0) return;

	// Idempotent: clear any prop container from a previous import.
	for (TActorIterator<AActor> It(World); It; ++It)
		if (It->Tags.Contains(RoadPropsTag)) It->Destroy();

	// Node id → world position (first way vertex carrying that id).
	TMap<int64, FVector> Pos;
	for (const FOSMRoadWay& Way : Ways)
	{
		if (Way.NodeIds.Num() != Way.PointsCm.Num()) continue;
		for (int32 i = 0; i < Way.NodeIds.Num(); ++i)
		{
			const int64 Id = Way.NodeIds[i];
			if (Id >= 0 && NodeFeatures.Contains(Id) && !Pos.Contains(Id) && !Way.PointsCm[i].ContainsNaN())
				Pos.Add(Id, Way.PointsCm[i]);
		}
	}
	if (Pos.Num() == 0) return;

	UStaticMesh* Cylinder = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	UStaticMesh* Cube     = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (!Cylinder && !Cube) return; // engine content missing — nothing to place

	AActor* Container = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity);
	if (!Container) return;
	Container->Tags.Add(RoadPropsTag);
#if WITH_EDITOR
	Container->SetActorLabel(TEXT("OSM_RoadProps"));
#endif
	USceneComponent* Root = NewObject<USceneComponent>(Container);
	Container->SetRootComponent(Root);
	Root->RegisterComponent();

	UMaterialInterface* BaseMat = LoadObject<UMaterialInterface>(
		nullptr, TEXT("/Game/Materials/M_OSMBuilding.M_OSMBuilding"));

	// Make one HISM for a (mesh, tint) prop kind.
	auto MakeHISM = [&](UStaticMesh* Mesh, const FLinearColor& Tint) -> UHierarchicalInstancedStaticMeshComponent*
	{
		if (!Mesh) return nullptr;
		auto* H = NewObject<UHierarchicalInstancedStaticMeshComponent>(Container);
		H->SetupAttachment(Root);
		H->SetStaticMesh(Mesh);
		H->SetMobility(EComponentMobility::Static);
		H->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		H->SetCanEverAffectNavigation(false);
		H->RegisterComponent();
#if WITH_EDITOR
		Container->AddInstanceComponent(H);
#endif
		if (BaseMat)
			if (UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(BaseMat, Container))
			{
				MID->SetVectorParameterValue(TEXT("BaseColor"), Tint);
				H->SetMaterial(0, MID);
			}
		return H;
	};

	// Bollard: slim ~1 m cylinder (basic cylinder is 100 cm tall/dia, origin
	// centred). Toll booth: ~3 m box. Basic shapes are centred, so lift by half.
	UHierarchicalInstancedStaticMeshComponent* Bollards = MakeHISM(Cylinder, FLinearColor(0.9f, 0.2f, 0.05f));
	UHierarchicalInstancedStaticMeshComponent* Tolls    = MakeHISM(Cube,     FLinearColor(0.85f, 0.8f, 0.4f));

	int32 NB = 0, NT = 0;
	for (const TPair<int64, FVector>& P : Pos)
	{
		const uint32 F = NodeFeatures[P.Key];
		if ((F & NF_Bollard) && Bollards)
		{
			const FVector Loc = P.Value + FVector(0, 0, 50.f);
			Bollards->AddInstance(FTransform(FRotator::ZeroRotator, Loc, FVector(0.2f, 0.2f, 1.0f)), /*bWorldSpace*/ true);
			++NB;
		}
		if ((F & NF_Toll) && Tolls)
		{
			const FVector Loc = P.Value + FVector(0, 0, 150.f);
			Tolls->AddInstance(FTransform(FRotator::ZeroRotator, Loc, FVector(3.0f, 3.0f, 3.0f)), /*bWorldSpace*/ true);
			++NT;
		}
	}

	if (NB == 0 && NT == 0)
	{
		Container->Destroy();
		return;
	}
	UE_LOG(LogOSMRoad, Log,
		TEXT("OSM.ImportRoads: node-feature props — %d bollards, %d toll booths instanced."), NB, NT);
}

// ---------------------------------------------------------------------------
// LoadRoadWaysFromFile — parse + project only (no world mutation)
// ---------------------------------------------------------------------------
bool OSMOverpassRoadImport::LoadRoadWaysFromFile(
	UWorld*              World,
	const FString&       FilePath,
	TArray<FOSMRoadWay>& OutWays,
	FString&             OutError)
{
	FString Json;
	if (!FFileHelper::LoadFileToString(Json, *FilePath))
	{
		OutError = FString::Printf(TEXT("Could not read '%s'."), *FilePath);
		return false;
	}

	OutWays.Reset();
	TMap<int64, uint32> NodeFeatures; // unused here
	if (!ParseOverpassJson(Json, OutWays, NodeFeatures, OutError))
	{
		return false;
	}

	AGeoReferencingSystem* Geo = AGeoReferencingSystem::GetGeoReferencingSystem(World);
	if (!Geo)
	{
		OutError = TEXT("No GeoReferencingSystem in level. Run OSM.ImportTerrain or OSM.ImportBBox first.");
		return false;
	}

	for (FOSMRoadWay& Way : OutWays)
	{
		for (FVector& P : Way.PointsCm)
		{
			FVector Engine;
			Geo->GeographicToEngine(FGeographicCoordinates(P.X /*lon*/, P.Y /*lat*/, 0.0), Engine);
			P.X = Engine.X;
			P.Y = Engine.Y;
			P.Z = 0.0;
		}
	}
	UE_LOG(LogOSMRoad, Log, TEXT("LoadRoadWaysFromFile: parsed + projected %d ways from '%s'."),
		OutWays.Num(), *FilePath);
	return true;
}

// ---------------------------------------------------------------------------
// ImportRoadsFromString — shared between HTTP and file paths
// ---------------------------------------------------------------------------
bool OSMOverpassRoadImport::ImportRoadsFromString(
	UWorld*                 World,
	const FString&          OverpassJson,
	FOSMRoadImportStats&    OutStats,
	FString&                OutError,
	double MinLon, double MinLat,
	double MaxLon, double MaxLat)
{
	// 1. Parse JSON (ways + tagged nodes)
	TArray<FOSMRoadWay> Ways;
	TMap<int64, uint32>  NodeFeatures; // nodeId → ENodeFeat bitmask
	if (!ParseOverpassJson(OverpassJson, Ways, NodeFeatures, OutError))
		return false;

	OutStats.WaysTotal = Ways.Num();
	UE_LOG(LogOSMRoad, Log, TEXT("OSM.ImportRoads: parsed %d ways, %d tagged nodes from JSON"),
		Ways.Num(), NodeFeatures.Num());

	// 1b. Clip ways to the import bbox (Overpass returns full geometry of any way
	//     touching the box, so pass-through roads can stretch far outside it).
	//     Disabled when the caller passes a degenerate bbox.
	if (MaxLon > MinLon && MaxLat > MinLat)
	{
		// Expand by 2% so roads still reach the landscape edges.
		const double MLon = (MaxLon - MinLon) * 0.02;
		const double MLat = (MaxLat - MinLat) * 0.02;
		const double XMin = MinLon - MLon, XMax = MaxLon + MLon;
		const double YMin = MinLat - MLat, YMax = MaxLat + MLat;

		TArray<FOSMRoadWay> Clipped;
		Clipped.Reserve(Ways.Num());
		for (const FOSMRoadWay& Way : Ways)
		{
			TArray<TArray<FVector>> Runs;
			TArray<TArray<int64>>   NodeRuns;
			ClipWayToBBoxN(Way.PointsCm, Way.NodeIds, XMin, YMin, XMax, YMax, Runs, NodeRuns);
			for (int32 r = 0; r < Runs.Num(); ++r)
			{
				FOSMRoadWay Part = Way;          // copy tags/class/flags
				Part.PointsCm = MoveTemp(Runs[r]);
				Part.NodeIds  = MoveTemp(NodeRuns[r]);
				Clipped.Add(MoveTemp(Part));
			}
		}
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: bbox clip — %d ways → %d in-bounds runs."),
			Ways.Num(), Clipped.Num());
		Ways = MoveTemp(Clipped);
	}

	// 2. Resolve GeoReferencingSystem (shared with buildings / terrain)
	AGeoReferencingSystem* Geo = AGeoReferencingSystem::GetGeoReferencingSystem(World);
	if (!Geo)
	{
		OutError = TEXT("No GeoReferencingSystem in level. Run OSM.ImportTerrain or OSM.ImportBBox first.");
		return false;
	}

	// 3. Compute bbox centre for stats from raw (lon, lat) BEFORE projection.
	{
		double SumLat = 0, SumLon = 0;
		int32  N      = 0;
		for (const FOSMRoadWay& Way : Ways)
			for (const FVector& P : Way.PointsCm)   // P = (lon, lat, 0) here
			{
				SumLon += P.X; SumLat += P.Y; ++N;
			}
		if (N > 0)
		{
			OutStats.CenterLon = SumLon / N;
			OutStats.CenterLat = SumLat / N;
		}
	}

	// 4. Project all points from (lon, lat) → engine XY cm (Z reset to 0).
	for (FOSMRoadWay& Way : Ways)
	{
		for (FVector& P : Way.PointsCm)
		{
			FVector Engine;
			Geo->GeographicToEngine(FGeographicCoordinates(P.X /*lon*/, P.Y /*lat*/, 0.0), Engine);
			P.X = Engine.X;
			P.Y = Engine.Y;
			P.Z = 0.0;
		}
	}

	// 4b. Terrain draping (D4): conform each centerline point to the Landscape
	//     surface so roads follow the ground. Done BEFORE any road mesh is
	//     spawned, so the downward traces hit only the terrain (not other roads).
	//     Bridges / positive-layer ways keep their grade-separation offset above
	//     terrain instead of being draped. Lateral embedding (cut/fill) is the
	//     separate OSM.CarveRoads pass (P4).
	{
		// Per-OSM-layer vertical separation for grade-separated ways (tunable).
		const double LayerHeightCm     = FMath::Max(0.f, CVarRoadLayerHeightCm.GetValueOnGameThread());
		const double BridgeClearanceCm = FMath::Max(0.f, CVarRoadBridgeClearanceCm.GetValueOnGameThread());
		int32 Sampled = 0, Filled = 0;
		for (FOSMRoadWay& Way : Ways)
		{
			const int32 N = Way.PointsCm.Num();
			if (N == 0) continue;

			const bool bElevated = Way.bBridge || Way.Layer > 0;
			if (bElevated)
			{
				// Bridges / positive-layer ways: instead of a flat ABSOLUTE Z
				// (which buries the deck where terrain is high and floats it
				// where terrain is low), anchor the deck to terrain. Base height
				// interpolates between the two endpoints' terrain heights, and a
				// grade-separation clearance is added with a sin taper (0 at the
				// ends → full in the middle) so the deck MEETS its ground
				// approaches at the shared junction nodes and rises off them.
				const double Clearance = Way.Layer * LayerHeightCm + (Way.bBridge ? BridgeClearanceCm : 0.0);

				bool bH0 = false, bH1 = false;
				double T0 = SampleTerrainZ(World, Way.PointsCm[0].X,     Way.PointsCm[0].Y,     bH0);
				double T1 = SampleTerrainZ(World, Way.PointsCm[N - 1].X, Way.PointsCm[N - 1].Y, bH1);
				if (!bH0 || !FMath::IsFinite(T0)) T0 = 0.0;
				if (!bH1 || !FMath::IsFinite(T1)) T1 = 0.0;

				TArray<double> Arc; Arc.SetNumZeroed(N);
				for (int32 i = 1; i < N; ++i)
					Arc[i] = Arc[i - 1] + FVector2D::Distance(FVector2D(Way.PointsCm[i]), FVector2D(Way.PointsCm[i - 1]));
				const double L = Arc[N - 1];

				for (int32 i = 0; i < N; ++i)
				{
					const double t    = (L > 1.0) ? Arc[i] / L : 0.0;
					const double Base = FMath::Lerp(T0, T1, t);
					const double Taper = FMath::Sin((float)(t * PI)); // 0 → 1 → 0
					Way.PointsCm[i].Z = Base + Clearance * Taper;
				}
				continue;
			}

			// Sample terrain per point, tracking which samples were valid.
			TArray<double> Zs;  Zs.SetNumZeroed(N);
			TArray<bool>   Ok;  Ok.Init(false, N);
			for (int32 i = 0; i < N; ++i)
			{
				bool bHit = false;
				const double Z = SampleTerrainZ(World, Way.PointsCm[i].X, Way.PointsCm[i].Y, bHit);
				if (bHit && FMath::IsFinite(Z)) { Zs[i] = Z; Ok[i] = true; ++Sampled; }
			}

			// Fill gaps from nearest valid neighbour (forward then backward) so a
			// point that traced off the landscape never creates a Z=0 cliff.
			double Last = 0.0; bool bHave = false;
			for (int32 i = 0; i < N; ++i)
			{
				if (Ok[i]) { Last = Zs[i]; bHave = true; }
				else if (bHave) { Zs[i] = Last; ++Filled; }
			}
			bHave = false;
			for (int32 i = N - 1; i >= 0; --i)
			{
				if (Ok[i]) { Last = Zs[i]; bHave = true; }
				else if (bHave) { Zs[i] = Last; }
			}

			for (int32 i = 0; i < N; ++i)
				Way.PointsCm[i].Z = Zs[i];
		}
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: terrain drape — %d points hit terrain, %d filled from neighbours."),
			Sampled, Filled);
	}

	// 4c. Smooth the longitudinal road grade. Draping conforms each point to the
	//     raw terrain, so the bed chases every bump; here we smooth the Z profile
	//     (the horizontal OSM path is untouched) and clamp extreme slopes so the
	//     spline deform — and the surface mesh — carve clean graded cuts/fills
	//     instead of a jittery surface. Endpoints are held fixed so shared
	//     junction nodes stay continuous; bridges/overpasses keep their deck Z.
	{
		constexpr int    SmoothPasses = 2;    // light: keep the bed close to terrain (smaller cuts/fills)
		constexpr double MaxGrade     = 0.12; // 12% max longitudinal slope
		int32 SmoothedWays = 0;
		for (FOSMRoadWay& Way : Ways)
		{
			const int32 N = Way.PointsCm.Num();
			if (N < 3) continue;
			if (Way.bBridge || Way.Layer > 0) continue; // keep elevated deck profile

			// [1 2 1] moving average on Z, endpoints fixed.
			for (int32 Pass = 0; Pass < SmoothPasses; ++Pass)
			{
				double Prev = Way.PointsCm[0].Z; // original Z of the left neighbour
				for (int32 i = 1; i < N - 1; ++i)
				{
					const double Cur  = Way.PointsCm[i].Z;
					const double Next = Way.PointsCm[i + 1].Z;
					Way.PointsCm[i].Z = 0.25 * Prev + 0.5 * Cur + 0.25 * Next;
					Prev = Cur;
				}
			}

			// Slope clamp (interior points only; endpoints anchor the grade).
			auto SegLen = [&](int32 a, int32 b)
			{
				return (double)FVector2D::Distance(FVector2D(Way.PointsCm[a]), FVector2D(Way.PointsCm[b]));
			};
			for (int32 i = 1; i <= N - 2; ++i)
			{
				const double MaxDz = MaxGrade * SegLen(i - 1, i);
				const double Dz = Way.PointsCm[i].Z - Way.PointsCm[i - 1].Z;
				if (Dz >  MaxDz) Way.PointsCm[i].Z = Way.PointsCm[i - 1].Z + MaxDz;
				else if (Dz < -MaxDz) Way.PointsCm[i].Z = Way.PointsCm[i - 1].Z - MaxDz;
			}
			for (int32 i = N - 2; i >= 1; --i)
			{
				const double MaxDz = MaxGrade * SegLen(i, i + 1);
				const double Dz = Way.PointsCm[i].Z - Way.PointsCm[i + 1].Z;
				if (Dz >  MaxDz) Way.PointsCm[i].Z = Way.PointsCm[i + 1].Z + MaxDz;
				else if (Dz < -MaxDz) Way.PointsCm[i].Z = Way.PointsCm[i + 1].Z - MaxDz;
			}
			++SmoothedWays;
		}
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: grade smoothing — %d ways smoothed (%d passes, %.0f%% max slope)."),
			SmoothedWays, SmoothPasses, MaxGrade * 100.0);
	}

	// 4c2. Junction vertex-snap (operator rule: define an X,Y extent; all vertices
	//      in it are ONE intersection; "highest Z is king" — every vertex rises to
	//      meet the tallest). Steps:
	//        - target Z per node = MAX of its copies (never lower a shared vertex);
	//        - cluster junction nodes within osm.RoadIntersectionExtentCm into one
	//          intersection (catches dual-carriageway / staggered / near-coincident
	//          nodes), and raise every cluster member to the cluster's MAX Z (king);
	//        - snap every copy of every shared node to that target (+ exact shared
	//          XY) so the merged mesh/spline see ONE height at the junction (kills
	//          the aggressive Z spikes / radical per-approach differences);
	//        - flat landing: from each junction walk each incident road outward and
	//          cosine-blend toward the junction Z within LandingRadius, so approaches
	//          meet the intersection level instead of plunging.
	//      (See ROAD_FEATURES_PLAN.md — this is Phase 1a; topology merge is 1b.)
	if (CVarRoadJunctionSnap.GetValueOnGameThread() != 0)
	{
		auto HasIds   = [](const FOSMRoadWay& W) { return W.NodeIds.Num() == W.PointsCm.Num(); };
		auto IsGround = [](const FOSMRoadWay& W) { return !(W.bBridge || W.bTunnel || W.Layer != 0); };

		// References to each node id across ground ways.
		struct FNodeRef { int32 Way; int32 Pt; };
		TMap<int64, TArray<FNodeRef>> Refs;
		for (int32 w = 0; w < Ways.Num(); ++w)
		{
			const FOSMRoadWay& Way = Ways[w];
			if (!IsGround(Way) || !HasIds(Way)) continue;
			for (int32 i = 0; i < Way.NodeIds.Num(); ++i)
				if (Way.NodeIds[i] >= 0) Refs.FindOrAdd(Way.NodeIds[i]).Add({ w, i });
		}

		// Per-node draped position + base target Z = MAX of all copies ("highest
		// Z is king" — a shared vertex is never lowered). Also track the most
		// major road class incident to each node (min enum = Motorway) for the
		// optional CityGML class-priority king mode.
		TMap<int64, double>  NodeZ;
		TMap<int64, FVector> NodePos;
		TMap<int64, uint8>   NodeClass; // most-major EOSMHighwayClass at the node
		for (TPair<int64, TArray<FNodeRef>>& Pair : Refs)
		{
			double  MaxZ = -DBL_MAX;
			FVector Pos  = FVector::ZeroVector;
			uint8   Best = (uint8)EOSMHighwayClass::Unknown;
			for (const FNodeRef& r : Pair.Value)
			{
				const FVector& P = Ways[r.Way].PointsCm[r.Pt];
				if (P.Z > MaxZ) MaxZ = P.Z;
				Pos = P;
				Best = FMath::Min(Best, (uint8)Ways[r.Way].Class);
			}
			NodeZ.Add(Pair.Key, MaxZ);
			NodePos.Add(Pair.Key, Pos);
			NodeClass.Add(Pair.Key, Best);
		}

		// Junction incidence (endpoint=1, interior=2 → >=3 is a junction).
		TMap<int64, int32> Incidence;
		for (const FOSMRoadWay& Way : Ways)
		{
			if (!IsGround(Way) || !HasIds(Way)) continue;
			const int32 N = Way.PointsCm.Num();
			for (int32 i = 0; i < N; ++i)
				if (Way.NodeIds[i] >= 0) Incidence.FindOrAdd(Way.NodeIds[i]) += (i == 0 || i == N - 1) ? 1 : 2;
		}
		auto IsJunction = [&](int64 Id) { const int32* C = Incidence.Find(Id); return C && *C >= 3; };

		// Intersection clustering: group junction nodes whose XY fall within the
		// extent into ONE intersection (union-find over a grid), then apply
		// "highest Z is king" — every member rises to the cluster's max Z.
		const double ExtentCm = FMath::Max(1.f, CVarRoadIntersectionExtentCm.GetValueOnGameThread());
		TArray<int64> JIds;
		for (const TPair<int64, int32>& P : Incidence)
			if (P.Value >= 3) JIds.Add(P.Key);

		const int32 J = JIds.Num();
		TArray<int32> Parent; Parent.SetNum(J);
		for (int32 i = 0; i < J; ++i) Parent[i] = i;
		auto Find = [&](int32 a) -> int32 { while (Parent[a] != a) { Parent[a] = Parent[Parent[a]]; a = Parent[a]; } return a; };
		auto Union = [&](int32 a, int32 b) { const int32 ra = Find(a), rb = Find(b); if (ra != rb) Parent[ra] = rb; };

		auto CellOf = [&](const FVector& P) { return FIntPoint(FMath::FloorToInt(P.X / ExtentCm), FMath::FloorToInt(P.Y / ExtentCm)); };
		TMap<FIntPoint, TArray<int32>> Grid;
		for (int32 j = 0; j < J; ++j)
			Grid.FindOrAdd(CellOf(NodePos[JIds[j]])).Add(j);
		for (int32 j = 0; j < J; ++j)
		{
			const FVector   Pj = NodePos[JIds[j]];
			const FIntPoint C  = CellOf(Pj);
			for (int32 dy = -1; dy <= 1; ++dy)
			for (int32 dx = -1; dx <= 1; ++dx)
			{
				const TArray<int32>* Bucket = Grid.Find(FIntPoint(C.X + dx, C.Y + dy));
				if (!Bucket) continue;
				for (int32 k : *Bucket)
					if (k > j && FVector2D::Distance(FVector2D(Pj), FVector2D(NodePos[JIds[k]])) <= ExtentCm)
						Union(j, k);
			}
		}
		// Per-cluster king: max-Z member (default, never lowers) AND the
		// most-major-class member (CityGML "through road defines the plane").
		const bool bClassKing = CVarRoadJunctionKingMode.GetValueOnGameThread() != 0;
		TMap<int32, double> RootMax;                 // cluster max Z
		TMap<int32, int32>  RootKingJ;               // member j with the max Z
		TMap<int32, uint8>  RootBestClass;           // cluster most-major class
		TMap<int32, double> RootClassZ;              // Z of that class-king member
		for (int32 j = 0; j < J; ++j)
		{
			const int32  Root = Find(j);
			const double Z    = NodeZ[JIds[j]]; // still the per-node base max here
			const uint8  Cls  = NodeClass[JIds[j]];
			if (double* M = RootMax.Find(Root))
			{
				if (Z > *M) { *M = Z; RootKingJ[Root] = j; }
				// class-king: smaller enum = more major; tie-break on higher Z.
				if (Cls < RootBestClass[Root] || (Cls == RootBestClass[Root] && Z > RootClassZ[Root]))
				{ RootBestClass[Root] = Cls; RootClassZ[Root] = Z; }
			}
			else
			{
				RootMax.Add(Root, Z); RootKingJ.Add(Root, j);
				RootBestClass.Add(Root, Cls); RootClassZ.Add(Root, Z);
			}
		}
		const int32 Clusters = RootMax.Num();

		// Chosen cluster target Z (mode-dependent). Reused by the snap, the flat
		// plate/landing, and the Phase-1b centroid so they never disagree.
		TMap<int32, double> RootTargetZ;
		for (const TPair<int32, double>& KV : RootMax)
			RootTargetZ.Add(KV.Key, bClassKing ? RootClassZ[KV.Key] : KV.Value);
		for (int32 j = 0; j < J; ++j)
			NodeZ[JIds[j]] = RootTargetZ[Find(j)];

		// Snap every copy of every shared node to the reconciled/king Z (+ exact
		// shared XY). Singletons are unchanged (target = own Z).
		int32 Reconciled = 0;
		for (TPair<int64, TArray<FNodeRef>>& Pair : Refs)
		{
			TArray<FNodeRef>& R = Pair.Value;
			const double    TZ = NodeZ[Pair.Key];
			const FVector2D XY(Ways[R[0].Way].PointsCm[R[0].Pt]);
			for (const FNodeRef& r : R)
			{
				FVector& P = Ways[r.Way].PointsCm[r.Pt];
				P.X = XY.X; P.Y = XY.Y; P.Z = TZ;
			}
			if (R.Num() > 1) ++Reconciled;
		}

		// Phase 1b — topological vertex merge. For every intersection cluster
		// that spans MORE THAN ONE junction node (dual carriageways, staggered
		// stop-lines, slip-road stubs, GPS jitter), collapse all its member
		// nodes into a single shared node: id → the king's id, XY → cluster
		// centroid, Z → king Z. Downstream (graph merge, spline bake, sidewalk
		// block tracing) keys on the node id, so this yields ONE clean junction
		// with ≤ road-count incident edges instead of a knot of tiny fans.
		int32 MergedClusters = 0, MergedNodes = 0, CollapsedPts = 0;
		if (CVarRoadIntersectionMerge.GetValueOnGameThread() != 0 && J > 0)
		{
			// Centroid accumulator per root (member XY average).
			TMap<int32, FVector2D> RootSum;
			TMap<int32, int32>     RootCnt;
			for (int32 j = 0; j < J; ++j)
			{
				const int32 Root = Find(j);
				RootSum.FindOrAdd(Root) += FVector2D(NodePos[JIds[j]]);
				RootCnt.FindOrAdd(Root) += 1;
			}

			// Build the remap only for multi-node clusters (singletons no-op).
			TMap<int64, int64> Remap;               // member node id → king id
			TMap<int64, FVector> RepPos;            // king id → (centroid, king Z)
			for (const TPair<int32, int32>& KP : RootCnt)
			{
				if (KP.Value < 2) continue;         // singleton cluster: nothing to merge
				const int32   Root   = KP.Key;
				const int64   KingId = JIds[RootKingJ[Root]];
				const FVector2D C    = RootSum[Root] / double(KP.Value);
				RepPos.Add(KingId, FVector(C.X, C.Y, RootTargetZ[Root]));
				++MergedClusters;
			}
			for (int32 j = 0; j < J; ++j)
			{
				const int64 KingId = JIds[RootKingJ[Find(j)]];
				if (RepPos.Contains(KingId) && JIds[j] != KingId)
				{
					Remap.Add(JIds[j], KingId);
					++MergedNodes;
				}
			}

			// Apply to ground ways: remap ids/positions, then drop consecutive
			// duplicates so a road that entered/left the junction collapses to a
			// single touch point at the merged node.
			if (Remap.Num() > 0)
			{
				for (FOSMRoadWay& Way : Ways)
				{
					if (!IsGround(Way) || !HasIds(Way)) continue;
					const int32 N = Way.PointsCm.Num();
					bool bTouched = false;
					for (int32 i = 0; i < N; ++i)
					{
						if (const int64* Rep = Remap.Find(Way.NodeIds[i]))
						{
							Way.NodeIds[i]  = *Rep;          // member → king id
							Way.PointsCm[i] = RepPos[*Rep];  // → centroid + king Z
							bTouched = true;
						}
						else if (const FVector* KP = RepPos.Find(Way.NodeIds[i]))
						{
							Way.PointsCm[i] = *KP;           // king's own copies → centroid too
							bTouched = true;
						}
					}
					if (!bTouched) continue;

					TArray<FVector> NewPts;
					TArray<int64>   NewIds;
					NewPts.Reserve(N); NewIds.Reserve(N);
					for (int32 i = 0; i < N; ++i)
					{
						const bool bDupId = NewIds.Num() > 0 && Way.NodeIds[i] >= 0
											&& NewIds.Last() == Way.NodeIds[i];
						if (bDupId) { ++CollapsedPts; continue; }
						NewPts.Add(Way.PointsCm[i]);
						NewIds.Add(Way.NodeIds[i]);
					}
					Way.PointsCm = MoveTemp(NewPts);
					Way.NodeIds  = MoveTemp(NewIds);
				}
			}
		}

		// Pass B — flat intersection plate + graded landing along each incident
		// road (CityGML: an Intersection is a coherent AREA). Within the plate
		// radius the road is held at the junction Z; from there out to the landing
		// radius it cosine-blends back to the natural profile.
		//
		// RoadBLD parity (bMatchIntersectionReferenceLineHeights): the radii are
		// WIDTH-SCALED per junction instead of fixed —
		//   plate   = widest incident arm's half-width + margin  (covers the whole
		//             junction polygon so every arm arrives at the same level);
		//   landing = plate + narrowest arm full width x BlendWidthMult
		//             (RoadBLD blends along the narrower road over NarrowWidth x 3).
		// The CVars osm.RoadJunctionFlatCm / LandingCm act as minimums, so alley
		// crossings keep a small plate while arterial junctions get a big one.
		const double MinLandingCm  = FMath::Max(50.f, CVarRoadJunctionLandingCm.GetValueOnGameThread());
		const double MinFlatCm     = FMath::Clamp(CVarRoadJunctionFlatCm.GetValueOnGameThread(),
									0.f, (float)MinLandingCm - 1.f);
		const double BlendMult     = FMath::Max(0.f, CVarRoadJunctionBlendWidthMult.GetValueOnGameThread());

		// Per-junction incident arm widths: widest half-width sizes the plate,
		// narrowest full width sizes the blend (RoadBLD's "narrower road").
		TMap<int64, double> NodePlateCm, NodeLandingCm;
		for (const TPair<int64, TArray<FNodeRef>>& Pair : Refs)
		{
			if (!IsJunction(Pair.Key)) continue;
			double MaxHalfW = 0.0, MinFullW = DBL_MAX;
			for (const FNodeRef& r : Pair.Value)
			{
				const double HalfW = OSMRoadGeom::RoadHalfWidthCm(Ways[r.Way]);
				MaxHalfW = FMath::Max(MaxHalfW, HalfW);
				MinFullW = FMath::Min(MinFullW, 2.0 * HalfW);
			}
			if (MinFullW == DBL_MAX) MinFullW = 0.0;
			const double Plate   = FMath::Max(MinFlatCm, MaxHalfW + kJunctionPlateMarginCm);
			const double Landing = FMath::Max(MinLandingCm, Plate + MinFullW * BlendMult);
			NodePlateCm.Add(Pair.Key, Plate);
			NodeLandingCm.Add(Pair.Key, Landing);
		}

		int64 Landed = 0, Plated = 0;
		for (int32 w = 0; w < Ways.Num(); ++w)
		{
			FOSMRoadWay& Way = Ways[w];
			if (!IsGround(Way) || !HasIds(Way)) continue;
			const int32 N = Way.PointsCm.Num();

			for (int32 j = 0; j < N; ++j)
			{
				if (!IsJunction(Way.NodeIds[j])) continue;
				const double JZ     = NodeZ[Way.NodeIds[j]];
				const double FlatCm = NodePlateCm.FindRef(Way.NodeIds[j], MinFlatCm);
				const double LandingRadiusCm = NodeLandingCm.FindRef(Way.NodeIds[j], MinLandingCm);
				const double BlendSpan = FMath::Max(1.0, LandingRadiusCm - FlatCm);

				// Blend outward in both directions until the landing radius or the
				// next junction, whichever comes first.
				auto Grade = [&](int32 step)
				{
					double acc = 0.0;
					for (int32 i = j + step; i >= 0 && i < N; i += step)
					{
						acc += FVector2D::Distance(FVector2D(Way.PointsCm[i]), FVector2D(Way.PointsCm[i - step]));
						if (acc >= LandingRadiusCm) break;
						if (IsJunction(Way.NodeIds[i])) break;
						if (acc <= FlatCm)
						{
							Way.PointsCm[i].Z = JZ;                            // flat plate
							++Plated;
						}
						else
						{
							const double t   = (acc - FlatCm) / BlendSpan;     // 0 → 1
							const double wgt = 0.5 * (1.0 + FMath::Cos(t * PI));// 1 → 0
							Way.PointsCm[i].Z = FMath::Lerp(Way.PointsCm[i].Z, JZ, wgt);
						}
						++Landed;
					}
				};
				Grade(+1);
				Grade(-1);
			}
		}

		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: junction vertex-snap — %d junctions in %d intersection clusters (extent %.0f cm), king=%s; ")
			TEXT("%d shared nodes reconciled; topo-merge folded %d nodes into %d clusters (%d pts collapsed); ")
			TEXT("%lld plate + %lld landing points (width-scaled radii, min flat %.0f cm / min landing %.0f cm, blend x%.1f)."),
			J, Clusters, ExtentCm, bClassKing ? TEXT("class") : TEXT("maxZ"),
			Reconciled, MergedNodes, MergedClusters, CollapsedPts, Plated, Landed, MinFlatCm, MinLandingCm, BlendMult);
	}

	// 4d. Junction-anchored continuity (operator: "lower roads should have risen
	//     to meet"). A road span between two junction nodes must not dive below
	//     the straight chord joining those junction heights — those nodes are
	//     SHARED across the network so their heights are consistent. Interior
	//     dips are lifted UP to the chord (the landscape deform then FILLS up to
	//     the road), while points already above the chord (hills) are untouched
	//     so cuts still work. Lift is clamped to avoid huge fills on long spans.
	{
		constexpr double SagTolCm  = 150.0;  // road may still dip this far below the chord
		// Lift is a SMALL junction reconciliation only, NOT a canyon filler. A
		// large cap (was 2000) built mesa embankments on steep terrain (the
		// landscape conform grades the fill back to terrain over only a few quads
		// → vertical cliffs). Real canyon crossings are handled as bridges/tunnels
		// from the OSM tags, not by lifting the bed.
		constexpr double MaxLiftCm = 400.0;

		auto HasIds = [](const FOSMRoadWay& W) { return W.NodeIds.Num() == W.PointsCm.Num(); };
		auto IsGround = [](const FOSMRoadWay& W) { return !(W.bBridge || W.bTunnel || W.Layer != 0); };

		// Node incidence: endpoint = 1, interior pass-through = 2 → >=3 is a junction.
		TMap<int64, int32> Incidence;
		for (const FOSMRoadWay& Way : Ways)
		{
			if (!IsGround(Way) || !HasIds(Way)) continue;
			const int32 N = Way.PointsCm.Num();
			if (N < 2) continue;
			for (int32 i = 0; i < N; ++i)
			{
				const int64 Id = Way.NodeIds[i];
				if (Id >= 0) Incidence.FindOrAdd(Id) += (i == 0 || i == N - 1) ? 1 : 2;
			}
		}
		auto IsJunction = [&](int64 Id) { const int32* C = Incidence.Find(Id); return C && *C >= 3; };

		int32 LiftedWays = 0; int64 LiftedPts = 0;
		for (FOSMRoadWay& Way : Ways)
		{
			if (!IsGround(Way) || !HasIds(Way)) continue;
			const int32 N = Way.PointsCm.Num();
			if (N < 3) continue;

			// Span breakpoints = endpoints + every junction node along the way.
			TArray<int32> Bps; Bps.Add(0);
			for (int32 i = 0; i < N; ++i)
				if (IsJunction(Way.NodeIds[i])) Bps.AddUnique(i);
			Bps.AddUnique(N - 1);
			Bps.Sort();

			bool bAny = false;
			for (int32 k = 0; k + 1 < Bps.Num(); ++k)
			{
				const int32 S = Bps[k];
				const int32 E = Bps[k + 1];
				if (E <= S + 1) continue; // no interior points to lift

				const double Z0 = Way.PointsCm[S].Z;
				const double Z1 = Way.PointsCm[E].Z;

				TArray<double> Arc; Arc.SetNumZeroed(E - S + 1);
				for (int32 i = S + 1; i <= E; ++i)
					Arc[i - S] = Arc[i - S - 1] + FVector2D::Distance(FVector2D(Way.PointsCm[i]), FVector2D(Way.PointsCm[i - 1]));
				const double L = Arc[E - S];
				if (L < 1.0) continue;

				for (int32 i = S + 1; i < E; ++i)
				{
					const double LineZ  = FMath::Lerp(Z0, Z1, Arc[i - S] / L);
					const double Cur    = Way.PointsCm[i].Z;
					double Target = LineZ - SagTolCm;
					if (Target > Cur)
					{
						if (Target - Cur > MaxLiftCm) Target = Cur + MaxLiftCm;
						Way.PointsCm[i].Z = Target;
						++LiftedPts; bAny = true;
					}
				}
			}
			if (bAny) ++LiftedWays;
		}
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: junction-anchored continuity — %d ways lifted, %lld points raised to chord."),
			LiftedWays, LiftedPts);
	}

	// 4d2. Alignment-feature pass (Phase 3, ROAD_FEATURES_PLAN §2A). Runs AFTER
	//      junction snap + continuity so endpoints are already at their king Z.
	//        - Link grade (slip lanes / on-off ramps): a highway=*_link is a
	//          connector between two junctions; instead of chasing terrain bumps
	//          it should ride a clean monotonic grade between its junction-anchored
	//          ends. We interpolate interior Z per span (breaking at any interior
	//          junction) — the ramp becomes a smooth ribbon, the deform fills/cuts
	//          terrain to it. Height authority: inherits the parent intersections.
	//        - Hairpin/gore recognisers are CENSUS ONLY (no geometry change yet):
	//          hairpins are already continuous (incidence 2 ⇒ never clustered as a
	//          junction), and gore/taper needs a mesh-builder width pass (later).
	if (CVarFeatureRecognize.GetValueOnGameThread() != 0)
	{
		auto HasIds   = [](const FOSMRoadWay& W) { return W.NodeIds.Num() == W.PointsCm.Num(); };
		auto IsGround = [](const FOSMRoadWay& W) { return !(W.bBridge || W.bTunnel || W.Layer != 0); };

		// Incidence (endpoint 1 / interior 2 → ≥3 junction), same basis as 4c2/4d.
		TMap<int64, int32> Incidence;
		for (const FOSMRoadWay& Way : Ways)
		{
			if (!IsGround(Way) || !HasIds(Way)) continue;
			const int32 N = Way.PointsCm.Num();
			if (N < 2) continue;
			for (int32 i = 0; i < N; ++i)
				if (Way.NodeIds[i] >= 0) Incidence.FindOrAdd(Way.NodeIds[i]) += (i == 0 || i == N - 1) ? 1 : 2;
		}
		auto IsJunction = [&](int64 Id) { const int32* C = Incidence.Find(Id); return C && *C >= 3; };

		// --- Link grade continuity ---------------------------------------------
		const bool bLinkGrade = CVarFeatLinkGrade.GetValueOnGameThread() != 0;
		int32 GradedLinks = 0; int64 GradedPts = 0;
		if (bLinkGrade)
		{
			for (FOSMRoadWay& Way : Ways)
			{
				if (!Way.bLink || !IsGround(Way) || !HasIds(Way)) continue;
				const int32 N = Way.PointsCm.Num();
				if (N < 3) continue;

				// Span breakpoints = endpoints + interior junctions (a link that
				// passes through a junction grades each sub-span to its own ends).
				TArray<int32> Bps; Bps.Add(0);
				for (int32 i = 1; i < N - 1; ++i)
					if (IsJunction(Way.NodeIds[i])) Bps.Add(i);
				Bps.Add(N - 1);

				bool bAny = false;
				for (int32 k = 0; k + 1 < Bps.Num(); ++k)
				{
					const int32 S = Bps[k], E = Bps[k + 1];
					if (E <= S + 1) continue;
					const double Z0 = Way.PointsCm[S].Z, Z1 = Way.PointsCm[E].Z;

					TArray<double> Arc; Arc.SetNumZeroed(E - S + 1);
					for (int32 i = S + 1; i <= E; ++i)
						Arc[i - S] = Arc[i - S - 1] + FVector2D::Distance(FVector2D(Way.PointsCm[i]), FVector2D(Way.PointsCm[i - 1]));
					const double L = Arc[E - S];
					if (L < 1.0) continue;

					for (int32 i = S + 1; i < E; ++i)
					{
						Way.PointsCm[i].Z = FMath::Lerp(Z0, Z1, Arc[i - S] / L);
						++GradedPts; bAny = true;
					}
				}
				if (bAny) ++GradedLinks;
			}
		}

		// --- Hairpin / switchback census (no geometry change) ------------------
		const double HairpinDeg = CVarFeatHairpinDeg.GetValueOnGameThread();
		int32 Hairpins = 0, LinkWays = 0, LinksAtJunction = 0;
		for (const FOSMRoadWay& Way : Ways)
		{
			if (!IsGround(Way) || !HasIds(Way)) continue;
			const int32 N = Way.PointsCm.Num();
			if (Way.bLink)
			{
				++LinkWays;
				if (N >= 2 && (IsJunction(Way.NodeIds[0]) || IsJunction(Way.NodeIds[N - 1])))
					++LinksAtJunction; // gore/taper candidate (mesh pass later)
			}
			if (HairpinDeg > 0.0 && N >= 3)
			{
				for (int32 i = 1; i < N - 1; ++i)
				{
					const FVector2D d0 = FVector2D(Way.PointsCm[i])     - FVector2D(Way.PointsCm[i - 1]);
					const FVector2D d1 = FVector2D(Way.PointsCm[i + 1]) - FVector2D(Way.PointsCm[i]);
					if (d0.IsNearlyZero() || d1.IsNearlyZero()) continue;
					const double Cos = FVector2D::DotProduct(d0.GetSafeNormal(), d1.GetSafeNormal());
					const double DeflDeg = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Cos, -1.0, 1.0)));
					if (DeflDeg >= HairpinDeg) ++Hairpins;
				}
			}
		}

		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: alignment features — %d links graded (%lld pts); ")
			TEXT("%d link ways (%d touch a junction → gore/taper candidates); %d hairpin bends (>=%.0f deg)."),
			GradedLinks, GradedPts, LinkWays, LinksAtJunction, Hairpins, HairpinDeg);

		// --- Local traffic calming: speed hump / table Z-bumps (Phase 5) -------
		// A traffic_calming=hump/table node ON a road raises the road Z locally
		// into a bump the deform then conforms terrain to. Layered on the
		// reconciled base (king priority rank 5). Skip junction nodes so we never
		// disturb an intersection king. Applied to every way sharing the node so
		// the bump is consistent, tapering cosine → 0 over the hump radius.
		if (CVarFeatCalming.GetValueOnGameThread() != 0 && NodeFeatures.Num() > 0)
		{
			const double HumpCm   = CVarFeatHumpCm.GetValueOnGameThread();
			const double RadiusCm = FMath::Max(1.f, CVarFeatHumpRadiusCm.GetValueOnGameThread());
			int32 Humps = 0; int64 BumpPts = 0;

			for (FOSMRoadWay& Way : Ways)
			{
				if (!IsGround(Way) || !HasIds(Way)) continue;
				const int32 N = Way.PointsCm.Num();
				if (N < 2) continue;

				for (int32 c = 0; c < N; ++c)
				{
					const int64 Id = Way.NodeIds[c];
					if (Id < 0 || IsJunction(Id)) continue;
					const uint32* F = NodeFeatures.Find(Id);
					if (!F || !(*F & NF_CalmingHump)) continue;

					// Raise the hump centre and taper along the road both ways.
					Way.PointsCm[c].Z += HumpCm;
					++Humps; ++BumpPts;
					auto Taper = [&](int32 step)
					{
						double acc = 0.0;
						for (int32 i = c + step; i >= 0 && i < N; i += step)
						{
							acc += FVector2D::Distance(FVector2D(Way.PointsCm[i]), FVector2D(Way.PointsCm[i - step]));
							if (acc >= RadiusCm) break;
							const double t = acc / RadiusCm;                    // 0 → 1
							const double w = 0.5 * (1.0 + FMath::Cos(t * PI));  // 1 → 0
							Way.PointsCm[i].Z += HumpCm * w;
							++BumpPts;
						}
					};
					Taper(+1);
					Taper(-1);
				}
			}
			UE_LOG(LogOSMRoad, Log,
				TEXT("OSM.ImportRoads: traffic calming — %d hump/table nodes bumped +%.0f cm (radius %.0f cm), %lld points raised."),
				Humps, HumpCm, RadiusCm, BumpPts);
		}
	}

	// 4e. Grade-separation census: how many ways OSM actually tags as bridge /
	//     tunnel / layer for this bbox. These are the ways that should NOT be
	//     draped+deformed flat into the terrain (they cross ravines etc.).
	{
		int32 NBridge = 0, NTunnel = 0, NCulvert = 0, NLayerPos = 0, NLayerNeg = 0;
		for (const FOSMRoadWay& Way : Ways)
		{
			if (Way.bBridge)   ++NBridge;
			if (Way.bTunnel)   ++NTunnel;
			if (Way.bCulvert)  ++NCulvert;
			if (Way.Layer > 0) ++NLayerPos;
			if (Way.Layer < 0) ++NLayerNeg;
		}
		const bool bHideTun = CVarRoadHideTunnels.GetValueOnGameThread() != 0;
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: grade-separation census — %d bridges, %d tunnels (%s), %d culverts (kept at grade), ")
			TEXT("%d layer>0, %d layer<0 (of %d ways)."),
			NBridge, NTunnel, bHideTun ? TEXT("hidden on surface") : TEXT("rendered as ground"),
			NCulvert, NLayerPos, NLayerNeg, Ways.Num());
	}

	// 4e2. Node-feature census (Phase 3.5 infra). Count road-feature nodes that
	//      actually lie ON an imported road way (node id shared), so later phases
	//      know what this bbox contains before we build anything for them. Pure
	//      recognition — no geometry change yet.
	if (NodeFeatures.Num() > 0)
	{
		TSet<int64> OnRoad;
		for (const FOSMRoadWay& Way : Ways)
			for (int64 Id : Way.NodeIds)
				if (Id >= 0 && NodeFeatures.Contains(Id)) OnRoad.Add(Id);

		int32 C_Cross = 0, C_Hump = 0, C_CalmOther = 0, C_Turn = 0, C_Bollard = 0,
			  C_Toll = 0, C_Level = 0, C_Mini = 0, C_Stop = 0, C_Sig = 0;
		for (int64 Id : OnRoad)
		{
			const uint32 F = NodeFeatures[Id];
			if (F & NF_Crossing)       ++C_Cross;
			if (F & NF_CalmingHump)    ++C_Hump;
			if (F & NF_CalmingOther)   ++C_CalmOther;
			if (F & NF_TurningCircle)  ++C_Turn;
			if (F & NF_Bollard)        ++C_Bollard;
			if (F & NF_Toll)           ++C_Toll;
			if (F & NF_LevelCrossing)  ++C_Level;
			if (F & NF_MiniRoundabout) ++C_Mini;
			if (F & NF_Stop)           ++C_Stop;
			if (F & NF_Signals)        ++C_Sig;
		}
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: node-feature census (on-road %d of %d tagged) — ")
			TEXT("%d crossings, %d humps/tables, %d other-calming, %d turning-circles, ")
			TEXT("%d bollards, %d toll, %d level-crossings, %d mini-roundabouts, %d stop, %d signals."),
			OnRoad.Num(), NodeFeatures.Num(),
			C_Cross, C_Hump, C_CalmOther, C_Turn, C_Bollard, C_Toll, C_Level, C_Mini, C_Stop, C_Sig);
	}

	// 4f. DEBUG: spline-preview mode. Draw the finalised road centerlines as
	//     persistent debug lines and STOP before mesh + deform, so the network
	//     (heights, junctions, bridges/tunnels) can be inspected first.
	if (CVarRoadDebugSplines.GetValueOnGameThread() != 0)
	{
		FlushPersistentDebugLines(World); // clear any previous preview

		int32 DrawnWays = 0, DrawnSegs = 0;
		for (const FOSMRoadWay& Way : Ways)
		{
			const int32 N = Way.PointsCm.Num();
			if (N < 2) continue;

			FColor Color = FColor::Green;                 // ground road
			if (Way.bTunnel)        Color = FColor::Cyan;
			else if (Way.bBridge)   Color = FColor::Orange;
			else if (Way.Layer > 0) Color = FColor::Yellow;

			const float Thickness = 60.f; // cm — fat so it reads from altitude

			for (int32 i = 0; i + 1 < N; ++i)
			{
				if (Way.PointsCm[i].ContainsNaN() || Way.PointsCm[i + 1].ContainsNaN()) continue;
				DrawDebugLine(World, Way.PointsCm[i], Way.PointsCm[i + 1], Color,
					/*bPersistent*/ true, /*LifeTime*/ -1.f, /*DepthPriority*/ 0, Thickness);
				++DrawnSegs;
			}

			// Mark junction-shared node positions with a small sphere.
			DrawDebugSphere(World, Way.PointsCm[0],          120.f, 6, FColor::Red, true, -1.f, 0, 20.f);
			DrawDebugSphere(World, Way.PointsCm[N - 1],      120.f, 6, FColor::Red, true, -1.f, 0, 20.f);
			++DrawnWays;
		}

		UE_LOG(LogOSMRoad, Warning,
			TEXT("OSM.ImportRoads: DEBUG spline-preview — drew %d ways, %d segments (green=ground, orange=bridge, cyan=tunnel, yellow=layer>0). ")
			TEXT("Mesh + deform SKIPPED. Set osm.RoadDebugSplines 0 for the full build."),
			DrawnWays, DrawnSegs);

		OutStats.RoadsCreated = DrawnWays;
		return true;
	}

	// 5. Find the target Landscape (the SRTM terrain imported by OSM.ImportTerrain).
	ALandscape* Landscape = nullptr;
	for (TActorIterator<ALandscape> It(World); It; ++It)
	{
		Landscape = *It;
		break;
	}
	if (!Landscape)
	{
		OutError = TEXT("No ALandscape in level. Run OSM.ImportTerrain for this bbox first.");
		return false;
	}

	// 5b. Cache the fully-projected, fully-attributed ways (ALL classes:
	//     drivable + footway/path/cycleway/steps) into the world subsystem so
	//     the "OSM Roads -> Points" PCG source node can build attributed points
	//     for the per-category graphs. The node re-drapes Z per point, so only
	//     engine-space X,Y and the parsed attributes need to be valid here.
	if (UOSMRoadDataSubsystem* RoadData = World->GetSubsystem<UOSMRoadDataSubsystem>())
	{
		RoadData->SetWays(Ways);
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: cached %d ways for PCG (OSM Roads -> Points node)."), Ways.Num());
	}

	// 6. Keep only drivable ways for the road network (footways/paths become
	//    sidewalks via PCG later). Build the spline input list. Phase 2: real
	//    tunnels are dropped here so they never reach the mesh or the deform —
	//    the surface road correctly disappears at the portal (culverts stay,
	//    they are at-grade roads with drainage passing beneath).
	const bool bHideTunnels = CVarRoadHideTunnels.GetValueOnGameThread() != 0;
	int32 HiddenTunnels = 0;
	TArray<FOSMRoadWay> RoadWays;
	RoadWays.Reserve(Ways.Num());
	for (const FOSMRoadWay& Way : Ways)
	{
		if (!OSMRoadClassify::IsDrivable(Way.Class))
		{
			++OutStats.WaysSkipped;
			continue;
		}
		if (bHideTunnels && Way.bTunnel)
		{
			++HiddenTunnels;
			++OutStats.WaysSkipped;
			continue;
		}
		RoadWays.Add(Way);
		++OutStats.WaysImported;
	}
	if (HiddenTunnels > 0)
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: hid %d tunnel way(s) from the surface (osm.RoadHideTunnels 0 to render them)."),
			HiddenTunnels);

	// 6a. DEBUG: build the NATIVE landscape splines and KEEP them persistent (no
	//     bake, no mesh) so the spline network can be inspected before anything
	//     else runs. This is the "see the landscape splines created before the
	//     mesh" preview the operator asked for.
	if (CVarRoadSplineDebug.GetValueOnGameThread() != 0)
	{
		int32 NumCP = 0, NumSeg = 0;
		UE_LOG(LogOSMRoad, Warning,
			TEXT("OSM.ImportRoads: DEBUG landscape-spline preview (keep, no bake/mesh) for %d road ways..."),
			RoadWays.Num());
		if (!OSMRoadSplineDeform::DeformViaLandscapeSplines(
				World, Landscape, RoadWays, NumCP, NumSeg, OutError, /*bDebugKeep*/ true))
			return false;
		OutStats.RoadsCreated = NumSeg;
		UE_LOG(LogOSMRoad, Warning,
			TEXT("OSM.ImportRoads DONE (spline DEBUG-KEEP): %d control points, %d segments left persistent. ")
			TEXT("Open Landscape mode → Manage → Splines to inspect. Set osm.RoadSplineDebug 0 to clear + build."),
			NumCP, NumSeg);
		return true;
	}

	// 6a2c. PIPELINE 4 — RoadNet engine (independent clean-room). Feed the parsed
	//       + draped OSM ways into our own URoadNetwork: staged rebuild (curves →
	//       endpoint-joint topology → corners → overlap masks → Clipper2 boolean
	//       surface → perimeter loops → mesh → commit). Fully independent of
	//       RoadBLD. Terrain corridors are sculpted first (same as Pipeline 2) so
	//       the eventual meshes sit on a flattened bed.
	//       NOTE: geometry commit stages are under construction; this currently
	//       runs curve + topology derivation and logs the pipeline.
	if (CVarRoadPipeline.GetValueOnGameThread() == 4)
	{
		int32 SculptWays = 0;
		int64 SculptCells = 0;
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: PIPELINE 4 — sculpting road corridors into the landscape (buffer %.1f m/side)..."),
			CVarRoadDeformBufferM.GetValueOnGameThread());
		if (!SculptRoadCorridors(World, Landscape, RoadWays, SculptWays, SculptCells, OutError))
			return false;
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: PIPELINE 4 — corridor sculpt done (%d corridors, %lld cells)."),
			SculptWays, SculptCells);

		int32 NumRoads = 0;
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: PIPELINE 4 (RoadNet engine) for %d road ways..."), RoadWays.Num());
		if (!OSMRoadNetBridge::BuildRoads(World, RoadWays, NumRoads, OutError))
			return false;
		OutStats.RoadsCreated = NumRoads;

		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads DONE (RoadNet): %d parsed, %d imported, %d skipped, %d RoadNet roads."),
			OutStats.WaysTotal, OutStats.WaysImported, OutStats.WaysSkipped, NumRoads);
		return true;
	}

	// 6a2b. PIPELINE 2 — RoadBuilder plugin engine. Feed the parsed + draped OSM
	//       ways straight into RoadBuilder (ARoadScene/ARoadActor/URoadStyle):
	//       per-lane carriageway triangulated between real boundary curves, curbs
	//       + sidewalks extruded from cross-sections, lane markings on boundaries,
	//       and CDT-filled junctions. Styles/shapes are built in C++ from the
	//       /Game/OSM materials (RoadBuilder ships no content).
	//       Terrain: the corridors are sculpted flat into the heightmap FIRST
	//       (same bed Z the ways carry, flat core = road + sidewalk + buffer),
	//       so the RoadBuilder meshes sit on a flattened bed instead of poking
	//       through the natural terrain.
	if (CVarRoadPipeline.GetValueOnGameThread() == 2)
	{
		// HARD EXTENT GUARD: clip every way to the landscape's world bounds.
		// The lon/lat bbox clip upstream only runs on the Overpass HTTP path
		// with a real bbox — file/GeoJSON imports skip it, and ways that run
		// past the terrain edge give RoadBuilder junction/corner curves that
		// try to close loops across the map. The landscape is the one
		// authoritative extent for what should exist, so cut everything else.
		if (ULandscapeInfo* ClipInfo = Landscape ? Landscape->GetLandscapeInfo() : nullptr)
		{
			int32 QMinX, QMinY, QMaxX, QMaxY;
			if (ClipInfo->GetLandscapeExtent(QMinX, QMinY, QMaxX, QMaxY))
			{
				const FTransform LtoW = Landscape->LandscapeActorToWorld();
				// Axis-aligned world bounds of the landscape's quad rect
				// (handles rotated/scaled landscapes via the 4 corners).
				double XMin = DBL_MAX, YMin = DBL_MAX, XMax = -DBL_MAX, YMax = -DBL_MAX;
				for (const FVector& Corner : {
					LtoW.TransformPosition(FVector((double)QMinX, (double)QMinY, 0.0)),
					LtoW.TransformPosition(FVector((double)QMaxX, (double)QMinY, 0.0)),
					LtoW.TransformPosition(FVector((double)QMinX, (double)QMaxY, 0.0)),
					LtoW.TransformPosition(FVector((double)QMaxX, (double)QMaxY, 0.0)) })
				{
					XMin = FMath::Min(XMin, Corner.X); XMax = FMath::Max(XMax, Corner.X);
					YMin = FMath::Min(YMin, Corner.Y); YMax = FMath::Max(YMax, Corner.Y);
				}

				const int32 BeforeWays = RoadWays.Num();
				int32 DroppedPts = 0;
				TArray<FOSMRoadWay> InBounds;
				InBounds.Reserve(RoadWays.Num());
				for (const FOSMRoadWay& Way : RoadWays)
				{
					TArray<TArray<FVector>> Runs;
					TArray<TArray<int64>>   NodeRuns;
					ClipWayToBBoxN(Way.PointsCm, Way.NodeIds, XMin, YMin, XMax, YMax, Runs, NodeRuns);
					int32 KeptPts = 0;
					for (int32 r = 0; r < Runs.Num(); ++r)
					{
						KeptPts += Runs[r].Num();
						FOSMRoadWay Part = Way;   // copy tags/class/flags
						Part.PointsCm = MoveTemp(Runs[r]);
						Part.NodeIds  = MoveTemp(NodeRuns[r]);
						InBounds.Add(MoveTemp(Part));
					}
					DroppedPts += Way.PointsCm.Num() - KeptPts;
				}
				RoadWays = MoveTemp(InBounds);
				UE_LOG(LogOSMRoad, Log,
					TEXT("OSM.ImportRoads: PIPELINE 2 — landscape extent guard: %d ways -> %d in-bounds runs (%d out-of-bounds points cut)."),
					BeforeWays, RoadWays.Num(), DroppedPts);
			}
		}

		int32 SculptWays = 0;
		int64 SculptCells = 0;
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: PIPELINE 2 — sculpting road corridors into the landscape (buffer %.1f m/side)..."),
			CVarRoadDeformBufferM.GetValueOnGameThread());
		if (!SculptRoadCorridors(World, Landscape, RoadWays, SculptWays, SculptCells, OutError))
			return false;
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: PIPELINE 2 — corridor sculpt done (%d corridors, %lld cells)."),
			SculptWays, SculptCells);

		int32 NumRoads = 0;
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: PIPELINE 2 (RoadBuilder engine) for %d road ways..."), RoadWays.Num());
		if (!OSMRoadBuilderBridge::BuildRoads(World, RoadWays, NumRoads, OutError))
			return false;
		OutStats.RoadsCreated = NumRoads;

		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads DONE (RoadBuilder): %d parsed, %d imported, %d skipped, %d RoadBuilder roads."),
			OutStats.WaysTotal, OutStats.WaysImported, OutStats.WaysSkipped, NumRoads);
		return true;
	}

	// 6a3. PIPELINE 1 — landscape-spline source + PCG graph (Yazan-style). Build a
	//      PERSISTENT ALandscapeSplineActor (width in CP->Width, per-class
	//      Segment->LayerName, control points shared at OSM nodes so junctions
	//      connect), deform the terrain via those splines, then (re)generate the
	//      PCG components so PCG_RoadGenerator meshes + dresses them. This is the
	//      path that reproduces the original polished output (grooved sidewalks,
	//      curbs, decals, per-class meshes) using the migrated /Game/OSM graph.
	if (CVarRoadPipeline.GetValueOnGameThread() == 1)
	{
		int32 NumCP = 0, NumSeg = 0;
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: PIPELINE 1 (landscape-spline + PCG graph) for %d road ways..."), RoadWays.Num());

		// Lane-logic pre-pass: optionally merge parallel carriageways into wider
		// multi-lane roads. PCG then renders the merged ways exactly as it would a
		// single wide road (width flows from RoadHalfWidthCm via the summed lanes).
		const TArray<FOSMRoadWay>* WaysForSpline = &RoadWays;
		TArray<FOSMRoadWay> MergedWays;
		if (CVarRoadMergeParallel.GetValueOnGameThread() != 0)
		{
			int32 MergedGroups = 0;
			OSMRoadLaneMerge::FMergeParams MergeParams;
			MergedWays = OSMRoadLaneMerge::MergeParallelCarriageways(RoadWays, MergeParams, MergedGroups);
			WaysForSpline = &MergedWays;
			UE_LOG(LogOSMRoad, Log,
				TEXT("OSM.ImportRoads: lane-merge pre-pass collapsed %d parallel groups (%d ways -> %d)."),
				MergedGroups, RoadWays.Num(), MergedWays.Num());
		}

		AActor* SplineActor = OSMRoadSplineSource::BuildRoadSplineSource(
			World, Landscape, *WaysForSpline, /*bDeformTerrain*/ true, NumCP, NumSeg, OutError);
		if (!SplineActor)
			return false;
		OutStats.RoadsCreated = NumSeg;

		// The PCG_RoadGenerator graph produces curbs, sidewalks, poles, crosswalks
		// and line decals — but no drivable road surface (the original relies on a
		// painted landscape). Build the carriageway from the SAME landscape-spline
		// actor the graph consumes, so the asphalt follows the identical curved
		// splines the curbs ride (not straight, junction-merged chords). PCG still
		// owns the curbs/sidewalks, so they don't double up.
		if (CVarRoadPCGCarriageway.GetValueOnGameThread() != 0)
		{
			OSMRoadMeshBuilder::BuildRoadSurfaceFromSplines(World, SplineActor);
		}

		// Paint the drivable carriageway as deferred asphalt decals (the
		// "texture paint" road surface). Projected full-width along the SAME
		// ways PCG renders, overlapping decals fill bends + intersections with
		// no junction gaps and no surface mesh. Controlled by osm.RoadSurfaceDecal.
		OSMRoadDecals::BuildRoadSurfaceDecals(World, *WaysForSpline);

		// Regenerate every PCG component in this world (matches the spline node's
		// own refresh). The PCG_RoadGenerator graph reads the spline actor via
		// LandscapeSplineToPoints and spawns the road/sidewalk/dressing meshes.
		int32 NumPCG = 0;
		for (TObjectIterator<UPCGComponent> It; It; ++It)
		{
			UPCGComponent* Comp = *It;
			if (!Comp || Comp->GetWorld() != World) continue;
			Comp->Generate(/*bForce*/ true);
			Comp->DirtyGenerated();
			++NumPCG;
		}

		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads DONE (spline+PCG): %d parsed, %d imported, %d skipped, %d control points, %d segments, %d PCG components regenerated.%s"),
			OutStats.WaysTotal, OutStats.WaysImported, OutStats.WaysSkipped, NumCP, NumSeg, NumPCG,
			NumPCG == 0 ? TEXT(" (no PCG component in level — add a PCG actor/volume running PCG_RoadGenerator)") : TEXT(""));
		return true;
	}

	// 6a2. Node-feature props (Phase 5): instanced bollards / toll booths at
	//      tagged road nodes. Runs for every real build mode (before the deform
	//      dispatch); a separate actor, so it never interferes with mesh/terrain.
	if (CVarFeatProps.GetValueOnGameThread() != 0)
		SpawnNodeFeatureProps(World, Ways, NodeFeatures);

	// 6b. DEFORM MODE 1 — native landscape-spline bake. Build a transient spline
	//     network (one control point per OSM node so junctions share a point and
	//     MERGE via the native cosine falloff), bake it into the heightmap with
	//     ULandscapeInfo::ApplySplines, then DISCARD the spline graph (the
	//     persistent graph was the stack-overflow source — baking keeps the
	//     deform without keeping the objects). Replaces BOTH heightmap passes.
	if (CVarRoadDeformMode.GetValueOnGameThread() == 1)
	{
		int32 NumCP = 0, NumSeg = 0;
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: DEFORM MODE 1 (landscape-spline bake) for %d road ways..."), RoadWays.Num());
		if (!OSMRoadSplineDeform::DeformViaLandscapeSplines(World, Landscape, RoadWays, NumCP, NumSeg, OutError))
			return false;
		OutStats.RoadsCreated = NumSeg;

		// Build the visible road + sidewalk mesh on the spline-baked bed. No
		// Pass-2 mesh-conform: the splines already conformed the terrain (a
		// second conform would fight the spline merge).
		OSMRoadMeshBuilder::FRoadMesh RoadMesh;
		OSMRoadMeshBuilder::BuildRoadAndSidewalks(World, RoadWays, &RoadMesh);

		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads DONE (spline-bake): %d parsed, %d imported, %d skipped, %d control points, %d segments."),
			OutStats.WaysTotal, OutStats.WaysImported, OutStats.WaysSkipped, NumCP, NumSeg);
		return true;
	}

	// 6c. DEFORM MODE 2 — legacy pre-carve path (kept for A/B): sculpt the road
	//     corridors into the heightmap FIRST, then build the mesh on that bed,
	//     then conform. This is the pre-carve order the operator moved away from.
	if (CVarRoadDeformMode.GetValueOnGameThread() == 2)
	{
		int32 ModifiedWays = 0;
		int64 ModifiedCells = 0;
		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads: DEFORM MODE 2 (legacy Pass-1 pre-carve) for %d road ways..."), RoadWays.Num());
		if (!SculptRoadCorridors(World, Landscape, RoadWays, ModifiedWays, ModifiedCells, OutError))
			return false;
		OutStats.RoadsCreated = ModifiedWays;

		OSMRoadMeshBuilder::FRoadMesh LegacyMesh;
		OSMRoadMeshBuilder::BuildRoadAndSidewalks(World, RoadWays, &LegacyMesh);

		int64 Pass2Cells = 0;
		if (!DeformLandscapeToRoadMesh(World, Landscape, LegacyMesh.Verts, LegacyMesh.Tris, Pass2Cells, OutError))
			return false;

		UE_LOG(LogOSMRoad, Log,
			TEXT("OSM.ImportRoads DONE (legacy): %d parsed, %d imported, %d skipped, %d corridors, %lld pass-1, %lld pass-2 cells."),
			OutStats.WaysTotal, OutStats.WaysImported, OutStats.WaysSkipped, ModifiedWays, ModifiedCells, Pass2Cells);
		return true;
	}

	// 7. Build the merged road + sidewalk mesh FIRST, directly on the draped
	//    terrain — the RoadBuilder model: lay the continuous road surface
	//    (junctions already merged into ONE polygon by OSMRoadGraph) and DON'T
	//    pre-carve the landscape. Capture the ground-level surface soup so the
	//    deform can conform the terrain to it next.
	OSMRoadMeshBuilder::FRoadMesh RoadMesh;
	OSMRoadMeshBuilder::BuildRoadAndSidewalks(World, RoadWays, &RoadMesh);
	OutStats.RoadsCreated = OutStats.WaysImported;

	// 7b. Dress the roads with the migrated /Game/OSM kit assets: dashed centre
	//     lines, street poles along the edges, guard rails on bridges. Additive
	//     + idempotent (own tagged container), so it can't disturb the surface.
	OSMRoadDressing::BuildRoadDressing(World, RoadWays);

	// 7c. Paint road-marking decals (crosswalks, centre/edge lines) onto the
	//     finished surface — the C++ replacement for Yazan's PCG_CrossWalk /
	//     BP_DecalRoad graph. Resolve each on-road highway=crossing node to a
	//     position + local road direction + half-width so the crosswalk spans
	//     the carriageway. Own tagged container: additive + idempotent.
	{
		TArray<OSMRoadDecals::FCrossing> Crossings;
		if (CVarFeatProps.GetValueOnGameThread() != 0 && NodeFeatures.Num() > 0)
		{
			TSet<int64> Seen;
			for (const FOSMRoadWay& Way : RoadWays)
			{
				const int32 N = Way.PointsCm.Num();
				if (N < 2 || Way.NodeIds.Num() != N) continue;
				if (!OSMRoadClassify::IsDrivable(Way.Class)) continue;
				if (Way.bBridge || Way.bTunnel || Way.Layer != 0) continue;

				const float HalfW = OSMRoadGeom::RoadHalfWidthCm(Way);
				for (int32 i = 0; i < N; ++i)
				{
					const int64 Id = Way.NodeIds[i];
					if (Id < 0 || Seen.Contains(Id)) continue;
					const uint32* F = NodeFeatures.Find(Id);
					if (!F || !(*F & NF_Crossing)) continue;
					if (Way.PointsCm[i].ContainsNaN()) continue;

					// Local road direction from the neighbouring vertices.
					const FVector P0 = Way.PointsCm[FMath::Max(0, i - 1)];
					const FVector P1 = Way.PointsCm[FMath::Min(N - 1, i + 1)];
					FVector2D Dir = FVector2D(P1.X - P0.X, P1.Y - P0.Y).GetSafeNormal();
					if (Dir.IsNearlyZero()) Dir = FVector2D(1.f, 0.f);

					OSMRoadDecals::FCrossing C;
					C.Pos = Way.PointsCm[i];
					C.Dir = Dir;
					C.HalfWidthCm = HalfW;
					Crossings.Add(C);
					Seen.Add(Id);
				}
			}
		}
		OSMRoadDecals::BuildRoadDecals(World, RoadWays, Crossings);
	}

	// 8. THEN deform: conform the landscape to the FINISHED mesh — the single
	//    deform pass, run AFTER the mesh exists (operator's order). The mesh has
	//    one top surface at any (X,Y) so junctions resolve to ONE height (no
	//    competing per-way flattens), and there is NO pre-carve, so the deep
	//    pre-carve trenches are gone. The conform only cuts/fills terrain that is
	//    above/below the actual road surface.
	int64 DeformCells = 0;
	UE_LOG(LogOSMRoad, Log,
		TEXT("OSM.ImportRoads: conforming landscape to finished road mesh (%d verts, %d tris)..."),
		RoadMesh.Verts.Num(), RoadMesh.Tris.Num() / 3);
	if (!DeformLandscapeToRoadMesh(World, Landscape, RoadMesh.Verts, RoadMesh.Tris, DeformCells, OutError))
	{
		return false;
	}

	UE_LOG(LogOSMRoad, Log,
		TEXT("OSM.ImportRoads DONE: %d ways parsed, %d imported, %d skipped, %lld cells conformed (deform AFTER mesh, no pre-carve)."),
		OutStats.WaysTotal, OutStats.WaysImported, OutStats.WaysSkipped, DeformCells);

	return true;
}

// ---------------------------------------------------------------------------
// ImportRoadsFromBBox — HTTP fetch from Overpass then delegate to FromString
// ---------------------------------------------------------------------------
bool OSMOverpassRoadImport::ImportRoadsFromBBox(
	UWorld*                 World,
	double MinLon, double MinLat,
	double MaxLon, double MaxLat,
	FOSMRoadImportStats&    OutStats,
	FString&                OutError)
{
	// Overpass QL: all highway ways with inline geometry, THEN the tagged nodes
	// belonging to those ways (crossings, humps, turning circles, bollards, toll,
	// level crossings, …) output with tags. `out body geom` gives way vertices
	// (geom) AND the per-way node-id list (body) AND tags — the node ids are what
	// the junction merge + block-sidewalk builders key off. NOTE: `out tags geom`
	// drops the node-id list, which silently breaks junctions and sidewalks.
	// `node(w); out tags` gives the on-road feature nodes keyed by the same ids
	// so we can match them to way vertices during parse.
	const FString Query = FString::Printf(
		TEXT("[out:json][timeout:120];")
		TEXT("way[highway](%.7f,%.7f,%.7f,%.7f)->.roads;")
		TEXT(".roads out body geom;")
		TEXT("node(w.roads);")
		TEXT("out tags;"),
		MinLat, MinLon, MaxLat, MaxLon);

	const FString Url = TEXT("https://overpass-api.de/api/interpreter?data=")
		+ FGenericPlatformHttp::UrlEncode(Query);

	UE_LOG(LogOSMRoad, Log, TEXT("OSM.ImportRoads: fetching bbox (%.5f,%.5f)-(%.5f,%.5f)..."),
		MinLon, MinLat, MaxLon, MaxLat);

	// Synchronous HTTP request (editor-only; not for runtime)
	bool bRequestDone  = false;
	bool bRequestOk    = false;
	FString ResponseBody;

	TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Req = FHttpModule::Get().CreateRequest();
	Req->SetURL(Url);
	Req->SetVerb(TEXT("GET"));
	Req->SetHeader(TEXT("User-Agent"), TEXT("OSMRoadCore/1.0 (Unreal Engine)"));
	Req->SetHeader(TEXT("Accept"),     TEXT("application/json"));

	Req->OnProcessRequestComplete().BindLambda(
		[&](FHttpRequestPtr, FHttpResponsePtr Resp, bool bOk)
		{
			bRequestOk   = bOk && Resp.IsValid() && Resp->GetResponseCode() == 200;
			ResponseBody = bRequestOk ? Resp->GetContentAsString() : FString();
			bRequestDone = true;
		});

	Req->ProcessRequest();

	// Tick the HTTP module until the request completes (editor-only blocking poll)
	while (!bRequestDone)
	{
		FHttpModule::Get().GetHttpManager().Tick(0.1f);
		FPlatformProcess::Sleep(0.1f);
	}

	if (!bRequestOk || ResponseBody.IsEmpty())
	{
		OutError = TEXT("Overpass HTTP request failed. Check internet connection / Overpass status.");
		return false;
	}

	// Cache for offline re-use / debugging
	FFileHelper::SaveStringToFile(ResponseBody,
		*(FPaths::ProjectSavedDir() / TEXT("osm_import_roads.json")));

	return ImportRoadsFromString(World, ResponseBody, OutStats, OutError,
		MinLon, MinLat, MaxLon, MaxLat);
}

#endif // WITH_EDITOR
