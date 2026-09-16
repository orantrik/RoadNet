// RoadNetwork.cpp — orchestration + early rebuild stages (§10.18).
#include "RoadNetwork.h"
#include "RoadNetMath.h"
#include "RoadNetJunctions.h"
#include "RoadNetStandards.h"
#include "RoadNetSurface.h"
#include "RoadNetMesh.h"
#include "RoadNetCurbs.h"
#include "RoadNetJunctionMarks.h"
#include "RoadNetZones.h"
#include "RoadNetTileActor.h"
#include "RoadNetTiles.h"
#include "EngineUtils.h"        // TActorIterator (tile registry rebuild)
#include "Polygon2.h"
#include "Algo/Reverse.h"
#include "RoadNetMarkings.h"
#include "RoadNetPerimeters.h"
#include "RoadNetLanes.h"
#include "RoadNetLog.h"
#include "Curve/PolygonOffsetUtils.h"
#include "Curve/PolygonIntersectionUtils.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMeshActor.h"
#include "Components/DynamicMeshComponent.h"
#include "Components/SplineComponent.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "Materials/MaterialInterface.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"

// Safety valve for the windowed (per-tile scoped) rebuild. When 0 every edit
// falls back to a full-network rebuild — the pre-tiling behaviour — so any
// suspected windowing artifact can be ruled out live without a recompile.
static TAutoConsoleVariable<int32> CVarRoadNetWindowedRebuild(
	TEXT("roadnet.WindowedRebuild"), 1,
	TEXT("1 = scope single-road edits to the spatial tiles they touch (fast). 0 = always full rebuild (safe fallback)."),
	ECVF_Default);

// Junction approach conditioning: thin the point cluster an import leaves around
// every intersection, then straighten what remains onto the approach tangent. On
// by default because the clustering is a defect of the import, not authoring —
// turn it off to compare against raw OSM geometry.
static TAutoConsoleVariable<int32> CVarRoadNetJunctionConditioning(
	TEXT("roadnet.JunctionConditioning"), 1,
	TEXT("1 = decluster and straighten road points near junctions during Smooth Roads and OSM import. 0 = leave imported points as-is."),
	ECVF_Default);

// § keep spline edits — user edits of the plan splines (centrelines and
// sidewalk rings) survive Build Street: harvested into the road polylines +
// persistent pins before every rebuild regenerates the splines.
static TAutoConsoleVariable<int32> CVarRoadNetKeepSplineEdits(
	TEXT("roadnet.KeepSplineEdits"),
	1,
	TEXT("1 = edits made to the plan splines (road centrelines, sidewalk rings) are harvested before a rebuild and re-applied, instead of being regenerated away. Default 1."),
	ECVF_Default);

// § ease heights — adjacent knots closer than the radius may not disagree in
// Z by more than the step. Two rulers, exposed on the OSM Roads panel.
// NOT static: RoadNetParcelAccess.cpp reads the same rulers for its field pass.
TAutoConsoleVariable<int32> CVarRoadNetEaseHeights(
	TEXT("roadnet.EaseHeights"),
	1,
	TEXT("1 = ease pass on the latent splines: knots closer than roadnet.EaseRadiusCm may not differ in Z by more than roadnet.EaseMaxStepCm. Default 1."),
	ECVF_Default);
TAutoConsoleVariable<float> CVarRoadNetEaseMaxStepCm(
	TEXT("roadnet.EaseMaxStepCm"),
	25.0f,
	TEXT("Ease heights: max Z difference (cm) between two knots inside the ease radius. Default 25."),
	ECVF_Default);
TAutoConsoleVariable<float> CVarRoadNetEaseRadiusCm(
	TEXT("roadnet.EaseRadiusCm"),
	600.0f,
	TEXT("Ease heights: only knots closer than this (cm) are eased against each other. Default 600."),
	ECVF_Default);

// § ease heights along ONE polyline: consecutive knots closer than RadiusCm
// (2D) are clamped to at most MaxStepCm of Z difference. Forward + backward
// sweep, two rounds, so a single spiked knot is pulled down from both sides
// while a long segment (> radius) may still carry a legitimate grade change.
static int32 EasePolylineZ(TArray<FVector>& P, bool bClosed, double MaxStepCm, double RadiusCm)
{
	if (P.Num() < 2 || MaxStepCm <= 0.0) { return 0; }
	int32 Moves = 0;
	auto Relax = [&](int32 A, int32 B)
	{
		const double D = FVector::Dist2D(P[A], P[B]);
		if (D > RadiusCm) { return; }
		const double dZ = P[B].Z - P[A].Z;
		if (FMath::Abs(dZ) <= MaxStepCm) { return; }
		P[B].Z = P[A].Z + FMath::Clamp(dZ, -MaxStepCm, MaxStepCm);
		++Moves;
	};
	const int32 N = P.Num();
	for (int32 Round = 0; Round < 2; ++Round)
	{
		for (int32 i = 1; i < N; ++i)      { Relax(i - 1, i); }
		if (bClosed)                        { Relax(N - 1, 0); }
		for (int32 i = N - 2; i >= 0; --i) { Relax(i + 1, i); }
		if (bClosed)                        { Relax(0, N - 1); }
	}
	return Moves;
}

// Pipeline tunables (§2.6). Kept local until a settings object is added.
namespace
{
	// §2.6 PolylineDensity is now a per-network property (URoadNetwork::PolylineDensityCm).
	constexpr double kAdaptiveTurnRad   = 0.0873;  // ~5° knot-preserve threshold
	constexpr double kRoadZLiftCm       = 12.0;    // lift above landscape (anti z-fight)
	constexpr double kJunctionMatchCm   = 600.0;   // persistent junction/roundabout match radius

	// How far painted markings sit above the asphalt. The paint and the slab are
	// draped off the same centrelines but triangulate differently, so they cannot
	// share a Z — but 4 cm (the old value) is a visible step at human height, and
	// the paint casting a shadow onto the road made it read as a floating sheet.
	// 1 cm reads flat and still clears the slab on a crowned or graded span.
	// ponytail: a constant, not a per-road figure. A road with an extreme crown
	// could still z-fight; the fix then is to drape the paint off the slab's own
	// surface rather than off the centreline, which needs the slab mesh first.
	constexpr double kMarkingLiftCm     = 1.0;

	// Typed-lane tint (bike green / parking amber). Must stay strictly BELOW the
	// paint: a parking bay's stall dividers live in the white bank, so an overlay
	// drawn over them turns the bay into a blank slab.
	constexpr double kLaneOverlayLiftCm = 0.5;

	// Paint and tint are decoration on the asphalt, not occluders. A sheet lifted
	// clear of the road that also casts its own shadow onto the road reads as
	// floating however small the gap — which is very likely what the "markings
	// float a few cm above the road" complaint actually is.
	bool LayerIsPaint(FName LayerName)
	{
		static const FName kWhite(TEXT("MarkingsWhite")), kYellow(TEXT("MarkingsYellow"));
		static const FName kBike(TEXT("LanesBike")),      kPark(TEXT("LanesParking"));
		return LayerName == kWhite || LayerName == kYellow
		    || LayerName == kBike  || LayerName == kPark;
	}

	constexpr double kMaxZGapCm         = 350.0;   // §10.12 at-grade crossing threshold
	constexpr double kEndpointWeldCm    = 400.0;   // §10.7 spatial endpoint weld radius

	// Trim Dist (cm, XY arc length) off one end of a polyline, moving the end
	// vertex inward and dropping any fully-consumed vertices. Used to pull a
	// median back from a junction so its rounded nose sits short of the crossing.
	void TrimPolylineEnd(TArray<FVector>& Pts, bool bFromStart, double Dist)
	{
		if (Pts.Num() < 2 || Dist <= 0.0) { return; }
		if (bFromStart)
		{
			double Acc = 0.0; int32 i = 0;
			for (; i + 1 < Pts.Num(); ++i)
			{
				const double Seg = FVector::Dist2D(Pts[i], Pts[i + 1]);
				if (Acc + Seg >= Dist)
				{
					const double T = (Dist - Acc) / FMath::Max(Seg, 1.0e-3);
					Pts[i] = FMath::Lerp(Pts[i], Pts[i + 1], T);
					break;
				}
				Acc += Seg;
			}
			if (i > 0) { Pts.RemoveAt(0, i); }
		}
		else
		{
			double Acc = 0.0; int32 i = Pts.Num() - 1;
			for (; i - 1 >= 0; --i)
			{
				const double Seg = FVector::Dist2D(Pts[i], Pts[i - 1]);
				if (Acc + Seg >= Dist)
				{
					const double T = (Dist - Acc) / FMath::Max(Seg, 1.0e-3);
					Pts[i] = FMath::Lerp(Pts[i], Pts[i - 1], T);
					break;
				}
				Acc += Seg;
			}
			if (i < Pts.Num() - 1) { Pts.RemoveAt(i + 1, Pts.Num() - 1 - i); }
		}
	}
}

int32 URoadNetwork::AddRoad(const FRoadDef& Road)
{
	FRoadDef Copy = Road;
	if (!Copy.Id.IsValid()) { Copy.Id = FGuid::NewGuid(); }
	return Roads.Add(MoveTemp(Copy));
}

void URoadNetwork::ResetRoads()
{
	Roads.Reset();
	// The user's plan-spline edits describe roads that no longer exist.
	PlanEditPins.Reset();
	PlanSplineBaselines.Reset();
}

int32 URoadNetwork::RemoveRoadsBySource(ERoadNetSource Source)
{
	return Roads.RemoveAll([Source](const FRoadDef& R) { return R.Source == Source; });
}

int32 URoadNetwork::FindRoadById(const FGuid& Id) const
{
	if (!Id.IsValid()) { return INDEX_NONE; }
	for (int32 i = 0; i < Roads.Num(); ++i) { if (Roads[i].Id == Id) { return i; } }
	return INDEX_NONE;
}

const FGuid& URoadNetwork::EnsureNetworkId()
{
	if (!NetworkId.IsValid()) { NetworkId = FGuid::NewGuid(); }
	return NetworkId;
}

void URoadNetwork::EnsureTileRegistry()
{
	if (bTileRegistryLoaded) { return; }
	bTileRegistryLoaded = true;

	TileActors.Reset();
	UWorld* World = WorldPtr.Get();
	if (!World) { return; }

	const FGuid& MyId = EnsureNetworkId();
	for (TActorIterator<ARoadNetTileActor> It(World); It; ++It)
	{
		ARoadNetTileActor* Tile = *It;
		if (!Tile || Tile->OwningNetworkId != MyId) { continue; }
		TileActors.Add(Tile->TileCoord, Tile);
	}
}

ARoadNetTileActor* URoadNetwork::GetOrCreateTile(const FIntPoint& Coord)
{
	EnsureTileRegistry();

	if (TWeakObjectPtr<ARoadNetTileActor>* Found = TileActors.Find(Coord))
	{
		if (ARoadNetTileActor* Existing = Found->Get()) { return Existing; }
		TileActors.Remove(Coord);
	}

	UWorld* World = WorldPtr.Get();
	if (!World) { return nullptr; }

	// Spawn at the origin with identity transform: committed geometry is stored
	// in WORLD coordinates (matching the former network-wide actors), so the
	// component-to-world transform must be identity. The actor's render/streaming
	// bounds still resolve to the cell because they derive from the (world-space)
	// mesh, not the pivot. (Local-space pivot handled in the streaming phase.)
	FActorSpawnParameters Params;
	Params.ObjectFlags |= RF_Transient; // made persistent in the streaming phase
	ARoadNetTileActor* Tile = World->SpawnActor<ARoadNetTileActor>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
	if (!Tile) { return nullptr; }
	Tile->Configure(Coord, TileSizeCm, EnsureNetworkId());
#if WITH_EDITOR
	Tile->SetActorLabel(Coord.Y == kJunKind
		? FString::Printf(TEXT("RoadNet_Jct_%d"), Coord.X)
		: FString::Printf(TEXT("RoadNet_Seg_%d"), Coord.X));
#endif
	TileActors.Add(Coord, Tile);
	return Tile;
}

FIntPoint URoadNetwork::SegTileKey(const FGuid& RoadId, int32 Arm)
{
	TPair<FGuid, int32> K(RoadId, FMath::Max(0, Arm));
	// Divided-road pairing: redirect a member carriageway to its pair's canonical
	// key so both carriageways (and their median) land on ONE tile.
	if (const TPair<FGuid, int32>* Canon = SegAlias.Find(K)) { K = *Canon; }
	if (const int32* Found = SegKeyOf.Find(K)) { return FIntPoint(*Found, kSegKind); }
	const int32 Id = NextSegId++;
	SegKeyOf.Add(K, Id);
	return FIntPoint(Id, kSegKind);
}

FIntPoint URoadNetwork::JunTileKey(const FVector2D& CentreCm)
{
	// ponytail: quantise the junction centroid to an 8 m grid with a 1-cell
	// neighbour search, so a centroid that drifts across a cell border between
	// windowed rebuilds keeps its id. Ceiling: two junctions closer than ~8 m
	// share a tile (they are usually one merged clip poly anyway). Upgrade path =
	// match by nearest stored centre within a tolerance.
	constexpr double Q = 800.0;
	const FIntPoint Cell(FMath::RoundToInt(CentreCm.X / Q), FMath::RoundToInt(CentreCm.Y / Q));
	for (int32 dy = -1; dy <= 1; ++dy)
	{
		for (int32 dx = -1; dx <= 1; ++dx)
		{
			if (const int32* Found = JunKeyOf.Find(FIntPoint(Cell.X + dx, Cell.Y + dy)))
			{
				return FIntPoint(*Found, kJunKind);
			}
		}
	}
	const int32 Id = NextJunId++;
	JunKeyOf.Add(Cell, Id);
	return FIntPoint(Id, kJunKind);
}

FIntPoint URoadNetwork::TopoKeyOf(const FVector& WorldPos, const FRoadNetRebuildContext& Ctx)
{
	using namespace UE::Geometry;
	const FVector2D P2(WorldPos.X, WorldPos.Y);

	// Inside a junction CARVE region? -> that junction tile (bbox pre-filter).
	for (const FRoadNetRebuildContext::FTopoJunctionRegion& JR : Ctx.TopoJunctions)
	{
		if (!JR.Box.bIsValid || !JR.Box.IsInside(P2)) { continue; }
		if (!Ctx.ZoneJunctionCarve.IsValidIndex(JR.Zone) ||
			!Ctx.ZoneJunctionCarve[JR.Zone].IsValidIndex(JR.Index)) { continue; }
		if (Ctx.ZoneJunctionCarve[JR.Zone][JR.Index].Contains(FVector2d(P2.X, P2.Y)))
		{
			return JR.Key;
		}
	}

	// else nearest road centreline -> its segment tile. Use the coarse sample
	// grid (3x3 neighbourhood) as a broadphase; fall back to a full scan only if
	// the neighbourhood is empty.
	constexpr double CellCm = 2000.0;
	const FIntPoint Home(FMath::FloorToInt(P2.X / CellCm), FMath::FloorToInt(P2.Y / CellCm));
	int32 Best = INDEX_NONE;
	int32 BestSeg = INDEX_NONE;
	double BestD = TNumericLimits<double>::Max();
	auto Consider = [&](int32 r)
	{
		const FRoadCurves* C = Ctx.Curves.Find(r);
		if (!C || C->Sampled.Num() < 2) { return; }
		const RoadNetMath::FProjectResult PR = RoadNetMath::ProjectToPolyline(C->Sampled, P2);
		if (PR.Distance < BestD) { BestD = PR.Distance; Best = r; BestSeg = PR.Segment; }
	};
	TSet<int32> Seen;
	for (int32 dy = -1; dy <= 1; ++dy)
	{
		for (int32 dx = -1; dx <= 1; ++dx)
		{
			if (const TArray<int32>* Cands = Ctx.RoadSampleGrid.Find(FIntPoint(Home.X + dx, Home.Y + dy)))
			{
				for (int32 r : *Cands) { if (!Seen.Contains(r)) { Seen.Add(r); Consider(r); } }
			}
		}
	}
	if (Best == INDEX_NONE)
	{
		for (const TPair<int32, FRoadCurves>& KV : Ctx.Curves) { Consider(KV.Key); }
	}
	if (Best != INDEX_NONE && Roads.IsValidIndex(Best) && Roads[Best].Id.IsValid())
	{
		// Which ARM of the road: the segment the point projects onto tells us its
		// inter-junction stretch, so a road crossing a junction lands on a distinct
		// tile on each side. If the projected segment is inside a junction (arm
		// -1), borrow the nearest surrounding arm so the point still lands on a
		// real segment tile.
		int32 Arm = 0;
		if (const TArray<int32>* Arms = Ctx.RoadSampleArm.Find(Best))
		{
			if (Arms->Num() > 0)
			{
				const int32 Si = FMath::Clamp(BestSeg, 0, Arms->Num() - 1);
				Arm = (*Arms)[Si];
				if (Arm < 0)
				{
					for (int32 d = 1; d < Arms->Num(); ++d)
					{
						if (Arms->IsValidIndex(Si - d) && (*Arms)[Si - d] >= 0) { Arm = (*Arms)[Si - d]; break; }
						if (Arms->IsValidIndex(Si + d) && (*Arms)[Si + d] >= 0) { Arm = (*Arms)[Si + d]; break; }
					}
					if (Arm < 0) { Arm = 0; }
				}
			}
		}
		return SegTileKey(Roads[Best].Id, Arm);
	}
	return FIntPoint(INDEX_NONE, kSegKind);
}

void URoadNetwork::BuildTopoAccel(FRoadNetRebuildContext& Ctx)
{
	using namespace UE::Geometry;
	Ctx.TopoJunctions.Reset();
	Ctx.RoadSampleGrid.Reset();

	// Grow each junction clip outward by the zone's sidewalk width (+margin) into
	// the CARVE region. The raw clip only covers the carriageway overlap, so a
	// road's sidewalk (which sits OUTSIDE the carriageway) never crosses it and
	// rides through the junction uncut — that's why the tile spanned the junction.
	// The carve reaches across the sidewalk band so surface AND sidewalk both
	// break at the junction. Tiles are cut/keyed against the carve below.
	Ctx.ZoneJunctionCarve.Reset();
	Ctx.ZoneJunctionCarve.SetNum(Ctx.ZoneJunctionClip.Num());

	// Junction DISC points per zone (centre + radius). The carriageway-overlap
	// clip only bites the EDGE of a through road at a T, so the through road's
	// centreline never enters it and rides across the junction uncut. A disc at
	// the junction POINT (which lies on the through-road centreline) spans the
	// road's width, so EVERY road through the junction gets carved → cut into two
	// segments. Same junctions the surface fills: same-grade crossings + N-way
	// (>=3 arm) joints. Grade-separated crossings (overpasses) are skipped so a
	// bridge is not sliced. 2-arm joints (continuations/seams) are NOT cut.
	const int32 NZ = Ctx.ZoneJunctionClip.Num();
	auto HalfW = [this](int32 r) -> double
	{
		return Roads.IsValidIndex(r) ? FMath::Max(50.0, (double)Roads[r].Lanes.HalfWidthCm()) : 50.0;
	};
	TMap<int32, int32> RoadZone;
	TArray<double> ZoneMaxSw; ZoneMaxSw.SetNumZeroed(NZ);
	for (int32 z = 0; z < Ctx.Zones.Num(); ++z)
	{
		for (int32 r : Ctx.Zones[z])
		{
			RoadZone.FindOrAdd(r) = z;
			if (z < NZ && Roads.IsValidIndex(r)) { ZoneMaxSw[z] = FMath::Max(ZoneMaxSw[z], (double)Roads[r].Lanes.SidewalkWidth); }
		}
	}
	TArray<TArray<TPair<FVector2D, double>>> ZonePts; ZonePts.SetNum(NZ);
	auto AddPt = [&ZonePts](int32 z, const FVector2D& P, double R)
	{
		if (!ZonePts.IsValidIndex(z)) { return; }
		for (TPair<FVector2D, double>& E : ZonePts[z])
		{
			if (FVector2D::DistSquared(E.Key, P) < FMath::Square(0.5 * FMath::Max(E.Value, R)))
			{
				E.Value = FMath::Max(E.Value, R); return;
			}
		}
		ZonePts[z].Emplace(P, R);
	};
	for (const FRoadNetCrossing& X : Ctx.Crossings)
	{
		const int32* za = RoadZone.Find(X.RoadA);
		if (!za) { continue; }
		if (RoadZone.FindRef(X.RoadB, -1) != *za) { continue; }
		if (FMath::Abs(X.Za - X.Zb) > 300.0) { continue; }   // overpass: not a junction
		const double SwA = ZoneMaxSw.IsValidIndex(*za) ? ZoneMaxSw[*za] : 0.0;
		AddPt(*za, X.Point, FMath::Max(HalfW(X.RoadA), HalfW(X.RoadB)) + SwA + 150.0);
	}
	for (const FRoadNetJoint& J : Ctx.Joints)
	{
		if (J.Arms.Num() < 3) { continue; }              // N-way only; 2-arm = continuation
		int32 z = INDEX_NONE; double MaxHalf = 0.0;
		for (const FRoadNetJointArm& A : J.Arms)
		{
			const int32* zz = RoadZone.Find(A.Road);
			if (!zz) { continue; }
			if (z == INDEX_NONE) { z = *zz; }
			if (*zz == z) { MaxHalf = FMath::Max(MaxHalf, HalfW(A.Road)); }
		}
		if (z == INDEX_NONE) { continue; }
		AddPt(z, J.Location, MaxHalf + (ZoneMaxSw.IsValidIndex(z) ? ZoneMaxSw[z] : 0.0) + 150.0);
	}

	for (int32 z = 0; z < NZ; ++z)
	{
		// (1) dilate the carriageway-overlap clip out across the sidewalk band.
		TArray<FGeneralPolygon2d> Carve;
		if (Ctx.ZoneJunctionClip[z].Num() > 0)
		{
			const double Grow = ZoneMaxSw[z] + 150.0;
			TArray<FGeneralPolygon2d> Dil;
			if (Grow > 1.0 && PolygonsOffset(Grow, Ctx.ZoneJunctionClip[z], Dil, /*bCopyInputOnFailure*/true,
					/*MiterLimit*/2.0, EPolygonOffsetJoinType::Round, EPolygonOffsetEndType::Polygon,
					/*MaxStepsPerRadian*/16.0, /*DefaultStepsPerRadianScale*/1.0e-3) && Dil.Num() > 0)
			{
				Carve = MoveTemp(Dil);
			}
			else { Carve = Ctx.ZoneJunctionClip[z]; }
		}

		// (2) add the junction discs so through-roads break too, unioned into the
		// clip so each junction stays ONE region (one junction tile).
		if (ZonePts.IsValidIndex(z) && ZonePts[z].Num() > 0)
		{
			TArray<FGeneralPolygon2d> Discs;
			for (const TPair<FVector2D, double>& E : ZonePts[z])
			{
				FGeneralPolygon2d D;
				RoadNetSurface::MakeDisc(E.Key, E.Value, /*Segments*/32, D);
				if (D.GetOuter().VertexCount() >= 3) { Discs.Add(MoveTemp(D)); }
			}
			if (Discs.Num() > 0)
			{
				TArray<FGeneralPolygon2d> Both = Carve; Both.Append(Discs);
				TArray<FGeneralPolygon2d> U;
				if (PolygonsUnion(Both, U, /*bCopyInputOnFailure*/true) && U.Num() > 0) { Carve = MoveTemp(U); }
				else { Carve.Append(Discs); }
			}
		}

		Ctx.ZoneJunctionCarve[z] = MoveTemp(Carve);
	}

	// Flat junction-region list (one entry per CARVE poly) with bbox + stable key.
	for (int32 z = 0; z < Ctx.ZoneJunctionCarve.Num(); ++z)
	{
		for (int32 i = 0; i < Ctx.ZoneJunctionCarve[z].Num(); ++i)
		{
			const TArray<FVector2d>& OV = Ctx.ZoneJunctionCarve[z][i].GetOuter().GetVertices();
			if (OV.Num() < 3) { continue; }
			FVector2D C(0, 0);
			FBox2D Box(ForceInit);
			for (const FVector2d& V : OV) { C += FVector2D(V.X, V.Y); Box += FVector2D(V.X, V.Y); }
			C /= (double)OV.Num();
			FRoadNetRebuildContext::FTopoJunctionRegion JR;
			JR.Box = Box;
			JR.Key = JunTileKey(C);
			JR.Zone = z;
			JR.Index = i;
			Ctx.TopoJunctions.Add(JR);
		}
	}

	// Coarse sample grid: mark every cell a road's sampled centreline passes
	// through, so point->segment lookups only test nearby roads.
	constexpr double CellCm = 2000.0;
	for (const TPair<int32, FRoadCurves>& KV : Ctx.Curves)
	{
		const TArray<FVector>& S = KV.Value.Sampled;
		for (const FVector& P : S)
		{
			const FIntPoint Cell(FMath::FloorToInt(P.X / CellCm), FMath::FloorToInt(P.Y / CellCm));
			Ctx.RoadSampleGrid.FindOrAdd(Cell).AddUnique(KV.Key);
		}
	}

	// Per-road arm index of each centreline sample: walk the sampled polyline and
	// bump the arm counter every time it LEAVES a junction clip, so each stretch
	// between two junctions is a distinct arm (samples inside a junction get -1).
	// This is the "cut at the junction" — a road passing through a junction is
	// two arms → two segment tiles.
	Ctx.RoadSampleArm.Reset();
	auto InAnyJct = [&Ctx](const FVector& P) -> bool
	{
		const FVector2D P2(P.X, P.Y);
		for (const FRoadNetRebuildContext::FTopoJunctionRegion& JR : Ctx.TopoJunctions)
		{
			if (!JR.Box.bIsValid || !JR.Box.IsInside(P2)) { continue; }
			if (!Ctx.ZoneJunctionCarve.IsValidIndex(JR.Zone) ||
				!Ctx.ZoneJunctionCarve[JR.Zone].IsValidIndex(JR.Index)) { continue; }
			if (Ctx.ZoneJunctionCarve[JR.Zone][JR.Index].Contains(FVector2d(P2.X, P2.Y))) { return true; }
		}
		return false;
	};
	for (const TPair<int32, FRoadCurves>& KV : Ctx.Curves)
	{
		const TArray<FVector>& S = KV.Value.Sampled;
		TArray<int32> Arm;
		Arm.SetNumUninitialized(S.Num());
		int32 Cur = -1;
		bool bPrevInside = true;   // so the first outside sample opens arm 0
		for (int32 i = 0; i < S.Num(); ++i)
		{
			if (InAnyJct(S[i])) { Arm[i] = -1; bPrevInside = true; }
			else { if (bPrevInside) { ++Cur; } Arm[i] = Cur; bPrevInside = false; }
		}
		Ctx.RoadSampleArm.Add(KV.Key, MoveTemp(Arm));
	}

	// Contiguous arm RUNS per road (index == arm value): a maximal span of samples
	// sharing one arm>=0. These are the inter-junction stretches each segment tile
	// is built from (cross-section + splines) and the units divided pairing works
	// on. Arm values are 0,1,2,... in order, so the run list is naturally indexed
	// by arm value.
	Ctx.RoadArmRuns.Reset();
	for (const TPair<int32, TArray<int32>>& KV : Ctx.RoadSampleArm)
	{
		const TArray<int32>& Arm = KV.Value;
		TArray<TPair<int32, int32>> Runs;
		int32 i = 0;
		while (i < Arm.Num())
		{
			if (Arm[i] < 0) { ++i; continue; }
			const int32 Av = Arm[i];
			int32 j = i;
			while (j + 1 < Arm.Num() && Arm[j + 1] == Av) { ++j; }
			// index in Runs must equal the arm value; arms are contiguous so this
			// holds, but guard against any gap by padding.
			while (Runs.Num() < Av) { Runs.Add(TPair<int32, int32>(-1, -1)); }
			Runs.Add(TPair<int32, int32>(i, j));
			i = j + 1;
		}
		Ctx.RoadArmRuns.Add(KV.Key, MoveTemp(Runs));
	}
}

void URoadNetwork::BuildDividedPairs(FRoadNetRebuildContext& Ctx)
{
	SegAlias.Reset();
	if (!bPairDividedRoads) { return; }

	// One record per one-way arm: mid-point, unit travel direction and the road's
	// half carriageway width, so we can test "opposite, parallel, side-by-side".
	struct FArmRec
	{
		int32 Road = INDEX_NONE;
		int32 Arm = 0;
		FGuid Id;
		FVector2D Mid = FVector2D::ZeroVector;
		FVector2D Dir = FVector2D(1, 0);   // unit start->end of the run
		double Half = 0.0;
		bool bTaken = false;
	};

	TArray<FArmRec> Arms;
	for (const TPair<int32, TArray<TPair<int32, int32>>>& KV : Ctx.RoadArmRuns)
	{
		const int32 r = KV.Key;
		if (!Roads.IsValidIndex(r) || !Roads[r].Id.IsValid()) { continue; }
		if (!Roads[r].Lanes.bOneway) { continue; }   // divided carriageways are one-way
		const FRoadCurves* C = Ctx.Curves.Find(r);
		if (!C) { continue; }
		const double Half = FMath::Max(50.0, (double)Roads[r].Lanes.HalfWidthCm());
		for (int32 av = 0; av < KV.Value.Num(); ++av)
		{
			const TPair<int32, int32>& Run = KV.Value[av];
			if (Run.Key < 0 || Run.Value <= Run.Key || !C->Sampled.IsValidIndex(Run.Value)) { continue; }
			const FVector A = C->Sampled[Run.Key];
			const FVector B = C->Sampled[Run.Value];
			FVector2D Dir(B.X - A.X, B.Y - A.Y);
			if (!Dir.Normalize()) { continue; }
			FArmRec Rec;
			Rec.Road = r; Rec.Arm = av; Rec.Id = Roads[r].Id;
			Rec.Mid = FVector2D(0.5 * (A.X + B.X), 0.5 * (A.Y + B.Y));
			Rec.Dir = Dir; Rec.Half = Half;
			Arms.Add(Rec);
		}
	}

	int32 Pairs = 0;
	const double MaxGap = FMath::Max(200.0, DividedRoadMaxGapCm);
	for (int32 a = 0; a < Arms.Num(); ++a)
	{
		if (Arms[a].bTaken) { continue; }
		int32 Best = INDEX_NONE;
		double BestScore = TNumericLimits<double>::Max();
		for (int32 b = a + 1; b < Arms.Num(); ++b)
		{
			if (Arms[b].bTaken || Arms[b].Road == Arms[a].Road) { continue; }
			// Opposite travel direction (divided carriageways run against each other).
			if (FVector2D::DotProduct(Arms[a].Dir, Arms[b].Dir) > -0.6) { continue; }
			// Side-by-side: lateral gap in (touching, MaxGap]; the midpoints must sit
			// roughly abeam (small longitudinal offset) so we don't pair end-to-end.
			const FVector2D D = Arms[b].Mid - Arms[a].Mid;
			const FVector2D Perp(-Arms[a].Dir.Y, Arms[a].Dir.X);
			const double Lat = FMath::Abs(FVector2D::DotProduct(D, Perp));
			const double Lon = FMath::Abs(FVector2D::DotProduct(D, Arms[a].Dir));
			const double MinLat = 0.5 * (Arms[a].Half + Arms[b].Half);
			if (Lat < MinLat || Lat > MaxGap) { continue; }
			if (Lon > MaxGap) { continue; }     // must be abeam, not sequential
			const double Score = Lat + 0.25 * Lon;
			if (Score < BestScore) { BestScore = Score; Best = b; }
		}
		if (Best == INDEX_NONE) { continue; }

		// Canonicalise to the smaller road index (deterministic within a rebuild;
		// the two are never the same road) so both members resolve identically
		// regardless of iteration order.
		FArmRec& A = Arms[a];
		FArmRec& Bx = Arms[Best];
		const TPair<FGuid, int32> KA(A.Id, A.Arm);
		const TPair<FGuid, int32> KB(Bx.Id, Bx.Arm);
		const TPair<FGuid, int32> Canon = (A.Road < Bx.Road) ? KA : KB;
		SegAlias.Add(KA, Canon);
		SegAlias.Add(KB, Canon);
		A.bTaken = Bx.bTaken = true;
		++Pairs;
		UE_LOG(LogRoadNet, Log,
			TEXT("[RoadNet][Divided] paired road %d arm %d <-> road %d arm %d (score=%.0f cm)."),
			A.Road, A.Arm, Bx.Road, Bx.Arm, BestScore);
	}
	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet][Divided] %d carriageway pair(s) merged into shared tiles (of %d one-way arms)."),
		Pairs, Arms.Num());
}

// ---------------------------------------------------------------------------
// § Street plan API — capture the sidewalk outer edges WITH Z while both the
// band polygons and the reconciled curves are alive. Before this existed the
// edge died inside the rebuild context and every downstream consumer (parcel
// seating, fences) had to re-guess it from the landscape — which is exactly
// how parcels and roads ended up disagreeing about shared ground.
// ---------------------------------------------------------------------------
void URoadNetwork::CaptureStreetPlan(FRoadNetRebuildContext& Ctx)
{
	PlanSidewalkEdges.Reset();

	for (int32 z = 0; z < Ctx.ZoneSidewalkPolys.Num(); ++z)
	{
		if (Ctx.ZoneSidewalkPolys[z].Num() == 0) { continue; }

		// Same height source the sidewalk mesh itself will sample, so the plan
		// edge and the committed pavement can never disagree.
		TArray<const TArray<FVector>*> CenterLines;
		if (Ctx.Zones.IsValidIndex(z))
		{
			for (const int32 r : Ctx.Zones[z])
			{
				if (const FRoadCurves* C = Ctx.Curves.Find(r))
				{
					if (C->Sampled.Num() >= 2) { CenterLines.Add(&C->Sampled); }
				}
			}
		}
		if (CenterLines.Num() == 0) { continue; }

		RoadNetMesh::FCenterlineHeightField Field;
		Field.Build(CenterLines);
		const double FallbackZ = Field.FirstZ();

		auto CaptureRing = [&](const UE::Geometry::FPolygon2d& Ring, bool bHole)
		{
			if (Ring.VertexCount() < 3) { return; }
			FRoadNetPlanEdge& Edge = PlanSidewalkEdges.AddDefaulted_GetRef();
			Edge.Zone  = z;
			Edge.bHole = bHole;
			Edge.Points.Reserve(Ring.VertexCount());
			for (const FVector2d& P : Ring.GetVertices())
			{
				Edge.Points.Emplace(P.X, P.Y,
					Field.SampleHeight(P.X, P.Y, FallbackZ) + SidewalkTopLiftCm);
			}
		};

		for (const UE::Geometry::FGeneralPolygon2d& GP : Ctx.ZoneSidewalkPolys[z])
		{
			CaptureRing(GP.GetOuter(), /*bHole*/false);
			for (const UE::Geometry::FPolygon2d& Hole : GP.GetHoles())
			{
				CaptureRing(Hole, /*bHole*/true);
			}
		}
	}

	// § keep spline edits: the rings above were just re-derived from the
	// surface union, so the user's sidewalk-knot edits go back on now — each
	// pin claims the nearest ring point to where the plan HAD that knot.
	int32 Pinned = 0;
	if (CVarRoadNetKeepSplineEdits.GetValueOnAnyThread() != 0)
	{
		for (const FRoadNetPlanPin& Pin : PlanEditPins)
		{
			if (!Pin.bWalk) { continue; }
			FVector* BestP = nullptr;
			double BestD = 500.0;
			for (FRoadNetPlanEdge& E : PlanSidewalkEdges)
			{
				for (FVector& P : E.Points)
				{
					const double D = FVector2D::Distance(FVector2D(P.X, P.Y), Pin.OrigXY);
					if (D < BestD) { BestD = D; BestP = &P; }
				}
			}
			if (BestP)
			{
				*BestP = FVector(Pin.XY.X, Pin.XY.Y, Pin.ZCm);
				++Pinned;
			}
		}
	}

	// § ease heights on every ring: adjacent knots inside the ease radius may
	// not disagree in Z by more than the ease step — pinned or not.
	int32 Eased = 0;
	if (CVarRoadNetEaseHeights.GetValueOnAnyThread() != 0)
	{
		const double MaxStep = FMath::Max(1.0, (double)CVarRoadNetEaseMaxStepCm.GetValueOnAnyThread());
		const double Radius  = FMath::Max(10.0, (double)CVarRoadNetEaseRadiusCm.GetValueOnAnyThread());
		for (FRoadNetPlanEdge& E : PlanSidewalkEdges)
		{
			Eased += EasePolylineZ(E.Points, /*bClosed*/true, MaxStep, Radius);
		}
	}

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] StreetPlan: captured %d sidewalk edge ring(s) with Z across %d zone(s); %d user pin(s) re-applied, %d knot(s) eased."),
		PlanSidewalkEdges.Num(), Ctx.ZoneSidewalkPolys.Num(), Pinned, Eased);
}

void URoadNetwork::BuildTilePartition(FRoadNetRebuildContext& Ctx)
{
	using namespace UE::Geometry;

	// § tiling v2 — OWNERSHIP BY CONSTRUCTION. Two days of routing heuristics
	// (centroid, nearest-vertex, majority vote, medial cuts) all failed the same
	// way: once roads are boolean-merged the geometry is anonymous, and ANY
	// post-hoc spatial guess has a counterexample (a parallel neighbour steals a
	// sidewalk). Here each segment tile's polygons are GENERATED from that arm
	// run's own centreline slice and bucketed under their tile key at creation —
	// there is no assignment step left to get wrong.
	TArray<TMap<FIntPoint, TArray<FGeneralPolygon2d>>>& Surf =
		Ctx.ZoneTileLayers.FindOrAdd(FName(TEXT("Surface")));
	TArray<TMap<FIntPoint, TArray<FGeneralPolygon2d>>>& Walk =
		Ctx.ZoneTileLayers.FindOrAdd(FName(TEXT("Sidewalks")));
	const int32 NZ = Ctx.Zones.Num();
	Surf.Reset(); Surf.SetNum(NZ);
	Walk.Reset(); Walk.SetNum(NZ);

	auto BoxOf = [](const FGeneralPolygon2d& GP) -> FBox2D
	{
		FBox2D B(ForceInit);
		for (const FVector2d& V : GP.GetOuter().GetVertices()) { B += FVector2D(V.X, V.Y); }
		return B;
	};

	// Both-sides self-check bookkeeping: bit 1 = left walk emitted, bit 2 = right.
	TMap<TPair<int32, int32>, uint8> SideSeen;
	int32 ArmsChecked = 0, OneSided = 0;

	for (int32 z = 0; z < NZ; ++z)
	{
		const TArray<FGeneralPolygon2d> Empty;
		const TArray<FGeneralPolygon2d>& Merged = Ctx.ZoneSurfacePolys.IsValidIndex(z) ? Ctx.ZoneSurfacePolys[z] : Empty;
		const TArray<FGeneralPolygon2d>& Band   = Ctx.ZoneSidewalkPolys.IsValidIndex(z) ? Ctx.ZoneSidewalkPolys[z] : Empty;
		const TArray<FGeneralPolygon2d>& Carve  = Ctx.ZoneJunctionCarve.IsValidIndex(z) ? Ctx.ZoneJunctionCarve[z] : Empty;
		if (Merged.Num() == 0 && Band.Num() == 0) { continue; }

		// Bboxes for the zone-wide operands. Every per-arm boolean below used to
		// take the WHOLE list — Carve is one polygon per junction (~800 in a city
		// zone) and each arm only touches two or three of them, so the booleans
		// were O(arms x junctions) and cost minutes. Narrowing by bbox first is
		// exact: a polygon whose bbox misses the query can't affect the result.
		auto BoxesOf = [&BoxOf](const TArray<FGeneralPolygon2d>& A)
		{
			TArray<FBox2D> B; B.Reserve(A.Num());
			for (const FGeneralPolygon2d& GP : A) { B.Add(BoxOf(GP)); }
			return B;
		};
		const TArray<FBox2D> CarveBoxes  = BoxesOf(Carve);
		const TArray<FBox2D> BandBoxes   = BoxesOf(Band);
		const TArray<FBox2D> MergedBoxes = BoxesOf(Merged);

		auto BoxOfArr = [&BoxOf](const TArray<FGeneralPolygon2d>& A)
		{
			FBox2D B(ForceInit);
			for (const FGeneralPolygon2d& GP : A) { B += BoxOf(GP); }
			return B;
		};
		// Returns Src itself when nothing can be dropped, so a zone whose band is
		// one big merged polygon is never deep-copied per arm.
		auto Narrow = [](const TArray<FGeneralPolygon2d>& Src, const TArray<FBox2D>& Boxes,
			const FBox2D& Q, TArray<FGeneralPolygon2d>& Scratch) -> const TArray<FGeneralPolygon2d>&
		{
			if (Src.Num() <= 1 || !Q.bIsValid) { return Src; }
			int32 Hits = 0;
			for (int32 i = 0; i < Src.Num(); ++i)
			{
				if (Boxes[i].bIsValid && Boxes[i].Intersect(Q)) { ++Hits; }
			}
			if (Hits == Src.Num()) { return Src; }
			Scratch.Reset();
			Scratch.Reserve(Hits);
			for (int32 i = 0; i < Src.Num(); ++i)
			{
				if (Boxes[i].bIsValid && Boxes[i].Intersect(Q)) { Scratch.Add(Src[i]); }
			}
			return Scratch;
		};
		TArray<FGeneralPolygon2d> NearCarve, NearBand, NearMerged;

		// ---- junction tiles: everything inside the carve, cut from the SAME
		// merged surface / band the segments are cut from → seams line up exactly.
		for (const FGeneralPolygon2d& CP : Carve)
		{
			const TArray<FVector2d>& OV = CP.GetOuter().GetVertices();
			if (OV.Num() < 3) { continue; }
			FVector2D C(0, 0);
			for (const FVector2d& V : OV) { C += FVector2D(V.X, V.Y); }
			C /= (double)OV.Num();
			const FIntPoint JKey = JunTileKey(C);
			const TArray<FGeneralPolygon2d> CPArr = { CP };
			const FBox2D CPBox = BoxOf(CP);
			TArray<FGeneralPolygon2d> Piece;
			const TArray<FGeneralPolygon2d>& MSrc = Narrow(Merged, MergedBoxes, CPBox, NearMerged);
			if (MSrc.Num() > 0 && PolygonsIntersection(MSrc, CPArr, Piece) && Piece.Num() > 0)
			{
				Surf[z].FindOrAdd(JKey).Append(MoveTemp(Piece));
			}
			TArray<FGeneralPolygon2d> WPiece;
			const TArray<FGeneralPolygon2d>& BSrc = Narrow(Band, BandBoxes, CPBox, NearBand);
			if (BSrc.Num() > 0 && PolygonsIntersection(BSrc, CPArr, WPiece) && WPiece.Num() > 0)
			{
				Walk[z].FindOrAdd(JKey).Append(MoveTemp(WPiece));
			}
		}

		// ---- segment tiles: one bucket per (road, arm run), alias-resolved so a
		// divided pair shares a tile. Deterministic emission order (road index,
		// arm index) + subtract-what-was-already-emitted resolves any physical
		// overlap (continuation seams, fused parallel walk strips) without theft:
		// a road always keeps the part bordering its own carriageway.
		TArray<FGeneralPolygon2d> SurfEmitted; TArray<FBox2D> SurfBoxes;
		TArray<FGeneralPolygon2d> WalkEmitted; TArray<FBox2D> WalkBoxes;

		// Subtract earlier emissions that bbox-overlap, then append + bucket.
		auto EmitPieces = [&](TArray<FGeneralPolygon2d>&& Pieces,
			TArray<FGeneralPolygon2d>& Emitted, TArray<FBox2D>& Boxes,
			TMap<FIntPoint, TArray<FGeneralPolygon2d>>& Buckets, const FIntPoint& Key) -> bool
		{
			if (Pieces.Num() == 0) { return false; }
			FBox2D PB(ForceInit);
			for (const FGeneralPolygon2d& GP : Pieces) { PB += BoxOf(GP); }
			TArray<FGeneralPolygon2d> Prior;
			for (int32 i = 0; i < Emitted.Num(); ++i)
			{
				if (Boxes[i].bIsValid && PB.bIsValid && Boxes[i].Intersect(PB)) { Prior.Add(Emitted[i]); }
			}
			if (Prior.Num() > 0)
			{
				TArray<FGeneralPolygon2d> Cut;
				if (RoadNetSurface::Difference(Pieces, Prior, Cut)) { Pieces = MoveTemp(Cut); }
			}
			if (Pieces.Num() == 0) { return false; }
			for (const FGeneralPolygon2d& GP : Pieces)
			{
				Emitted.Add(GP);
				Boxes.Add(BoxOf(GP));
			}
			Buckets.FindOrAdd(Key).Append(MoveTemp(Pieces));
			return true;
		};

		for (int32 r : Ctx.Zones[z])
		{
			if (!Roads.IsValidIndex(r) || !Roads[r].Id.IsValid()) { continue; }
			const FRoadCurves* C = Ctx.Curves.Find(r);
			const TArray<TPair<int32, int32>>* Runs = Ctx.RoadArmRuns.Find(r);
			if (!C || !Runs || C->Sampled.Num() < 2) { continue; }
			const int32 N = C->Sampled.Num();
			const bool bEdges = (C->LeftEdge.Num() == N && C->RightEdge.Num() == N);
			const FRoadNetLaneSpec& L = Roads[r].Lanes;
			const double Half = FMath::Max(50.0, (double)L.HalfWidthCm());
			const double SwW = (double)L.SidewalkWidth;
			const double SwIn  = FMath::Max(1.0, Half - 30.0);
			// + the deepest bay: at a pocket the carriageway edge sits that far
			// out, so the walk behind it would fall outside a constant-Half mask.
			const double SwOut = Half + SwW + 60.0 + Roads[r].MaxBayDepthCm();

			for (int32 av = 0; av < Runs->Num(); ++av)
			{
				const TPair<int32, int32>& Run = (*Runs)[av];
				if (Run.Key < 0 || Run.Value <= Run.Key || Run.Value >= N) { continue; }
				// Pad one sample past each end so the slice's caps land INSIDE the
				// junction carve; subtracting the carve then trims to the exact
				// boundary (no hairline gap at the cut).
				const int32 Lo = FMath::Max(0, Run.Key - 1);
				const int32 Hi = FMath::Min(N - 1, Run.Value + 1);
				TArray<FVector> Sub(&C->Sampled[Lo], Hi - Lo + 1);

				const FIntPoint Key = SegTileKey(Roads[r].Id, av);
				if (Key.X == INDEX_NONE) { continue; }

				// Carriageway: this arm's own outline minus the junction carve.
				FGeneralPolygon2d Outline;
				bool bOutline = false;
				if (bEdges)
				{
					FRoadCurves Slice;
					Slice.Sampled  = Sub;
					Slice.LeftEdge  = TArray<FVector>(&C->LeftEdge[Lo],  Hi - Lo + 1);
					Slice.RightEdge = TArray<FVector>(&C->RightEdge[Lo], Hi - Lo + 1);
					bOutline = RoadNetSurface::BuildRoadOutline(Slice, Outline);
				}
				if (!bOutline)
				{
					bOutline = RoadNetSurface::BuildSideRibbon(Sub, -Half, +Half, Outline);
				}
				if (bOutline)
				{
					TArray<FGeneralPolygon2d> Piece = { Outline };
					if (Carve.Num() > 0)
					{
						const TArray<FGeneralPolygon2d>& CSrc =
							Narrow(Carve, CarveBoxes, BoxOfArr(Piece), NearCarve);
						TArray<FGeneralPolygon2d> Cut;
						if (CSrc.Num() > 0 && RoadNetSurface::Difference(Piece, CSrc, Cut)) { Piece = MoveTemp(Cut); }
					}
					EmitPieces(MoveTemp(Piece), SurfEmitted, SurfBoxes, Surf[z], Key);
				}

				// Sidewalks: BOTH enabled sides from THIS arm's slice, clipped to
				// the zone band (band already excludes every carriageway → no
				// flaps) and cut at the carve. Emitted under the SAME key as the
				// carriageway — theft is structurally impossible.
				if (SwW > 0.0 && Band.Num() > 0)
				{
					auto EmitSide = [&](double InOff, double OutOff, uint8 SideBit)
					{
						FGeneralPolygon2d Ribbon;
						if (!RoadNetSurface::BuildSideRibbon(Sub, InOff, OutOff, Ribbon)) { return; }
						const TArray<FGeneralPolygon2d> RArr = { Ribbon };
						TArray<FGeneralPolygon2d> Piece;
						const TArray<FGeneralPolygon2d>& WBSrc =
							Narrow(Band, BandBoxes, BoxOf(Ribbon), NearBand);
						if (WBSrc.Num() == 0) { return; }
						if (!PolygonsIntersection(WBSrc, RArr, Piece) || Piece.Num() == 0) { return; }
						if (Carve.Num() > 0)
						{
							const TArray<FGeneralPolygon2d>& CSrc =
								Narrow(Carve, CarveBoxes, BoxOfArr(Piece), NearCarve);
							TArray<FGeneralPolygon2d> Cut;
							if (CSrc.Num() > 0 && RoadNetSurface::Difference(Piece, CSrc, Cut)) { Piece = MoveTemp(Cut); }
						}
						if (EmitPieces(MoveTemp(Piece), WalkEmitted, WalkBoxes, Walk[z], Key))
						{
							SideSeen.FindOrAdd(TPair<int32, int32>(r, av)) |= SideBit;
						}
					};
					if (L.bSidewalkLeft)  { EmitSide(+SwIn, +SwOut, 1); }
					if (L.bSidewalkRight) { EmitSide(-SwIn, -SwOut, 2); }
				}
			}
		}

		// ---- residual sweep: what generation didn't cover — junction blend fill
		// outside the carve (2-arm continuation welds) and band end-caps at dead
		// ends. These are slivers ON a road/seam, so the point resolver is exact
		// for them (the failure mode was OFFSET geometry, which no longer gets
		// here). Without this sweep the welds would be visible holes.
		auto SweepResidual = [&](const TArray<FGeneralPolygon2d>& Source,
			const TArray<FGeneralPolygon2d>& Emitted,
			TMap<FIntPoint, TArray<FGeneralPolygon2d>>& Buckets)
		{
			if (Source.Num() == 0) { return; }
			TArray<FGeneralPolygon2d> Residual = Source;
			if (Carve.Num() > 0)
			{
				TArray<FGeneralPolygon2d> Cut;
				if (RoadNetSurface::Difference(Residual, Carve, Cut)) { Residual = MoveTemp(Cut); }
			}
			if (Emitted.Num() > 0 && Residual.Num() > 0)
			{
				TArray<FGeneralPolygon2d> Cut;
				if (RoadNetSurface::Difference(Residual, Emitted, Cut)) { Residual = MoveTemp(Cut); }
			}
			for (FGeneralPolygon2d& GP : Residual)
			{
				const TArray<FVector2d>& OV = GP.GetOuter().GetVertices();
				if (OV.Num() < 3) { continue; }
				// Boolean noise along coincident edges (raw outline vs merged
				// boundary) makes hairline slivers — drop them; keep real blend
				// fills (weld discs, band end-caps), which are far larger.
				if (FMath::Abs(GP.GetOuter().SignedArea()) < 500.0) { continue; }
				FVector2D PC(0, 0);
				for (const FVector2d& V : OV) { PC += FVector2D(V.X, V.Y); }
				PC /= (double)OV.Num();
				const FIntPoint Key = TopoKeyOf(FVector(PC.X, PC.Y, 0.0), Ctx);
				if (Key.X == INDEX_NONE) { continue; }
				Buckets.FindOrAdd(Key).Add(MoveTemp(GP));
			}
		};
		SweepResidual(Merged, SurfEmitted, Surf[z]);
		SweepResidual(Band, WalkEmitted, Walk[z]);
	}

	// ---- both-sides self-check: the exact invariant every v1 attempt violated.
	// A (road, arm) with BOTH sides enabled that emitted one walk but not the
	// other is named loudly. (Zero-sided short stubs are legitimate — a run can
	// sit entirely between two carves' band edges.)
	for (const TPair<TPair<int32, int32>, uint8>& KV : SideSeen)
	{
		const int32 r = KV.Key.Key;
		if (!Roads.IsValidIndex(r)) { continue; }
		const FRoadNetLaneSpec& L = Roads[r].Lanes;
		if (!L.bSidewalkLeft || !L.bSidewalkRight || L.SidewalkWidth <= 0.f) { continue; }
		++ArmsChecked;
		if (KV.Value != 3)
		{
			++OneSided;
			UE_LOG(LogRoadNet, Warning,
				TEXT("[RoadNet][TILECHK] road %d arm %d emitted only its %s sidewalk — other side missing from its tile."),
				r, KV.Key.Value, (KV.Value & 1) ? TEXT("LEFT") : TEXT("RIGHT"));
		}
	}
	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet][TILECHK] tile partition: %d arm(s) with both walks checked, %d one-sided (expected 0)."),
		ArmsChecked, OneSided);
}

void URoadNetwork::RetireAllTiles()
{
	EnsureTileRegistry();
	for (TPair<FIntPoint, TWeakObjectPtr<ARoadNetTileActor>>& KV : TileActors)
	{
		if (ARoadNetTileActor* Tile = KV.Value.Get()) { Tile->Destroy(); }
	}
	TileActors.Reset();
}

bool URoadNetwork::MoveRoadPoint(int32 RoadIdx, int32 PointIdx, const FVector& NewWorldPos)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	FRoadDef& R = Roads[RoadIdx];
	if (!R.Ref.IsValidIndex(PointIdx)) { return false; }
	R.Ref[PointIdx] = NewWorldPos;
	// Keep any per-point elevation override in sync with the moved point.
	if (R.Elev.IsValidIndex(PointIdx)) { R.Elev[PointIdx] = NewWorldPos.Z; }
	return true;
}

bool URoadNetwork::DeleteRoadPoint(int32 RoadIdx, int32 PointIdx, bool& bOutRoadRemoved)
{
	bOutRoadRemoved = false;
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	FRoadDef& R = Roads[RoadIdx];
	if (!R.Ref.IsValidIndex(PointIdx)) { return false; }

	const int32 RefBefore = R.Ref.Num();
	R.Ref.RemoveAt(PointIdx);
	if (R.Elev.IsValidIndex(PointIdx)) { R.Elev.RemoveAt(PointIdx); }
	if (R.NodeIds.IsValidIndex(PointIdx)) { R.NodeIds.RemoveAt(PointIdx); }
	UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet][DEL] DeleteRoadPoint road=%d idx=%d: Ref %d -> %d"),
		RoadIdx, PointIdx, RefBefore, R.Ref.Num());

	if (R.Ref.Num() < 2)
	{
		Roads.RemoveAt(RoadIdx);
		bOutRoadRemoved = true;
	}
	return true;
}

int32 URoadNetwork::DeleteRoadPointsSplitting(int32 RoadIdx, const TArray<int32>& PointIdxToRemove)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return 1; }
	const FRoadDef Base = Roads[RoadIdx];   // template for the surviving pieces
	const int32 N = Base.Ref.Num();

	TSet<int32> Rem;
	for (int32 p : PointIdxToRemove) { if (p >= 0 && p < N) { Rem.Add(p); } }
	if (Rem.Num() == 0) { return 1; }

	// Surviving indices split into runs of consecutive originals: a removed point
	// between two survivors ends the current run, which is exactly the gap.
	TArray<TArray<int32>> Runs;
	TArray<int32> Cur;
	for (int32 i = 0; i < N; ++i)
	{
		if (Rem.Contains(i)) { if (Cur.Num() > 0) { Runs.Add(MoveTemp(Cur)); Cur.Reset(); } continue; }
		Cur.Add(i);
	}
	if (Cur.Num() > 0) { Runs.Add(MoveTemp(Cur)); }

	auto SliceInto = [&Base](const TArray<int32>& Run) -> FRoadDef
	{
		FRoadDef Out = Base;                 // keep lanes / source / sidewalk / etc.
		Out.Ref.Reset(); Out.Elev.Reset(); Out.NodeIds.Reset();
		for (int32 idx : Run)
		{
			Out.Ref.Add(Base.Ref[idx]);
			if (Base.Elev.IsValidIndex(idx))    { Out.Elev.Add(Base.Elev[idx]); }
			if (Base.NodeIds.IsValidIndex(idx)) { Out.NodeIds.Add(Base.NodeIds[idx]); }
		}
		return Out;
	};

	TArray<FRoadDef> Pieces;
	for (const TArray<int32>& Run : Runs)
	{
		if (Run.Num() >= 2) { Pieces.Add(SliceInto(Run)); }
	}

	if (Pieces.Num() == 0)
	{
		Roads.RemoveAt(RoadIdx);
		UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet][DEL] SplitDelete road=%d: no surviving run >=2 pts, road removed"), RoadIdx);
		return 0;
	}

	// First piece keeps the original slot + Id (stable); extra runs become new
	// roads with fresh Ids (indices only grow, so no cached index shifts down).
	Pieces[0].Id = Base.Id;
	Roads[RoadIdx] = Pieces[0];
	for (int32 i = 1; i < Pieces.Num(); ++i)
	{
		FRoadDef P = Pieces[i];
		P.Id = FGuid::NewGuid();
		AddRoad(P);
	}
	UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet][DEL] SplitDelete road=%d: %d pts removed -> %d piece(s) (%s)"),
		RoadIdx, Rem.Num(), Pieces.Num(), Pieces.Num() > 1 ? TEXT("SPLIT with gap") : TEXT("trimmed"));
	return Pieces.Num();
}

bool URoadNetwork::RemoveRoad(int32 RoadIdx)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	Roads.RemoveAt(RoadIdx);
	return true;
}

namespace
{
	// Closest point on a polyline to a 2-D query, returning the lerped 3-D point
	// (so the merged midline carries a sensible Z from each member).
	FVector RoadNetClosestOnPolyline(const FVector2D& Q, const TArray<FVector>& Poly)
	{
		double Best = TNumericLimits<double>::Max();
		FVector BestPt = Poly.Num() ? Poly[0] : FVector::ZeroVector;
		for (int32 i = 0; i + 1 < Poly.Num(); ++i)
		{
			const FVector2D A(Poly[i]);
			const FVector2D B(Poly[i + 1]);
			const FVector2D AB = B - A;
			const double LenSq = AB.SizeSquared();
			const double T = LenSq > UE_DOUBLE_SMALL_NUMBER
				? FMath::Clamp((double)FVector2D::DotProduct(Q - A, AB) / LenSq, 0.0, 1.0) : 0.0;
			const FVector2D Cl = A + AB * T;
			const double D = FVector2D::Distance(Q, Cl);
			if (D < Best) { Best = D; BestPt = FMath::Lerp(Poly[i], Poly[i + 1], T); }
		}
		return BestPt;
	}
}

bool URoadNetwork::MergeRoads(TArrayView<const int32> RoadIndices)
{
	// Unique, valid indices only.
	TArray<int32> Idx;
	for (int32 i : RoadIndices) { if (Roads.IsValidIndex(i)) { Idx.AddUnique(i); } }
	if (Idx.Num() < 2) { return false; }

	// Primary = longest centreline (keeps its identity/tags for the merged road).
	auto ArcLen = [&](int32 r) { return RoadNetMath::TotalLength(Roads[r].Ref); };
	Idx.Sort([&](const int32& A, const int32& B) { return ArcLen(A) > ArcLen(B); });
	const int32 Primary = Idx[0];

	// Resample the primary centreline evenly, then at each sample average the
	// nearest point of every other member → the cluster midline.
	const double PrimLen = FMath::Max(1.0, ArcLen(Primary));
	const int32  NS      = FMath::Clamp(Roads[Primary].Ref.Num(), 2, 512);
	TArray<FVector> Base;
	RoadNetMath::ResampleByArcLength(Roads[Primary].Ref, FMath::Max(50.0, PrimLen / (NS - 1)), Base);
	if (Base.Num() < 2) { Base = Roads[Primary].Ref; }

	TArray<FVector> Mid;
	Mid.Reserve(Base.Num());
	for (const FVector& P : Base)
	{
		FVector Sum = P;
		int32   Cnt = 1;
		for (int32 k = 1; k < Idx.Num(); ++k)
		{
			Sum += RoadNetClosestOnPolyline(FVector2D(P), Roads[Idx[k]].Ref);
			++Cnt;
		}
		Mid.Add(Sum / (double)Cnt);
	}

	// Lane count = sum of members' effective lanes; sidewalks = OR (widest wins).
	// ZoneGraph is ORed too: the merged road covers every member's ground, so
	// dropping the flag would silently lose a choice made before the merge.
	int32 TotalLanes = 0;
	bool  bSwL = false, bSwR = false, bZone = false;
	float SwW = 0.f;
	for (int32 r : Idx)
	{
		TotalLanes += FMath::Max(1, Roads[r].Lanes.EffectiveLaneCount());
		bSwL |= Roads[r].Lanes.bSidewalkLeft;
		bSwR |= Roads[r].Lanes.bSidewalkRight;
		bZone |= Roads[r].bZoneGraph;
		SwW = FMath::Max(SwW, Roads[r].Lanes.SidewalkWidth);
	}

	FRoadDef Merged = Roads[Primary];   // inherit class / source / name / grade
	Merged.Id = FGuid::NewGuid();
	Merged.Ref = MoveTemp(Mid);
	Merged.bZoneGraph = bZone;
	Merged.Elev.Reset();
	Merged.NodeIds.Reset();             // synthetic midline shares no OSM node
	Merged.StartLinks.Reset();
	Merged.EndLinks.Reset();

	FRoadNetLaneSpec& L = Merged.Lanes;
	L.DetailedLanes.Reset();            // fall back to the summed count model
	L.LaneWidths.Reset();
	L.Total    = FMath::Max(1, TotalLanes);
	L.Forward  = 0;
	L.Backward = 0;
	L.bOneway  = false;
	L.bSidewalkLeft  = bSwL;
	L.bSidewalkRight = bSwR;
	if (SwW > 0.f) { L.SidewalkWidth = SwW; }

	// Remove members high-index-first (so earlier indices stay valid), then add.
	TArray<int32> ToRemove = Idx;
	ToRemove.Sort([](const int32& A, const int32& B) { return A > B; });
	for (int32 r : ToRemove) { Roads.RemoveAt(r); }
	AddRoad(Merged);

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] MergeRoads: folded %d roads into 1 (%d lanes)."),
		Idx.Num(), L.Total);
	return true;
}

namespace
{
	bool RayCircleHit(const FVector2D& Origin, const FVector2D& Dir, const FVector2D& C,
		double R, FVector2D& Out)
	{
		const FVector2D D = Dir.GetSafeNormal();
		if (D.IsNearlyZero() || R < 1.0) { return false; }
		const FVector2D F = Origin - C;
		const double B = 2.0 * FVector2D::DotProduct(D, F);
		const double Cd = FVector2D::DotProduct(F, F) - R * R;
		const double Disc = B * B - 4.0 * Cd;
		if (Disc < 0.0) { return false; }
		const double S = FMath::Sqrt(Disc);
		const double T0 = (-B - S) * 0.5;
		const double T1 = (-B + S) * 0.5;
		double T = -1.0;
		if (T0 > 1.0) { T = T0; }
		if (T1 > 1.0 && (T < 0.0 || T1 < T)) { T = T1; }
		if (T < 0.0)
		{
			if (T0 >= 0.0) { T = T0; }
			else if (T1 >= 0.0) { T = T1; }
		}
		if (T < 0.0) { return false; }
		Out = Origin + D * T;
		return true;
	}

	bool RoadLooksClosed(const TArray<FVector>& Ref)
	{
		return Ref.Num() >= 3 && FVector::DistSquaredXY(Ref[0], Ref.Last()) < FMath::Square(80.0);
	}
}

bool URoadNetwork::CleanRoundabout(TArrayView<const int32> RoadIndices)
{
	TArray<int32> Idx;
	for (int32 i : RoadIndices) { if (Roads.IsValidIndex(i)) { Idx.AddUnique(i); } }
	if (Idx.Num() < 1) { return false; }

	TArray<FVector2D> Pts;
	double SumZ = 0.0;
	int32 ZN = 0;
	for (int32 r : Idx)
	{
		for (const FVector& P : Roads[r].Ref)
		{
			Pts.Emplace(P.X, P.Y);
			SumZ += P.Z; ++ZN;
		}
	}
	FVector2D Centre;
	double Radius = 0.0;
	if (!RoadNetMath::FitCircle(Pts, Centre, Radius) || Radius < 200.0) { return false; }

	auto ArcLen = [&](int32 r) { return RoadNetMath::TotalLength(Roads[r].Ref); };
	Idx.Sort([&](const int32& A, const int32& B) { return ArcLen(A) > ArcLen(B); });
	const int32 Primary = Idx[0];

	const int32 Segs = FMath::Clamp(FMath::RoundToInt(Radius / 100.0), 16, 96);
	TArray<FVector> Ring;
	RoadNetMath::SampleCircle(Centre, Radius, ZN > 0 ? SumZ / (double)ZN : 0.0, Segs, Ring);

	FRoadDef Circ = Roads[Primary];
	Circ.Id = FGuid::NewGuid();
	Circ.Ref = MoveTemp(Ring);
	Circ.Elev.Reset();
	Circ.NodeIds.Reset();
	Circ.StartLinks.Reset();
	Circ.EndLinks.Reset();
	Circ.OuterEdgeLeft.Reset();
	Circ.OuterEdgeRight.Reset();
	Circ.ParkingBays.Reset();
	Circ.Crossings.Reset();

	const float CircW = FMath::Max(350.f, Circ.Lanes.HalfWidthCm() * 2.f);

	TArray<int32> ToRemove = Idx;
	ToRemove.Sort([](const int32& A, const int32& B) { return A > B; });
	for (int32 r : ToRemove) { Roads.RemoveAt(r); }
	const int32 RingIdx = AddRoad(Circ);

	const double Snap = FMath::Max(800.0, Radius * 0.35);
	for (int32 r = 0; r < Roads.Num(); ++r)
	{
		if (r == RingIdx) { continue; }
		FRoadDef& R = Roads[r];
		if (R.Ref.Num() < 2) { continue; }

		auto RetrimEnd = [&](bool bStart)
		{
			const int32 EndI = bStart ? 0 : (R.Ref.Num() - 1);
			const int32 InI  = bStart ? 1 : (R.Ref.Num() - 2);
			const FVector2D P(R.Ref[EndI].X, R.Ref[EndI].Y);
			const double Dist = FVector2D::Distance(P, Centre);
			if (FMath::Abs(Dist - Radius) > Snap) { return; }
			const FVector2D Interior(R.Ref[InI].X, R.Ref[InI].Y);
			FVector2D Dir = P - Interior;
			if (Dir.IsNearlyZero()) { Dir = P - Centre; }
			FVector2D Hit;
			if (!RayCircleHit(Interior, Dir, Centre, Radius, Hit))
			{
				const FVector2D Rad = (P - Centre).GetSafeNormal();
				if (Rad.IsNearlyZero()) { return; }
				Hit = Centre + Rad * Radius;
			}
			R.Ref[EndI].X = Hit.X;
			R.Ref[EndI].Y = Hit.Y;
		};
		RetrimEnd(true);
		RetrimEnd(false);
	}

	UpsertRoundaboutAt(Centre, (float)Radius, CircW);
	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] CleanRoundabout: %d roads -> ring R=%.0f cm"),
		Idx.Num(), Radius);
	return true;
}

const FRoadNetRoundaboutConfig* URoadNetwork::FindRoundaboutNear(const FVector2D& Loc) const
{
	const FRoadNetRoundaboutConfig* Best = nullptr;
	double BestD2 = TNumericLimits<double>::Max();
	for (const FRoadNetRoundaboutConfig& Cfg : RoundaboutConfigs)
	{
		const double Lim = FMath::Max(kJunctionMatchCm, (double)Cfg.InscribedRadiusCm);
		const double D2 = FVector2D::DistSquared(Cfg.Location, Loc);
		if (D2 < Lim * Lim && D2 < BestD2) { BestD2 = D2; Best = &Cfg; }
	}
	return Best;
}

int32 URoadNetwork::UpsertRoundaboutAt(const FVector2D& Loc, float InscribedRadiusCm, float CirculatoryWidthCm)
{
	Modify();
	int32 BestIdx = INDEX_NONE;
	double BestD2 = FMath::Square(FMath::Max(kJunctionMatchCm, (double)InscribedRadiusCm));
	for (int32 i = 0; i < RoundaboutConfigs.Num(); ++i)
	{
		const double D2 = FVector2D::DistSquared(RoundaboutConfigs[i].Location, Loc);
		if (D2 < BestD2) { BestD2 = D2; BestIdx = i; }
	}
	if (BestIdx == INDEX_NONE)
	{
		FRoadNetRoundaboutConfig Cfg;
		BestIdx = RoundaboutConfigs.Add(Cfg);
	}
	FRoadNetRoundaboutConfig& C = RoundaboutConfigs[BestIdx];
	C.Location = Loc;
	C.InscribedRadiusCm = FMath::Max(200.f, InscribedRadiusCm);
	if (CirculatoryWidthCm > 0.f) { C.CirculatoryWidthCm = CirculatoryWidthCm; }
	return BestIdx;
}

bool URoadNetwork::InsertRoadPoint(int32 RoadIdx, int32 AfterIdx, const FVector& Pos)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	FRoadDef& R = Roads[RoadIdx];
	if (AfterIdx < 0 || AfterIdx >= R.Ref.Num()) { return false; }
	const int32 At = AfterIdx + 1;
	R.Ref.Insert(Pos, At);
	if (R.Elev.Num() > 0)    { R.Elev.Insert(Pos.Z, FMath::Min(At, R.Elev.Num())); }
	if (R.NodeIds.Num() > 0) { R.NodeIds.Insert((int64)-1, FMath::Min(At, R.NodeIds.Num())); }
	return true;
}

namespace
{
	// Typical lane width (cm) for each authored lane type. Bicycle paths are
	// narrow, parking bays are wide enough for a parked car, everything else is
	// a standard driving lane.
	double LaneTypeDefaultWidthCm(ERoadNetLaneType Type)
	{
		switch (Type)
		{
			case ERoadNetLaneType::Bicycle:  return 150.0; // 1.5 m
			case ERoadNetLaneType::Parking:  return 250.0; // 2.5 m
			case ERoadNetLaneType::Sidewalk: return 200.0; // 2 m
			case ERoadNetLaneType::Median:   return 300.0; // 3 m
			default:                         return 350.0; // 3.5 m driving lane
		}
	}

	// Recompute every authored lane's CenterOffset + Side by stacking widths
	// left→right, centred on the reference line. A central median (MedianHalf>0)
	// opens a gap in the middle. Lanes are assumed already ordered left→right.
	void RelayoutLanes(TArray<FRoadNetLane>& Lanes, double /*MedianHalfCm*/)
	{
		FRoadNetLaneSpec::RelayoutStack(Lanes);
	}

	void SyncFlagsFromLanes(FRoadNetLaneSpec& L)
	{
		if (!L.HasDetailedLanes()) { return; }
		L.bMedian = false;
		L.bSidewalkLeft = false;
		L.bSidewalkRight = false;
		for (const FRoadNetLane& Ln : L.DetailedLanes)
		{
			if (Ln.Type == ERoadNetLaneType::Median)
			{
				L.bMedian = true;
				L.MedianWidth = Ln.Width;
			}
			else if (Ln.Type == ERoadNetLaneType::Sidewalk)
			{
				if (Ln.CenterOffset < 0.0) { L.bSidewalkLeft = true; }
				else { L.bSidewalkRight = true; }
				L.SidewalkWidth = Ln.Width;
			}
		}
	}

	void MigrateFlagsToLanes(FRoadNetLaneSpec& L)
	{
		auto HasType = [&L](ERoadNetLaneType T) -> bool
		{
			for (const FRoadNetLane& Ln : L.DetailedLanes)
			{
				if (Ln.Type == T) { return true; }
			}
			return false;
		};
		auto MakeSlot = [](ERoadNetLaneType T, float W) -> FRoadNetLane
		{
			FRoadNetLane Ln;
			Ln.LaneId = FGuid::NewGuid();
			Ln.Type = T;
			Ln.Direction = ERoadNetLaneDirection::None;
			Ln.Width = FMath::Max(30.f, W);
			Ln.Side = (T == ERoadNetLaneType::Median) ? ERoadNetSide::Center : ERoadNetSide::Right;
			return Ln;
		};
		if (L.bMedian && !HasType(ERoadNetLaneType::Median) && L.DetailedLanes.Num() >= 1)
		{
			int32 Mid = 0;
			for (int32 i = 0; i < L.DetailedLanes.Num(); ++i)
			{
				if (L.DetailedLanes[i].Type != ERoadNetLaneType::Sidewalk) { Mid = i + 1; }
			}
			Mid = FMath::Clamp(Mid / 2, 0, L.DetailedLanes.Num());
			L.DetailedLanes.Insert(MakeSlot(ERoadNetLaneType::Median, L.MedianWidth), Mid);
		}
		if (L.bSidewalkLeft && L.SidewalkWidth > 0.f
			&& (L.DetailedLanes.Num() == 0 || L.DetailedLanes[0].Type != ERoadNetLaneType::Sidewalk))
		{
			L.DetailedLanes.Insert(MakeSlot(ERoadNetLaneType::Sidewalk, L.SidewalkWidth), 0);
		}
		if (L.bSidewalkRight && L.SidewalkWidth > 0.f
			&& (L.DetailedLanes.Num() == 0 || L.DetailedLanes.Last().Type != ERoadNetLaneType::Sidewalk))
		{
			L.DetailedLanes.Add(MakeSlot(ERoadNetLaneType::Sidewalk, L.SidewalkWidth));
		}
	}

	// Every median change moves every authored lane, because the gap it opens is
	// part of the same stack. Count-model roads get that from ResolveLanes() on
	// read, but a road with DetailedLanes stores its offsets, so without this the
	// lanes keep the PREVIOUS median's spacing and the carriageway either overlaps
	// the median or leaves a hole beside it.
	void RestackForMedian(FRoadNetLaneSpec& L)
	{
		if (L.HasDetailedLanes())
		{
			MigrateFlagsToLanes(L);
			RelayoutLanes(L.DetailedLanes, 0.0);
			SyncFlagsFromLanes(L);
		}
	}

	void EnsureDetailedLanes(FRoadNetLaneSpec& L, bool bDriveOnLeft)
	{
		if (!L.HasDetailedLanes()) { L.DetailedLanes = L.ResolveLanes(bDriveOnLeft); }
		L.DetailedLanes.Sort([](const FRoadNetLane& A, const FRoadNetLane& B)
			{ return A.CenterOffset < B.CenterOffset; });
		MigrateFlagsToLanes(L);
		RelayoutLanes(L.DetailedLanes, 0.0);
		SyncFlagsFromLanes(L);
	}
}

bool URoadNetwork::AddLane(int32 RoadIdx, ERoadNetSide Side)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;

	// Authored road: append a normal lane on the far end of the chosen side.
	if (L.HasDetailedLanes())
	{
		FRoadNetLane NL;
		NL.LaneId = FGuid::NewGuid();
		NL.Type   = ERoadNetLaneType::Normal;
		NL.Width  = (float)LaneTypeDefaultWidthCm(ERoadNetLaneType::Normal);
		if (Side == ERoadNetSide::Left) { L.DetailedLanes.Insert(NL, 0); }
		else                            { L.DetailedLanes.Add(NL); }
		RelayoutLanes(L.DetailedLanes, 0.0);
		SyncFlagsFromLanes(L);
		return true;
	}

	const bool bDirectional = (L.Forward > 0 || L.Backward > 0);
	if (!bDirectional)
	{
		L.Total = FMath::Max(1, L.Total) + 1;   // Left/Right/Center all just widen
		return true;
	}
	if (Side == ERoadNetSide::Left) { ++L.Backward; }   // Center folds to the forward bank
	else                            { ++L.Forward;  }
	L.Total = L.Forward + L.Backward;
	return true;
}

bool URoadNetwork::RemoveLane(int32 RoadIdx, ERoadNetSide Side)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;

	// Authored road: drop the outermost lane on the chosen side (keep >=1).
	if (L.HasDetailedLanes())
	{
		if (L.DetailedLanes.Num() <= 1) { return false; }
		if (Side == ERoadNetSide::Left) { L.DetailedLanes.RemoveAt(0); }
		else                            { L.DetailedLanes.RemoveAt(L.DetailedLanes.Num() - 1); }
		RelayoutLanes(L.DetailedLanes, 0.0);
		SyncFlagsFromLanes(L);
		return true;
	}

	const bool bDirectional = (L.Forward > 0 || L.Backward > 0);
	if (!bDirectional)
	{
		if (L.Total <= 1) { return false; }     // keep at least one lane
		--L.Total;
		return true;
	}
	if (Side == ERoadNetSide::Left)
	{
		if (L.Backward <= 0) { return false; }
		--L.Backward;
	}
	else
	{
		if (L.Forward <= 0) { return false; }
		--L.Forward;
	}
	// Never strip the road to zero lanes — undo the decrement if it would.
	if (L.Forward + L.Backward < 1)
	{
		if (Side == ERoadNetSide::Left) { ++L.Backward; } else { ++L.Forward; }
		return false;
	}
	L.Total = L.Forward + L.Backward;
	return true;
}

int32 URoadNetwork::GetLaneCount(int32 RoadIdx) const
{
	if (!Roads.IsValidIndex(RoadIdx)) { return 0; }
	return Roads[RoadIdx].Lanes.EffectiveLaneCount();
}

TArray<FRoadNetLane> URoadNetwork::GetLanesLeftToRight(int32 RoadIdx) const
{
	if (!Roads.IsValidIndex(RoadIdx)) { return {}; }
	TArray<FRoadNetLane> Lanes = Roads[RoadIdx].Lanes.ResolveLanes(bDriveOnLeft);
	Lanes.Sort([](const FRoadNetLane& A, const FRoadNetLane& B)
		{ return A.CenterOffset < B.CenterOffset; });
	return Lanes;
}

bool URoadNetwork::MaterializeLanes(int32 RoadIdx)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	EnsureDetailedLanes(Roads[RoadIdx].Lanes, bDriveOnLeft);
	return true;
}

int32 URoadNetwork::InsertLaneRelative(int32 RoadIdx, int32 LaneLtoR, bool bRightSide)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return INDEX_NONE; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	EnsureDetailedLanes(L, bDriveOnLeft);
	if (!L.DetailedLanes.IsValidIndex(LaneLtoR)) { return INDEX_NONE; }

	FRoadNetLane NL;
	NL.LaneId = FGuid::NewGuid();
	NL.Type   = ERoadNetLaneType::Normal;
	NL.Width  = (float)LaneTypeDefaultWidthCm(ERoadNetLaneType::Normal);

	const int32 Pos = bRightSide ? (LaneLtoR + 1) : LaneLtoR;
	L.DetailedLanes.Insert(NL, Pos);
	RelayoutLanes(L.DetailedLanes, 0.0);
	SyncFlagsFromLanes(L);

	// New left→right index of the originally selected lane (so the caller keeps
	// its highlight): unchanged when we inserted to its right, +1 to its left.
	return bRightSide ? LaneLtoR : (LaneLtoR + 1);
}

ERoadNetLaneType URoadNetwork::CycleLaneType(int32 RoadIdx, int32 LaneLtoR, int32 Dir)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return ERoadNetLaneType::Normal; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	EnsureDetailedLanes(L, bDriveOnLeft);
	if (!L.DetailedLanes.IsValidIndex(LaneLtoR)) { return ERoadNetLaneType::Normal; }

	// Author cycle order: driving → bicycle path → parking bay → driving.
	static const ERoadNetLaneType kCycle[] = {
		ERoadNetLaneType::Normal, ERoadNetLaneType::Bicycle, ERoadNetLaneType::Parking };
	constexpr int32 N = UE_ARRAY_COUNT(kCycle);

	FRoadNetLane& Ln = L.DetailedLanes[LaneLtoR];
	int32 Cur = 0;
	for (int32 i = 0; i < N; ++i) { if (kCycle[i] == Ln.Type) { Cur = i; break; } }
	const int32 Next = ((Cur + (Dir >= 0 ? 1 : -1)) % N + N) % N;
	Ln.Type  = kCycle[Next];
	Ln.Width = (float)LaneTypeDefaultWidthCm(Ln.Type);
	RelayoutLanes(L.DetailedLanes, 0.0);
	SyncFlagsFromLanes(L);
	return Ln.Type;
}

namespace
{
	// Width bounds for the cross-section editor's drag. The floor is about a
	// kerb strip and the ceiling about a bus lane plus margin: outside that a
	// drag has stopped describing a lane and started describing a mistake.
	constexpr double kMinLaneWidthCm = 60.0;
	constexpr double kMaxLaneWidthCm = 800.0;

	// The lane at a left→right index, or null. DetailedLanes is kept in
	// left→right order by EnsureDetailedLanes and RelayoutLanes, so the index
	// the editor hands back is a direct index — the same assumption
	// InsertLaneRelative and CycleLaneType already make.
	FRoadNetLane* LaneAt(FRoadNetLaneSpec& L, int32 LaneLtoR)
	{
		return L.DetailedLanes.IsValidIndex(LaneLtoR) ? &L.DetailedLanes[LaneLtoR] : nullptr;
	}
}

bool URoadNetwork::SetLaneWidth(int32 RoadIdx, int32 LaneLtoR, double WidthCm)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	EnsureDetailedLanes(L, bDriveOnLeft);
	FRoadNetLane* Ln = LaneAt(L, LaneLtoR);
	if (!Ln) { return false; }

	Ln->Width = (float)FMath::Clamp(WidthCm, kMinLaneWidthCm, kMaxLaneWidthCm);
	RelayoutLanes(L.DetailedLanes, 0.0);
	SyncFlagsFromLanes(L);
	return true;
}

bool URoadNetwork::SetLaneType(int32 RoadIdx, int32 LaneLtoR, ERoadNetLaneType Type)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	EnsureDetailedLanes(L, bDriveOnLeft);
	FRoadNetLane* Ln = LaneAt(L, LaneLtoR);
	if (!Ln) { return false; }

	Ln->Type = Type;
	if (Type == ERoadNetLaneType::Shoulder
		|| Type == ERoadNetLaneType::Border || Type == ERoadNetLaneType::Parking
		|| Type == ERoadNetLaneType::Median || Type == ERoadNetLaneType::Sidewalk)
	{
		Ln->Direction = ERoadNetLaneDirection::None;
	}
	else if (Ln->Direction == ERoadNetLaneDirection::None)
	{
		Ln->Direction = (Type == ERoadNetLaneType::CenterTurn)
			? ERoadNetLaneDirection::Both : ERoadNetLaneDirection::FromSide;
	}
	RelayoutLanes(L.DetailedLanes, 0.0);
	SyncFlagsFromLanes(L);
	return true;
}

bool URoadNetwork::SetLaneTurnRole(int32 RoadIdx, int32 LaneLtoR, ERoadNetTurnRole Role)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	EnsureDetailedLanes(L, bDriveOnLeft);
	FRoadNetLane* Ln = LaneAt(L, LaneLtoR);
	if (!Ln) { return false; }
	Ln->TurnRole = Role;
	return true;
}

ERoadNetTurnRole URoadNetwork::CycleLaneTurnRole(int32 RoadIdx, int32 LaneLtoR, int32 Dir)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return ERoadNetTurnRole::Auto; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	EnsureDetailedLanes(L, bDriveOnLeft);
	FRoadNetLane* Ln = LaneAt(L, LaneLtoR);
	if (!Ln) { return ERoadNetTurnRole::Auto; }

	static const ERoadNetTurnRole kCycle[] = {
		ERoadNetTurnRole::Auto, ERoadNetTurnRole::Through, ERoadNetTurnRole::Left,
		ERoadNetTurnRole::Right, ERoadNetTurnRole::UTurn, ERoadNetTurnRole::None };
	constexpr int32 N = UE_ARRAY_COUNT(kCycle);
	int32 Cur = 0;
	for (int32 i = 0; i < N; ++i) { if (kCycle[i] == Ln->TurnRole) { Cur = i; break; } }
	const int32 Next = ((Cur + (Dir >= 0 ? 1 : -1)) % N + N) % N;
	Ln->TurnRole = kCycle[Next];
	return Ln->TurnRole;
}

namespace
{
	struct FRoadProj
	{
		int32 Road = INDEX_NONE;
		double Arc = 0.0;
		double SignedOff = 0.0;
		double D2 = TNumericLimits<double>::Max();
	};

	FRoadProj ProjectToNearestRoad(const TArray<FRoadDef>& Roads, const FVector& Probe, double MaxD2)
	{
		FRoadProj Best;
		Best.D2 = MaxD2;
		for (int32 r = 0; r < Roads.Num(); ++r)
		{
			const TArray<FVector>& Ref = Roads[r].Ref;
			double Acc = 0.0;
			for (int32 i = 0; i + 1 < Ref.Num(); ++i)
			{
				const FVector2D A(Ref[i].X, Ref[i].Y), B(Ref[i + 1].X, Ref[i + 1].Y);
				const FVector2D AB = B - A;
				const double Len2 = AB.SizeSquared();
				if (Len2 <= KINDA_SMALL_NUMBER) { continue; }
				const FVector2D Q(Probe.X, Probe.Y);
				const double T = FMath::Clamp(FVector2D::DotProduct(Q - A, AB) / Len2, 0.0, 1.0);
				const FVector2D P = A + AB * T;
				const double D2 = FVector2D::DistSquared(Q, P);
				if (D2 < Best.D2)
				{
					Best.D2 = D2;
					Best.Road = r;
					const double Len = FMath::Sqrt(Len2);
					Best.Arc = Acc + Len * T;
					const FVector2D Tan = AB / Len;
					const FVector2D Rt(Tan.Y, -Tan.X);
					Best.SignedOff = FVector2D::DotProduct(Q - P, Rt);
				}
				Acc += FMath::Sqrt(Len2);
			}
		}
		return Best;
	}

	ERoadNetTurnRole MovementToKind(RoadNetJunctions::EMovement Mv)
	{
		using EMovement = RoadNetJunctions::EMovement;
		switch (Mv)
		{
		case EMovement::Left:  return ERoadNetTurnRole::Left;
		case EMovement::Right: return ERoadNetTurnRole::Right;
		case EMovement::UTurn: return ERoadNetTurnRole::UTurn;
		default:               return ERoadNetTurnRole::Through;
		}
	}

	bool RoleAllowsKind(ERoadNetTurnRole Role, ERoadNetTurnRole Kind)
	{
		if (Role == ERoadNetTurnRole::None) { return false; }
		if (Role == ERoadNetTurnRole::Auto) { return true; }
		return Role == Kind;
	}

	FName CurbPaintHISMName(ERoadNetCurbPaintType Type)
	{
		switch (Type)
		{
		case ERoadNetCurbPaintType::WhiteBlack: return FName(TEXT("CurbWhiteBlack"));
		case ERoadNetCurbPaintType::WhiteRed:   return FName(TEXT("CurbWhiteRed"));
		case ERoadNetCurbPaintType::WhiteBlue:  return FName(TEXT("CurbWhiteBlue"));
		default:                                return FName(TEXT("CurbGray"));
		}
	}

	static const FName kCurbHISMKeys[] = {
		FName(TEXT("CurbA")), FName(TEXT("CurbB")),
		FName(TEXT("CurbGray")), FName(TEXT("CurbWhiteBlack")),
		FName(TEXT("CurbWhiteRed")), FName(TEXT("CurbWhiteBlue")) };

	double RingAreaCm2(const TArray<FVector>& Ring)
	{
		const int32 N = Ring.Num();
		if (N < 3) { return 0.0; }
		double A = 0.0;
		for (int32 i = 0; i < N; ++i)
		{
			const FVector& P = Ring[i];
			const FVector& Q = Ring[(i + 1) % N];
			A += P.X * Q.Y - Q.X * P.Y;
		}
		return FMath::Abs(A) * 0.5;
	}

	bool PointInRing2(const TArray<FVector2d>& V, const FVector2d& P)
	{
		bool bIn = false;
		const int32 N = V.Num();
		for (int32 i = 0, j = N - 1; i < N; j = i++)
		{
			const bool bCross = ((V[i].Y > P.Y) != (V[j].Y > P.Y));
			if (bCross)
			{
				const double X = (V[j].X - V[i].X) * (P.Y - V[i].Y) / (V[j].Y - V[i].Y + 1.e-12) + V[i].X;
				if (P.X < X) { bIn = !bIn; }
			}
		}
		return bIn;
	}
}

int32 URoadNetwork::AddPlacedIsland(const TArray<FVector>& Ring)
{
	if (Ring.Num() < 3) { return INDEX_NONE; }
	if (RingAreaCm2(Ring) < RoadNetStandards::MinIslandAreaCm2()) { return INDEX_NONE; }
	FRoadNetIsland Isl;
	Isl.Id = FGuid::NewGuid();
	Isl.Ring = Ring;
	Isl.SmoothCm = 150.f;
	return PlacedIslands.Add(Isl);
}

namespace
{
	bool SegSeg2(const FVector2D& A, const FVector2D& B, const FVector2D& C, const FVector2D& D, FVector2D& Out)
	{
		const FVector2D R = B - A, S = D - C;
		const double Den = R.X * S.Y - R.Y * S.X;
		if (FMath::IsNearlyZero(Den)) { return false; }
		const FVector2D QP = C - A;
		const double T = (QP.X * S.Y - QP.Y * S.X) / Den;
		const double U = (QP.X * R.Y - QP.Y * R.X) / Den;
		if (T < 0.0 || T > 1.0 || U < 0.0 || U > 1.0) { return false; }
		Out = A + R * T;
		return true;
	}

	void GestureRingHits(const TArray<FVector>& Gesture, const TArray<FVector>& Ring, TArray<FVector2D>& Hits)
	{
		Hits.Reset();
		if (Gesture.Num() < 2 || Ring.Num() < 3) { return; }
		for (int32 g = 0; g + 1 < Gesture.Num(); ++g)
		{
			const FVector2D A(Gesture[g].X, Gesture[g].Y), B(Gesture[g + 1].X, Gesture[g + 1].Y);
			for (int32 i = 0; i < Ring.Num(); ++i)
			{
				const FVector& P = Ring[i];
				const FVector& Q = Ring[(i + 1) % Ring.Num()];
				FVector2D H;
				if (SegSeg2(A, B, FVector2D(P.X, P.Y), FVector2D(Q.X, Q.Y), H))
				{
					Hits.AddUnique(H);
				}
			}
		}
	}
}

void URoadNetwork::TryCutIslandPaths(const TArray<FVector>& Gesture)
{
	if (Gesture.Num() < 2) { return; }
	Modify();
	for (FRoadNetIsland& Isl : PlacedIslands)
	{
		if (Isl.Ring.Num() < 3) { continue; }
		TArray<FVector2D> Hits;
		GestureRingHits(Gesture, Isl.Ring, Hits);

		TArray<FVector2d> Loop;
		for (const FVector& P : Isl.Ring) { Loop.Emplace(P.X, P.Y); }
		FVector2D Inside = FVector2D::ZeroVector;
		bool bInside = false;
		for (const FVector& P : Gesture)
		{
			if (PointInRing2(Loop, FVector2d(P.X, P.Y)))
			{
				Inside = FVector2D(P.X, P.Y);
				bInside = true;
				break;
			}
		}

		if (Hits.Num() < 2 && bInside)
		{
			// One crossing (or start inside): shoot through along the gesture.
			FVector2D Dir(Gesture.Last().X - Gesture[0].X, Gesture.Last().Y - Gesture[0].Y);
			if (!Dir.Normalize()) { continue; }
			const FVector2D Origin = Hits.Num() == 1 ? Hits[0] : Inside;
			const FVector2D Far = Origin + Dir * 20000.0;
			const FVector2D Near = Origin - Dir * 20000.0;
			TArray<FVector> Ray = {
				FVector(Near.X, Near.Y, 0), FVector(Far.X, Far.Y, 0) };
			GestureRingHits(Ray, Isl.Ring, Hits);
		}
		if (Hits.Num() < 2) { continue; }

		FRoadNetIslandPath Path;
		Path.A = Hits[0];
		Path.B = Hits[0];
		double Best = 0.0;
		for (int32 i = 1; i < Hits.Num(); ++i)
		{
			const double D = FVector2D::DistSquared(Hits[0], Hits[i]);
			if (D > Best) { Best = D; Path.B = Hits[i]; }
		}
		if (Best < FMath::Square(80.0)) { continue; }
		Path.WidthCm = 250.f;
		Isl.Paths.Add(Path);
	}
}

int32 URoadNetwork::AdjustIslandSmoothNear(const FVector& WorldHit, float DeltaCm)
{
	Modify();
	int32 Best = INDEX_NONE;
	double BestD2 = FMath::Square(4000.0);
	for (int32 i = 0; i < PlacedIslands.Num(); ++i)
	{
		const FRoadNetIsland& Isl = PlacedIslands[i];
		if (Isl.Ring.Num() < 3) { continue; }
		FVector C(0, 0, 0);
		for (const FVector& P : Isl.Ring) { C += P; }
		C /= (double)Isl.Ring.Num();
		const double D2 = FVector::DistSquaredXY(WorldHit, C);
		if (D2 < BestD2) { BestD2 = D2; Best = i; }
	}
	if (Best == INDEX_NONE) { return INDEX_NONE; }
	FRoadNetIsland& Isl = PlacedIslands[Best];
	Isl.SmoothCm = FMath::Clamp(Isl.SmoothCm + DeltaCm, 0.f, 800.f);
	return Best;
}

int32 URoadNetwork::AddCurbPaintNear(const FVector& WorldHit, ERoadNetCurbPaintType Type)
{
	Modify();
	EnsureTileRegistry();

	// Prefer the nearest existing kerb instance so island / junction stones paint
	// even when they are not a clean centreline projection.
	FVector At = WorldHit;
	double BestInstD2 = FMath::Square(600.0);
	for (TPair<FIntPoint, TWeakObjectPtr<ARoadNetTileActor>>& KV : TileActors)
	{
		ARoadNetTileActor* Tile = KV.Value.Get();
		if (!Tile) { continue; }
		for (const FName Key : kCurbHISMKeys)
		{
			UHierarchicalInstancedStaticMeshComponent* H = Tile->FindHISM(Key);
			if (!H) { continue; }
			const int32 N = H->GetInstanceCount();
			for (int32 i = 0; i < N; ++i)
			{
				FTransform Xf;
				if (!H->GetInstanceTransform(i, Xf, /*bWorldSpace*/true)) { continue; }
				const double D2 = FVector::DistSquaredXY(WorldHit, Xf.GetLocation());
				if (D2 < BestInstD2) { BestInstD2 = D2; At = Xf.GetLocation(); }
			}
		}
	}

	const FRoadProj P = ProjectToNearestRoad(Roads, At, FMath::Square(4000.0));
	if (BestInstD2 >= FMath::Square(600.0) && P.Road == INDEX_NONE) { return 0; }

	FRoadNetCurbPaint Sample;
	Sample.Id = FGuid::NewGuid();
	Sample.Road = P.Road;
	Sample.Side = (P.SignedOff >= 0.0) ? ERoadNetSide::Right : ERoadNetSide::Left;
	Sample.DistanceCm = (float)P.Arc;
	Sample.Type = Type;
	Sample.World = At;

	constexpr float kMergeCm = 120.f;
	for (FRoadNetCurbPaint& Existing : CurbPaints)
	{
		if (FVector::DistSquaredXY(Existing.World, Sample.World) <= FMath::Square((double)kMergeCm)
			|| (Existing.Road == Sample.Road && Existing.Road != INDEX_NONE
				&& Existing.Side == Sample.Side
				&& FMath::Abs(Existing.DistanceCm - Sample.DistanceCm) <= kMergeCm))
		{
			Existing.Type = Type;
			Existing.World = At;
			return RebucketCurbPaint();
		}
	}
	CurbPaints.Add(Sample);
	return RebucketCurbPaint();
}

bool URoadNetwork::LookupCurbPaint(const FVector& WorldLoc, ERoadNetCurbPaintType& OutType) const
{
	int32 Best = INDEX_NONE;
	double BestD2 = FMath::Square(500.0);
	for (int32 i = 0; i < CurbPaints.Num(); ++i)
	{
		const FRoadNetCurbPaint& S = CurbPaints[i];
		if (S.World.IsNearlyZero()) { continue; }
		const double D2 = FVector::DistSquaredXY(S.World, WorldLoc);
		if (D2 < BestD2) { BestD2 = D2; Best = i; }
	}
	if (Best != INDEX_NONE)
	{
		OutType = CurbPaints[Best].Type;
		return true;
	}

	const FRoadProj P = ProjectToNearestRoad(Roads, WorldLoc, FMath::Square(4000.0));
	if (P.Road == INDEX_NONE) { return false; }
	const ERoadNetSide Side = (P.SignedOff >= 0.0) ? ERoadNetSide::Right : ERoadNetSide::Left;
	float BestArc = 500.f;
	for (int32 i = 0; i < CurbPaints.Num(); ++i)
	{
		const FRoadNetCurbPaint& S = CurbPaints[i];
		if (S.Road != P.Road || S.Side != Side) { continue; }
		const float D = FMath::Abs(S.DistanceCm - (float)P.Arc);
		if (D < BestArc) { BestArc = D; Best = i; }
	}
	if (Best == INDEX_NONE) { return false; }
	OutType = CurbPaints[Best].Type;
	return true;
}

int32 URoadNetwork::RebucketCurbPaint()
{
	if (!WorldPtr.IsValid()) { return 0; }
	EnsureTileRegistry();

	UStaticMesh* Mesh = CurbMesh ? CurbMesh.Get() : nullptr;
	if (!Mesh) { Mesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Game/PCG/Assets/Meshes/SM_Curb2.SM_Curb2")); }
	if (!Mesh) { Mesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")); }
	if (!Mesh) { return 0; }

	auto MatFor = [this](ERoadNetCurbPaintType Type) -> UMaterialInterface*
	{
		UMaterialInterface* M = nullptr;
		switch (Type)
		{
		case ERoadNetCurbPaintType::WhiteBlack: M = CurbPaintWhiteBlack.Get(); break;
		case ERoadNetCurbPaintType::WhiteRed:   M = CurbPaintWhiteRed.Get(); break;
		case ERoadNetCurbPaintType::WhiteBlue:  M = CurbPaintWhiteBlue.Get(); break;
		default:                                M = CurbPaintGray.Get(); break;
		}
		if (M) { return M; }
		if (Type == ERoadNetCurbPaintType::WhiteBlack && MarkingWhiteMaterial) { return MarkingWhiteMaterial.Get(); }
		if (Type == ERoadNetCurbPaintType::WhiteRed && MarkingYellowMaterial) { return MarkingYellowMaterial.Get(); }
		if (Type == ERoadNetCurbPaintType::Gray && CurbMaterial1) { return CurbMaterial1.Get(); }
		if (CurbMaterial0) { return CurbMaterial0.Get(); }
		return LoadObject<UMaterialInterface>(nullptr,
			TEXT("/Engine/EngineMaterials/DefaultWhiteGrid.DefaultWhiteGrid"));
	};

	int32 Painted = 0;
	for (TPair<FIntPoint, TWeakObjectPtr<ARoadNetTileActor>>& KV : TileActors)
	{
		ARoadNetTileActor* Tile = KV.Value.Get();
		if (!Tile) { continue; }

		TArray<FTransform> Insts;
		for (const FName Key : kCurbHISMKeys)
		{
			if (UHierarchicalInstancedStaticMeshComponent* H = Tile->FindHISM(Key))
			{
				const int32 N = H->GetInstanceCount();
				for (int32 i = 0; i < N; ++i)
				{
					FTransform Xf;
					if (H->GetInstanceTransform(i, Xf, /*bWorldSpace*/true)) { Insts.Add(Xf); }
				}
				H->ClearInstances();
			}
		}
		if (Insts.Num() == 0) { continue; }

		TMap<UHierarchicalInstancedStaticMeshComponent*, TArray<FTransform>> Batches;
		int32 iCurb = 0;
		for (const FTransform& Xf : Insts)
		{
			ERoadNetCurbPaintType Paint;
			UHierarchicalInstancedStaticMeshComponent* H = nullptr;
			if (LookupCurbPaint(Xf.GetLocation(), Paint))
			{
				UMaterialInterface* PaintMat = MatFor(Paint);
				H = Tile->GetOrCreateHISM(CurbPaintHISMName(Paint), Mesh, PaintMat);
				if (H && PaintMat)
				{
					const int32 Slots = Mesh->GetStaticMaterials().Num();
					for (int32 s = 0; s < FMath::Max(1, Slots); ++s) { H->SetMaterial(s, PaintMat); }
				}
				++Painted;
			}
			else
			{
				const bool bA = ((iCurb++ & 1) == 0);
				H = Tile->GetOrCreateHISM(bA ? FName(TEXT("CurbA")) : FName(TEXT("CurbB")),
					Mesh, bA ? CurbMaterial0.Get() : CurbMaterial1.Get());
			}
			if (H) { Batches.FindOrAdd(H).Add(Xf); }
		}
		for (TPair<UHierarchicalInstancedStaticMeshComponent*, TArray<FTransform>>& B : Batches)
		{
			B.Key->AddInstances(B.Value, /*bShouldReturnIndices*/false, /*bWorldSpace*/true);
		}
	}
	return Painted;
}

bool URoadNetwork::SetLaneDirection(int32 RoadIdx, int32 LaneLtoR, ERoadNetLaneDirection Dir)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	EnsureDetailedLanes(L, bDriveOnLeft);
	FRoadNetLane* Ln = LaneAt(L, LaneLtoR);
	if (!Ln) { return false; }

	Ln->Direction = Dir;
	return true;
}

bool URoadNetwork::RemoveLaneAt(int32 RoadIdx, int32 LaneLtoR)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	EnsureDetailedLanes(L, bDriveOnLeft);
	if (!L.DetailedLanes.IsValidIndex(LaneLtoR) || L.DetailedLanes.Num() <= 1) { return false; }

	L.DetailedLanes.RemoveAt(LaneLtoR);
	RelayoutLanes(L.DetailedLanes, 0.0);
	SyncFlagsFromLanes(L);
	return true;
}

// Self-check for the lane setters and the junction rules: run `RoadNet.LaneSelfCheck`
// from the console.
//
// Two families of invariant, both expensive to debug any other way.
//
// CONTIGUITY — lane i's right edge lands exactly on lane i+1's left edge. The
// cross-section editor's boundary picking and its "trade width between neighbours" drag
// both assume it, and if a relayout ever left a gap the drag would grab a boundary that
// is not where it is drawn, which is a bug you would chase in the UI for an afternoon
// before suspecting the model.
//
// JUNCTION RULES (כרך 2) — lane continuity across a junction, the drop landing on the
// rightmost lane, the Table 5.1 taper length, and the junction grade bounds. These are
// geometric facts about a whole network, so the alternative to asserting them is
// eyeballing junctions in the viewport and hoping.
void URoadNetwork::RunSelfCheck()
{
	bool bOK = true;
	auto Expect = [&bOK](bool bCond, const TCHAR* What)
	{
		if (!bCond)
		{
			bOK = false;
			UE_LOG(LogRoadNet, Error, TEXT("LaneSelfCheck FAILED: %s"), What);
		}
	};

	// ---- part 1: the cross-section lane setters -----------------------------
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef R;
		R.Ref = { FVector::ZeroVector, FVector(10000.0, 0.0, 0.0) };
		R.Lanes.Total = 4;
		const int32 Idx = Net->AddRoad(R);

		auto Contiguous = [Net, Idx]()
		{
			const TArray<FRoadNetLane> Ls = Net->GetLanesLeftToRight(Idx);
			for (int32 i = 1; i < Ls.Num(); ++i)
			{
				const double PrevHi = Ls[i - 1].CenterOffset + 0.5 * Ls[i - 1].Width;
				const double ThisLo = Ls[i].CenterOffset - 0.5 * Ls[i].Width;
				if (FMath::Abs(PrevHi - ThisLo) > 0.5) { return false; }
			}
			return Ls.Num() > 0;
		};

		Expect(Net->GetLanesLeftToRight(Idx).Num() == 4, TEXT("a 4-lane road resolves to 4 lanes"));
		Expect(Contiguous(), TEXT("lanes are contiguous before any edit"));

		Expect(Net->SetLaneWidth(Idx, 0, 1.0), TEXT("SetLaneWidth accepts lane 0"));
		Expect(Net->GetLanesLeftToRight(Idx)[0].Width >= 59.9f, TEXT("a sub-minimum width clamps up"));
		Expect(Contiguous(), TEXT("lanes stay contiguous after a width clamp"));

		Expect(Net->SetLaneWidth(Idx, 3, 99999.0), TEXT("SetLaneWidth accepts the last lane"));
		Expect(Net->GetLanesLeftToRight(Idx)[3].Width <= 800.1f, TEXT("an over-maximum width clamps down"));
		Expect(Contiguous(), TEXT("lanes stay contiguous after a wide lane"));

		Expect(!Net->SetLaneWidth(Idx, 99, 350.0), TEXT("a bad lane index is refused"));
		Expect(!Net->SetLaneWidth(999, 0, 350.0), TEXT("a bad road index is refused"));

		// Typing a lane Median keeps it in the stack as a real slot. The whole
		// carriageway restacks around it, which is what keeps markings on their
		// lane boundaries instead of sliding relative to a centre-pinned gap.
		const int32 LanesBefore = Net->GetLanesLeftToRight(Idx).Num();
		const float TakenWidth = Net->GetLanesLeftToRight(Idx)[1].Width;
		const double OffBefore0 = Net->GetLanesLeftToRight(Idx)[0].CenterOffset;
		const double OffBeforeLast = Net->GetLanesLeftToRight(Idx).Last().CenterOffset;
		Expect(Net->SetLaneType(Idx, 1, ERoadNetLaneType::Median), TEXT("SetLaneType accepts a median"));
		Expect(Net->IsMedian(Idx), TEXT("typing a lane Median turns the road's median on"));
		Expect(FMath::IsNearlyEqual(Net->GetMedianWidth(Idx), TakenWidth, 0.5f),
			TEXT("the median inherits the width of the lane it replaced"));
		Expect(Net->GetLanesLeftToRight(Idx).Num() == LanesBefore,
			TEXT("the median stays in the stack as a lane slot"));
		Expect(Net->GetLanesLeftToRight(Idx)[1].Type == ERoadNetLaneType::Median,
			TEXT("lane 1 is the median slot"));
		Expect(Net->GetLanesLeftToRight(Idx)[0].CenterOffset < OffBefore0,
			TEXT("the left carriageway moves out when a median is typed"));
		Expect(Net->GetLanesLeftToRight(Idx).Last().CenterOffset > OffBeforeLast,
			TEXT("the right carriageway moves out when a median is typed"));

		auto MedianLane = [Net, Idx]() -> FRoadNetLane
		{
			for (const FRoadNetLane& Ln : Net->GetLanesLeftToRight(Idx))
			{
				if (Ln.Type == ERoadNetLaneType::Median) { return Ln; }
			}
			return FRoadNetLane();
		};

		const int32 NumLanes = Net->GetLanesLeftToRight(Idx).Num();
		for (int32 i = 0; i < NumLanes; ++i)
		{
			if (Net->GetLanesLeftToRight(Idx)[i].Type != ERoadNetLaneType::Median)
			{
				Net->SetLaneWidth(Idx, i, 350.0);
			}
		}
		Net->SetMedianWidth(Idx, 800.f);
		{
			const FRoadNetLane M = MedianLane();
			Expect(M.Type == ERoadNetLaneType::Median, TEXT("a median lane exists after SetMedianWidth"));
			Expect(FMath::IsNearlyEqual(M.Width, 800.f, 1.f),
				TEXT("a median wider than its lanes still occupies its own width in the stack"));
			Expect(FMath::IsNearlyEqual(M.CenterOffset, 0.0, 1.0),
				TEXT("the median slot straddles the reference line, where the 3-D strip is built"));
		}

		Net->SetMedianWidth(Idx, 300.f);
		Expect(FMath::IsNearlyEqual(MedianLane().Width, 300.f, 1.f),
			TEXT("the median slot tracks the median width"));

		// The count model and the authored model must report the same half-width
		// for one road, or its sidewalk masks and terrain corridor shift the
		// moment it is materialised into authored lanes.
		{
			FRoadNetLaneSpec Spec;
			Spec.Total = 4;
			Spec.LaneWidthDefault = 350.f;
			Spec.bMedian = true;
			Spec.MedianWidth = 800.f;
			const float CountHalf = Spec.HalfWidthCm();
			EnsureDetailedLanes(Spec, /*bDriveOnLeft*/false);
			RelayoutLanes(Spec.DetailedLanes, 0.0);
			Expect(FMath::IsNearlyEqual(CountHalf, Spec.HalfWidthCm(), 1.f),
				TEXT("both lane models report the same half-width for one road"));
		}

		Expect(Net->SetLaneType(Idx, 1, ERoadNetLaneType::Normal), TEXT("SetLaneType accepts driving"));
		Expect(Net->GetLanesLeftToRight(Idx)[1].Direction != ERoadNetLaneDirection::None,
			TEXT("a lane that becomes drivable gets a direction back"));

		Expect(Net->SetLaneTurnRole(Idx, 1, ERoadNetTurnRole::Right), TEXT("SetLaneTurnRole persists"));
		Expect(Net->GetLanesLeftToRight(Idx)[1].TurnRole == ERoadNetTurnRole::Right,
			TEXT("turn role survives GetLanesLeftToRight"));
		Expect(Net->CycleLaneTurnRole(Idx, 1, 1) == ERoadNetTurnRole::UTurn,
			TEXT("CycleLaneTurnRole advances Right -> UTurn"));

		for (int32 i = 0; i < 3; ++i) { Net->RemoveLaneAt(Idx, 0); }
		Expect(Net->GetLanesLeftToRight(Idx).Num() == 1, TEXT("lanes can be removed down to one"));
		Expect(!Net->RemoveLaneAt(Idx, 0), TEXT("the last lane is refused"));
		Expect(Net->GetLanesLeftToRight(Idx).Num() == 1, TEXT("the refused removal changed nothing"));
	}

	// ---- part 1a2: a 2+2 separator stays coherent when a median is inserted ---
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef R;
		R.Ref = { FVector::ZeroVector, FVector(10000.0, 0.0, 0.0) };
		R.Lanes.Total = 4;
		R.Lanes.LaneWidthDefault = 350.f;
		const int32 Idx = Net->AddRoad(R);
		Net->SetLaneWidth(Idx, 0, 350.0);

		TArray<double> EdgeBefore;
		int32 DrivenBefore = 0;
		for (const FRoadNetLane& Ln : Net->GetLanesLeftToRight(Idx))
		{
			if (!Ln.bDrivable()) { continue; }
			++DrivenBefore;
			EdgeBefore.Add(Ln.CenterOffset - 0.5 * Ln.Width);
			EdgeBefore.Add(Ln.CenterOffset + 0.5 * Ln.Width);
		}
		Expect(DrivenBefore == 4, TEXT("a 2+2 starts as four driven lanes"));

		Expect(Net->ToggleMedian(Idx), TEXT("ToggleMedian inserts a median slot"));
		Net->SetMedianWidth(Idx, 300.f);

		TArray<double> EdgeAfter;
		int32 DrivenAfter = 0;
		bool bHasMedian = false;
		for (const FRoadNetLane& Ln : Net->GetLanesLeftToRight(Idx))
		{
			if (Ln.Type == ERoadNetLaneType::Median)
			{
				bHasMedian = true;
				Expect(FMath::IsNearlyEqual(Ln.Width, 300.f, 1.f), TEXT("the median slot is 3 m"));
				continue;
			}
			if (!Ln.bDrivable()) { continue; }
			++DrivenAfter;
			EdgeAfter.Add(Ln.CenterOffset - 0.5 * Ln.Width);
			EdgeAfter.Add(Ln.CenterOffset + 0.5 * Ln.Width);
		}
		Expect(bHasMedian, TEXT("the stack carries a median lane"));
		Expect(DrivenAfter == DrivenBefore, TEXT("driven lane count is unchanged"));
		Expect(EdgeAfter.Num() == EdgeBefore.Num(), TEXT("every driven edge still exists"));
		const double Shift = 150.0;
		for (int32 i = 0; i < EdgeBefore.Num(); ++i)
		{
			const double Expected = (EdgeBefore[i] < 0.0) ? (EdgeBefore[i] - Shift) : (EdgeBefore[i] + Shift);
			Expect(FMath::IsNearlyEqual(EdgeAfter[i], Expected, 1.5),
				TEXT("every driven edge moved by half the median, and stayed on its lane"));
		}
	}

	// ---- part 1b: the UK / drive-on-left flip -------------------------------
	// Regression: assigning bDriveOnLeft used to BE the flip, which does nothing at all
	// on a road with authored lanes, because ResolveLanes hands DetailedLanes back
	// untouched. Assert the flip reaches the stored lanes and leaves the stack intact.
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef R;
		R.Ref = { FVector::ZeroVector, FVector(10000.0, 0.0, 0.0) };
		R.Lanes.Total = 4;
		const int32 Idx = Net->AddRoad(R);
		Net->SetLaneWidth(Idx, 0, 350.0);   // any lane edit materialises DetailedLanes

		// Where forward traffic sits, summed. Sign says which side of the reference line.
		auto ForwardSide = [Net, Idx]()
		{
			double Sum = 0.0;
			for (const FRoadNetLane& L : Net->GetLanesLeftToRight(Idx))
			{
				if (L.Direction == ERoadNetLaneDirection::Forward) { Sum += L.CenterOffset; }
			}
			return Sum;
		};

		// The constructor seeds handedness from roadnet.DriveOnLeft, so pin it first
		// rather than assuming a fresh network drives on the right.
		Net->SetDriveOnLeft(false);
		const double RightHand = ForwardSide();
		Expect(Net->SetDriveOnLeft(true), TEXT("a real handedness change reports true"));
		const double LeftHand = ForwardSide();
		Expect(RightHand * LeftHand < 0.0,
			TEXT("a UK flip moves forward traffic to the other side of the reference line"));
		Expect(!Net->SetDriveOnLeft(true), TEXT("flipping to the handedness already set is a no-op"));

		const TArray<FRoadNetLane> Flipped = Net->GetLanesLeftToRight(Idx);
		bool bContiguous = Flipped.Num() == 4;
		for (int32 i = 1; i < Flipped.Num(); ++i)
		{
			const double PrevHi = Flipped[i - 1].CenterOffset + 0.5 * Flipped[i - 1].Width;
			const double ThisLo = Flipped[i].CenterOffset - 0.5 * Flipped[i].Width;
			if (FMath::Abs(PrevHi - ThisLo) > 0.5) { bContiguous = false; }
		}
		Expect(bContiguous, TEXT("lanes stay contiguous after a UK flip"));
	}

	// ---- part 1d: a parking bay stays out of the junction, and on one kerb ---
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->ParkingBayJunctionSetbackCm = 1200.f;
		Net->ParkingStallWidthCm = 250.f;
		Net->ParkingBayLengthCm  = 0.f;   // as long as the road allows

		// Cross streets meeting the main road at x = 0 and x = 2000, so both of its
		// ends are real junctions rather than free ends. GetJunctionEnds finds them
		// by the same endpoint weld the rebuild uses, which is what lets a bay
		// added from the panel — with no rebuild in between — know they are there.
		auto AddCross = [&Net](double X)
		{
			FRoadDef C;
			C.Ref = { FVector(X, 0.0, 0.0), FVector(X, 5000.0, 0.0) };
			C.Lanes.Total = 2;
			Net->AddRoad(C);
		};
		AddCross(0.0);
		AddCross(2000.0);

		// 20 m of road between two junctions: 12 m of setback at each end leaves
		// nothing. This used to fall back to a full-length bay with no setback at
		// all, parking cars in the junction it was supposed to avoid.
		FRoadDef Short;
		Short.Ref = { FVector::ZeroVector, FVector(2000.0, 0.0, 0.0) };
		Short.Lanes.Total = 2;
		const int32 ShortIdx = Net->AddRoad(Short);
		{
			bool bS = false, bE = false;
			Net->GetJunctionEnds(ShortIdx, bS, bE);
			Expect(bS && bE, TEXT("both ends of the short road are seen as junctions"));
		}
		Expect(Net->AddStandardParkingBay(ShortIdx, ERoadNetSide::Right,
			ERoadNetParkingLayout::Perpendicular, -1.0) == INDEX_NONE,
			TEXT("a road too short to clear its junctions gets no parking bay"));
		Expect(Net->GetRoads()[ShortIdx].ParkingBays.Num() == 0,
			TEXT("a refused parking bay leaves the road untouched"));

		// 100 m of road from the junction at x = 0 to a free end: a bay fits, on
		// the requested kerb only, set back from the junction but not from the end.
		FRoadDef Long;
		Long.Ref = { FVector::ZeroVector, FVector(10000.0, 0.0, 0.0) };
		Long.Lanes.Total = 2;
		const int32 LongIdx = Net->AddRoad(Long);
		const int32 BayIdx = Net->AddStandardParkingBay(LongIdx, ERoadNetSide::Right,
			ERoadNetParkingLayout::Perpendicular, -1.0);
		Expect(BayIdx != INDEX_NONE, TEXT("a long enough road accepts a parking bay"));
		Expect(Net->GetRoads()[LongIdx].ParkingBays.Num() == 1,
			TEXT("asking for one parking bay adds one, not one per kerb"));
		if (Net->GetRoads()[LongIdx].ParkingBays.Num() == 1)
		{
			const FRoadNetParkingBay& B = Net->GetRoads()[LongIdx].ParkingBays[0];
			Expect(B.Side == ERoadNetSide::Right, TEXT("the bay lands on the requested kerb"));
			Expect(B.StartArcCm >= 1200.f - 1.f,
				TEXT("the bay stays clear of the junction at the road's start"));
			Expect(B.StartArcCm + B.LengthCm <= 10000.f + 1.f,
				TEXT("the bay stays on the road"));
		}
	}

	// ---- part 1c: stop bars follow traffic, not arm geometry ----------------
	// Regression: BuildJoint striped EVERY arm, because FApproach carried no
	// direction data at all — so a one-way exit arm, which no traffic enters, got a
	// full stop bar facing nothing.
	{
		using namespace RoadNetJunctionMarks;
		TArray<UE::Geometry::FGeneralPolygon2d> White;
		TArray<FSignal> Signals;

		FApproach Ap;
		Ap.StopPos = FVector2D::ZeroVector;
		Ap.Outward = FVector2D(1, 0);      // so Rin (the lateral axis) is +Y
		Ap.HalfWidthCm = 700.0;

		TArray<FApproach> Arms;
		Arms.Add(Ap);
		Arms[0].bHasEnteringTraffic = false;
		BuildJoint(FVector2D::ZeroVector, 0.0, Arms, ERoadNetJunctionPreset::StopLine,
			/*bDriveOnLeft*/false, White, Signals);
		Expect(White.Num() == 0, TEXT("a one-way exit arm emits no stop bar"));
		Expect(Signals.Num() == 0, TEXT("a one-way exit arm gets no signal head"));

		// A divided two-way arm: the bar covers the entering lanes only, starting at
		// the median edge. Measured from the reference line it would begin inside the
		// median, which is the "bar crosses the median" half of the same bug.
		White.Reset(); Signals.Reset();
		Arms[0].bHasEnteringTraffic = true;
		Arms[0].EnterLoCm = 150.0;   // median edge
		Arms[0].EnterHiCm = 850.0;   // kerb
		BuildJoint(FVector2D::ZeroVector, 0.0, Arms, ERoadNetJunctionPreset::StopLine,
			/*bDriveOnLeft*/false, White, Signals);
		Expect(White.Num() == 1, TEXT("a two-way arm emits exactly one stop bar"));
		if (White.Num() == 1)
		{
			double Lo = TNumericLimits<double>::Max(), Hi = -TNumericLimits<double>::Max();
			for (const FVector2d& V : White[0].GetOuter().GetVertices())
			{
				Lo = FMath::Min(Lo, V.Y); Hi = FMath::Max(Hi, V.Y);
			}
			Expect(FMath::IsNearlyEqual(Lo, 150.0, 1.0) && FMath::IsNearlyEqual(Hi, 850.0, 1.0),
				TEXT("the bar spans the entering lanes, starting at the median edge"));
		}

		// A driver must meet the bar BEFORE the zebra, or they stop with the bonnet
		// over the crossing. Outward is +X here, so "further out" is a larger X: the
		// bar's near edge has to be past the zebra's far edge.
		White.Reset(); Signals.Reset();
		BuildJoint(FVector2D::ZeroVector, 0.0, Arms, ERoadNetJunctionPreset::StopAndCrosswalk,
			/*bDriveOnLeft*/false, White, Signals);

		// The bar is the one polygon spanning the entering lanes; every zebra stripe
		// is 50 cm across. Split them on that, then compare their extents along X.
		double BarNearX   = TNumericLimits<double>::Max();
		double ZebraNearX = TNumericLimits<double>::Max();
		double ZebraFarX  = -TNumericLimits<double>::Max();
		int32  Bars = 0, Stripes = 0;
		for (const UE::Geometry::FGeneralPolygon2d& P : White)
		{
			double X0 = TNumericLimits<double>::Max(), X1 = -TNumericLimits<double>::Max();
			double Y0 = TNumericLimits<double>::Max(), Y1 = -TNumericLimits<double>::Max();
			for (const FVector2d& V : P.GetOuter().GetVertices())
			{
				X0 = FMath::Min(X0, V.X); X1 = FMath::Max(X1, V.X);
				Y0 = FMath::Min(Y0, V.Y); Y1 = FMath::Max(Y1, V.Y);
			}
			if (Y1 - Y0 > 200.0)
			{
				++Bars;
				BarNearX = FMath::Min(BarNearX, X0);
			}
			else
			{
				++Stripes;
				ZebraNearX = FMath::Min(ZebraNearX, X0);
				ZebraFarX  = FMath::Max(ZebraFarX, X1);
			}
		}
		Expect(Bars == 1, TEXT("a crossing approach still emits exactly one stop bar"));
		Expect(Stripes > 1, TEXT("a crossing approach emits a zebra band"));
		Expect(Bars == 1 && Stripes > 1 && BarNearX > ZebraFarX,
			TEXT("the stop bar sits behind the crosswalk, not on top of it"));
		// The band is as long as the setting says, so the knob is really wired
		// through rather than the paint keeping its own constant.
		Expect(Stripes > 1 && FMath::IsNearlyEqual(ZebraFarX - ZebraNearX,
				RoadNetJunctionMarks::CrosswalkLengthCm(), 1.0),
			TEXT("the crosswalk is as long as roadnet.CrosswalkLengthCm asks for"));
	}

	// ---- part 2: the standards tables --------------------------------------
	{
		using namespace RoadNetStandards;

		Expect(DesignSpeedKph(ERoadNetClass::Residential, 0) == 50,
			TEXT("a road with no authored speed falls back to its class default"));
		Expect(DesignSpeedKph(ERoadNetClass::Residential, 80) == 80,
			TEXT("an authored speed outranks the class default"));
		Expect(DesignSpeedKph(ERoadNetClass::Motorway, 0) > DesignSpeedKph(ERoadNetClass::Residential, 0),
			TEXT("a motorway is designed faster than a residential street"));

		// Table 5.1 — a 1:N taper closes a lane of width W over N*W.
		Expect(FMath::IsNearlyEqual(LaneDropTaperLengthCm(50, 350.0), 40.0 * 350.0, 1.0),
			TEXT("a 50 km/h lane drop tapers at 1:40 (Table 5.1)"));
		Expect(LaneDropTaperRatio(100) > LaneDropTaperRatio(50),
			TEXT("a faster road gets a longer, flatter lane-drop taper"));
		Expect(LeftTurnTaperRatio(60) < LeftTurnTaperRatio(90),
			TEXT("the left-turn taper stretches from 1:10 to 1:15 above 80 km/h"));

		Expect(FMath::IsNearlyEqual(ResultantGrade(0.03, 0.04), 0.05, 1e-6),
			TEXT("the resultant grade combines longitudinal and crossfall"));

		// §8.2.3's 1% is a drainage FLOOR, not a ceiling — the standard says
		// «לא יפחת מ-1%», "shall not fall below". This pair of assertions is
		// here because the rule reads like a ceiling (the quantity is named
		// "maximum resultant grade") and was in fact implemented as one.
		Expect(MinJunctionResultantGrade() < MaxArmGradeAtJunction(80),
			TEXT("the §8.2.3 drainage floor sits BELOW the Table 8.1 arm ceiling"));
		Expect(ResultantGrade(MaxArmGradeAtJunction(100), 0.0) > MinJunctionResultantGrade(),
			TEXT("an arm at its Table 8.1 max grade drains, i.e. clears the 1% floor"));
		Expect(MaxArmGradeAtJunction(100) < MaxArmGradeAtJunction(60),
			TEXT("a faster arm is held to a flatter grade through the junction"));
	}

	// ---- part 3: junction movement classification --------------------------
	{
		using namespace RoadNetJunctions;

		// A cross junction, its four arms pointing out along ±X and ±Y. A
		// vehicle arriving on the +X arm is travelling in the −X direction, and
		// Unreal is left-handed (+X forward, +Y right), so from that heading
		// the driver's right is −Y and their left is +Y.
		const double PlusX = 0.0, PlusY = HALF_PI, MinusX = PI, MinusY = -HALF_PI;

		Expect(Classify(PlusX, MinusX) == EMovement::Through,
			TEXT("opposite arms are a through movement"));
		Expect(Classify(PlusX, PlusX) == EMovement::UTurn,
			TEXT("leaving the way you came is a U-turn"));
		Expect(Classify(PlusX, MinusY) == EMovement::Right,
			TEXT("a cross junction's right turn is classified right"));
		Expect(Classify(PlusX, PlusY) == EMovement::Left,
			TEXT("a cross junction's left turn is classified left"));

		// Kerb ordering: rank 0 must be the lane against the kerb, which is the
		// rightmost lane driving on the right and the leftmost driving on the left.
		FRoadNetLaneSpec Spec;
		Spec.Total = 4;
		for (const bool bLeftHand : { false, true })
		{
			const TArray<FRoadNetLane> Lanes = Spec.ResolveLanes(bLeftHand);
			TArray<int32> Ordered;
			OrderLanesFromKerb(Lanes, /*bAtStart*/false, /*bEntering*/true, bLeftHand, Ordered);
			Expect(Ordered.Num() == 2, TEXT("half a 4-lane two-way road enters the joint"));
			if (Ordered.Num() == 2)
			{
				const double Kerb  = Lanes[Ordered[0]].CenterOffset;
				const double Inner = Lanes[Ordered[1]].CenterOffset;
				Expect(bLeftHand ? (Kerb < Inner) : (Kerb > Inner),
					TEXT("rank 0 is the kerb lane under both traffic handednesses"));
			}
		}
	}

	// ---- part 4: junction lane continuity and the rightmost drop -----------
	// A 4-lane road running head-on into a 2-lane one. Two lanes arrive, one
	// leaves: כרך 2 §5.2.1 forbids losing that lane inside the junction, so
	// §5.2.4 must carry it out and drop it on the rightmost lane beyond.
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());

		FRoadDef Wide;
		Wide.Ref = { FVector(-10000.0, 0.0, 0.0), FVector(0.0, 0.0, 0.0) };
		Wide.Lanes.Total = 4;
		Net->AddRoad(Wide);

		FRoadDef Narrow;
		Narrow.Ref = { FVector(0.0, 0.0, 0.0), FVector(10000.0, 0.0, 0.0) };
		Narrow.Lanes.Total = 2;
		const int32 NarrowIdx = Net->AddRoad(Narrow);

		for (const bool bLeftHand : { false, true })
		{
			Net->bDriveOnLeft = bLeftHand;

			FRoadNetRebuildContext Ctx;
			Net->BuildEndpointJoints(Ctx);
			Net->BuildJunctionChannelization(Ctx);

			// The shared node, as opposed to the two dead ends.
			const FRoadNetJoint* Shared = nullptr;
			for (const FRoadNetJoint& J : Ctx.Joints)
			{
				if (J.Arms.Num() == 2) { Shared = &J; break; }
			}
			Expect(Shared != nullptr, TEXT("the two roads weld into one shared joint"));
			if (!Shared) { continue; }

			Expect(Shared->Arms.IsValidIndex(Shared->MainA) && Shared->Arms.IsValidIndex(Shared->MainB),
				TEXT("two opposed arms elect a main axis (§3.1.4)"));
			Expect(Shared->Kind == ERoadNetJointKind::Split,
				TEXT("arms that disagree on lane count make a Split joint"));

			const FRoadNetArmWidening* W = nullptr;
			for (const FRoadNetArmWidening& Cand : Ctx.ArmWidenings)
			{
				if (Cand.Road == NarrowIdx) { W = &Cand; break; }
			}
			Expect(W != nullptr, TEXT("the surplus lane is carried onto the narrow arm, not swallowed"));
			if (!W) { continue; }

			Expect(W->bAtStart, TEXT("the widening sits at the end of the arm that touches the junction"));

			// §5.2.4 — the lane dropped is always the rightmost, so the extra
			// width grows on the kerb side. Forward traffic stacks on the +offset
			// side driving on the right and the −offset side driving on the left.
			Expect(W->Side == (bLeftHand ? ERoadNetSide::Left : ERoadNetSide::Right),
				TEXT("the dropped lane is the rightmost one (§5.2.4)"));

			// The widening reaches full width at the junction and nothing past
			// the taper — that is what "the drop happens OUTSIDE" means.
			const double Len = 10000.0;
			Expect(FMath::IsNearlyEqual(W->BulgeAt(0.0, Len), W->WidthCm, 1.0),
				TEXT("the arm is at full extra width where it meets the junction"));
			Expect(W->BulgeAt(W->TaperCm + 1.0, Len) <= 0.0,
				TEXT("the extra width is gone past the taper"));

			const double LaneW = 350.0;   // the default the 2-lane road resolves to
			Expect(FMath::IsNearlyEqual(W->TaperCm,
				RoadNetStandards::LaneDropTaperLengthCm(RoadNetStandards::DesignSpeedKph(ERoadNetClass::Residential, 0), LaneW), 1.0),
				TEXT("the taper is the Table 5.1 length for the design speed"));
			Expect(FMath::IsNearlyEqual(W->WidthCm, LaneW, 1.0),
				TEXT("exactly the surplus lane's worth of width is carried out"));
		}
	}

	// ---- part 1e: a tee ramps up onto the road it joins ---------------------
	// The stem of a tee ENDS on the through road's centreline, so the two never
	// weld — the through road runs straight past instead of ending there. That
	// left the stem with only its own dead-end joint, pinned to its draped ground
	// level, while the through road was lifted to the junction level. The join
	// was a vertical wall at the mouth of the tee.
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());

		// Through road held 5 m above the stem, both dead flat, so any Z the
		// grade pass produces is its own doing and not terrain noise.
		constexpr double kThroughZ = 500.0;
		FRoadDef Through;
		Through.Ref = { FVector(-10000.0, 0.0, kThroughZ), FVector(10000.0, 0.0, kThroughZ) };
		Through.Lanes.Total = 4;
		const int32 ThroughIdx = Net->AddRoad(Through);

		// The stem runs a metre past the centreline, the way a drawn or imported
		// one does, so the crossing is unambiguous rather than an exact endpoint
		// touch. Its far end is a genuine dead end and must stay one.
		FRoadDef Stem;
		Stem.Ref = { FVector(0.0, 10000.0, 0.0), FVector(0.0, -100.0, 0.0) };
		Stem.Lanes.Total = 2;
		const int32 StemIdx = Net->AddRoad(Stem);

		FRoadNetRebuildContext Ctx;
		Ctx.Pending = { ThroughIdx, StemIdx };
		Ctx.TestAgainst = Ctx.Pending;
		Net->BuildCurves(Ctx);
		Net->BuildEndpointJoints(Ctx);
		Net->BuildCrossings(Ctx);
		Net->BuildVerticalAlignment(Ctx);

		Expect(Ctx.Crossings.Num() > 0, TEXT("the stem crosses the through road it ends on"));

		const FRoadCurves* SC = Ctx.Curves.Find(StemIdx);
		Expect(SC != nullptr && SC->Sampled.Num() >= 2, TEXT("the stem has a profile"));
		if (SC && SC->Sampled.Num() >= 2)
		{
			// The mouth must sit at the junction, not 5 m below it.
			Expect(FMath::IsNearlyEqual(SC->Sampled.Last().Z, kThroughZ, 1.0),
				TEXT("the stem meets the through road at the junction's level"));

			// And it must get there by ramping, not by jumping: no step between
			// neighbouring samples bigger than the grade cap allows.
			double WorstStepCm = 0.0, WorstGrade = 0.0;
			for (int32 i = 1; i < SC->Sampled.Num(); ++i)
			{
				const double dZ = FMath::Abs(SC->Sampled[i].Z - SC->Sampled[i - 1].Z);
				const double dS = FMath::Max(1.0, FVector::Dist2D(SC->Sampled[i - 1], SC->Sampled[i]));
				WorstStepCm = FMath::Max(WorstStepCm, dZ);
				WorstGrade  = FMath::Max(WorstGrade,  dZ / dS);
			}
			Expect(WorstGrade <= 0.13,
				TEXT("the stem ramps onto the junction instead of stepping up to it"));
			UE_LOG(LogRoadNet, Display,
				TEXT("  tee ramp: mouth %.0f cm, worst sample step %.0f cm (%.1f%% grade)"),
				SC->Sampled.Last().Z, WorstStepCm, WorstGrade * 100.0);
		}

		// The through road is the one that keeps its own level (כרך 2 §8.5.3).
		const FRoadCurves* TC = Ctx.Curves.Find(ThroughIdx);
		if (TC && TC->Sampled.Num() >= 2)
		{
			Expect(FMath::IsNearlyEqual(TC->Sampled[0].Z, kThroughZ, 1.0)
				&& FMath::IsNearlyEqual(TC->Sampled.Last().Z, kThroughZ, 1.0),
				TEXT("the through road holds its own level through the tee"));
		}
	}

	// Packed XY/Z knots and accidental cross-slope between parallel beds.
	{
		TArray<FVector> Packed = {
			FVector(0.0, 0.0, 0.0),
			FVector(2.0, 0.0, 200.0),
			FVector(1000.0, 0.0, 0.0)
		};
		const int32 Dropped = RoadNetMath::CollapsePackedSamples(Packed, 50.0, 0.12);
		Expect(Dropped == 1 && Packed.Num() == 2,
			TEXT("a 2 cm Z-cluster is dropped, leaving the two honest endpoints"));
		if (Packed.Num() == 2)
		{
			Expect(FMath::IsNearlyEqual(Packed[0].Z, 0.0, 0.5)
				&& FVector::Dist2D(Packed[0], Packed[1]) > 50.0,
				TEXT("the survivor of a packed Z-cluster keeps the previous Z, not the spike"));
		}
	}

	// Ring hygiene before triangulation: packed boundary clusters and collinear
	// slivers are dropped; an honest ring is untouched.
	{
		TArray<FVector2D> Dirty = {
			FVector2D(0.0, 0.0),
			FVector2D(10.0, 0.5),        // packed against the corner (10 cm)
			FVector2D(500.0, 0.5),       // collinear on the bottom edge (0.5 cm off)
			FVector2D(1000.0, 0.0),
			FVector2D(1000.0, 1000.0),
			FVector2D(0.0, 1000.0)
		};
		const int32 Removed = RoadNetMath::CleanPolygonRing(Dirty, 25.0, 1.5);
		Expect(Removed == 2 && Dirty.Num() == 4,
			TEXT("a packed corner cluster and a collinear sliver vertex are dropped from the ring"));

		TArray<FVector2D> Square = {
			FVector2D(0.0, 0.0), FVector2D(1000.0, 0.0),
			FVector2D(1000.0, 1000.0), FVector2D(0.0, 1000.0)
		};
		Expect(RoadNetMath::CleanPolygonRing(Square, 25.0, 1.5) == 0 && Square.Num() == 4,
			TEXT("an honest square ring is left alone"));
	}

	// § ease heights: a single spiked knot inside the radius is pulled to
	// within the step of its neighbours; a genuine grade over a segment longer
	// than the radius is left alone.
	{
		TArray<FVector> Spiked = {
			FVector(0.0, 0.0, 100.0),
			FVector(200.0, 0.0, 100.0),
			FVector(400.0, 0.0, 480.0),   // the rogue knot (+380 over 2 m)
			FVector(600.0, 0.0, 100.0),
			FVector(800.0, 0.0, 100.0)
		};
		EasePolylineZ(Spiked, /*bClosed*/false, /*MaxStepCm*/25.0, /*RadiusCm*/600.0);
		Expect(FMath::Abs(Spiked[2].Z - Spiked[1].Z) <= 25.5 &&
		       FMath::Abs(Spiked[3].Z - Spiked[2].Z) <= 25.5,
			TEXT("a spiked knot is eased to within the max step of both neighbours"));

		TArray<FVector> LongGrade = {
			FVector(0.0, 0.0, 100.0),
			FVector(1000.0, 0.0, 400.0)   // 3 m rise over 10 m — beyond the radius
		};
		Expect(EasePolylineZ(LongGrade, false, 25.0, 600.0) == 0 &&
		       LongGrade[1].Z == 400.0,
			TEXT("a grade across a segment longer than the ease radius is untouched"));
	}

	UE_LOG(LogRoadNet, Display, TEXT("LaneSelfCheck: %s"), bOK ? TEXT("PASS") : TEXT("FAIL"));
}

namespace
{
	FAutoConsoleCommand GLaneSelfCheckCmd(
		TEXT("RoadNet.LaneSelfCheck"),
		TEXT("Assert the lane setters' invariants (clamping, contiguity, the last-lane refusal) and the כרך 2 junction rules (lane continuity, the rightmost drop, Table 5.1 tapers, the 1% grade limit). Logs PASS or FAIL."),
		FConsoleCommandDelegate::CreateStatic(&URoadNetwork::RunSelfCheck));
}

namespace
{
	// Reference-polyline arc length (2-D, cm).
	double RefArcLength(const TArray<FVector>& Ref)
	{
		double L = 0.0;
		for (int32 i = 1; i < Ref.Num(); ++i) { L += FVector::Dist2D(Ref[i - 1], Ref[i]); }
		return L;
	}

	// Evenly spaced flat knot count for an outer-edge profile: ~1 knot / 15 m,
	// clamped so short roads still get a handful and long roads stay editable.
	int32 EdgeKnotCountFor(double LengthCm)
	{
		return FMath::Clamp(FMath::FloorToInt(LengthCm / 1500.0) + 1, 3, 24);
	}

	TArray<FRoadNetEdgeKnot>& OuterEdgeSide(FRoadDef& R, ERoadNetSide Side)
	{
		return (Side == ERoadNetSide::Left) ? R.OuterEdgeLeft : R.OuterEdgeRight;
	}
}

void URoadNetwork::GetOuterEdgeForDisplay(int32 RoadIdx, ERoadNetSide Side, TArray<FRoadNetEdgeKnot>& Out) const
{
	Out.Reset();
	if (!Roads.IsValidIndex(RoadIdx)) { return; }
	const FRoadDef& R = Roads[RoadIdx];
	const TArray<FRoadNetEdgeKnot>& Existing = (Side == ERoadNetSide::Left) ? R.OuterEdgeLeft : R.OuterEdgeRight;
	if (Existing.Num() > 0) { Out = Existing; return; }

	// Synthesize a flat profile at the uniform ±HalfWidth so the Edge tool has
	// handles even before the road is edited (materialised on first drag).
	const double Len  = RefArcLength(R.Ref);
	const double Half = FMath::Max(50.0, (double)R.Lanes.HalfWidthCm());
	const double Sign = (Side == ERoadNetSide::Left) ? -1.0 : +1.0;
	const int32  N    = EdgeKnotCountFor(Len);
	Out.SetNum(N);
	for (int32 k = 0; k < N; ++k)
	{
		Out[k].Distance = (N > 1) ? (Len * k / (N - 1)) : 0.0;
		Out[k].Offset   = Sign * Half;
	}
}

bool URoadNetwork::EnsureOuterEdgeProfile(int32 RoadIdx, ERoadNetSide Side)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	TArray<FRoadNetEdgeKnot>& Arr = OuterEdgeSide(Roads[RoadIdx], Side);
	if (Arr.Num() == 0) { GetOuterEdgeForDisplay(RoadIdx, Side, Arr); }
	return Arr.Num() > 0;
}

void URoadNetwork::SetOuterEdgeKnotOffset(int32 RoadIdx, ERoadNetSide Side, int32 KnotIdx, double Offset)
{
	if (!EnsureOuterEdgeProfile(RoadIdx, Side)) { return; }
	TArray<FRoadNetEdgeKnot>& Arr = OuterEdgeSide(Roads[RoadIdx], Side);
	if (!Arr.IsValidIndex(KnotIdx)) { return; }
	// Keep the edge on its own side of the centreline (≥ +50 for Right, ≤ −50
	// for Left) so a drag can't fold the carriageway inside-out.
	Arr[KnotIdx].Offset = (Side == ERoadNetSide::Left)
		? FMath::Min(Offset, -50.0)
		: FMath::Max(Offset, +50.0);
}

int32 URoadNetwork::AddOuterEdgeKnot(int32 RoadIdx, ERoadNetSide Side, double Distance, double Offset)
{
	if (!EnsureOuterEdgeProfile(RoadIdx, Side)) { return INDEX_NONE; }
	TArray<FRoadNetEdgeKnot>& Arr = OuterEdgeSide(Roads[RoadIdx], Side);
	FRoadNetEdgeKnot K;
	K.Distance = FMath::Max(0.0, Distance);
	K.Offset   = (Side == ERoadNetSide::Left) ? FMath::Min(Offset, -50.0) : FMath::Max(Offset, +50.0);
	int32 Pos = 0;
	while (Pos < Arr.Num() && Arr[Pos].Distance < K.Distance) { ++Pos; }
	Arr.Insert(K, Pos);
	return Pos;
}

bool URoadNetwork::RemoveOuterEdgeKnot(int32 RoadIdx, ERoadNetSide Side, int32 KnotIdx)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	TArray<FRoadNetEdgeKnot>& Arr = OuterEdgeSide(Roads[RoadIdx], Side);
	if (!Arr.IsValidIndex(KnotIdx) || Arr.Num() <= 2) { return false; }
	Arr.RemoveAt(KnotIdx);
	return true;
}

bool URoadNetwork::ToggleMedian(int32 RoadIdx)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	EnsureDetailedLanes(L, bDriveOnLeft);

	int32 MedianIdx = INDEX_NONE;
	for (int32 i = 0; i < L.DetailedLanes.Num(); ++i)
	{
		if (L.DetailedLanes[i].Type == ERoadNetLaneType::Median) { MedianIdx = i; break; }
	}
	if (MedianIdx != INDEX_NONE)
	{
		L.DetailedLanes.RemoveAt(MedianIdx);
		L.bMedian = false;
	}
	else
	{
		FRoadNetLane M;
		M.LaneId = FGuid::NewGuid();
		M.Type = ERoadNetLaneType::Median;
		M.Direction = ERoadNetLaneDirection::None;
		M.Side = ERoadNetSide::Center;
		M.Width = FMath::Max(30.f, L.MedianWidth);
		int32 Mid = 0;
		for (int32 i = 0; i < L.DetailedLanes.Num(); ++i)
		{
			if (L.DetailedLanes[i].Type != ERoadNetLaneType::Sidewalk) { ++Mid; }
		}
		Mid = FMath::Clamp(Mid / 2, 0, L.DetailedLanes.Num());
		L.DetailedLanes.Insert(M, Mid);
		L.bMedian = true;
	}
	RelayoutLanes(L.DetailedLanes, 0.0);
	SyncFlagsFromLanes(L);
	return L.bMedian;
}

ERoadNetMedianEdge URoadNetwork::CycleMedianEdge(int32 RoadIdx, int32 Dir)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return ERoadNetMedianEdge::Plantable; }
	constexpr int32 N = 4; // Plantable, CurbOnly, SidewalkAndCurb, PlantableWalkCurb
	const int32 V = ((int32)Roads[RoadIdx].Lanes.MedianEdge + (Dir >= 0 ? 1 : N - 1)) % N;
	SetMedianEdge(RoadIdx, (ERoadNetMedianEdge)V);
	return Roads[RoadIdx].Lanes.MedianEdge;
}

void URoadNetwork::SetMedianEdge(int32 RoadIdx, ERoadNetMedianEdge Edge)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	L.bMedian = true;   // choosing an edge treatment implies a median
	L.MedianEdge = Edge;
	RestackForMedian(L);
}

float URoadNetwork::SetMedianWidth(int32 RoadIdx, float WidthCm)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return 0.f; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	L.MedianWidth = FMath::Clamp(WidthCm, 30.f, 2000.f);
	EnsureDetailedLanes(L, bDriveOnLeft);
	for (FRoadNetLane& Ln : L.DetailedLanes)
	{
		if (Ln.Type == ERoadNetLaneType::Median) { Ln.Width = L.MedianWidth; }
	}
	if (!L.bMedian)
	{
		L.bMedian = true;
		MigrateFlagsToLanes(L);
	}
	RelayoutLanes(L.DetailedLanes, 0.0);
	SyncFlagsFromLanes(L);
	return L.MedianWidth;
}

float URoadNetwork::AdjustMedianWidth(int32 RoadIdx, float DeltaCm)
{
	return Roads.IsValidIndex(RoadIdx)
		? SetMedianWidth(RoadIdx, Roads[RoadIdx].Lanes.MedianWidth + DeltaCm)
		: 0.f;
}

bool URoadNetwork::IsMedian(int32 RoadIdx) const
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	const FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	if (L.bMedian) { return true; }
	for (const FRoadNetLane& Ln : L.DetailedLanes)
	{
		if (Ln.Type == ERoadNetLaneType::Median) { return true; }
	}
	return false;
}

float URoadNetwork::GetMedianWidth(int32 RoadIdx) const
{
	if (!Roads.IsValidIndex(RoadIdx)) { return 0.f; }
	const FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	for (const FRoadNetLane& Ln : L.DetailedLanes)
	{
		if (Ln.Type == ERoadNetLaneType::Median) { return Ln.Width; }
	}
	return L.MedianWidth;
}

ERoadNetMedianEdge URoadNetwork::GetMedianEdge(int32 RoadIdx) const
{
	return Roads.IsValidIndex(RoadIdx)
		? Roads[RoadIdx].Lanes.MedianEdge : ERoadNetMedianEdge::Plantable;
}

double URoadNetwork::AdjustJunctionSmoothing(double DeltaCm)
{
	JunctionSmoothingCm = FMath::Clamp(JunctionSmoothingCm + DeltaCm, 0.0, 300.0);
	return JunctionSmoothingCm;
}

bool URoadNetwork::SetDriveOnLeft(bool bNewDriveOnLeft)
{
	if (bDriveOnLeft == bNewDriveOnLeft) { return false; }
	bDriveOnLeft = bNewDriveOnLeft;

	// Count-model roads need nothing: ResolveLanes mirrors them from the flag on every
	// read. Authored lanes are the problem — ResolveLanes returns DetailedLanes verbatim,
	// so their stored Direction keeps the old country's traffic forever. Re-deriving them
	// from the counts would fix the handedness by throwing away every per-lane edit, so
	// flip the stored Direction in place instead.
	//
	// The lane STACK deliberately does not move: where a road splits its carriageway is a
	// physical fact about that road, and changing country only changes which direction
	// uses which side of the split. RelayoutLanes then re-derives Side from the offsets so
	// Side and Direction cannot drift apart.
	for (FRoadDef& R : Roads)
	{
		if (!R.Lanes.HasDetailedLanes()) { continue; }
		for (FRoadNetLane& L : R.Lanes.DetailedLanes)
		{
			// FromSide / Both / None already mean the same thing on either side of the
			// road (bTravelsForward folds bDriveOnLeft into the FromSide answer), so
			// touching them here would flip that traffic twice.
			if (L.Direction == ERoadNetLaneDirection::Forward)
			{
				L.Direction = ERoadNetLaneDirection::Backward;
			}
			else if (L.Direction == ERoadNetLaneDirection::Backward)
			{
				L.Direction = ERoadNetLaneDirection::Forward;
			}
		}
		RelayoutLanes(R.Lanes.DetailedLanes, 0.0);
		SyncFlagsFromLanes(R.Lanes);
	}
	return true;
}

URoadNetwork::URoadNetwork()
{
	// A network spawned by hand-drawing (rather than by an import that pushes
	// panel settings) still has to land in the right country, so seed handedness
	// from the console variable the panel keeps in step.
	if (IConsoleVariable* CVar = IConsoleManager::Get().FindConsoleVariable(TEXT("roadnet.DriveOnLeft")))
	{
		bDriveOnLeft = CVar->GetInt() != 0;
	}

	// Seed the street-furniture menu. Types default to DISABLED so a fresh
	// network (and city-scale OSM imports) stay clean — tick a type in the panel
	// to place it. With no mesh/Blueprint assigned each enabled type instances a
	// grey placeholder box so the layout reads before real assets are wired.
	auto MakeType = [](const TCHAR* InName, ERoadNetFurniturePlacement Place,
		float Spacing, float SideOffset, const FVector& Extent) -> FRoadNetFurnitureType
	{
		FRoadNetFurnitureType T;
		T.bEnabled            = false;
		T.Name                = FName(InName);
		T.Placement           = Place;
		T.SpacingCm           = Spacing;
		T.SideOffsetCm        = SideOffset;
		T.PlaceholderExtentCm = Extent;
		return T;
	};
	FurnitureTypes.Add(MakeType(TEXT("Bench"),     ERoadNetFurniturePlacement::SpacedPoints,  2500.f, 140.f, FVector(90.f,  35.f,  45.f)));
	FurnitureTypes.Add(MakeType(TEXT("GuardRail"), ERoadNetFurniturePlacement::Continuous,      400.f,  20.f, FVector(200.f,  8.f,  55.f)));
	FurnitureTypes.Add(MakeType(TEXT("BusStop"),   ERoadNetFurniturePlacement::SpacedPoints, 30000.f, 180.f, FVector(300.f, 120.f, 120.f)));
	FurnitureTypes.Add(MakeType(TEXT("Kiosk"),     ERoadNetFurniturePlacement::SpacedPoints, 20000.f, 200.f, FVector(200.f, 200.f, 250.f)));
}

void URoadNetwork::SetAllSidewalkWidth(float WidthCm)
{
	DefaultSidewalkWidthCm = FMath::Clamp(WidthCm, 0.f, 2000.f);
	for (FRoadDef& R : Roads)
	{
		R.Lanes.SidewalkWidth = DefaultSidewalkWidthCm;
		if (DefaultSidewalkWidthCm > 0.f)
		{
			R.Lanes.bSidewalkLeft  = true;
			R.Lanes.bSidewalkRight = true;
		}
	}
}

int32 URoadNetwork::SmoothAllRoads(float SimplifyTolCm, float CornerAngleDeg, float CornerMaxCutCm)
{
	const double Tol = FMath::Max(1.0, (double)SimplifyTolCm);
	const double CosLimit = FMath::Cos(FMath::DegreesToRadians(FMath::Clamp(CornerAngleDeg, 1.f, 90.f)));
	const double MaxCut = FMath::Max(50.0, (double)CornerMaxCutCm);

	// Points that carry topology are untouchable: a node id shared with another
	// road (or reused within one road, e.g. a loop) is a junction weld.
	TMap<int64, int32> NodeUse;
	for (const FRoadDef& R : Roads)
	{
		for (int64 Id : R.NodeIds) { if (Id >= 0) { NodeUse.FindOrAdd(Id)++; } }
	}

	// Iterative Ramer–Douglas–Peucker over Ref[A..B] (XY deviation), collecting
	// KEPT indices so the parallel NodeIds/Elev arrays can follow.
	auto RdpKeep = [Tol](const TArray<FVector>& P, int32 A, int32 B, TSet<int32>& Keep)
	{
		TArray<TPair<int32, int32>> Stack;
		Stack.Emplace(A, B);
		while (Stack.Num() > 0)
		{
			const TPair<int32, int32> Range = Stack.Pop();
			const int32 Lo = Range.Key, Hi = Range.Value;
			if (Hi - Lo < 2) { continue; }
			const FVector2D S(P[Lo].X, P[Lo].Y), E(P[Hi].X, P[Hi].Y);
			int32 Worst = INDEX_NONE;
			double WorstD = Tol;
			for (int32 i = Lo + 1; i < Hi; ++i)
			{
				double T;
				const FVector2D Q(P[i].X, P[i].Y);
				const FVector2D C = RoadNetMath::ClosestOnSegment(S, E, Q, T);
				const double D = FVector2D::Distance(Q, C);
				if (D > WorstD) { WorstD = D; Worst = i; }
			}
			if (Worst != INDEX_NONE)
			{
				Keep.Add(Worst);
				Stack.Emplace(Lo, Worst);
				Stack.Emplace(Worst, Hi);
			}
		}
	};

	int32 Changed = 0, PtsBefore = 0, PtsAfter = 0, Corners = 0;
	for (FRoadDef& R : Roads)
	{
		const int32 N = R.Ref.Num();
		if (N < 3) { continue; }
		const bool bIds = (R.NodeIds.Num() == N);

		auto IsProtected = [&](int32 i) -> bool
		{
			if (i == 0 || i == N - 1) { return true; }
			if (!bIds) { return false; }
			const int64 Id = R.NodeIds[i];
			return Id >= 0 && NodeUse.FindRef(Id) >= 2;
		};

		// 1) simplify each span between protected anchors.
		TSet<int32> Keep;
		int32 Anchor = 0;
		Keep.Add(0);
		for (int32 i = 1; i < N; ++i)
		{
			if (!IsProtected(i)) { continue; }
			Keep.Add(i);
			RdpKeep(R.Ref, Anchor, i, Keep);
			Anchor = i;
		}
		TArray<int32> Kept = Keep.Array();
		Kept.Sort();

		// 2) corner-cut kept UNPROTECTED points whose turn exceeds the limit:
		// replace the corner with two points pulled toward its neighbours (the
		// rebuild's G2 spline then rounds through the gap instead of kinking).
		TArray<FVector> NewRef;
		TArray<int64> NewIds;
		NewRef.Reserve(Kept.Num() + 8);
		NewIds.Reserve(Kept.Num() + 8);
		for (int32 k = 0; k < Kept.Num(); ++k)
		{
			const int32 i = Kept[k];
			const FVector P = R.Ref[i];
			const int64 Id = bIds ? R.NodeIds[i] : (int64)-1;
			bool bCut = false;
			if (k > 0 && k < Kept.Num() - 1 && !IsProtected(i))
			{
				const FVector& Pv = R.Ref[Kept[k - 1]];
				const FVector& Nx = R.Ref[Kept[k + 1]];
				FVector2D A(P.X - Pv.X, P.Y - Pv.Y);
				FVector2D B(Nx.X - P.X, Nx.Y - P.Y);
				const double LA = A.Size(), LB = B.Size();
				if (LA > 1.0 && LB > 1.0)
				{
					A /= LA; B /= LB;
					if (FVector2D::DotProduct(A, B) < CosLimit)   // sharper than limit
					{
						const double CutA = FMath::Min(MaxCut, 0.35 * LA);
						const double CutB = FMath::Min(MaxCut, 0.35 * LB);
						NewRef.Add(P + (Pv - P).GetSafeNormal() * CutA);
						NewIds.Add(-1);
						NewRef.Add(P + (Nx - P).GetSafeNormal() * CutB);
						NewIds.Add(-1);
						++Corners;
						bCut = true;
					}
				}
			}
			if (!bCut) { NewRef.Add(P); NewIds.Add(Id); }
		}

		if (NewRef.Num() < 2 || (NewRef.Num() == N && Corners == 0)) { continue; }
		PtsBefore += N;
		PtsAfter += NewRef.Num();
		R.Ref = MoveTemp(NewRef);
		if (bIds) { R.NodeIds = MoveTemp(NewIds); } else { R.NodeIds.Reset(); }
		// Elev rides parallel to Ref (kept in sync by point edits) — rebuild it
		// from the new points' Z so the arrays stay aligned.
		if (R.Elev.Num() > 0)
		{
			R.Elev.SetNum(R.Ref.Num());
			for (int32 i = 0; i < R.Ref.Num(); ++i) { R.Elev[i] = R.Ref[i].Z; }
		}
		++Changed;
	}

	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet] SmoothAllRoads: %d/%d road(s) simplified (%d -> %d pts, %d corner(s) rounded; tol %.0f cm, corner > %.0f deg)."),
		Changed, Roads.Num(), PtsBefore, PtsAfter, Corners, Tol, CornerAngleDeg);

	// RDP deliberately never touches a protected point, so it cannot clear the
	// cluster sitting on top of a junction — that is what these two do. Decluster
	// before straightening: blending a tangent through points a few centimetres
	// apart just reproduces the kink at a smaller scale.
	if (CVarRoadNetJunctionConditioning.GetValueOnAnyThread() != 0)
	{
		Changed = FMath::Max(Changed, DeclusterNearJunctions());
		Changed = FMath::Max(Changed, StraightenJunctionApproaches());
	}
	Changed = FMath::Max(Changed, DeclusterPackedAlongRoads());
	return Changed;
}

namespace
{
	// Node ids that carry topology across the whole network. A count of 2 or more
	// means the point is a weld: two roads share it, or one road returns to it.
	void BuildNodeUse(const TArray<FRoadDef>& Roads, TMap<int64, int32>& Out)
	{
		Out.Reset();
		for (const FRoadDef& R : Roads)
		{
			for (int64 Id : R.NodeIds) { if (Id >= 0) { Out.FindOrAdd(Id)++; } }
		}
	}

	// Is Ref[i] of this road untouchable? Endpoints anchor the road; a shared node
	// id is a junction weld.
	bool IsProtectedPoint(const FRoadDef& R, int32 i, const TMap<int64, int32>& NodeUse)
	{
		const int32 N = R.Ref.Num();
		if (i <= 0 || i >= N - 1) { return true; }
		if (R.NodeIds.Num() != N) { return false; }
		const int64 Id = R.NodeIds[i];
		return Id >= 0 && NodeUse.FindRef(Id) >= 2;
	}
}

int32 URoadNetwork::DeclusterNearJunctions(double RadiusCm, double MinSpacingCm)
{
	const double R2 = FMath::Square(FMath::Max(1.0, RadiusCm));
	const double MinSpacing = FMath::Max(1.0, MinSpacingCm);

	TMap<int64, int32> NodeUse;
	BuildNodeUse(Roads, NodeUse);

	int32 Changed = 0, Dropped = 0;
	for (FRoadDef& Road : Roads)
	{
		const int32 N = Road.Ref.Num();
		if (N < 3) { continue; }
		const bool bIds = (Road.NodeIds.Num() == N);

		TArray<int32> Protected;
		for (int32 i = 0; i < N; ++i)
		{
			if (IsProtectedPoint(Road, i, NodeUse)) { Protected.Add(i); }
		}

		auto NearAJunction = [&](const FVector& P) -> bool
		{
			for (int32 pi : Protected)
			{
				if (FVector::DistSquaredXY(P, Road.Ref[pi]) <= R2) { return true; }
			}
			return false;
		};

		TArray<int32> Keep;
		Keep.Reserve(N);
		int32 Last = 0;
		Keep.Add(0);
		for (int32 i = 1; i < N; ++i)
		{
			if (IsProtectedPoint(Road, i, NodeUse))
			{
				Keep.Add(i);
				Last = i;
				continue;
			}
			// Thinning only applies inside the junction's reach; the open road
			// keeps whatever density the import gave it.
			if (NearAJunction(Road.Ref[i]) &&
				FVector::DistXY(Road.Ref[i], Road.Ref[Last]) < MinSpacing)
			{
				continue;
			}
			Keep.Add(i);
			Last = i;
		}
		if (Keep.Num() == N) { continue; }

		TArray<FVector> NewRef;
		TArray<int64> NewIds;
		NewRef.Reserve(Keep.Num());
		NewIds.Reserve(Keep.Num());
		for (int32 i : Keep)
		{
			NewRef.Add(Road.Ref[i]);
			if (bIds) { NewIds.Add(Road.NodeIds[i]); }
		}

		Dropped += N - NewRef.Num();
		Road.Ref = MoveTemp(NewRef);
		if (bIds) { Road.NodeIds = MoveTemp(NewIds); } else { Road.NodeIds.Reset(); }
		if (Road.Elev.Num() > 0)
		{
			Road.Elev.SetNum(Road.Ref.Num());
			for (int32 i = 0; i < Road.Ref.Num(); ++i) { Road.Elev[i] = Road.Ref[i].Z; }
		}
		++Changed;
	}

	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet] DeclusterNearJunctions: dropped %d clustered point(s) across %d/%d road(s) (radius %.0f cm, min spacing %.0f cm)."),
		Dropped, Changed, Roads.Num(), RadiusCm, MinSpacingCm);
	return Changed;
}

int32 URoadNetwork::DeclusterPackedAlongRoads()
{
	const IConsoleVariable* SlopeCV = IConsoleManager::Get().FindConsoleVariable(TEXT("osm.RoadGradeMaxSlope"));
	const double MaxSlope = FMath::Max(0.005, SlopeCV ? (double)SlopeCV->GetFloat() : 0.12);
	const double MinXY = FMath::Max(50.0, 0.25 * FMath::Max(25.0, PolylineDensityCm));

	TMap<int64, int32> NodeUse;
	BuildNodeUse(Roads, NodeUse);

	int32 Changed = 0, Dropped = 0;
	for (FRoadDef& Road : Roads)
	{
		const int32 N = Road.Ref.Num();
		if (N < 3) { continue; }
		const bool bIds = (Road.NodeIds.Num() == N);

		TBitArray<> AlwaysKeep;
		AlwaysKeep.Init(false, N);
		for (int32 i = 0; i < N; ++i)
		{
			if (IsProtectedPoint(Road, i, NodeUse)) { AlwaysKeep[i] = true; }
		}

		TArray<int32> Kept;
		const int32 Here = RoadNetMath::CollapsePackedSamples(Road.Ref, MinXY, MaxSlope, &AlwaysKeep, &Kept);
		if (Here <= 0) { continue; }

		if (bIds)
		{
			TArray<int64> NewIds;
			NewIds.Reserve(Kept.Num());
			for (int32 i : Kept) { NewIds.Add(Road.NodeIds[i]); }
			Road.NodeIds = MoveTemp(NewIds);
		}
		else
		{
			Road.NodeIds.Reset();
		}
		if (Road.Elev.Num() > 0)
		{
			Road.Elev.SetNum(Road.Ref.Num());
			for (int32 i = 0; i < Road.Ref.Num(); ++i) { Road.Elev[i] = Road.Ref[i].Z; }
		}
		Dropped += Here;
		++Changed;
	}

	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet] DeclusterPackedAlongRoads: dropped %d packed point(s) across %d/%d road(s) (min XY %.0f cm, max slope %.1f%%)."),
		Dropped, Changed, Roads.Num(), MinXY, MaxSlope * 100.0);
	return Changed;
}

int32 URoadNetwork::StraightenJunctionApproaches(double LengthCm)
{
	const double Len = FMath::Max(1.0, LengthCm);

	TMap<int64, int32> NodeUse;
	BuildNodeUse(Roads, NodeUse);

	int32 Changed = 0, Moved = 0;
	for (FRoadDef& Road : Roads)
	{
		const int32 N = Road.Ref.Num();
		if (N < 3) { continue; }

		// Read from the untouched original so an approach blended from one end
		// cannot feed a different answer to the pass coming from the other.
		const TArray<FVector> Src = Road.Ref;
		bool bAny = false;

		// Walk out from each protected point in one direction (Step is +1 or -1),
		// blending the points it passes onto the straight approach tangent.
		auto BlendFrom = [&](int32 Anchor, int32 Step)
		{
			const FVector& P = Src[Anchor];

			// Tangent = direction to the first point at or beyond Len. Using a
			// point that far out rather than the immediate neighbour is the whole
			// trick: the neighbour may itself be part of the kink we are removing.
			double Arc = 0.0;
			int32 Far = Anchor;
			for (int32 i = Anchor + Step; i >= 0 && i < N; i += Step)
			{
				Arc += FVector::DistXY(Src[i], Src[i - Step]);
				Far = i;
				if (Arc >= Len) { break; }
			}
			if (Far == Anchor) { return; }
			FVector T = Src[Far] - P;
			T.Z = 0.0;
			if (!T.Normalize()) { return; }

			double S = 0.0;
			for (int32 i = Anchor + Step; i >= 0 && i < N; i += Step)
			{
				S += FVector::DistXY(Src[i], Src[i - Step]);
				if (S >= Len) { break; }
				if (IsProtectedPoint(Road, i, NodeUse)) { break; }

				// Full correction at the junction, none at Len out, smooth in
				// between — a linear taper would leave a visible crease where the
				// straightened stretch rejoins the untouched road.
				const double W = 0.5 * (1.0 + FMath::Cos(PI * S / Len));
				const FVector Target = P + T * S;
				FVector& Q = Road.Ref[i];
				const FVector Blended = FMath::Lerp(Src[i], Target, W);
				if (!FVector::PointsAreNear(FVector(Blended.X, Blended.Y, 0.0),
					FVector(Q.X, Q.Y, 0.0), 1.0))
				{
					++Moved;
					bAny = true;
				}
				Q.X = Blended.X;
				Q.Y = Blended.Y;
			}
		};

		for (int32 i = 0; i < N; ++i)
		{
			if (!IsProtectedPoint(Road, i, NodeUse)) { continue; }
			if (i + 1 < N) { BlendFrom(i, +1); }
			if (i - 1 >= 0) { BlendFrom(i, -1); }
		}

		if (bAny)
		{
			if (Road.Elev.Num() == Road.Ref.Num())
			{
				for (int32 i = 0; i < Road.Ref.Num(); ++i) { Road.Elev[i] = Road.Ref[i].Z; }
			}
			++Changed;
		}
	}

	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet] StraightenJunctionApproaches: relaxed %d point(s) across %d/%d road(s) (approach %.0f cm)."),
		Moved, Changed, Roads.Num(), LengthCm);
	return Changed;
}

bool URoadNetwork::ToggleBikePath(int32 RoadIdx)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;

	const bool bOn = !(L.bBikePathLeft || L.bBikePathRight);
	L.bBikePathLeft = bOn;
	L.bBikePathRight = bOn;
	if (bOn)
	{
		if (L.BikePathWidth <= 0.f) { L.BikePathWidth = 200.f; }
		// See the header: an outboard track with no footway to be outboard OF
		// would sit straight against the kerb, which is not what was asked for.
		if (!L.bSidewalkLeft && !L.bSidewalkRight)
		{
			L.bSidewalkLeft  = true;
			L.bSidewalkRight = true;
			L.SidewalkWidth  = FMath::Max(L.SidewalkWidth, 200.f);
		}
	}
	return bOn;
}

float URoadNetwork::AdjustSidewalkWidth(int32 RoadIdx, float DeltaCm)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return 0.f; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	EnsureDetailedLanes(L, bDriveOnLeft);
	L.SidewalkWidth = FMath::Clamp(L.SidewalkWidth + DeltaCm, 0.f, 2000.f);
	int32 Touched = 0;
	for (FRoadNetLane& Ln : L.DetailedLanes)
	{
		if (Ln.Type == ERoadNetLaneType::Sidewalk)
		{
			Ln.Width = FMath::Max(30.f, L.SidewalkWidth);
			++Touched;
		}
	}
	if (Touched == 0 && L.SidewalkWidth > 0.f)
	{
		L.bSidewalkLeft = true;
		L.bSidewalkRight = true;
		MigrateFlagsToLanes(L);
	}
	RelayoutLanes(L.DetailedLanes, 0.0);
	SyncFlagsFromLanes(L);
	return L.SidewalkWidth;
}

int32 URoadNetwork::AddStandardParkingBay(int32 RoadIdx, ERoadNetSide Side, ERoadNetParkingLayout Layout,
	double CenterArcCm)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return INDEX_NONE; }
#if WITH_EDITOR
	MarkRoadForUndoRebuild(RoadIdx);
#endif
	FRoadNetParkingBay Bay;
	Bay.Side    = (Side == ERoadNetSide::Left) ? ERoadNetSide::Left : ERoadNetSide::Right;
	Bay.Layout  = Layout;
	Bay.AngleDeg = ParkingAngleDeg;
	if (Layout == ERoadNetParkingLayout::Parallel)
	{
		Bay.StallWidthCm = ParkingParallelLengthCm; // along-kerb stall length
		Bay.StallDepthCm = ParkingParallelDepthCm;  // out-from-kerb depth
	}
	else
	{
		Bay.StallWidthCm = ParkingStallWidthCm;
		Bay.StallDepthCm = ParkingStallDepthCm;
	}

	const double L       = RoadNetMath::TotalLength(Roads[RoadIdx].Ref);
	const double Setback = FMath::Max(0.0, (double)ParkingBayJunctionSetbackCm);
	const double Taper   = FMath::Max(0.0, (double)ParkingBayTaperCm);

	// The setback exists to keep stalls out of a junction, so an end that meets no
	// junction does not need it — a cul-de-sac or a road that simply stops can park
	// right up to its end.
	bool bJointAtStart = false, bJointAtEnd = false;
	GetJunctionEnds(RoadIdx, bJointAtStart, bJointAtEnd);

	const double Lo = (bJointAtStart ? Setback : 0.0) + Taper;
	const double Hi = FMath::Max(Lo, L - ((bJointAtEnd ? Setback : 0.0) + Taper));
	const double Available = FMath::Max(0.0, Hi - Lo);
	const double Want = (ParkingBayLengthCm > 0.f)
		? FMath::Min((double)ParkingBayLengthCm, Available) : Available;

	Bay.TaperCm = ParkingBayTaperCm;
	if (Want < 2.0 * (double)Bay.StallWidthCm)
	{
		// Not enough road between the junctions for two stalls. This used to fall
		// back to a full-length bay with the setback dropped, which is exactly the
		// bay the setback exists to prevent: stalls sitting in the junction. Refuse
		// instead — the caller reports it and the user shortens the bay or setback.
		UE_LOG(LogRoadNet, Warning,
			TEXT("[RoadNet] AddStandardParkingBay: road %d has %.0f cm clear of its junctions, "
			     "needs %.0f cm for two %.0f cm stalls — no bay added."),
			RoadIdx, Available, 2.0 * (double)Bay.StallWidthCm, (double)Bay.StallWidthCm);
		return INDEX_NONE;
	}

	// Centre on the authored edge point when given; otherwise mid-road.
	const double Mid = (CenterArcCm >= 0.0) ? CenterArcCm : (0.5 * L);
	double S0 = Mid - 0.5 * Want;
	S0 = FMath::Clamp(S0, Lo, Hi - Want);
	Bay.StartArcCm = (float)S0;
	Bay.LengthCm   = (float)Want;
	return Roads[RoadIdx].ParkingBays.Add(Bay);
}

int32 URoadNetwork::AddBikeCrossing(const TArray<FVector>& Path, float WidthCm)
{
	if (Path.Num() < 2) { return INDEX_NONE; }
	if (RoadNetMath::TotalLength(Path) < 90.0) { return INDEX_NONE; }  // shorter than one block pitch

	FRoadNetBikeCrossing X;
	X.Id = FGuid::NewGuid();
	X.Path = Path;
	X.WidthCm = FMath::Max(100.f, WidthCm);
	return BikeCrossings.Add(MoveTemp(X));
}

int32 URoadNetwork::ClearParkingBays(int32 RoadIdx)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return 0; }
#if WITH_EDITOR
	MarkRoadForUndoRebuild(RoadIdx);
#endif
	const int32 N = Roads[RoadIdx].ParkingBays.Num();
	Roads[RoadIdx].ParkingBays.Reset();
	return N;
}

// 2-D crossing of two segments, INTERIOR hits only. The interior test is what
// makes SplitRoadsAtCrossings terminate: once both roads have been cut, they
// merely touch at a shared endpoint, which no longer reports as a crossing.
static bool SegmentCrossXY(const FVector& A0, const FVector& A1,
	const FVector& B0, const FVector& B1, double& OutTa, double& OutTb)
{
	const FVector2D P(A0.X, A0.Y), R(A1.X - A0.X, A1.Y - A0.Y);
	const FVector2D Q(B0.X, B0.Y), S(B1.X - B0.X, B1.Y - B0.Y);
	const double Den = R.X * S.Y - R.Y * S.X;
	if (FMath::Abs(Den) < 1e-9) { return false; } // parallel or degenerate
	const FVector2D QP = Q - P;
	OutTa = (QP.X * S.Y - QP.Y * S.X) / Den;
	OutTb = (QP.X * R.Y - QP.Y * R.X) / Den;
	return OutTa > 0.0 && OutTa < 1.0 && OutTb > 0.0 && OutTb < 1.0;
}

int32 URoadNetwork::SplitRoadAt(int32 RoadIdx, double ArcCm)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return INDEX_NONE; }
	FRoadDef& A = Roads[RoadIdx];
	if (!A.IsValid()) { return INDEX_NONE; }

	TArray<double> CL;
	RoadNetMath::CumulativeLength(A.Ref, CL);
	const double Len = CL.Last();

	// Inside the weld radius of an end there is already an endpoint that will
	// collapse into this joint, so a cut here buys nothing and leaves a stub.
	if (ArcCm <= kEndpointWeldCm || ArcCm >= Len - kEndpointWeldCm) { return INDEX_NONE; }

	int32 Seg = 0;
	while (Seg + 2 < CL.Num() && CL[Seg + 1] <= ArcCm) { ++Seg; }
	const double SegLen = FMath::Max(1e-6, CL[Seg + 1] - CL[Seg]);
	const double T = FMath::Clamp((ArcCm - CL[Seg]) / SegLen, 0.0, 1.0);
	const FVector P = FMath::Lerp(A.Ref[Seg], A.Ref[Seg + 1], T);

	// The cut point becomes an endpoint of BOTH halves. That shared coordinate is
	// the whole point: FindSpatialJoint welds it with the other road's cut into a
	// single joint, and the arms follow.
	TArray<FVector> Head, Tail;
	Head.Reserve(Seg + 2);
	for (int32 i = 0; i <= Seg; ++i) { Head.Add(A.Ref[i]); }
	Head.Add(P);
	Tail.Reserve(A.Ref.Num() - Seg);
	Tail.Add(P);
	for (int32 i = Seg + 1; i < A.Ref.Num(); ++i) { Tail.Add(A.Ref[i]); }
	if (Head.Num() < 2 || Tail.Num() < 2) { return INDEX_NONE; }

	FRoadDef B = A;             // class, design speed, lanes, name, source inherited
	B.Id = FGuid::NewGuid();
	B.Ref = MoveTemp(Tail);

	// Per-point elevation override, interpolated at the cut then partitioned.
	TArray<double> EHead, ETail;
	if (A.Elev.Num() == CL.Num())
	{
		const double EP = FMath::Lerp(A.Elev[Seg], A.Elev[Seg + 1], T);
		for (int32 i = 0; i <= Seg; ++i) { EHead.Add(A.Elev[i]); }
		EHead.Add(EP);
		ETail.Add(EP);
		for (int32 i = Seg + 1; i < A.Elev.Num(); ++i) { ETail.Add(A.Elev[i]); }
	}
	B.Elev = MoveTemp(ETail);

	// The cut is not an OSM node, so the id arrays stop lining up with Ref across
	// it, and the authored endpoint links now describe the wrong ends. Clearing
	// both costs only node-id seam merging; keeping a stale id would weld this
	// junction to an unrelated one, which is far worse than losing the hint.
	A.NodeIds.Reset();   B.NodeIds.Reset();
	A.StartLinks.Reset(); A.EndLinks.Reset();
	B.StartLinks.Reset(); B.EndLinks.Reset();

	// Arc-parameterised authored data: keep whatever lies wholly on one side and
	// rebase the far side. Anything straddling the cut is dropped rather than
	// guessed at, and said out loud.
	{
		TArray<FRoadNetEdgeKnot> HR, TR, HL, TL;
		auto SplitKnots = [&](const TArray<FRoadNetEdgeKnot>& In,
			TArray<FRoadNetEdgeKnot>& OutH, TArray<FRoadNetEdgeKnot>& OutT)
		{
			for (const FRoadNetEdgeKnot& K : In)
			{
				if (K.Distance <= ArcCm) { OutH.Add(K); }
				else { FRoadNetEdgeKnot N = K; N.Distance -= ArcCm; OutT.Add(N); }
			}
		};
		SplitKnots(A.OuterEdgeRight, HR, TR);
		SplitKnots(A.OuterEdgeLeft,  HL, TL);
		B.OuterEdgeRight = MoveTemp(TR); B.OuterEdgeLeft = MoveTemp(TL);
		A.OuterEdgeRight = MoveTemp(HR); A.OuterEdgeLeft = MoveTemp(HL);
	}

	int32 DroppedBays = 0;
	{
		TArray<FRoadNetParkingBay> HB, TB;
		for (const FRoadNetParkingBay& Bay : A.ParkingBays)
		{
			const double S0 = Bay.StartArcCm;
			const double S1 = (Bay.LengthCm > 0.f) ? (S0 + Bay.LengthCm) : Len;
			if (S1 <= ArcCm) { HB.Add(Bay); }
			else if (S0 >= ArcCm)
			{
				FRoadNetParkingBay N = Bay;
				N.StartArcCm = (float)(S0 - ArcCm);
				TB.Add(N);
			}
			else { ++DroppedBays; }
		}
		B.ParkingBays = MoveTemp(TB);
		A.ParkingBays = MoveTemp(HB);
	}
	{
		TArray<FRoadNetCrossingMark> HC, TC;
		for (const FRoadNetCrossingMark& X : A.Crossings)
		{
			if (X.DistanceCm <= ArcCm) { HC.Add(X); }
			else { FRoadNetCrossingMark N = X; N.DistanceCm -= (float)ArcCm; TC.Add(N); }
		}
		B.Crossings = MoveTemp(TC);
		A.Crossings = MoveTemp(HC);
	}

	A.Ref  = MoveTemp(Head);
	A.Elev = MoveTemp(EHead);

#if DO_CHECK
	// The cut must conserve the road, and the halves must share the cut point
	// EXACTLY — that shared coordinate is the only reason BuildJoints welds them
	// into one node, so if it ever drifts the junction silently fails to form and
	// we are back to a crossing that looks like a crossroads.
	{
		TArray<double> HeadCL, TailCL;
		RoadNetMath::CumulativeLength(A.Ref, HeadCL);
		RoadNetMath::CumulativeLength(B.Ref, TailCL);
		ensureMsgf(FMath::IsNearlyEqual(HeadCL.Last() + TailCL.Last(), Len, 1.0),
			TEXT("SplitRoadAt: %.1f + %.1f != %.1f cm"), HeadCL.Last(), TailCL.Last(), Len);
		ensureMsgf(A.Ref.Last().Equals(B.Ref[0], 0.01),
			TEXT("SplitRoadAt: halves do not share the cut point"));
	}
#endif

	if (DroppedBays > 0)
	{
		UE_LOG(LogRoadNet, Warning,
			TEXT("[RoadNet] SplitRoadAt: dropped %d parking bay(s) spanning the cut on road %d."),
			DroppedBays, RoadIdx);
	}
	// Every mutation of A is done: Roads.Add may reallocate and invalidate it.
	return Roads.Add(MoveTemp(B));
}

int32 URoadNetwork::SplitRoadsAtCrossings(double MaxZGapCm)
{
	// Crossings we looked at and could not act on (both cuts fell inside the weld
	// radius, i.e. the roads already meet here). Without this the rescan would
	// find the same pair forever. Quantised to 10 cm, which is far below the
	// 400 cm weld radius, so it can never merge two distinct junctions.
	TSet<FIntPoint> Settled;
	auto KeyOf = [](const FVector2D& P)
	{
		return FIntPoint(FMath::RoundToInt(P.X / 10.0), FMath::RoundToInt(P.Y / 10.0));
	};

	int32 Created = 0;
	// ponytail: full rescan after each split, O(splits x roads^2 x segments).
	// Fine for a hand-drawn network of tens of roads; if this is ever run over a
	// city import, reuse the uniform-grid broadphase already in BuildCrossings.
	for (int32 Guard = 0; Guard < 512; ++Guard)
	{
		int32 RoadA = INDEX_NONE, RoadB = INDEX_NONE;
		double ArcA = 0.0, ArcB = 0.0;
		FVector2D Hit = FVector2D::ZeroVector;
		bool bFound = false;

		for (int32 a = 0; a < Roads.Num() && !bFound; ++a)
		{
			if (!Roads[a].IsValid()) { continue; }
			TArray<double> CLa; RoadNetMath::CumulativeLength(Roads[a].Ref, CLa);
			for (int32 b = a + 1; b < Roads.Num() && !bFound; ++b)
			{
				if (!Roads[b].IsValid()) { continue; }
				TArray<double> CLb; RoadNetMath::CumulativeLength(Roads[b].Ref, CLb);
				for (int32 i = 0; i + 1 < Roads[a].Ref.Num() && !bFound; ++i)
				{
					for (int32 j = 0; j + 1 < Roads[b].Ref.Num() && !bFound; ++j)
					{
						double Ta = 0.0, Tb = 0.0;
						if (!SegmentCrossXY(Roads[a].Ref[i], Roads[a].Ref[i + 1],
							Roads[b].Ref[j], Roads[b].Ref[j + 1], Ta, Tb)) { continue; }

						// A bridge crosses on purpose — only weld what meets at grade.
						const double Za = FMath::Lerp(Roads[a].Ref[i].Z, Roads[a].Ref[i + 1].Z, Ta);
						const double Zb = FMath::Lerp(Roads[b].Ref[j].Z, Roads[b].Ref[j + 1].Z, Tb);
						if (FMath::Abs(Za - Zb) > MaxZGapCm) { continue; }

						const FVector Pt = FMath::Lerp(Roads[a].Ref[i], Roads[a].Ref[i + 1], Ta);
						const FVector2D Pt2(Pt.X, Pt.Y);
						if (Settled.Contains(KeyOf(Pt2))) { continue; }

						RoadA = a; RoadB = b; Hit = Pt2; bFound = true;
						ArcA = CLa[i] + Ta * (CLa[i + 1] - CLa[i]);
						ArcB = CLb[j] + Tb * (CLb[j + 1] - CLb[j]);
					}
				}
			}
		}
		if (!bFound) { break; }

		// Split the higher index first so the lower one's arc stays measured on an
		// untouched road. (Roads.Add appends, so neither index moves regardless.)
		const int32 NewB = SplitRoadAt(RoadB, ArcB);
		const int32 NewA = SplitRoadAt(RoadA, ArcA);
		Created += (NewA != INDEX_NONE ? 1 : 0) + (NewB != INDEX_NONE ? 1 : 0);

		// Neither road could be cut: both ends are already inside the weld radius,
		// so this is a junction as far as BuildJoints is concerned. Retire the
		// point and keep scanning — breaking here would abandon crossings
		// elsewhere in the network.
		if (NewA == INDEX_NONE && NewB == INDEX_NONE) { Settled.Add(KeyOf(Hit)); }
	}

	if (Created > 0)
	{
		UE_LOG(LogRoadNet, Log,
			TEXT("[RoadNet] SplitRoadsAtCrossings: %d new road(s) — crossings are now junctions."),
			Created);
	}
	return Created;
}

int32 URoadNetwork::ToggleCrossingNear(const FVector2D& WorldXY, double PickRadiusCm,
	bool& bOutAdded, double MergeCm)
{
	bOutAdded = false;

	// Nearest point on any road's centreline, and the arc distance to it. The
	// crossing is stored by arc distance rather than by world position so it
	// travels with the road when a control point is dragged.
	const FVector Probe(WorldXY.X, WorldXY.Y, 0.0);
	int32 BestRoad = INDEX_NONE;
	double BestD2 = PickRadiusCm * PickRadiusCm;
	double BestArc = 0.0;

	for (int32 r = 0; r < Roads.Num(); ++r)
	{
		const TArray<FVector>& Ref = Roads[r].Ref;
		double Acc = 0.0;
		for (int32 i = 0; i + 1 < Ref.Num(); ++i)
		{
			const double SegLen = FVector::Dist2D(Ref[i], Ref[i + 1]);
			if (SegLen <= KINDA_SMALL_NUMBER) { continue; }
			const FVector C = FMath::ClosestPointOnSegment(Probe, Ref[i], Ref[i + 1]);
			const double D2 = FVector::DistSquaredXY(Probe, C);
			if (D2 < BestD2)
			{
				BestD2 = D2;
				BestRoad = r;
				BestArc = Acc + FVector::Dist2D(Ref[i], C);
			}
			Acc += SegLen;
		}
	}
	if (BestRoad == INDEX_NONE) { return INDEX_NONE; }

#if WITH_EDITOR
	MarkRoadForUndoRebuild(BestRoad);
#endif

	// Clicking an existing crossing takes it away — one gesture, both ways,
	// which is how the junction preset and island toggles already behave.
	TArray<FRoadNetCrossingMark>& Marks = Roads[BestRoad].Crossings;
	for (int32 i = 0; i < Marks.Num(); ++i)
	{
		if (FMath::Abs((double)Marks[i].DistanceCm - BestArc) <= MergeCm)
		{
			Marks.RemoveAt(i);
			return BestRoad;
		}
	}

	FRoadNetCrossingMark X;
	X.DistanceCm = (float)BestArc;
	Marks.Add(X);
	bOutAdded = true;
	return BestRoad;
}

int32 URoadNetwork::AddCrossingAt(const FVector2D& WorldXY, float DepthCm, double PickRadiusCm,
	double MergeCm)
{
	const FRoadProj P = ProjectToNearestRoad(Roads, FVector(WorldXY.X, WorldXY.Y, 0.0),
		PickRadiusCm * PickRadiusCm);
	if (P.Road == INDEX_NONE) { return INDEX_NONE; }

#if WITH_EDITOR
	MarkRoadForUndoRebuild(P.Road);
#endif

	const float Depth = FMath::Clamp(DepthCm, 200.f, 1000.f);
	TArray<FRoadNetCrossingMark>& Marks = Roads[P.Road].Crossings;
	for (FRoadNetCrossingMark& M : Marks)
	{
		if (FMath::Abs((double)M.DistanceCm - P.Arc) <= MergeCm)
		{
			M.DepthCm = Depth;
			return P.Road;
		}
	}
	FRoadNetCrossingMark X;
	X.DistanceCm = (float)P.Arc;
	X.DepthCm = Depth;
	Marks.Add(X);
	return P.Road;
}

#if WITH_EDITOR
void URoadNetwork::BeginAuthoringEdit()
{
	UndoRebuildRoads.Reset();
}

void URoadNetwork::MarkRoadForUndoRebuild(int32 RoadIdx)
{
	if (Roads.IsValidIndex(RoadIdx)) { UndoRebuildRoads.AddUnique(RoadIdx); }
}

void URoadNetwork::NotifyAuthoringUndoRedo()
{
	if (const AActor* Owner = GetTypedOuter<AActor>())
	{
		if (UWorld* World = Owner->GetWorld()) { WorldPtr = World; }
	}
	// Keep UndoRebuildRoads so Redo can window the same roads; BeginAuthoringEdit
	// clears it on the next real edit.
	TArray<int32> Window = UndoRebuildRoads;
	for (int32 i = Window.Num() - 1; i >= 0; --i)
	{
		if (!Roads.IsValidIndex(Window[i])) { Window.RemoveAtSwap(i); }
	}
	if (Window.Num() > 0) { Rebuild(Window); }
	else { Rebuild(); }
}

void URoadNetwork::PostEditUndo()
{
	Super::PostEditUndo();
	NotifyAuthoringUndoRedo();
}
#endif

void URoadNetwork::RebuildLatent()
{
	bLatentRebuild = true;
	Rebuild();
	bLatentRebuild = false;
}

// Also mirror the carriageway EDGE splines and the sidewalk OUTER-EDGE rings
// as plan splines (the sidewalk outer edge is the exact line parcels snap to,
// so this is the QA view for road-parcel connections). Off by default: it
// roughly quadruples the plan component count.
static TAutoConsoleVariable<int32> CVarRoadNetPlanEdgeSplines(
	TEXT("roadnet.PlanEdgeSplines"),
	0,
	TEXT("1 = plan splines also show the carriageway edges (white) and sidewalk outer-edge rings (green, the line parcels snap to), not just the centrelines. Default 0."),
	ECVF_Default);

// ---------------------------------------------------------------------------
// § plan splines — EVERYTHING is created from a spline, so the spline must be
// visible. Mirror the reconciled centrelines (post-alignment, post-solver Z)
// onto the owning actor as real USplineComponents, one per road, so the latent
// plan can be inspected in the viewport BEFORE any terrain or mesh work: what
// you see after Import Roads is exactly what the conform and the mesh will be
// built from. Transient and editor-only: rebuilt from the plan every time,
// never saved, never cooked. With roadnet.PlanEdgeSplines=1 the carriageway
// edges and the sidewalk outer-edge rings appear too.
// ---------------------------------------------------------------------------
void URoadNetwork::RefreshPlanSplines(FRoadNetRebuildContext& Ctx)
{
	AActor* Owner = Cast<AActor>(GetOuter());
	if (!Owner) { return; }
	static const FName kPlanTag(TEXT("RoadNetPlanSpline"));

	TArray<USplineComponent*> Old;
	Owner->GetComponents<USplineComponent>(Old);
	for (USplineComponent* S : Old)
	{
		if (S && S->ComponentHasTag(kPlanTag)) { S->DestroyComponent(); }
	}
	// The old splines are gone, so their edit baselines mean nothing now.
	PlanSplineBaselines.Reset();

	int32 Made = 0;
	// IdTag names what a spline IS so HarvestPlanSplineEdits can route an edit
	// back to the data: "RoadNetPlanRoad:<idx>" = centreline of that road,
	// "RoadNetPlanWalk" = sidewalk outer ring. Carriageway edges carry no id —
	// they are DERIVED from centreline + widths, so edit the centreline.
	auto MakePlan = [&](const TArray<FVector>& P, const FLinearColor& Color, bool bClosed,
		const FName IdTag)
	{
		if (P.Num() < 2) { return; }
		USplineComponent* S = NewObject<USplineComponent>(Owner, NAME_None, RF_Transient);
		S->ComponentTags.Add(kPlanTag);
		if (!IdTag.IsNone()) { S->ComponentTags.Add(IdTag); }
		S->bIsEditorOnly = true;
		if (USceneComponent* Root = Owner->GetRootComponent())
		{
			S->SetupAttachment(Root);
		}
		S->RegisterComponent();
		S->ClearSplinePoints(false);

		// Every Nth sample, LINEAR: the plan IS this polyline. Curve
		// interpolation between plan knots would show geometry that is not in
		// the plan — mangled-looking tangents where the road bends hard.
		const int32 Step = FMath::Max(1, P.Num() / 100);
		for (int32 i = 0; i < P.Num(); i += Step)
		{
			S->AddSplinePoint(P[i], ESplineCoordinateSpace::World, false);
		}
		if (!bClosed && (P.Num() - 1) % Step != 0)
		{
			S->AddSplinePoint(P.Last(), ESplineCoordinateSpace::World, false);
		}
		for (int32 i = 0; i < S->GetNumberOfSplinePoints(); ++i)
		{
			S->SetSplinePointType(i, ESplinePointType::Linear, false);
		}
		S->SetClosedLoop(bClosed, false);
#if WITH_EDITORONLY_DATA
		S->EditorUnselectedSplineSegmentColor = Color;
		S->bShouldVisualizeScale = false;
#endif
		S->UpdateSpline();

		// § keep spline edits: remember the as-built points so the next
		// rebuild can tell which knots the USER moved.
		TArray<FVector> Baseline;
		Baseline.Reserve(S->GetNumberOfSplinePoints());
		for (int32 i = 0; i < S->GetNumberOfSplinePoints(); ++i)
		{
			Baseline.Add(S->GetLocationAtSplinePoint(i, ESplineCoordinateSpace::World));
		}
		PlanSplineBaselines.Add(S, MoveTemp(Baseline));
		++Made;
	};

	const FLinearColor kCentre(0.05f, 0.75f, 1.0f);   // plan blue
	const FLinearColor kEdge(0.9f, 0.9f, 0.9f);       // carriageway edge white
	const FLinearColor kWalk(0.1f, 1.0f, 0.25f);      // sidewalk outer edge green
	static const FName kWalkTag(TEXT("RoadNetPlanWalk"));
	const bool bEdges = CVarRoadNetPlanEdgeSplines.GetValueOnAnyThread() != 0;

	for (const TPair<int32, FRoadCurves>& KV : Ctx.Curves)
	{
		MakePlan(KV.Value.Sampled, kCentre, /*bClosed*/false,
			FName(*FString::Printf(TEXT("RoadNetPlanRoad:%d"), KV.Key)));
		if (bEdges)
		{
			MakePlan(KV.Value.LeftEdge,  kEdge, /*bClosed*/false, NAME_None);
			MakePlan(KV.Value.RightEdge, kEdge, /*bClosed*/false, NAME_None);
		}
	}
	// The sidewalk OUTER edge (with reconciled Z, sidewalk-top height): the
	// line every street-bound parcel vertex was snapped onto. If a parcel
	// corner is not ON a green ring, the connection is wrong — that is the QA.
	if (bEdges)
	{
		for (const FRoadNetPlanEdge& E : PlanSidewalkEdges)
		{
			MakePlan(E.Points, kWalk, /*bClosed*/true, kWalkTag);
		}
	}

	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet] PlanSplines: %d plan spline(s) mirrored onto %s (%s) — the latent plan, visible before any mesh exists."),
		Made, *Owner->GetName(),
		bEdges ? TEXT("centrelines + carriageway edges + sidewalk edge rings") : TEXT("centrelines only"));
}

// ---------------------------------------------------------------------------
// § keep spline edits — the user shapes the street ON the plan splines, then
// presses Build Street. Without this, the rebuild regenerates every plan
// spline from the source data and the edits evaporate. So BEFORE the rebuild
// touches anything, diff every live plan spline against the baseline stored at
// its creation, and route each moved knot back to the data:
//   centreline knot : XY goes straight into the road polyline (Roads[].Ref,
//                     persistent) + a Z pin (the vertical solver recomputes Z
//                     every rebuild, so the chosen height must be re-applied).
//   sidewalk knot   : a full-position pin (the ring is derived from the
//                     surface union each rebuild; re-applied after capture).
//   carriageway edge: derived from centreline + lane widths — not harvested.
// ---------------------------------------------------------------------------
void URoadNetwork::HarvestPlanSplineEdits()
{
	if (CVarRoadNetKeepSplineEdits.GetValueOnAnyThread() == 0) { return; }
	AActor* Owner = Cast<AActor>(GetOuter());
	if (!Owner || PlanSplineBaselines.IsEmpty()) { return; }
	static const FName kPlanTag(TEXT("RoadNetPlanSpline"));
	static const FName kWalkTag(TEXT("RoadNetPlanWalk"));

	// Editing the same knot again must UPDATE its pin, not stack a second one:
	// a new edit's plan position is the previous edit's placed position (the
	// splines were rebuilt from the pinned plan), so match against both.
	auto UpsertPin = [this](const FVector2D& OrigXY, const FVector& Edited, bool bWalk)
	{
		for (FRoadNetPlanPin& Pin : PlanEditPins)
		{
			if (Pin.bWalk == bWalk &&
				(FVector2D::Distance(Pin.XY, OrigXY) < 150.0 ||
				 FVector2D::Distance(Pin.OrigXY, OrigXY) < 150.0))
			{
				Pin.OrigXY = OrigXY;
				Pin.XY = FVector2D(Edited.X, Edited.Y);
				Pin.ZCm = (float)Edited.Z;
				return;
			}
		}
		FRoadNetPlanPin Pin;
		Pin.OrigXY = OrigXY;
		Pin.XY = FVector2D(Edited.X, Edited.Y);
		Pin.ZCm = (float)Edited.Z;
		Pin.bWalk = bWalk;
		PlanEditPins.Add(Pin);
	};

	int32 CentreEdits = 0, WalkEdits = 0;
	TArray<USplineComponent*> Splines;
	Owner->GetComponents<USplineComponent>(Splines);
	for (USplineComponent* S : Splines)
	{
		if (!S || !S->ComponentHasTag(kPlanTag)) { continue; }
		const TArray<FVector>* Base = PlanSplineBaselines.Find(S);
		if (!Base) { continue; }

		int32 RoadIdx = INDEX_NONE;
		bool bWalk = false;
		for (const FName& Tag : S->ComponentTags)
		{
			FString TagStr = Tag.ToString();
			if (Tag == kWalkTag) { bWalk = true; }
			else if (TagStr.RemoveFromStart(TEXT("RoadNetPlanRoad:"))) { RoadIdx = FCString::Atoi(*TagStr); }
		}
		if (RoadIdx == INDEX_NONE && !bWalk) { continue; } // derived edge spline

		const int32 N = FMath::Min(S->GetNumberOfSplinePoints(), Base->Num());
		if (S->GetNumberOfSplinePoints() != Base->Num())
		{
			UE_LOG(LogRoadNet, Warning,
				TEXT("[RoadNet] KeepSplineEdits: a plan spline changed point COUNT — only knots up to the common count are kept. Shape by MOVING knots, not adding/deleting."));
		}
		for (int32 i = 0; i < N; ++i)
		{
			const FVector E = S->GetLocationAtSplinePoint(i, ESplineCoordinateSpace::World);
			const FVector B = (*Base)[i];
			if (E.Equals(B, 1.0)) { continue; } // untouched (1 cm)

			if (RoadIdx != INDEX_NONE && Roads.IsValidIndex(RoadIdx))
			{
				// The plan knot is a decimated SAMPLE of the curve, so it has
				// no 1:1 source vertex — move the nearest road vertex to it.
				// ponytail: one plan knot moves one Ref vertex; a curve denser
				// than the plan decimation follows approximately. Upgrade path
				// is a windowed re-fit of Ref around the edit.
				TArray<FVector>& Ref = Roads[RoadIdx].Ref;
				int32 Best = INDEX_NONE;
				double BestD = 1500.0;
				for (int32 k = 0; k < Ref.Num(); ++k)
				{
					const double D = FVector::Dist2D(Ref[k], B);
					if (D < BestD) { BestD = D; Best = k; }
				}
				if (Best != INDEX_NONE)
				{
					Ref[Best].X = E.X;
					Ref[Best].Y = E.Y;
				}
				UpsertPin(FVector2D(B.X, B.Y), E, /*bWalk=*/false);
				++CentreEdits;
			}
			else if (bWalk)
			{
				UpsertPin(FVector2D(B.X, B.Y), E, /*bWalk=*/true);
				++WalkEdits;
			}
		}
	}
	if (CentreEdits + WalkEdits > 0)
	{
		UE_LOG(LogRoadNet, Log,
			TEXT("[RoadNet] KeepSplineEdits: harvested %d centreline + %d sidewalk knot edit(s) -> %d pin(s) held across rebuilds."),
			CentreEdits, WalkEdits, PlanEditPins.Num());
	}
}

// ---------------------------------------------------------------------------
// § keep spline edits + ease heights on the CURVES, right after the vertical
// solver: the solver recomputed every Z from scratch, so the user's pinned
// heights go back on now — nearest sample within 5 m of each pin takes the
// pin's Z, edges follow by the same delta — and then the ease pass sweeps
// every centreline so no two adjacent samples inside the ease radius disagree
// by more than the ease step (spike filter; long segments still grade freely).
// ---------------------------------------------------------------------------
void URoadNetwork::ApplyPlanEditPins(FRoadNetRebuildContext& Ctx)
{
	const bool bKeep = CVarRoadNetKeepSplineEdits.GetValueOnAnyThread() != 0;
	const bool bEase = CVarRoadNetEaseHeights.GetValueOnAnyThread() != 0;
	if (!bKeep && !bEase) { return; }

	int32 Pinned = 0;
	if (bKeep)
	{
		for (const FRoadNetPlanPin& Pin : PlanEditPins)
		{
			if (Pin.bWalk) { continue; } // sidewalk pins land in CaptureStreetPlan
			FRoadCurves* BestC = nullptr;
			int32 BestI = INDEX_NONE;
			double BestD = 500.0;
			for (TPair<int32, FRoadCurves>& KV : Ctx.Curves)
			{
				for (int32 i = 0; i < KV.Value.Sampled.Num(); ++i)
				{
					const double D = FVector2D::Distance(
						FVector2D(KV.Value.Sampled[i].X, KV.Value.Sampled[i].Y), Pin.XY);
					if (D < BestD) { BestD = D; BestC = &KV.Value; BestI = i; }
				}
			}
			if (!BestC) { continue; }
			const double dZ = (double)Pin.ZCm - BestC->Sampled[BestI].Z;
			BestC->Sampled[BestI].Z = Pin.ZCm;
			if (BestC->LeftEdge.IsValidIndex(BestI))  { BestC->LeftEdge[BestI].Z  += dZ; }
			if (BestC->RightEdge.IsValidIndex(BestI)) { BestC->RightEdge[BestI].Z += dZ; }
			++Pinned;
		}
	}

	int32 Eased = 0;
	if (bEase)
	{
		const double MaxStep = FMath::Max(1.0, (double)CVarRoadNetEaseMaxStepCm.GetValueOnAnyThread());
		const double Radius  = FMath::Max(10.0, (double)CVarRoadNetEaseRadiusCm.GetValueOnAnyThread());
		for (TPair<int32, FRoadCurves>& KV : Ctx.Curves)
		{
			TArray<FVector>& P = KV.Value.Sampled;
			TArray<double> ZBefore;
			ZBefore.Reserve(P.Num());
			for (const FVector& V : P) { ZBefore.Add(V.Z); }
			const int32 Moves = EasePolylineZ(P, /*bClosed*/false, MaxStep, Radius);
			if (Moves == 0) { continue; }
			Eased += Moves;
			// Edges ride the centreline: apply the same per-sample delta so
			// the crossfall the solver set is preserved exactly.
			for (int32 i = 0; i < P.Num(); ++i)
			{
				const double dZ = P[i].Z - ZBefore[i];
				if (dZ == 0.0) { continue; }
				if (KV.Value.LeftEdge.IsValidIndex(i))  { KV.Value.LeftEdge[i].Z  += dZ; }
				if (KV.Value.RightEdge.IsValidIndex(i)) { KV.Value.RightEdge[i].Z += dZ; }
			}
		}
	}

	if (Pinned + Eased > 0)
	{
		UE_LOG(LogRoadNet, Log,
			TEXT("[RoadNet] PlanEditPins: %d pinned height(s) re-applied, %d sample(s) eased (step %.0f cm within %.0f cm)."),
			Pinned, Eased,
			CVarRoadNetEaseMaxStepCm.GetValueOnAnyThread(), CVarRoadNetEaseRadiusCm.GetValueOnAnyThread());
	}
}

void URoadNetwork::Rebuild(TArrayView<const int32> Modified, const FBox2D& DirtyRegionWorld)
{
	const double T0 = FPlatformTime::Seconds();

	// § keep spline edits: read the user's plan-spline edits BEFORE anything
	// regenerates those splines out from under them.
	HarvestPlanSplineEdits();

	auto EnsureIds = [](auto& Arr)
	{
		for (auto& E : Arr) { if (!E.Id.IsValid()) { E.Id = FGuid::NewGuid(); } }
	};
	EnsureIds(PlacedMarks);
	EnsureIds(PlacedIslands);
	EnsureIds(BikeCrossings);
	EnsureIds(CurbPaints);

	FRoadNetRebuildContext Ctx;
	Ctx.ExplicitDirtyBox = DirtyRegionWorld;

	// The terrain-conform caches are transient, so right after a level load they
	// hold nothing. A windowed pass would then leave DeformCorridors/ConformTris
	// describing ONLY the edited roads, and the landscape sculpt would ramp just
	// those while every other road kept whatever bed it had — the "conform terrain
	// only works on part of the network" symptom. The header documents that the
	// first rebuild of a session is a full one; enforce it rather than assume it.
	// Costs one full rebuild per session; windowing resumes from the next edit.
	const bool bCachesCold = Roads.Num() > 0 && DeformCache.IsEmpty() && ConformCache.IsEmpty();
	if (Modified.IsEmpty() || bCachesCold)
	{
		if (bCachesCold && !Modified.IsEmpty())
		{
			UE_LOG(LogRoadNet, Log,
				TEXT("[RoadNet] Rebuild: terrain-conform caches are cold (first rebuild since load) — "
					 "promoting this windowed pass to a full rebuild so the conform sees the whole network."));
			Ctx.ExplicitDirtyBox = FBox2D(ForceInit);   // a scoped commit would defeat the promotion
		}
		Ctx.Modified.SetNumUninitialized(Roads.Num());
		for (int32 i = 0; i < Roads.Num(); ++i) { Ctx.Modified[i] = i; }
	}
	else
	{
		Ctx.Modified = TArray<int32>(Modified.GetData(), Modified.Num());
	}

	for (int32 i : Ctx.Modified)
	{
		if (!Roads.IsValidIndex(i) || !Roads[i].Lanes.HasDetailedLanes()) { continue; }
		MigrateFlagsToLanes(Roads[i].Lanes);
		RelayoutLanes(Roads[i].Lanes.DetailedLanes, 0.0);
		SyncFlagsFromLanes(Roads[i].Lanes);
	}

	for (FRoadDef& R : Roads)
	{
		if (!RoadLooksClosed(R.Ref) || R.Ref.Num() < 4) { continue; }
		FVector2D Cent(0, 0);
		for (const FVector& P : R.Ref) { Cent += FVector2D(P.X, P.Y); }
		Cent /= (double)R.Ref.Num();
		const FRoadNetRoundaboutConfig* Rb = FindRoundaboutNear(Cent);
		if (!Rb || Rb->CirculatoryWidthCm < 50.f) { continue; }
		const float Want = Rb->CirculatoryWidthCm;
		const float Have = R.Lanes.HalfWidthCm() * 2.f;
		if (FMath::Abs(Have - Want) < 8.f) { continue; }
		FRoadNetLaneSpec& L = R.Lanes;
		if (L.HasDetailedLanes())
		{
			double Driven = 0.0;
			for (const FRoadNetLane& Ln : L.DetailedLanes)
			{
				if (!Ln.bOutboard() && Ln.Type != ERoadNetLaneType::Median) { Driven += Ln.Width; }
			}
			if (Driven < 50.0) { continue; }
			const double S = (double)Want / Driven;
			for (FRoadNetLane& Ln : L.DetailedLanes)
			{
				if (!Ln.bOutboard() && Ln.Type != ERoadNetLaneType::Median) { Ln.Width = (float)(Ln.Width * S); }
			}
			RelayoutLanes(L.DetailedLanes, 0.0);
			SyncFlagsFromLanes(L);
		}
		else
		{
			const int32 N = FMath::Max(1, L.EffectiveLaneCount());
			L.LaneWidthDefault = Want / (float)N;
		}
	}

	// Per-stage timing so large imports show where time goes (§10.18 profiling).
	// For city-scale rebuilds we also log each stage AS it finishes, so a slow
	// stage is visible immediately (the summary line only prints at the end).
	const bool bTraceStages = Roads.Num() > 50;
	auto Now = []() { return FPlatformTime::Seconds(); };
	auto Trace = [&](const TCHAR* Name, double Dt)
	{
		if (bTraceStages)
		{
			UE_LOG(LogRoadNet, Log, TEXT("[RoadNet]   stage %-11s %8.1f ms"), Name, Dt * 1000.0);
		}
	};

	const double tA = Now();
	DeterminePendingRoads(Ctx);   // §10.17 scope
	// Joints only read Roads[i].Ref / NodeIds, so the topology is available
	// before any geometry — the vertical alignment stage needs it to know where
	// the junctions (its vertical points of intersection) are.
	BuildEndpointJoints(Ctx);     // §10.7 topology from shared node ids
	// Channelization only reads the joints and the lane spec, and it decides
	// how wide each arm has to be, so it has to land before the curves are
	// offset — otherwise a lane drop would have to re-cut geometry afterwards.
	BuildJunctionChannelization(Ctx);   // כרך 2 §5.2.4 / §6 / §7
	BuildCurves(Ctx);             // §10.2–§10.4 reference line + outer edges
	const double tCurves = Now();  Trace(TEXT("curves"), tCurves - tA);
	BuildCrossings(Ctx);          // §10.12 grid broadphase (shared by zones+surface)
	const double tCross = Now();   Trace(TEXT("crossings"), tCross - tCurves);
	BuildVerticalAlignment(Ctx);  // § tangent grades between junctions + plates
	ApplyPlanEditPins(Ctx);       // § keep user heights + ease adjacent samples
	const double tGrade = Now();   Trace(TEXT("grade"), tGrade - tCross);

	// Snapshot the smoothed+densified centrelines for OSMRoadCore's terrain
	// conform (see GetDeformCorridors). These are the SAME polylines the mesh is
	// built from, so ramping the landscape along them makes the flattened
	// corridor hug the real road instead of the sparse source knots.
	//
	// Under a WINDOWED rebuild Ctx.Curves only holds the edit window, so update
	// the per-GUID corridor cache for the roads we recomputed, prune dead roads,
	// then reassemble the WHOLE-network DeformCorridors from the cache — the
	// landscape sculpt always receives every ground corridor, not just the edit.
	{
		for (const TPair<int32, FRoadCurves>& KV : Ctx.Curves)
		{
			const int32 RoadIdx = KV.Key;
			const FRoadCurves& C = KV.Value;
			if (!Roads.IsValidIndex(RoadIdx) || C.Sampled.Num() < 2) { continue; }
			const FRoadDef& R = Roads[RoadIdx];
			FRoadNetDeformCorridor Cor;
			Cor.Points  = C.Sampled;
			const double Half = FMath::Max(50.0, (double)R.Lanes.HalfWidthCm());
			const double Walk = (R.Lanes.bSidewalkLeft || R.Lanes.bSidewalkRight)
				? (double)FMath::Max(0.f, R.Lanes.SidewalkWidth) : 0.0;
			// + an outboard cycle track, which lies beyond the walk: leave it out
			// and the track falls off the flattened corridor onto raw terrain.
			const double Track = (R.Lanes.bBikePathLeft || R.Lanes.bBikePathRight)
				? (double)FMath::Max(0.f, R.Lanes.BikePathWidth) : 0.0;
			// + the deepest parking pocket, so the terrain corridor covers the
			// inclave instead of stopping at the nominal footprint edge.
			Cor.FlatHalfCm = Half + Walk + Track + R.MaxBayDepthCm();
			Cor.bBridge = R.bBridge;
			Cor.bTunnel = R.bTunnel;
			Cor.Layer   = R.Layer;
			DeformCache.Add(R.Id, MoveTemp(Cor));
		}
		// Prune corridors for roads that no longer exist.
		TSet<FGuid> Live;
		for (const FRoadDef& R : Roads) { Live.Add(R.Id); }
		for (auto It = DeformCache.CreateIterator(); It; ++It)
		{
			if (!Live.Contains(It.Key())) { It.RemoveCurrent(); }
		}
		// Reassemble the full-network corridor list.
		DeformCorridors.Reset(DeformCache.Num());
		for (const TPair<FGuid, FRoadNetDeformCorridor>& KV : DeformCache)
		{
			DeformCorridors.Add(KV.Value);
		}
	}
	const double tSnap = Now();    Trace(TEXT("corridors"), tSnap - tGrade);
	BuildZones(Ctx);              // §10.12 grade-separation layering
	const double tZones = Now();   Trace(TEXT("zones"), tZones - tSnap);
	BuildSurfaceUnion(Ctx);       // §10.9 Clipper2 boolean-union per zone
	CaptureStreetPlan(Ctx);       // § street plan API: sidewalk edges with Z
	const double tSurface = Now(); Trace(TEXT("surface"), tSurface - tZones);

	// LATENT-ONLY stop (the street-plan order): the plan now exists — curves,
	// reconciled alignment, deform corridors, surface plan, sidewalk edges. Snap
	// the parcel splines to it, reconcile every spline against its neighbours,
	// mirror the road centrelines as VISIBLE spline components, and return
	// WITHOUT committing a single triangle. The stale conform soup is dropped so
	// the landscape conform that follows reads the latent corridors, never
	// yesterday's mesh. RebuildSerial is deliberately NOT bumped: nothing
	// visible changed, and the editor mode's conform watch must not fire off a
	// half-built state.
	if (bLatentRebuild)
	{
		BindParcelsToStreet(Ctx);
		RefreshPlanSplines(Ctx);
		ConformVerts.Reset();
		ConformTris.Reset();
		UE_LOG(LogRoadNet, Log,
			TEXT("[RoadNet] Rebuild (latent): %d road(s), %d curve(s), %d joint(s), %d corridor(s) — plan captured, splines reconciled, no meshes committed."),
			Roads.Num(), Ctx.Curves.Num(), Ctx.Joints.Num(), DeformCorridors.Num());
		return;
	}

	BuildJunctionMarkings(Ctx);   // §2 junction paint (stop/crosswalk) + signals
	BuildJunctionIslands(Ctx);    // § corner channelizing grass islands (per-junction)
	const double tJunc = Now();    Trace(TEXT("junctionmarks"), tJunc - tSurface);
	BuildPerimeterLoops(Ctx);     // §10.11 perimeter loops (PCG export)
	const double tPerim = Now();   Trace(TEXT("perimeters"), tPerim - tJunc);
	BuildLaneGraph(Ctx);          // §12.2 lane connectivity across joints
	const double tGraph = Now();   Trace(TEXT("lanegraph"), tGraph - tPerim);
	BuildLaneRibbons(Ctx);        // §12.1 per-lane ribbon polys
	const double tRibbon = Now();  Trace(TEXT("laneribbon"), tRibbon - tGraph);
	BuildStandardParkingBays(Ctx);// § standard stalls → parking overlay + white stall lines
	BuildBikeCrossings(Ctx);      // § elephant's-footprint cycle crossings → white bank
	BuildFurniture(Ctx);          // § street-furniture placement sampling (spaced + continuous)
	const double tExtras = Now();  Trace(TEXT("streetextras"), tExtras - tRibbon);
	CommitGeometry(Ctx);          // §10.15 triangulate + spawn surface actor
	const double tCommit = Now();  Trace(TEXT("commit"), tCommit - tExtras);

	int32 Intersections = 0, Seams = 0;
	for (const FRoadNetJoint& J : Ctx.Joints)
	{
		if (J.Kind == ERoadNetJointKind::Intersection) { ++Intersections; }
		else if (J.Kind == ERoadNetJointKind::Seam)     { ++Seams; }
	}

	// A CROSSING is two centrelines overlapping in plan (BuildCrossings). A
	// JUNCTION is two road ENDS meeting (BuildJoints only ever calls AddArm at
	// bStart/!bStart). Only the second grows arms, and turn arrows, stop lines,
	// corner islands and signals every one require Arms.Num() >= 3. So drawing
	// one road straight over another builds a silent nothing: the surfaces union
	// and it LOOKS like a crossroads, while the arm count never leaves 2 and
	// CommitLaneMarks discards every connection. That combination is always a
	// mistake worth naming — but only at grade, since a genuine bridge or
	// underpass crosses on purpose and must not be reported.
	if (Intersections == 0 && Ctx.Crossings.Num() > 0)
	{
		int32 AtGrade = 0;
		for (const FRoadNetCrossing& X : Ctx.Crossings)
		{
			if (FMath::Abs(X.Za - X.Zb) < 200.0) { ++AtGrade; } // >2 m apart = grade separated
		}
		if (AtGrade > 0)
		{
			UE_LOG(LogRoadNet, Warning,
				TEXT("[RoadNet] %d at-grade crossing(s) but 0 intersections: these roads overlap mid-span instead of ending at a shared node, so no junction arms exist. Turn arrows, stop lines and corner islands stay empty until a road ENDS at the crossing point — press 'Make Junctions' on the OSM Roads > Roads tab to split them."),
				AtGrade);
		}
	}

	const double AreaM2 = RoadNetSurface::TotalArea(Ctx.SurfacePolys) / 1.0e4; // cm^2 -> m^2
	const double WalkM2 = RoadNetSurface::TotalArea(Ctx.SidewalkPolys) / 1.0e4;
	const double Ms = (FPlatformTime::Seconds() - T0) * 1000.0;
	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet] Rebuild: %d roads (%d modified, %d pending), %d curves, %d crossings, %d joints (%d intersections, %d seams), %d grade zones, %d surface polys (%.0f m^2), %d sidewalk polys (%.0f m^2), %d perimeter loops, %d lane connections in %.2f ms."),
		Roads.Num(), Ctx.Modified.Num(), Ctx.Pending.Num(), Ctx.Curves.Num(), Ctx.Crossings.Num(),
		Ctx.Joints.Num(), Intersections, Seams, Ctx.Zones.Num(),
		Ctx.SurfacePolys.Num(), AreaM2, Ctx.SidewalkPolys.Num(), WalkM2, Ctx.PerimeterLoops.Num(),
		Ctx.LaneConnections.Num(), Ms);
	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet] Rebuild stages (ms): curves+joints %.1f, crossings %.1f, grade %.1f, corridors %.1f, zones %.1f, surface %.1f, perimeters %.1f, lanegraph %.1f, commit %.1f."),
		(tCurves - tA) * 1000.0, (tCross - tCurves) * 1000.0, (tGrade - tCross) * 1000.0,
		(tSnap - tGrade) * 1000.0, (tZones - tSnap) * 1000.0, (tSurface - tZones) * 1000.0,
		(tPerim - tSurface) * 1000.0, (tGraph - tPerim) * 1000.0, (tCommit - tRibbon) * 1000.0);

	++RebuildSerial;
}

void URoadNetwork::DeterminePendingRoads(FRoadNetRebuildContext& Ctx) const
{
	const int32 N = Roads.Num();

	// Max distance a road's geometry (half-width + sidewalk + junction fillet/
	// clearance) can reach from its centreline; generous so a border junction is
	// always recomputed with its neighbours.
	constexpr double kGeomReachCm = 12000.0; // 120 m

	// CORRIDOR of grid cells a road occupies: every cell its polyline passes
	// through, plus a ring of neighbours within ExpandCm. This is the key to
	// cheap windowed edits — a long/diagonal road marks only the thin band of
	// cells along its length, NOT the full bounding rectangle (which for a
	// diagonal arterial covers half the city and drags in every road inside it).
	// The walk samples each source segment at <= half a tile so no crossed cell
	// is skipped.
	auto RoadCorridorCells = [this](int32 Idx, double ExpandCm, TSet<FIntPoint>& Out)
	{
		if (!Roads.IsValidIndex(Idx)) { return; }
		const TArray<FVector>& Ref = Roads[Idx].Ref;
		if (Ref.Num() == 0) { return; }
		const int32 Ring = FMath::CeilToInt(ExpandCm / FMath::Max(1.0, TileSizeCm));
		const double Step = 0.5 * FMath::Max(1.0, TileSizeCm);
		auto AddPt = [&](double X, double Y)
		{
			const FIntPoint C = RoadNetTiles::WorldToTile(X, Y, TileSizeCm);
			for (int32 dy = -Ring; dy <= Ring; ++dy)
			{
				for (int32 dx = -Ring; dx <= Ring; ++dx)
				{
					Out.Add(FIntPoint(C.X + dx, C.Y + dy));
				}
			}
		};
		if (Ref.Num() == 1) { AddPt(Ref[0].X, Ref[0].Y); return; }
		for (int32 i = 0; i + 1 < Ref.Num(); ++i)
		{
			const FVector& A = Ref[i];
			const FVector& B = Ref[i + 1];
			const double Len = FVector2D::Distance(FVector2D(A.X, A.Y), FVector2D(B.X, B.Y));
			const int32 Steps = FMath::Max(1, (int32)FMath::CeilToInt(Len / Step));
			for (int32 s = 0; s <= Steps; ++s)
			{
				const double t = (double)s / (double)Steps;
				AddPt(FMath::Lerp(A.X, B.X, t), FMath::Lerp(A.Y, B.Y, t));
			}
		}
	};

	// Precompute every road's corridor once (reused for the dirty set, the
	// pending test, and the per-GUID cache refresh). Cheap: cost scales with
	// total road length / tile size, not with road count squared.
	TArray<TSet<FIntPoint>> Corridors;
	Corridors.SetNum(N);
	for (int32 i = 0; i < N; ++i) { RoadCorridorCells(i, kGeomReachCm, Corridors[i]); }

	// Refresh the per-GUID corridor tracker for the NEXT rebuild (its previous
	// value is read below to clear a moved road's old cells). Prunes dead GUIDs.
	auto RefreshFootprints = [&]()
	{
		TSet<FGuid> Live;
		for (int32 i = 0; i < N; ++i)
		{
			const FGuid& Id = Roads[i].Id;
			if (!Id.IsValid()) { continue; }
			Live.Add(Id);
			LastRoadCells.Add(Id, Corridors[i].Array());
		}
		for (auto It = LastRoadCells.CreateIterator(); It; ++It)
		{
			if (!Live.Contains(It.Key())) { It.RemoveCurrent(); }
		}
	};

	// A full rebuild: Rebuild() fills Modified with every index when the caller
	// passes none, and any caller that touches all roads lands here too. The
	// CVar safety valve forces full when windowing is disabled.
	const bool bWindowingEnabled = CVarRoadNetWindowedRebuild.GetValueOnAnyThread() != 0;
	// An explicit dirty region (junction edit) is always a windowed commit — the
	// caller has told us exactly which area changed, so never fall back to full
	// just because every road was passed as "modified".
	// Force a full rebuild when the topological id maps are empty (a fresh
	// session / post-reload): a windowed edit would otherwise assign fresh
	// segment/junction ids from 0 and collide with tile actors already loaded
	// from the level. One full rebuild reassigns every id and recreates tiles.
	const bool bFull = !bWindowingEnabled || SegKeyOf.Num() == 0 ||
		(Ctx.Modified.Num() >= N && !Ctx.ExplicitDirtyBox.bIsValid);
	if (bFull)
	{
		Ctx.bFullCommit = true;
		Ctx.DirtyTiles.Reset();
		Ctx.Pending.SetNumUninitialized(N);
		Ctx.TestAgainst.SetNumUninitialized(N);
		for (int32 i = 0; i < N; ++i) { Ctx.Pending[i] = i; Ctx.TestAgainst[i] = i; }
		RefreshFootprints();
		return;
	}

	// ---- windowed scope --------------------------------------------------
	Ctx.DirtyTiles.Reset();
	if (Ctx.ExplicitDirtyBox.bIsValid)
	{
		// Junction edit: dirty ONLY the cells over the explicit region (the
		// junction disc), regardless of how long the arm roads are. Expanded by
		// the geometry reach so a junction near a cell border still commits the
		// neighbour cell its fillet/paint spills into.
		RoadNetTiles::TilesOverlappingBox(
			Ctx.ExplicitDirtyBox.Min, Ctx.ExplicitDirtyBox.Max,
			TileSizeCm, kGeomReachCm, Ctx.DirtyTiles);
	}
	else
	{
		// Dirty cells = the corridor of every modified road's CURRENT geometry,
		// UNION its previous corridor (so a move clears the cells it left behind).
		for (int32 Idx : Ctx.Modified)
		{
			if (!Roads.IsValidIndex(Idx)) { continue; }
			Ctx.DirtyTiles.Append(Corridors[Idx]);
			if (const TArray<FIntPoint>* Prev = LastRoadCells.Find(Roads[Idx].Id))
			{
				for (const FIntPoint& C : *Prev) { Ctx.DirtyTiles.Add(C); }
			}
		}
	}

	// Nothing valid to scope → safest is a full rebuild.
	if (Ctx.DirtyTiles.Num() == 0)
	{
		Ctx.bFullCommit = true;
		Ctx.Pending.SetNumUninitialized(N);
		Ctx.TestAgainst.SetNumUninitialized(N);
		for (int32 i = 0; i < N; ++i) { Ctx.Pending[i] = i; Ctx.TestAgainst[i] = i; }
		RefreshFootprints();
		return;
	}

	Ctx.bFullCommit = false;

	// Window roads = any road whose corridor shares a cell with the dirty set,
	// so junction unions/height blends at the window border see their
	// neighbours. Because each corridor already carries a reach ring, a road
	// abutting the dirty region (e.g. across a junction) is included.
	Ctx.Pending.Reset();
	for (int32 i = 0; i < N; ++i)
	{
		bool bHit = false;
		for (const FIntPoint& C : Corridors[i])
		{
			if (Ctx.DirtyTiles.Contains(C)) { bHit = true; break; }
		}
		if (bHit) { Ctx.Pending.Add(i); }
	}
	Ctx.TestAgainst = Ctx.Pending;
	RefreshFootprints();
}

void URoadNetwork::BuildCurves(FRoadNetRebuildContext& Ctx) const
{
	for (int32 Idx : Ctx.Pending)
	{
		if (!Roads.IsValidIndex(Idx)) { continue; }
		const FRoadDef& R = Roads[Idx];
		if (!R.IsValid()) { continue; }

		FRoadCurves C;
		C.RoadIndex = Idx;
		const double DensityCm = FMath::Max(25.0, PolylineDensityCm);
		const double MinXY = FMath::Max(50.0, 0.25 * DensityCm);
		const IConsoleVariable* SlopeCV = IConsoleManager::Get().FindConsoleVariable(TEXT("osm.RoadGradeMaxSlope"));
		const double MaxSlope = FMath::Max(0.005, SlopeCV ? (double)SlopeCV->GetFloat() : 0.12);
		// Packed Ref knots make the G2 spline oscillate; collapse a COPY so authored
		// points survive in the editor until DeclusterPackedAlongRoads runs at import.
		TArray<FVector> Knots = R.Ref;
		RoadNetMath::CollapsePackedSamples(Knots, MinXY, MaxSlope);
		TArray<FVector> Smooth;
		RoadNetMath::SmoothG2Spline(Knots, DensityCm, Smooth);
		RoadNetMath::ResampleByArcLength(Smooth, DensityCm, C.Sampled, kAdaptiveTurnRad);
		RoadNetMath::CollapsePackedSamples(C.Sampled, MinXY, MaxSlope);
		if (C.Sampled.Num() < 2) { continue; }

		// Grade the longitudinal profile into a clean ramp: the G2 spline through
		// draped knots OVERSHOOTS in Z between sparse points, and the drape itself
		// carries terrain micro-bumps — both show up as steps/washboard once the
		// terrain is conformed to the bed. A local straight-line fit of Z vs arc
		// length removes those while preserving the real slope exactly. Plan (XY)
		// geometry is untouched; the SAME sampled Z feeds the mesh AND the terrain
		// corridor, so they stay in agreement (no re-introduced poke-through).
		RoadNetMath::SmoothProfileZ(C.Sampled, FMath::Max(0.0, GradeSmoothingM) * 100.0);

		const double Half = FMath::Max(50.0, (double)R.Lanes.HalfWidthCm());
		C.Length = RoadNetMath::TotalLength(C.Sampled);

		// Outer edges: uniform ±Half unless an authored profile exists for that
		// side (Edge tool), in which case sample the profile (arc length → signed
		// lateral offset) per densified vertex and offset variably. The +offset
		// side maps to LeftEdge (see FRoadCurves), the −offset side to RightEdge.
		auto SampleProfile = [](const TArray<FRoadNetEdgeKnot>& Knots, double S, double Fallback) -> double
		{
			if (Knots.Num() == 0) { return Fallback; }
			if (Knots.Num() == 1) { return Knots[0].Offset; }
			if (S <= Knots[0].Distance) { return Knots[0].Offset; }
			if (S >= Knots.Last().Distance) { return Knots.Last().Offset; }
			for (int32 k = 0; k + 1 < Knots.Num(); ++k)
			{
				const double D0 = Knots[k].Distance, D1 = Knots[k + 1].Distance;
				if (S >= D0 && S <= D1)
				{
					const double T = (D1 - D0) > KINDA_SMALL_NUMBER ? (S - D0) / (D1 - D0) : 0.0;
					return FMath::Lerp(Knots[k].Offset, Knots[k + 1].Offset, T);
				}
			}
			return Fallback;
		};

		auto ArcLengths = [](const TArray<FVector>& P, TArray<double>& Out)
		{
			Out.SetNumUninitialized(P.Num());
			double Acc = 0.0;
			Out[0] = 0.0;
			for (int32 i = 1; i < P.Num(); ++i)
			{
				Acc += FVector::Dist2D(P[i - 1], P[i]);
				Out[i] = Acc;
			}
		};

		// Parking bays cut an INCLAVE: each bay pushes the carriageway outer edge
		// out by its stall depth across the bay window, ramped over the entry and
		// exit tapers. The sidewalk band is derived as dilate(carriageway) minus
		// carriageway, so the walk retreats around the bulge on its own and the
		// pocket appears — no separate sidewalk geometry needed.
		// Junction lane drops and turn bays widen the same edge (כרך 2 §5.2.4,
		// §6, §7) — full extra width at the junction, tapered away from it.
		TArray<const FRoadNetArmWidening*> Widenings;
		for (const FRoadNetArmWidening& W : Ctx.ArmWidenings)
		{
			if (W.Road == Idx) { Widenings.Add(&W); }
		}

		if (R.OuterEdgeRight.Num() > 0 || R.OuterEdgeLeft.Num() > 0 || R.ParkingBays.Num() > 0 || Widenings.Num() > 0)
		{
			TArray<double> S; ArcLengths(C.Sampled, S);
			const double ArcLen = S.Last();
			TArray<double> OffR, OffL;
			OffR.SetNumUninitialized(C.Sampled.Num());
			OffL.SetNumUninitialized(C.Sampled.Num());
			for (int32 i = 0; i < C.Sampled.Num(); ++i)
			{
				double BulgeR = 0.0, BulgeL = 0.0;   // + side, − side
				for (const FRoadNetParkingBay& Bay : R.ParkingBays)
				{
					const double B = Bay.BulgeAt(S[i], ArcLen);
					double& Dst = (Bay.Side == ERoadNetSide::Left) ? BulgeL : BulgeR;
					Dst = FMath::Max(Dst, B);
				}
				// Two widenings on the same side ADD (a right-turn bay beyond a
				// dropped through lane is two lanes of extra width), unlike two
				// parking bays which overlap and take the deeper.
				for (const FRoadNetArmWidening* W : Widenings)
				{
					const double B = W->BulgeAt(S[i], ArcLen);
					((W->Side == ERoadNetSide::Left) ? BulgeL : BulgeR) += B;
				}
				OffR[i] = SampleProfile(R.OuterEdgeRight, S[i], +Half) + BulgeR; // +side
				OffL[i] = SampleProfile(R.OuterEdgeLeft,  S[i], -Half) - BulgeL; // −side
			}
			RoadNetMath::OffsetPolylineVariable(C.Sampled, OffR, C.LeftEdge);
			RoadNetMath::OffsetPolylineVariable(C.Sampled, OffL, C.RightEdge);
		}
		else
		{
			RoadNetMath::OffsetPolyline(C.Sampled, +Half, C.LeftEdge);
			RoadNetMath::OffsetPolyline(C.Sampled, -Half, C.RightEdge);
		}

		Ctx.Curves.Add(Idx, MoveTemp(C));
	}
}

void URoadNetwork::BuildCrossings(FRoadNetRebuildContext& Ctx) const
{
	// All 2-D centerline crossings between DIFFERENT roads, via a uniform-grid
	// broadphase. Replaces the two former O(N^2) pair loops (zones + surface) —
	// at city scale (100s of roads, 10,000s of segments) that was the hang.
	Ctx.Crossings.Reset();

	struct FSeg { int32 Road; FVector2D P0, P1; double Z0, Z1; };
	TArray<FSeg> Segs;
	Segs.Reserve(Ctx.Curves.Num() * 16);
	for (const TPair<int32, FRoadCurves>& Pair : Ctx.Curves)
	{
		const TArray<FVector>& S = Pair.Value.Sampled;
		for (int32 i = 0; i + 1 < S.Num(); ++i)
		{
			Segs.Add({ Pair.Key,
				FVector2D(S[i].X, S[i].Y), FVector2D(S[i + 1].X, S[i + 1].Y),
				S[i].Z, S[i + 1].Z });
		}
	}
	if (Segs.Num() < 2) { return; }

	// Cell ~5x the sample density: few segments per cell, cheap neighbour tests.
	constexpr double CellCm = 1000.0;
	auto Floor = [](double V) { return (int32)FMath::FloorToInt(V / CellCm); };

	TMultiMap<FIntPoint, int32> Grid;
	Grid.Reserve(Segs.Num() * 2);
	for (int32 s = 0; s < Segs.Num(); ++s)
	{
		const FSeg& G = Segs[s];
		const int32 X0 = Floor(FMath::Min(G.P0.X, G.P1.X)), X1 = Floor(FMath::Max(G.P0.X, G.P1.X));
		const int32 Y0 = Floor(FMath::Min(G.P0.Y, G.P1.Y)), Y1 = Floor(FMath::Max(G.P0.Y, G.P1.Y));
		for (int32 cx = X0; cx <= X1; ++cx)
		{
			for (int32 cy = Y0; cy <= Y1; ++cy) { Grid.Add(FIntPoint(cx, cy), s); }
		}
	}

	TArray<FIntPoint> Cells;
	Grid.GetKeys(Cells);
	TSet<uint64> Tested;
	Tested.Reserve(Segs.Num() * 2);
	TArray<int32> Bucket;
	for (const FIntPoint& Cell : Cells)
	{
		Bucket.Reset();
		Grid.MultiFind(Cell, Bucket);
		for (int32 i = 0; i < Bucket.Num(); ++i)
		{
			for (int32 j = i + 1; j < Bucket.Num(); ++j)
			{
				int32 a = Bucket[i], b = Bucket[j];
				if (Segs[a].Road == Segs[b].Road) { continue; }
				const uint64 Key = ((uint64)FMath::Min(a, b) << 32) | (uint32)FMath::Max(a, b);
				if (Tested.Contains(Key)) { continue; }
				Tested.Add(Key);

				const FSeg& A = Segs[a];
				const FSeg& B = Segs[b];
				FVector2D Hit; double Ta, Tb;
				if (RoadNetMath::SegmentIntersect2D(A.P0, A.P1, B.P0, B.P1, Hit, Ta, Tb))
				{
					FRoadNetCrossing X;
					X.RoadA = A.Road; X.RoadB = B.Road;
					X.Point = Hit;
					X.Za = FMath::Lerp(A.Z0, A.Z1, Ta);
					X.Zb = FMath::Lerp(B.Z0, B.Z1, Tb);
					Ctx.Crossings.Add(X);
				}
			}
		}
	}
}

namespace
{
	// Bearing of a road end as seen FROM the node, looking outward along the
	// road. Measured over a span rather than off the first segment: OSM
	// geometry routinely opens with a 30 cm stub, and a bearing taken off that
	// is noise, which would scramble the anticlockwise arm order.
	double OutwardBearing(const TArray<FVector>& Ref, bool bAtStart)
	{
		constexpr double kSpanCm = 1500.0;   // 15 m — past the stub, short enough to stay local
		if (Ref.Num() < 2) { return 0.0; }

		const int32 Node = bAtStart ? 0 : Ref.Num() - 1;
		const int32 Step = bAtStart ? 1 : -1;
		const FVector& P0 = Ref[Node];

		FVector Far = Ref[Node + Step];
		double Walked = FVector::Dist2D(P0, Far);
		for (int32 i = Node + 2 * Step; Walked < kSpanCm && Ref.IsValidIndex(i); i += Step)
		{
			Walked += FVector::Dist2D(Far, Ref[i]);
			Far = Ref[i];
		}

		const FVector2D D(Far.X - P0.X, Far.Y - P0.Y);
		return D.IsNearlyZero() ? 0.0 : FMath::Atan2(D.Y, D.X);
	}

	// Elect the main axis through a junction (כרך 2 §3.1.4): the through road
	// keeps its alignment and its crossfall, and every minor arm is designed
	// into it. Pick the most nearly straight-through pair of arms, preferring
	// the higher road class and then the larger lane count.
	//
	// A pair must be within kStraightToleranceDeg of opposite to count as an
	// axis at all. A Y junction where nothing is straight elects nothing, which
	// is exactly the signal the elevation stage uses to fall back to the plane
	// method (§8.5.4).
	void ElectMainAxis(FRoadNetJoint& J)
	{
		J.MainA = J.MainB = INDEX_NONE;
		if (J.Arms.Num() < 2) { return; }

		constexpr double kStraightToleranceDeg = 45.0;
		const double MinDot = FMath::Cos(FMath::DegreesToRadians(180.0 - kStraightToleranceDeg));

		// Score a candidate pair: highest road class first (lower ERoadNetClass
		// ordinal is the more important road), then lane count, then whichever
		// pair runs straightest through the node.
		int32  BestClass = TNumericLimits<int32>::Max();
		int32  BestLanes = -1;
		double BestCos   = 0.0;

		for (int32 a = 0; a < J.Arms.Num(); ++a)
		{
			for (int32 b = a + 1; b < J.Arms.Num(); ++b)
			{
				// Both bearings point outward, so a straight through-road has
				// them opposed: cos of the angle between them near -1.
				const double Cos = FMath::Cos(J.Arms[a].BearingRad - J.Arms[b].BearingRad);
				if (Cos > MinDot) { continue; }   // too bent to be one road through

				const int32 Cls   = FMath::Min((int32)J.Arms[a].Class, (int32)J.Arms[b].Class);
				const int32 Lanes = J.Arms[a].DrivableLanes + J.Arms[b].DrivableLanes;

				const bool bBetter =
					 (Cls <  BestClass) ||
					((Cls == BestClass) && (Lanes >  BestLanes)) ||
					((Cls == BestClass) && (Lanes == BestLanes) && (Cos < BestCos));
				if (!bBetter) { continue; }

				BestClass = Cls;
				BestLanes = Lanes;
				BestCos   = Cos;
				J.MainA   = a;
				J.MainB   = b;
			}
		}

		if (J.MainA != INDEX_NONE)
		{
			J.Arms[J.MainA].bMain = true;
			J.Arms[J.MainB].bMain = true;
		}
	}
}

void URoadNetwork::BuildEndpointJoints(FRoadNetRebuildContext& Ctx) const
{
	// Derive endpoint topology (§10.7). Each road end contributes an arm to the
	// node it sits on. Two mechanisms, unified so OSM and hand-drawn roads share
	// junctions (and mixed OSM+hand-drawn junctions form correctly):
	//   1. Shared OSM node id  — exact grouping via a node->joint map.
	//   2. Spatial weld        — ends within kEndpointWeldCm collapse to one
	//                            joint (the only topology signal hand-drawn roads
	//                            have, since they carry no NodeIds).
	// A uniform grid over joint locations keeps the spatial weld near-O(n).
	const double WeldCm = kEndpointWeldCm;
	const double CellCm = FMath::Max(WeldCm, 1.0);
	const double WeldR2 = FMath::Square(WeldCm);

	TArray<FRoadNetJoint> Joints;
	TMap<int64, int32> NodeToJoint;      // OSM node id  -> index into Joints
	TMultiMap<FIntPoint, int32> Grid;    // grid cell    -> joint indices

	auto CellOf = [CellCm](const FVector2D& P)
	{
		return FIntPoint(FMath::FloorToInt(P.X / CellCm), FMath::FloorToInt(P.Y / CellCm));
	};

	auto FindSpatialJoint = [&](const FVector2D& P) -> int32
	{
		const FIntPoint C = CellOf(P);
		int32 Best = INDEX_NONE;
		double BestD2 = WeldR2;
		TArray<int32> Cands;
		for (int32 dx = -1; dx <= 1; ++dx)
		{
			for (int32 dy = -1; dy <= 1; ++dy)
			{
				Cands.Reset();
				Grid.MultiFind(FIntPoint(C.X + dx, C.Y + dy), Cands);
				for (int32 J : Cands)
				{
					const double D2 = FVector2D::DistSquared(Joints[J].Location, P);
					if (D2 < BestD2) { BestD2 = D2; Best = J; }
				}
			}
		}
		return Best;
	};

	auto AddArm = [&](int32 RoadIdx, bool bStart)
	{
		const FRoadDef& R = Roads[RoadIdx];
		if (!R.IsValid()) { return; }
		const FVector& P = bStart ? R.Ref[0] : R.Ref.Last();
		const FVector2D P2(P.X, P.Y);
		const int64 Node = (R.NodeIds.Num() > 0) ? (bStart ? R.NodeIds[0] : R.NodeIds.Last()) : -1;

		int32 JIdx = INDEX_NONE;
		if (Node >= 0)
		{
			if (int32* Found = NodeToJoint.Find(Node)) { JIdx = *Found; }
		}
		if (JIdx == INDEX_NONE) { JIdx = FindSpatialJoint(P2); }
		if (JIdx == INDEX_NONE)
		{
			FRoadNetJoint J;
			J.NodeId = Node;
			J.Location = P2;
			J.Z = P.Z;
			JIdx = Joints.Add(MoveTemp(J));
			Grid.Add(CellOf(P2), JIdx);
		}
		if (Node >= 0)
		{
			if (Joints[JIdx].NodeId < 0) { Joints[JIdx].NodeId = Node; }
			NodeToJoint.Add(Node, JIdx);
		}
		FRoadNetJointArm Arm;
		Arm.Road     = RoadIdx;
		Arm.bAtStart = bStart;
		Arm.Class    = R.Class;
		Arm.DesignSpeedKph = RoadNetStandards::DesignSpeedKph(R.Class, R.DesignSpeedKph);
		Arm.HalfWidthCm    = FMath::Max(50.0, (double)R.Lanes.HalfWidthCm());
		Arm.BearingRad     = OutwardBearing(R.Ref, bStart);
		for (const FRoadNetLane& L : R.Lanes.ResolveLanes(bDriveOnLeft))
		{
			if (L.bDrivable()) { ++Arm.DrivableLanes; }
		}
		Joints[JIdx].Arms.Add(Arm);
	};

	for (int32 i = 0; i < Roads.Num(); ++i)
	{
		if (!Roads[i].IsValid()) { continue; }
		AddArm(i, /*bStart*/true);
		AddArm(i, /*bStart*/false);
	}

	Ctx.Joints.Reserve(Joints.Num());
	for (FRoadNetJoint& J : Joints)
	{
		const int32 Deg = J.Arms.Num();
		if (Deg <= 1)      { J.Kind = ERoadNetJointKind::Terminal; }
		else if (Deg == 2) { J.Kind = ERoadNetJointKind::Seam; }        // continuity — refine by name/class later
		else               { J.Kind = ERoadNetJointKind::Intersection; }

		// Anticlockwise by outward bearing, so adjacent entries are adjacent
		// approaches. Everything downstream that reasons about "the next arm
		// round" — corner wedges, splitter islands, turn classification —
		// depends on this order and not on road index.
		J.Arms.Sort([](const FRoadNetJointArm& A, const FRoadNetJointArm& B)
		{
			return A.BearingRad < B.BearingRad;
		});
		ElectMainAxis(J);

		Ctx.Joints.Add(MoveTemp(J));
	}
}

void URoadNetwork::BuildZones(FRoadNetRebuildContext& Ctx) const
{
	// Partition roads-with-curves into grade-separated layers (§10.12).
	TArray<int32> WithCurves;
	WithCurves.Reserve(Ctx.Curves.Num());
	for (const TPair<int32, FRoadCurves>& Pair : Ctx.Curves) { WithCurves.Add(Pair.Key); }

	RoadNetZones::PartitionLayers(WithCurves, Ctx.Curves, Roads, Ctx.Joints, Ctx.Crossings, kMaxZGapCm, Ctx.Zones);
}

void URoadNetwork::BuildSurfaceUnion(FRoadNetRebuildContext& Ctx) const
{
	// Union each zone independently (§10.9). Roads only merge with others at the
	// same grade, so overpasses/underpasses stay separate — no cross-level blobs.
	Ctx.ZoneSurfacePolys.Reset();
	Ctx.ZoneSurfacePolys.SetNum(Ctx.Zones.Num());
	Ctx.ZoneSidewalkPolys.Reset();
	Ctx.ZoneSidewalkPolys.SetNum(Ctx.Zones.Num());
	Ctx.ZoneMarkingWhitePolys.Reset();
	Ctx.ZoneMarkingWhitePolys.SetNum(Ctx.Zones.Num());
	Ctx.ZoneMarkingYellowPolys.Reset();
	Ctx.ZoneMarkingYellowPolys.SetNum(Ctx.Zones.Num());
	Ctx.ZoneJunctionClip.Reset();
	Ctx.ZoneJunctionClip.SetNum(Ctx.Zones.Num());
	Ctx.ZoneMedianPolys.Reset();
	Ctx.ZoneMedianPolys.SetNum(Ctx.Zones.Num());
	Ctx.ZoneMedianWalkPolys.Reset();
	Ctx.ZoneMedianWalkPolys.SetNum(Ctx.Zones.Num());
	Ctx.ZoneBikePathPolys.Reset();
	Ctx.ZoneBikePathPolys.SetNum(Ctx.Zones.Num());
	Ctx.SurfacePolys.Reset();
	Ctx.SidewalkPolys.Reset();

	auto HalfWidth = [this](int32 RoadIdx) -> double
	{
		return Roads.IsValidIndex(RoadIdx) ? FMath::Max(50.0, (double)Roads[RoadIdx].Lanes.HalfWidthCm()) : 50.0;
	};

	for (int32 z = 0; z < Ctx.Zones.Num(); ++z)
	{
		const TArray<int32>& ZoneRoads = Ctx.Zones[z];

		TArray<const FRoadCurves*> Ptrs;
		Ptrs.Reserve(ZoneRoads.Num());
		for (int32 RoadIdx : ZoneRoads)
		{
			if (const FRoadCurves* C = Ctx.Curves.Find(RoadIdx)) { Ptrs.Add(C); }
		}

		// ---- junction fillet discs (§10.8): collect junction points ----------
		TSet<int32> ZoneSet(ZoneRoads);
		TArray<TPair<FVector2D, double>> JPts;  // (location, radius)

		auto AddJPoint = [&JPts](const FVector2D& P, double R)
		{
			for (TPair<FVector2D, double>& E : JPts)
			{
				if (FVector2D::DistSquared(E.Key, P) < FMath::Square(0.5 * FMath::Max(E.Value, R)))
				{
					E.Value = FMath::Max(E.Value, R);   // merge into nearby point
					return;
				}
			}
			JPts.Emplace(P, R);
		};

		// (a) welded endpoint joints whose arms lie in this zone. This covers both
		// N-way intersections (3+ arms) AND end-to-end seams (2 arms): two roads
		// meeting at a node leave a wedge/notch between their outlines, and the
		// node itself may be a genuine gap because arms weld up to kEndpointWeldCm
		// apart without touching. Both are filled by the CONVEX HULL of the arms'
		// own carriageway edge ends.
		//
		// This used to be a disc of radius max(arm half-width), which is where the
		// "blob" came from: a circle has no orientation, so a 2-lane road meeting a
		// 4-lane one got the 4-lane road's 7 m radius stamped across it and bulged
		// 3.5 m past both its kerbs. The kerb ribbon, which is traced off this same
		// merged boundary, then ran into the bulge and got cut off mid-stone.
		//
		// The hull cannot do that: every vertex of it IS an arm's own edge point, so
		// the fill is as wide as the arms are and no wider, it leans the way they
		// lean, and a skewed junction gets a skewed fill. That is also what כרך 2
		// §2.1 defines the junction area to be — the region bounded by the arms'
		// edge lines and their imaginary extension.
		auto ArmEndpoint = [&](const FRoadNetJointArm& Arm) -> FVector2D
		{
			if (const FRoadCurves* C = Ctx.Curves.Find(Arm.Road))
			{
				const FVector& P = Arm.bAtStart ? C->Sampled[0] : C->Sampled.Last();
				return FVector2D(P.X, P.Y);
			}
			const FVector& P = Arm.bAtStart ? Roads[Arm.Road].Ref[0] : Roads[Arm.Road].Ref.Last();
			return FVector2D(P.X, P.Y);
		};

		// The two outer-edge corners this arm presents to the node. Falls back to
		// the centreline endpoint offset by the arm's half width when the arm has
		// no resolved curves (hand-drawn road not yet sampled), so a joint never
		// silently loses its fill.
		auto ArmEdgeEnds = [&](const FRoadNetJointArm& Arm, TArray<FVector2D>& Out)
		{
			const FRoadCurves* C = Ctx.Curves.Find(Arm.Road);
			if (C && C->LeftEdge.Num() > 0 && C->RightEdge.Num() > 0)
			{
				const FVector& L = Arm.bAtStart ? C->LeftEdge[0]  : C->LeftEdge.Last();
				const FVector& R = Arm.bAtStart ? C->RightEdge[0] : C->RightEdge.Last();
				Out.Emplace(L.X, L.Y);
				Out.Emplace(R.X, R.Y);
				return;
			}
			const FVector2D P = ArmEndpoint(Arm);
			const FVector2D N(-FMath::Sin(Arm.BearingRad), FMath::Cos(Arm.BearingRad));
			const double H = FMath::Max(50.0, (double)Arm.HalfWidthCm);
			Out.Add(P + N * H);
			Out.Add(P - N * H);
		};

		TArray<UE::Geometry::FGeneralPolygon2d> Fills;   // node fills (hulls)

		for (const FRoadNetJoint& J : Ctx.Joints)
		{
			// Terminal (dead-end) joints have a single arm — nothing to bridge.
			if (J.Arms.Num() < 2) { continue; }

			double MaxHalf = 0.0;
			FVector2D Centroid(0, 0);
			int32 InZoneArms = 0;
			TArray<FVector2D> EdgeEnds;
			for (const FRoadNetJointArm& Arm : J.Arms)
			{
				if (!ZoneSet.Contains(Arm.Road)) { continue; }
				Centroid += ArmEndpoint(Arm);
				MaxHalf = FMath::Max(MaxHalf, HalfWidth(Arm.Road));
				ArmEdgeEnds(Arm, EdgeEnds);
				++InZoneArms;
			}
			if (InZoneArms < 2 || MaxHalf <= 0.0) { continue; }
			Centroid /= (double)InZoneArms;

			UE::Geometry::FGeneralPolygon2d Hull;
			if (RoadNetSurface::MakeHull(EdgeEnds, Hull)) { Fills.Add(MoveTemp(Hull)); }

			// The smoothing pass still works in discs, and its radius has to reach
			// the farthest arm endpoint plus that road's half width or it would
			// clip the corner it is meant to round. Note this no longer adds any
			// pavement — it only says how much of the surface to look at.
			double R = MaxHalf;
			for (const FRoadNetJointArm& Arm : J.Arms)
			{
				if (!ZoneSet.Contains(Arm.Road)) { continue; }
				const double D = FVector2D::Distance(Centroid, ArmEndpoint(Arm));
				R = FMath::Max(R, D + HalfWidth(Arm.Road));
			}
			AddJPoint(Centroid, R);
		}

		// (b) centerline crossings between roads in this zone (X and touching-T).
		// Uses the precomputed shared crossings (grid broadphase) instead of a
		// per-zone O(N^2) segment sweep.
		//
		// A crossing needs NO fill: two full-width bands laid across each other
		// already overlap over the whole crossing, so the union covers it. It is
		// still a junction point for smoothing, which is what the corners of that
		// overlap need.
		for (const FRoadNetCrossing& X : Ctx.Crossings)
		{
			if (!ZoneSet.Contains(X.RoadA) || !ZoneSet.Contains(X.RoadB)) { continue; }
			const double R = FMath::Max(HalfWidth(X.RoadA), HalfWidth(X.RoadB));
			AddJPoint(X.Point, R);
		}

		// JunctionSmoothingCm drives the morphological close (round corners / gap
		// bridging). Keep a small floor so abutting arms always weld.
		const double CloseCm = FMath::Max(2.0, JunctionSmoothingCm);

		// Per-junction smoothing: every junction point gets its own close radius,
		// its override if it has one and the network default otherwise. Cheap:
		// for a windowed junction edit JPts holds only the edited junction.
		TArray<RoadNetSurface::FJunctionClose> JClose;
		{
			// Match a JPt to a stored override the same way the interactive picker
			// does (nearest config within tolerance).
			JClose.Reserve(JPts.Num());
			for (const TPair<FVector2D, double>& E : JPts)
			{
				double e = JunctionSmoothingCm;
				double BestD2 = FMath::Square(kJunctionMatchCm);
				for (const FRoadNetJunctionConfig& Cfg : JunctionConfigs)
				{
					if (Cfg.SmoothingCm < 0.f) { continue; }
					const double D2 = FVector2D::DistSquared(Cfg.Location, E.Key);
					if (D2 < BestD2) { BestD2 = D2; e = Cfg.SmoothingCm; }
				}
				RoadNetSurface::FJunctionClose J;
				J.Center = E.Key;
				J.FillRadiusCm = E.Value;
				J.CloseCm = FMath::Max(0.0, e);
				JClose.Add(J);
			}
		}

		// Always round LOCALLY, never with one global close. A global close of
		// radius e bridges any gap up to 2e wide anywhere in the zone, so at the
		// old 20 cm default it was harmless and at 150 it would weld two roads
		// running 2 m apart into one blob — the exact failure the per-zone union
		// exists to avoid. The local path welds the surface with a 5 cm hairline
		// and then rounds each junction inside its own disc, so a bigger radius
		// buys rounder corners and nothing else.
		RoadNetSurface::BuildMergedSurface(Ptrs, Ctx.ZoneSurfacePolys[z], CloseCm,
			&Fills, JClose.Num() > 0 ? &JClose : nullptr);
		Ctx.SurfacePolys.Append(Ctx.ZoneSurfacePolys[z]);

		// ---- true junction area (§2.1 "שטח הצומת") for clipping paint ----------
		// Per HNCH vol.2 §2.1, the junction is bounded by the roads' edge lines and
		// their imaginary extension. Geometrically that boundary is the mutual
		// OVERLAP of the crossing carriageways (a disc ignores width/angle/skew).
		// We union those overlaps, then dilate by the stop-line setback, and use
		// the result to clip lane markings + ribbons so paint ends at the junction.
		{
			using namespace UE::Geometry;

			// Per-road carriageway outlines (edge-to-edge) for this zone.
			TMap<int32, FGeneralPolygon2d> Outlines;
			Outlines.Reserve(ZoneRoads.Num());
			for (int32 RoadIdx : ZoneRoads)
			{
				if (const FRoadCurves* C = Ctx.Curves.Find(RoadIdx))
				{
					FGeneralPolygon2d O;
					if (RoadNetSurface::BuildRoadOutline(*C, O)) { Outlines.Add(RoadIdx, MoveTemp(O)); }
				}
			}

			TArray<FGeneralPolygon2d> Overlaps;
			TSet<uint64> DonePairs;
			auto PackPair = [](int32 a, int32 b) -> uint64
			{
				if (a > b) { Swap(a, b); }
				return ((uint64)(uint32)a << 32) | (uint32)b;
			};
			auto AddOverlap = [&](int32 A, int32 B)
			{
				if (A == B) { return; }
				bool bDup = false; DonePairs.Add(PackPair(A, B), &bDup);
				if (bDup) { return; }
				const FGeneralPolygon2d* PA = Outlines.Find(A);
				const FGeneralPolygon2d* PB = Outlines.Find(B);
				if (!PA || !PB) { return; }
				const FGeneralPolygon2d InA[] = { *PA };
				const FGeneralPolygon2d InB[] = { *PB };
				TArray<FGeneralPolygon2d> Out;
				if (PolygonsIntersection(InA, InB, Out)) { Overlaps.Append(MoveTemp(Out)); }
			};

			// Connected road pairs = shared endpoint joints (each arm pair) and
			// centerline crossings, both restricted to this zone. A junction has
			// ≥3 arms (§2.2); 2-arm joints are continuations/corners where paint
			// runs through, so they don't define a clip region.
			for (const FRoadNetJoint& J : Ctx.Joints)
			{
				if (J.Arms.Num() < 3) { continue; }
				for (int32 i = 0; i < J.Arms.Num(); ++i)
				{
					for (int32 j = i + 1; j < J.Arms.Num(); ++j)
					{
						const int32 A = J.Arms[i].Road, B = J.Arms[j].Road;
						if (ZoneSet.Contains(A) && ZoneSet.Contains(B)) { AddOverlap(A, B); }
					}
				}
			}
			for (const FRoadNetCrossing& X : Ctx.Crossings)
			{
				if (ZoneSet.Contains(X.RoadA) && ZoneSet.Contains(X.RoadB)) { AddOverlap(X.RoadA, X.RoadB); }
			}

			TArray<FGeneralPolygon2d>& Clip = Ctx.ZoneJunctionClip[z];
			if (Overlaps.Num() > 0)
			{
				// The clip is what every junction element is positioned from —
				// stop bars, zebras and signal masts all sit where a centreline
				// leaves it — so it has to describe the pavement that will
				// ACTUALLY be there.
				//
				// This region is the mutual overlap of the crossing carriageways,
				// which is the junction BEFORE smoothing. The close that follows
				// in BuildSurfaceUnion fills the concave wedge between each pair
				// of adjacent arms with a fillet of its own radius, so the real
				// pavement boundary runs up to that much further back up every
				// approach than the overlap does. Set back by only
				// JunctionClearanceCm and the stop bar lands inside the fillet —
				// a small error at the old 20 cm default, and a very visible one
				// now the default rounds properly.
				//
				// Each overlap is one junction, so its own smoothing applies:
				// a junction tuned down with [ does not get a neighbour's setback.
				TArray<FGeneralPolygon2d> SetBack;
				SetBack.Reserve(Overlaps.Num());
				for (const FGeneralPolygon2d& Poly : Overlaps)
				{
					const TArray<FVector2d>& PV = Poly.GetOuter().GetVertices();
					if (PV.Num() < 3) { continue; }

					FVector2D Mid(0, 0);
					for (const FVector2d& V : PV) { Mid += FVector2D(V.X, V.Y); }
					Mid /= (double)PV.Num();

					const double Setback = FMath::Max(0.0, JunctionClearanceCm)
					                     + FMath::Max(0.0, ResolveJunctionSmoothingNear(Mid));

					TArray<FGeneralPolygon2d> One;
					One.Add(Poly);
					TArray<FGeneralPolygon2d> Dilated;
					if (Setback > 1.0 && PolygonsOffset(Setback, One, Dilated, /*bCopyInputOnFailure*/true,
							/*MiterLimit*/2.0, EPolygonOffsetJoinType::Round,
							EPolygonOffsetEndType::Polygon, /*MaxStepsPerRadian*/16.0,
							/*DefaultStepsPerRadianScale*/1.0e-3))
					{
						SetBack.Append(MoveTemp(Dilated));
					}
					else { SetBack.Add(Poly); }
				}

				if (SetBack.Num() > 0 && !PolygonsUnion(SetBack, Clip, /*bCopyInputOnFailure*/true))
				{
					Clip = MoveTemp(SetBack);
				}
			}
		}

		// ---- sidewalk band (§8.12): dilate merged carriageway, subtract road ---
		// Deriving the sidewalk from the ALREADY-merged surface makes it hug the
		// road (junction discs included) with continuous, flap-free edges — the
		// per-road ribbon approach left overlapping square end-caps at seams.
		// Per-side requests are honoured afterwards by intersecting the band with
		// only the enabled sides of each road.
		{
			using namespace UE::Geometry;

			auto SidewalkW = [](const FRoadNetLaneSpec& L, bool bLeft) -> double
			{
				for (const FRoadNetLane& Ln : L.DetailedLanes)
				{
					if (Ln.Type != ERoadNetLaneType::Sidewalk) { continue; }
					if (bLeft == (Ln.CenterOffset < 0.0)) { return (double)FMath::Max(0.f, Ln.Width); }
				}
				if ((bLeft && L.bSidewalkLeft) || (!bLeft && L.bSidewalkRight))
				{
					return (double)FMath::Max(0.f, L.SidewalkWidth);
				}
				return 0.0;
			};

			double MaxW = 0.0;
			bool bAnySide = false;
			for (int32 RoadIdx : ZoneRoads)
			{
				if (!Roads.IsValidIndex(RoadIdx)) { continue; }
				const FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
				const double WL = SidewalkW(L, true), WR = SidewalkW(L, false);
				if (WL > 0.0 || WR > 0.0) { bAnySide = true; }
				MaxW = FMath::Max(MaxW, FMath::Max(WL, WR));
			}

			const TArray<FGeneralPolygon2d>& RoadPolys = Ctx.ZoneSurfacePolys[z];
			if (bAnySide && MaxW > 0.0 && RoadPolys.Num() > 0)
			{
				TArray<FGeneralPolygon2d> Dilated;
				const bool bOff = PolygonsOffset(
					MaxW, RoadPolys, Dilated, /*bCopyInputOnFailure*/true,
					/*MiterLimit*/2.0, EPolygonOffsetJoinType::Round,
					EPolygonOffsetEndType::Polygon, /*MaxStepsPerRadian*/16.0,
					/*DefaultStepsPerRadianScale*/1.0e-3);

				TArray<FGeneralPolygon2d> Band;
				if (bOff && PolygonsDifference(Dilated, RoadPolys, Band) && Band.Num() > 0)
				{
					TArray<FGeneralPolygon2d> Masks;
					for (int32 RoadIdx : ZoneRoads)
					{
						const FRoadCurves* C = Ctx.Curves.Find(RoadIdx);
						if (!C || !Roads.IsValidIndex(RoadIdx)) { continue; }
						const FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
						const double Half = HalfWidth(RoadIdx);
						const double In = FMath::Max(1.0, Half - 30.0);
						const double WL = SidewalkW(L, true), WR = SidewalkW(L, false);
						FGeneralPolygon2d Mask;
						if (WL > 0.0)
						{
							const double Out = Half + WL + 60.0 + Roads[RoadIdx].MaxBayDepthCm();
							if (RoadNetSurface::BuildSideRibbon(C->Sampled, +In, +Out, Mask)) { Masks.Add(Mask); }
						}
						if (WR > 0.0)
						{
							const double Out = Half + WR + 60.0 + Roads[RoadIdx].MaxBayDepthCm();
							if (RoadNetSurface::BuildSideRibbon(C->Sampled, -In, -Out, Mask)) { Masks.Add(Mask); }
						}
					}
					TArray<FGeneralPolygon2d> MaskU;
					if (Masks.Num() > 0 && PolygonsUnion(Masks, MaskU, /*bCopyInputOnFailure*/true)
						&& PolygonsIntersection(Band, MaskU, Ctx.ZoneSidewalkPolys[z]))
					{
						// per-road width mask stored
					}
					else
					{
						Ctx.ZoneSidewalkPolys[z] = MoveTemp(Band);
					}
					Ctx.SidewalkPolys.Append(Ctx.ZoneSidewalkPolys[z]);
				}
			}
		}

		// ---- outboard cycle track (separated cycling) ------------------------
		// A bike path BEYOND the footway, so the sidewalk (and its kerb) is what
		// stands between riders and moving traffic, rather than a painted line.
		// Same recipe as the sidewalk band — dilate the merged carriageway,
		// subtract it, mask to the requested sides — with one addition: the
		// sidewalk band built just above is subtracted out. That is what places
		// the walk in between, and it is exact, whereas offsetting by a
		// zone-wide sidewalk width would shove a road with a narrow walk out to
		// match the widest walk in the zone. Must stay AFTER the sidewalk block.
		{
			using namespace UE::Geometry;

			double MaxOut = 0.0;
			bool bAnyTrack = false;
			auto WalkWidthOf = [](const FRoadNetLaneSpec& L) -> double
			{
				return (L.bSidewalkLeft || L.bSidewalkRight)
					? (double)FMath::Max(0.f, L.SidewalkWidth) : 0.0;
			};
			for (int32 RoadIdx : ZoneRoads)
			{
				if (!Roads.IsValidIndex(RoadIdx)) { continue; }
				const FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
				if (L.BikePathWidth <= 0.f || (!L.bBikePathLeft && !L.bBikePathRight)) { continue; }
				bAnyTrack = true;
				MaxOut = FMath::Max(MaxOut, WalkWidthOf(L) + (double)L.BikePathWidth);
			}

			const TArray<FGeneralPolygon2d>& RoadPolys = Ctx.ZoneSurfacePolys[z];
			if (bAnyTrack && MaxOut > 0.0 && RoadPolys.Num() > 0)
			{
				TArray<FGeneralPolygon2d> Dilated, Band;
				const bool bOff = PolygonsOffset(
					MaxOut, RoadPolys, Dilated, /*bCopyInputOnFailure*/true,
					/*MiterLimit*/2.0, EPolygonOffsetJoinType::Round,
					EPolygonOffsetEndType::Polygon, /*MaxStepsPerRadian*/16.0,
					/*DefaultStepsPerRadianScale*/1.0e-3);

				if (bOff && PolygonsDifference(Dilated, RoadPolys, Band) && Band.Num() > 0)
				{
					// Carve the footway out, so the track starts where the walk ends.
					if (Ctx.ZoneSidewalkPolys[z].Num() > 0)
					{
						TArray<FGeneralPolygon2d> Outboard;
						if (PolygonsDifference(Band, Ctx.ZoneSidewalkPolys[z], Outboard))
						{
							Band = MoveTemp(Outboard);
						}
					}

					// Mask to the requested sides. The inner edge is already set by
					// the carve above, so the ribbon starts at the carriageway and
					// is deliberately generous — it only decides WHICH side.
					TArray<FGeneralPolygon2d> Masks;
					for (int32 RoadIdx : ZoneRoads)
					{
						const FRoadCurves* C = Ctx.Curves.Find(RoadIdx);
						if (!C || !Roads.IsValidIndex(RoadIdx)) { continue; }
						const FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
						const double BW = (double)L.BikePathWidth;
						if (BW <= 0.0) { continue; }
						const double Half = HalfWidth(RoadIdx);
						const double In  = FMath::Max(1.0, Half - 30.0);
						const double Out = Half + WalkWidthOf(L) + BW + 60.0
						                 + Roads[RoadIdx].MaxBayDepthCm();
						FGeneralPolygon2d Mask;
						if (L.bBikePathLeft  && RoadNetSurface::BuildSideRibbon(C->Sampled, +In, +Out, Mask)) { Masks.Add(Mask); }
						if (L.bBikePathRight && RoadNetSurface::BuildSideRibbon(C->Sampled, -In, -Out, Mask)) { Masks.Add(Mask); }
					}

					TArray<FGeneralPolygon2d> MaskU;
					if (Masks.Num() > 0 && PolygonsUnion(Masks, MaskU, /*bCopyInputOnFailure*/true))
					{
						PolygonsIntersection(Band, MaskU, Ctx.ZoneBikePathPolys[z]);
					}
				}
			}
		}

		// ---- central median islands (§ divided road) -------------------------
		// A raised grass island filling the carriageway gap: green fill bordered
		// by a constant-width kerb (emitted in CommitCurbs from this outline).
		// The reference line is split into runs OUTSIDE the junction region and
		// each run is pulled back + capped with a rounded nose, so the island
		// closes cleanly short of every intersection instead of ploughing
		// through. Routed to the soil (grass) or walkable (concrete) layer.
		{
			using namespace UE::Geometry;
			const TArray<FGeneralPolygon2d>& Clip = Ctx.ZoneJunctionClip[z];
			auto InClip = [&Clip](const FVector& P)
			{
				const FVector2d Q(P.X, P.Y);
				for (const FGeneralPolygon2d& GP : Clip) { if (GP.Contains(Q)) { return true; } }
				return false;
			};
			for (int32 RoadIdx : ZoneRoads)
			{
				if (!Roads.IsValidIndex(RoadIdx) || !Roads[RoadIdx].Lanes.bMedian) { continue; }
				const FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
				const FRoadCurves* C = Ctx.Curves.Find(RoadIdx);
				if (!C || C->Sampled.Num() < 2) { continue; }
				const double MedianHalf = (double)L.MedianHalfCm();
				if (MedianHalf < 10.0) { continue; }
				const double Pullback = MedianHalf + 60.0;   // nose clearance from junction

				const TArray<FVector>& S = C->Sampled;
				const ERoadNetMedianEdge Edge = L.MedianEdge;
				const bool bWalk = (Edge == ERoadNetMedianEdge::SidewalkAndCurb);

				// PlantableWalkCurb builds a concentric island: an outer concrete
				// walk band (donut) + a kerbed green planter in the centre. Only
				// attempt it if the median is wide enough for a real walk band
				// plus a planter; otherwise fall back to a plain concrete median.
				constexpr double kMedianWalkBandCm   = 130.0; // concrete walk band width
				constexpr double kMinPlanterHalfCm   = 40.0;  // min planter half-width
				const bool bWalkPlanter = (Edge == ERoadNetMedianEdge::PlantableWalkCurb) &&
					(MedianHalf - kMedianWalkBandCm >= kMinPlanterHalfCm);

				TArray<FGeneralPolygon2d>& Dst = bWalk ? Ctx.ZoneMedianWalkPolys[z] : Ctx.ZoneMedianPolys[z];

				int32 i = 0;
				while (i < S.Num())
				{
					while (i < S.Num() && InClip(S[i])) { ++i; }
					const int32 RunStart = i;
					while (i < S.Num() && !InClip(S[i])) { ++i; }
					const int32 RunEnd = i;               // exclusive
					if (RunEnd - RunStart < 2) { continue; }

					// A run end that was cut by the junction region gets pulled back
					// and closed with a rounded nose. A FREE road end (nothing clipped
					// before/after) is capped flat (butt) so the median ends naturally
					// flush with the sidewalks at the end of the road, not as a bulge.
					const bool bStartAtJunction = (RunStart > 0);
					const bool bEndAtJunction   = (RunEnd < S.Num());
					const bool bRoundEnds       = bStartAtJunction || bEndAtJunction;

					TArray<FVector> Run(&S[RunStart], RunEnd - RunStart);
					if (bStartAtJunction) { TrimPolylineEnd(Run, /*fromStart*/true,  Pullback); }
					if (bEndAtJunction)   { TrimPolylineEnd(Run, /*fromStart*/false, Pullback); }
					if (Run.Num() < 2) { continue; }

					TArray<FGeneralPolygon2d> Outer;
					if (!RoadNetSurface::BuildPathRibbon(Run, MedianHalf, Outer, 16.0, bRoundEnds))
					{
						continue;
					}

					if (bWalkPlanter)
					{
						// Inner green planter (matching end caps, inset by the band).
						TArray<FGeneralPolygon2d> Inner;
						const bool bInner = RoadNetSurface::BuildPathRibbon(
							Run, MedianHalf - kMedianWalkBandCm, Inner, 16.0, bRoundEnds);

						// Concrete walk = outer MINUS planter → a curbed ring band.
						TArray<FGeneralPolygon2d> Band;
						if (bInner && Inner.Num() > 0 &&
							RoadNetSurface::Difference(Outer, Inner, Band) && Band.Num() > 0)
						{
							Ctx.ZoneMedianWalkPolys[z].Append(MoveTemp(Band));
							Ctx.ZoneMedianPolys[z].Append(MoveTemp(Inner)); // green centre
						}
						else
						{
							// Planter failed/degenerate → keep the whole island as walk.
							Ctx.ZoneMedianWalkPolys[z].Append(MoveTemp(Outer));
						}
					}
					else
					{
						Dst.Append(MoveTemp(Outer));
					}
				}
			}
		}

		// Safety net: subtract the junction region from the finished median polys
		// on EVERY side. The run-split + pull-back already keeps the rounded nose
		// clear of the junction, but coarse centreline sampling can let the
		// point-in-polygon split miss one approach, leaving the median poking into
		// the crossing on that side. This boolean clip guarantees symmetric
		// clearance (it is a no-op where the nose is already pulled back).
		if (Ctx.ZoneJunctionClip[z].Num() > 0)
		{
			using namespace UE::Geometry;
			auto ClipToJunction = [&](TArray<FGeneralPolygon2d>& Polys)
			{
				if (Polys.Num() == 0) { return; }
				TArray<FGeneralPolygon2d> Trimmed;
				if (RoadNetSurface::Difference(Polys, Ctx.ZoneJunctionClip[z], Trimmed))
				{
					Polys = MoveTemp(Trimmed);
				}
			};
			ClipToJunction(Ctx.ZoneMedianPolys[z]);
			ClipToJunction(Ctx.ZoneMedianWalkPolys[z]);
		}

		// ---- lane markings (§8.10): per-road stripes within this zone ----------
		// Which road ends meet a junction, so a crossing at a junction mouth does not
		// paint a second stop bar for traffic the junction has already released.
		// Ctx.Joints is built earlier in the rebuild; if it is empty nothing is
		// skipped, which is the old behaviour.
		TMap<int32, TPair<bool, bool>> JointEnds;
		for (const FRoadNetJoint& J : Ctx.Joints)
		{
			if (J.Arms.Num() < 2) { continue; }   // a terminal node is not a junction
			for (const FRoadNetJointArm& Arm : J.Arms)
			{
				TPair<bool, bool>& E = JointEnds.FindOrAdd(Arm.Road, TPair<bool, bool>(false, false));
				if (Arm.bAtStart) { E.Key = true; } else { E.Value = true; }
			}
		}

		TArray<UE::Geometry::FGeneralPolygon2d> White, Yellow;
		for (int32 RoadIdx : ZoneRoads)
		{
			const FRoadCurves* C = Ctx.Curves.Find(RoadIdx);
			if (C && Roads.IsValidIndex(RoadIdx))
			{
				const TPair<bool, bool>* Ends = JointEnds.Find(RoadIdx);
				RoadNetMarkings::BuildRoadMarkings(Roads[RoadIdx], *C, bDriveOnLeft,
					Ends ? Ends->Key : false, Ends ? Ends->Value : false, White, Yellow);
			}
		}

		// ---- white EDGE line from the carriageway boundary (§8.10) -------------
		// Instead of per-road straight offsets (which can't turn a corner), stroke
		// the merged surface boundary eroded inward by the edge inset. Because that
		// boundary already rounds around junction/sidewalk corners, the edge line
		// curves with them — and it covers hole (block) edges too.
		{
			using namespace UE::Geometry;
			constexpr double kEdgeInsetCm  = 38.0;  // must match RoadNetMarkings
			constexpr double kEdgeHalfCm   = 7.0;   // 14 cm solid line
			TArray<FGeneralPolygon2d> Eroded;
			if (Ctx.ZoneSurfacePolys[z].Num() > 0 &&
				PolygonsOffset(-kEdgeInsetCm, Ctx.ZoneSurfacePolys[z], Eroded,
					/*bCopyInputOnFailure*/false, /*MiterLimit*/2.0,
					EPolygonOffsetJoinType::Round, EPolygonOffsetEndType::Polygon,
					/*MaxStepsPerRadian*/16.0, /*DefaultStepsPerRadianScale*/1.0e-3))
			{
				auto Stroke = [&](const TArray<FVector2d>& Ring)
				{
					if (Ring.Num() < 3) { return; }
					TArray<FVector> Path;
					Path.Reserve(Ring.Num() + 1);
					for (const FVector2d& V : Ring) { Path.Emplace(V.X, V.Y, 0.0); }
					const FVector First = Path[0];
					Path.Add(First); // close the loop
					TArray<FGeneralPolygon2d> Strip;
					if (RoadNetSurface::BuildPathRibbon(Path, kEdgeHalfCm, Strip))
					{
						White.Append(MoveTemp(Strip));
					}
				};
				for (const FGeneralPolygon2d& GP : Eroded)
				{
					Stroke(GP.GetOuter().GetVertices());
					for (const TPolygon2<double>& H : GP.GetHoles()) { Stroke(H.GetVertices()); }
				}
			}
		}

		// Markings end at the junction (§2.1): subtract the true edge-line junction
		// area (plus stop-line setback), not a circular disc.
		const TArray<UE::Geometry::FGeneralPolygon2d>& ClipRegion = Ctx.ZoneJunctionClip[z];
		auto ClipToJunction = [&ClipRegion](TArray<UE::Geometry::FGeneralPolygon2d>& In,
			TArray<UE::Geometry::FGeneralPolygon2d>& Dst)
		{
			if (In.Num() > 0 && ClipRegion.Num() > 0) { RoadNetSurface::Difference(In, ClipRegion, Dst); }
			else                                      { Dst = MoveTemp(In); }
		};
		ClipToJunction(White,  Ctx.ZoneMarkingWhitePolys[z]);
		ClipToJunction(Yellow, Ctx.ZoneMarkingYellowPolys[z]);
	}
}

void URoadNetwork::BuildPerimeterLoops(FRoadNetRebuildContext& Ctx) const
{
	Ctx.PerimeterLoops.Reset();
	RoadNetPerimeters::ExtractLoops(Ctx.ZoneSurfacePolys, Ctx.Zones, Ctx.Curves, kRoadZLiftCm, Ctx.PerimeterLoops);
}

void URoadNetwork::BuildLaneGraph(FRoadNetRebuildContext& Ctx) const
{
	Ctx.LaneConnections.Reset();
	if (!bBuildLaneGraph) { return; }

	// §12.2 — RoadBLD ships no routing graph, so we build our own. The old
	// version connected every incoming lane to every outgoing one, which
	// produced a cross-product of movements: a 3x3 lane junction claimed nine
	// ways to go straight on. Real junctions match lanes by POSITION.
	//
	// כרך 2 §5.2.1: a through movement pairs lane-for-lane counting out from
	// the kerb, and the count must not fall — where it would, the surplus lane
	// has already been carried out of the junction by the channelization stage,
	// so here it simply has somewhere to go. Turns come off the lane the turn
	// is signed from: the kerb lane for a right turn (§6), the centre lane for
	// a left (§7).
	using namespace RoadNetJunctions;

	// Cache resolved lanes + their two centreline endpoints per road so a road
	// touching two joints is only offset once.
	//
	// Reserved up front because the matching below holds the entering arm's
	// cache entry while looking the leaving arm's up: without the reservation
	// that second insert could rehash and leave the first pointer dangling.
	// Only road indices are ever added, so Roads.Num() is the true upper bound.
	struct FRoadLaneCache { TArray<FRoadNetLane> Lanes; TArray<FVector> StartPt, EndPt; };
	TMap<int32, FRoadLaneCache> Cache;
	Cache.Reserve(Roads.Num());

	auto GetCache = [&](int32 RoadIdx) -> const FRoadLaneCache*
	{
		if (const FRoadLaneCache* Hit = Cache.Find(RoadIdx)) { return Hit; }
		const FRoadCurves* C = Ctx.Curves.Find(RoadIdx);
		if (!C || C->Sampled.Num() < 2 || !Roads.IsValidIndex(RoadIdx)) { return nullptr; }

		FRoadLaneCache New;
		New.Lanes = Roads[RoadIdx].Lanes.ResolveLanes(bDriveOnLeft);
		New.StartPt.Reserve(New.Lanes.Num());
		New.EndPt.Reserve(New.Lanes.Num());
		TArray<FVector> CL;
		for (const FRoadNetLane& L : New.Lanes)
		{
			RoadNetLanes::BuildLaneCenterline(C->Sampled, L, CL);
			New.StartPt.Add(CL.Num() > 0 ? CL[0] : C->Sampled[0]);
			New.EndPt.Add(CL.Num() > 0 ? CL.Last() : C->Sampled.Last());
		}
		return &Cache.Add(RoadIdx, MoveTemp(New));
	};

	int32 Reductions = 0;

	for (int32 ji = 0; ji < Ctx.Joints.Num(); ++ji)
	{
		const FRoadNetJoint& J = Ctx.Joints[ji];
		if (J.Arms.Num() < 2) { continue; } // a terminal end connects to nothing

		const bool bSeam = (J.Arms.Num() == 2);

		for (int32 ai = 0; ai < J.Arms.Num(); ++ai)
		{
			const FRoadNetJointArm& In = J.Arms[ai];
			const FRoadLaneCache* InC = GetCache(In.Road);
			if (!InC) { continue; }

			TArray<int32> Entering;
			OrderLanesFromKerb(InC->Lanes, In.bAtStart, /*bEntering*/true, bDriveOnLeft, Entering);
			if (Entering.Num() == 0) { continue; }

			for (int32 bi = 0; bi < J.Arms.Num(); ++bi)
			{
				if (bi == ai) { continue; }
				const FRoadNetJointArm& Out = J.Arms[bi];
				if (Out.Road == In.Road) { continue; }   // no U-turn back onto the same arm

				const FRoadLaneCache* OutC = GetCache(Out.Road);
				if (!OutC) { continue; }

				TArray<int32> Leaving;
				OrderLanesFromKerb(OutC->Lanes, Out.bAtStart, /*bEntering*/false, bDriveOnLeft, Leaving);
				if (Leaving.Num() == 0) { continue; }

				const EMovement Mv = Classify(In.BearingRad, Out.BearingRad);

				// Which entering lanes feed this movement, and which leaving
				// lanes they land in. Through pairs by kerb rank; a turn comes
				// off the one lane the standard signs it from.
				auto Emit = [&](int32 FromLane, int32 ToLane)
				{
					FRoadNetLaneConnection Cn;
					Cn.From.Road = In.Road;   Cn.From.Lane = FromLane;
					Cn.To.Road   = Out.Road;  Cn.To.Lane   = ToLane;
					Cn.Joint   = ji;
					Cn.Entry   = In.bAtStart  ? InC->StartPt[FromLane] : InC->EndPt[FromLane];
					Cn.Exit    = Out.bAtStart ? OutC->StartPt[ToLane]  : OutC->EndPt[ToLane];
					Cn.bThrough = (Mv == EMovement::Through);
					Cn.Kind = MovementToKind(Mv);
					Ctx.LaneConnections.Add(Cn);
				};

				switch (Mv)
				{
				case EMovement::Through:
				{
					// Lane i from the kerb continues as lane i from the kerb.
					const int32 Pairs = FMath::Min(Entering.Num(), Leaving.Num());
					for (int32 i = 0; i < Pairs; ++i) { Emit(Entering[i], Leaving[i]); }

					// §5.2.1 — a through movement must never lose a lane inside
					// the junction. Channelization widens the far arm so this
					// cannot happen; if it still does, the surplus traffic has
					// to merge somewhere, so send it to the kerb lane and say so.
					for (int32 i = Pairs; i < Entering.Num(); ++i)
					{
						Emit(Entering[i], Leaving[0]);
						++Reductions;
					}
					break;
				}
				case EMovement::Right:
					// §6 — the right turn is taken from the kerb lane and lands
					// in the kerb lane of the arm it turns into.
					Emit(Entering[0], Leaving[0]);
					break;

				case EMovement::Left:
					// §7 — the left turn is taken from the innermost lane and
					// lands in the innermost lane of the receiving arm.
					Emit(Entering.Last(), Leaving.Last());
					break;

				case EMovement::UTurn:
					Emit(Entering.Last(), Leaving.Last());
					break;

				default:
					break;
				}

				// A 2-arm seam is a continuation, so its through pairs are the
				// only movements; nothing else can be reached from it.
				if (bSeam) { break; }
			}
		}
	}

	if (Reductions > 0)
	{
		UE_LOG(LogRoadNet, Warning,
			TEXT("[RoadNet][LANES] %d through movement(s) still lose a lane inside a junction (כרך 2 §5.2.1). The drop belongs outside the junction on the rightmost lane — check bChannelizeJunctions and the arms' lane counts."),
			Reductions);
	}
}

bool URoadNetwork::IsTileInCommitScope(const FIntPoint& Coord, const FRoadNetRebuildContext& Ctx) const
{
	return Ctx.bFullCommit || Ctx.DirtyTiles.Contains(Coord);
}

void URoadNetwork::PrepareTilesForCommit(FRoadNetRebuildContext& Ctx)
{
	EnsureTileRegistry();
	if (Ctx.bFullCommit)
	{
		for (TPair<FIntPoint, TWeakObjectPtr<ARoadNetTileActor>>& KV : TileActors)
		{
			if (ARoadNetTileActor* T = KV.Value.Get()) { T->ClearForRebuild(); }
		}
	}
	else
	{
		for (const FIntPoint& C : Ctx.DirtyTiles)
		{
			if (TWeakObjectPtr<ARoadNetTileActor>* F = TileActors.Find(C))
			{
				if (ARoadNetTileActor* T = F->Get()) { T->ClearForRebuild(); }
			}
		}
	}
}

void URoadNetwork::RetireEmptyTiles(FRoadNetRebuildContext& Ctx)
{
	TArray<FIntPoint> ToRemove;
	for (TPair<FIntPoint, TWeakObjectPtr<ARoadNetTileActor>>& KV : TileActors)
	{
		ARoadNetTileActor* T = KV.Value.Get();
		if (!T) { ToRemove.Add(KV.Key); continue; }
		// Only retire tiles this pass was allowed to rewrite; clean tiles keep
		// whatever they already hold (a windowed edit must not delete them).
		const bool bConsider = Ctx.bFullCommit || Ctx.DirtyTiles.Contains(KV.Key);
		if (bConsider && T->IsEmptyTile()) { T->Destroy(); ToRemove.Add(KV.Key); }
	}
	for (const FIntPoint& C : ToRemove) { TileActors.Remove(C); }
}

int32 URoadNetwork::CommitMarkingDecals(
	FName LayerName,
	const TArray<TArray<UE::Geometry::FGeneralPolygon2d>>& ZonePolys,
	UMaterialInterface* DecalMaterial, FRoadNetRebuildContext& Ctx)
{
	if (!DecalMaterial || !WorldPtr.IsValid()) { return 0; }

	// Projection depth. Deep enough to survive the road's own grade and the
	// terrain conform underneath it, shallow enough not to reach a bridge deck or
	// an underpass roof passing overhead.
	constexpr double kDecalHalfDepthCm = 60.0;

	int32 Placed = 0;
	for (int32 z = 0; z < ZonePolys.Num(); ++z)
	{
		if (ZonePolys[z].Num() == 0 || !Ctx.Zones.IsValidIndex(z)) { continue; }

		TArray<const TArray<FVector>*> CenterLines;
		for (int32 RoadIdx : Ctx.Zones[z])
		{
			if (const FRoadCurves* C = Ctx.Curves.Find(RoadIdx)) { CenterLines.Add(&C->Sampled); }
		}
		RoadNetMesh::FCenterlineHeightField Field;
		Field.Build(CenterLines);
		const double FallbackZ = Field.FirstZ();

		for (const UE::Geometry::FGeneralPolygon2d& GP : ZonePolys[z])
		{
			const TArray<FVector2d>& V = GP.GetOuter().GetVertices();
			if (V.Num() < 3) { continue; }

			// Oriented box from the longest edge. Every mark in these banks is a
			// thin quad (a line, a dash, a bar, a zebra stripe), so the longest
			// edge IS the mark's length and this is exact.
			// ponytail: a non-convex mark (an arrow head) gets its bounding box
			// instead, which is the right footprint for a decal anyway — the
			// material's alpha carries the shape.
			int32 Longest = 0;
			double LongestLen2 = -1.0;
			for (int32 i = 0; i < V.Num(); ++i)
			{
				const double L2 = (V[(i + 1) % V.Num()] - V[i]).SizeSquared();
				if (L2 > LongestLen2) { LongestLen2 = L2; Longest = i; }
			}
			FVector2d Axis = V[(Longest + 1) % V.Num()] - V[Longest];
			if (!Axis.Normalize()) { continue; }
			const FVector2d Perp(-Axis.Y, Axis.X);

			double LoA = TNumericLimits<double>::Max(), HiA = -LoA;
			double LoP = TNumericLimits<double>::Max(), HiP = -LoP;
			for (const FVector2d& P : V)
			{
				const double A = P.Dot(Axis), B = P.Dot(Perp);
				LoA = FMath::Min(LoA, A); HiA = FMath::Max(HiA, A);
				LoP = FMath::Min(LoP, B); HiP = FMath::Max(HiP, B);
			}
			const double HalfLen  = 0.5 * (HiA - LoA);
			const double HalfWide = 0.5 * (HiP - LoP);
			if (HalfLen < 1.0 || HalfWide < 1.0) { continue; }   // sub-centimetre sliver

			const FVector2d C =
				Axis * (0.5 * (LoA + HiA)) + Perp * (0.5 * (LoP + HiP));
			const FVector Loc(C.X, C.Y,
				Field.SampleHeight(C.X, C.Y, FallbackZ) + kRoadZLiftCm + kMarkingLiftCm);

			const FIntPoint Coord = TopoKeyOf(Loc, Ctx);
			if (Coord.X == INDEX_NONE || !IsTileInCommitScope(Coord, Ctx)) { continue; }
			ARoadNetTileActor* Tile = GetOrCreateTile(Coord);
			if (!Tile) { continue; }

			// DecalSize is (depth, half-width across the yaw, half-length along it).
			if (Tile->AddRoadDecal(DecalMaterial, Loc,
				(float)FMath::RadiansToDegrees(FMath::Atan2(Axis.Y, Axis.X)),
				FVector(kDecalHalfDepthCm, HalfWide, HalfLen)))
			{
				++Placed;
			}
		}
	}

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] CommitMarkingDecals(%s): %d decals."),
		*LayerName.ToString(), Placed);
	return Placed;
}

int32 URoadNetwork::CommitLayer(
	FName LayerName,
	const TArray<TArray<UE::Geometry::FGeneralPolygon2d>>& ZonePolys,
	double ExtraLiftCm, FColor Color, UMaterialInterface* Material, FRoadNetRebuildContext& Ctx,
	bool bBakeLaneColors, bool bWorldUVs, bool bConformSurface, bool bSkirtToGround)
{
	UWorld* World = WorldPtr.Get();
	if (!World) { return 0; }

	// A zone drives the terrain conform only if ALL its roads are ground-level
	// (not a bridge/tunnel/elevated layer) — otherwise an overpass deck would
	// carve the ground up to its own height. Matches SculptCorridorsToBed's
	// IsDeformable rule.
	auto ZoneIsGround = [&](int32 z) -> bool
	{
		if (!Ctx.Zones.IsValidIndex(z)) { return false; }
		for (int32 r : Ctx.Zones[z])
		{
			if (Roads.IsValidIndex(r))
			{
				const FRoadDef& R = Roads[r];
				if (R.bBridge || R.bTunnel || R.Layer != 0) { return false; }
			}
		}
		return true;
	};

	// Baked per-lane shading colours (linear). Alternating asphalt banks plus a
	// base for junction fill / off-lane areas — same palette the old lifted
	// ribbon overlay used, now folded into the ONE carriageway mesh so nothing
	// z-intersects the road.
	const FVector3f BaseCol(FLinearColor(FColor(38, 38, 42)).R, FLinearColor(FColor(38, 38, 42)).G, FLinearColor(FColor(38, 38, 42)).B);
	const FVector3f EvenCol(FLinearColor(FColor(58, 58, 64)).R, FLinearColor(FColor(58, 58, 64)).G, FLinearColor(FColor(58, 58, 64)).B);
	const FVector3f OddCol (FLinearColor(FColor(44, 44, 49)).R, FLinearColor(FColor(44, 44, 49)).G, FLinearColor(FColor(44, 44, 49)).B);
	const bool bBake = bBakeLaneColors && bShowLaneRibbons;

	// Mesh each grade zone with ONLY its own centerline heights (so overpasses
	// keep their elevation), but CLIP each zone's polygons to every grid cell
	// they overlap and accumulate one mesh PER CELL — so the layer is committed
	// into the per-cell tile actors instead of one network-wide actor. Clipping
	// to the tile square (rather than assigning whole triangles) keeps tile
	// borders clean, and passing the FULL zone centrelines to every cell means
	// adjacent cells agree on Z at the shared border (no cracks in elevation).
	// Heap-allocated per cell so the mesh keeps a STABLE address: appending a new
	// cell can grow/rehash the map, and FDynamicMesh3's attribute overlays hold a
	// raw back-pointer to their parent mesh — relocating a by-value mesh would
	// leave those pointers dangling and crash the next append (SetTriangle).
	TMap<FIntPoint, TUniquePtr<UE::Geometry::FDynamicMesh3>> TileMeshes;
	int32 Tris = 0;
	// § tiling v2: layers bucketed at generation (BuildTilePartition) consume
	// their buckets directly — ownership was fixed at creation, nothing is
	// re-assigned here. Remaining (on-road overlay) layers use the simple
	// carve-cut + point-resolver fallback below.
	TArray<TMap<FIntPoint, TArray<UE::Geometry::FGeneralPolygon2d>>>* PreBuckets =
		Ctx.ZoneTileLayers.Find(LayerName);
	for (int32 z = 0; z < ZonePolys.Num(); ++z)
	{
		const bool bBucketed = (PreBuckets && PreBuckets->IsValidIndex(z));
		if (bBucketed ? ((*PreBuckets)[z].Num() == 0) : (ZonePolys[z].Num() == 0)) { continue; }

		TArray<const TArray<FVector>*> CenterLines;
		for (int32 RoadIdx : Ctx.Zones[z])
		{
			if (const FRoadCurves* C = Ctx.Curves.Find(RoadIdx)) { CenterLines.Add(&C->Sampled); }
		}

		// Per-zone lane-shade lookup: project the point onto the nearest road in
		// this zone, then pick the lane band it lands in. Scoped to zone roads so
		// the search stays small.
		const TArray<int32>& ZoneRoads = Ctx.Zones[z];
		// O(1) membership so the shared road grid's candidates can be filtered
		// back down to this zone.
		TSet<int32> ZoneRoadSet;
		ZoneRoadSet.Reserve(ZoneRoads.Num());
		ZoneRoadSet.Append(ZoneRoads);
		TFunction<FVector3f(double, double)> ShadeFn =
			[this, &Ctx, &ZoneRoads, &ZoneRoadSet, BaseCol, EvenCol, OddCol](double X, double Y) -> FVector3f
		{
			const FVector2D Q(X, Y);
			int32 BestRoad = INDEX_NONE;
			double BestDist = TNumericLimits<double>::Max();
			double BestOffset = 0.0;
			auto Consider = [&](int32 r)
			{
				const FRoadCurves* C = Ctx.Curves.Find(r);
				if (!C || C->Sampled.Num() < 2) { return; }
				const RoadNetMath::FProjectResult PR = RoadNetMath::ProjectToPolyline(C->Sampled, Q);
				if (PR.Distance < BestDist) { BestDist = PR.Distance; BestRoad = r; BestOffset = PR.Offset; }
			};
			// Narrow to nearby roads off the coarse sample grid (same 3x3 idiom as
			// SegTileKeyForPoint). Projecting every road in the zone per vertex made
			// this O(vertices x zone-roads x polyline-points) and stalled city commits.
			constexpr double CellCm = 2000.0;
			const FIntPoint Home(FMath::FloorToInt(X / CellCm), FMath::FloorToInt(Y / CellCm));
			TArray<int32, TInlineAllocator<32>> Cands;
			for (int32 dy = -1; dy <= 1; ++dy)
			{
				for (int32 dx = -1; dx <= 1; ++dx)
				{
					if (const TArray<int32>* Cell = Ctx.RoadSampleGrid.Find(FIntPoint(Home.X + dx, Home.Y + dy)))
					{
						for (int32 r : *Cell) { if (ZoneRoadSet.Contains(r)) { Cands.AddUnique(r); } }
					}
				}
			}
			for (int32 r : Cands) { Consider(r); }
			if (BestRoad == INDEX_NONE) { for (int32 r : ZoneRoads) { Consider(r); } }
			if (BestRoad == INDEX_NONE || !Roads.IsValidIndex(BestRoad)) { return BaseCol; }
			const TArray<FRoadNetLane> Lanes = Roads[BestRoad].Lanes.ResolveLanes(bDriveOnLeft);
			for (int32 i = 0; i < Lanes.Num(); ++i)
			{
				const double Lo = Lanes[i].CenterOffset - 0.5 * (double)Lanes[i].Width;
				const double Hi = Lanes[i].CenterOffset + 0.5 * (double)Lanes[i].Width;
				if (BestOffset >= Lo && BestOffset <= Hi) { return (i % 2 == 0) ? EvenCol : OddCol; }
			}
			return BaseCol;
		};

		const bool bZoneGround = bConformSurface && ZoneIsGround(z);

		// Fallback routing (on-road overlays only — markings/bike/parking/median):
		// carve junction parts to the junction tile, then resolve each remaining
		// piece by its centroid. Those pieces sit INSIDE one arm's carriageway, so
		// the point lookup is containment, not a guess. Offset geometry (sidewalks)
		// and the surface are bucketed at generation and never take this path.
		auto Centroid2D = [](const UE::Geometry::FGeneralPolygon2d& GP, FVector2D& Out) -> bool
		{
			const TArray<FVector2d>& OV = GP.GetOuter().GetVertices();
			if (OV.Num() < 3) { return false; }
			FVector2D C(0, 0);
			for (const FVector2d& V : OV) { C += FVector2D(V.X, V.Y); }
			Out = C / (double)OV.Num();
			return true;
		};

		// This zone's junction regions (bbox + stable key), for the cut. Uses the
		// CARVE (sidewalk-spanning) region so the sidewalk breaks at the junction
		// too, not just the carriageway.
		struct FZoneJun { FBox2D Box = FBox2D(ForceInit); TArray<UE::Geometry::FGeneralPolygon2d> Poly; FIntPoint Key = FIntPoint(0, 0); };
		TArray<FZoneJun> ZoneJuns;
		if (Ctx.ZoneJunctionCarve.IsValidIndex(z))
		{
			for (const UE::Geometry::FGeneralPolygon2d& GP : Ctx.ZoneJunctionCarve[z])
			{
				FVector2D C;
				if (!Centroid2D(GP, C)) { continue; }
				FZoneJun J;
				for (const FVector2d& V : GP.GetOuter().GetVertices()) { J.Box += FVector2D(V.X, V.Y); }
				J.Poly = { GP };
				J.Key = JunTileKey(C);
				ZoneJuns.Add(MoveTemp(J));
			}
		}

		TMap<FIntPoint, TArray<UE::Geometry::FGeneralPolygon2d>> ZoneTilePolys;
		if (bBucketed)
		{
			// § tiling v2: consume the pre-bucketed tile map (built at generation
			// with ownership fixed at creation), filtered to the commit window.
			for (TPair<FIntPoint, TArray<UE::Geometry::FGeneralPolygon2d>>& KV : (*PreBuckets)[z])
			{
				if (KV.Value.Num() == 0 || !IsTileInCommitScope(KV.Key, Ctx)) { continue; }
				ZoneTilePolys.Add(KV.Key, MoveTemp(KV.Value));
			}
		}
		else for (const UE::Geometry::FGeneralPolygon2d& GP : ZonePolys[z])
		{
			FBox2D PB(ForceInit);
			for (const FVector2d& V : GP.GetOuter().GetVertices()) { PB += FVector2D(V.X, V.Y); }
			if (!PB.bIsValid) { continue; }

			// Carve the junction part(s) out (only junctions whose bbox overlaps).
			TArray<UE::Geometry::FGeneralPolygon2d> Remainder = { GP };
			for (const FZoneJun& J : ZoneJuns)
			{
				if (Remainder.Num() == 0) { break; }
				if (!J.Box.bIsValid || !J.Box.Intersect(PB)) { continue; }
				TArray<UE::Geometry::FGeneralPolygon2d> Inter;
				if (PolygonsIntersection(Remainder, J.Poly, Inter) && Inter.Num() > 0)
				{
					if (IsTileInCommitScope(J.Key, Ctx)) { ZoneTilePolys.FindOrAdd(J.Key).Append(Inter); }
					TArray<UE::Geometry::FGeneralPolygon2d> Diff;
					if (RoadNetSurface::Difference(Remainder, J.Poly, Diff)) { Remainder = MoveTemp(Diff); }
				}
			}

			// Remaining pieces are on-road overlay ribbons INSIDE one arm's
			// carriageway — the centroid resolver is exact containment for them.
			for (UE::Geometry::FGeneralPolygon2d& Piece : Remainder)
			{
				FVector2D PC;
				if (!Centroid2D(Piece, PC)) { continue; }
				const FIntPoint Key = TopoKeyOf(FVector(PC.X, PC.Y, 0.0), Ctx);
				if (Key.X == INDEX_NONE || !IsTileInCommitScope(Key, Ctx)) { continue; }
				ZoneTilePolys.FindOrAdd(Key).Add(MoveTemp(Piece));
			}
		}

		for (TPair<FIntPoint, TArray<UE::Geometry::FGeneralPolygon2d>>& KV : ZoneTilePolys)
		{
			const FIntPoint Coord = KV.Key;
			TArray<UE::Geometry::FGeneralPolygon2d>& Polys = KV.Value;
			if (Polys.Num() == 0) { continue; }

			TUniquePtr<UE::Geometry::FDynamicMesh3>& TMPtr = TileMeshes.FindOrAdd(Coord);
			if (!TMPtr) { TMPtr = MakeUnique<UE::Geometry::FDynamicMesh3>(); }
			UE::Geometry::FDynamicMesh3& TM = *TMPtr;
			const int32 TID0 = TM.MaxTriangleID();
			Tris += RoadNetMesh::AppendSurfaceMesh(
				Polys, CenterLines, kRoadZLiftCm + ExtraLiftCm, TM,
				bBake ? &ShadeFn : nullptr,
				/*bComputeUVs*/true, /*UVUnitCm*/100.0, /*bGradientNormals*/true,
				/*bWorldUVs*/bWorldUVs, /*bSkirtToGround*/bSkirtToGround);

			// Terrain-conform soup (world cm; tiles are spawned at origin/identity),
			// cached PER CELL so a windowed rebuild can reassemble the whole-network
			// conform arrays from clean cells + this pass's dirty cells.
			if (bZoneGround)
			{
				FRoadNetTileConform& TC = ConformCache.FindOrAdd(Coord);
				for (int32 tid = TID0; tid < TM.MaxTriangleID(); ++tid)
				{
					if (!TM.IsTriangle(tid)) { continue; }
					// Skirt walls are near-vertical and dive BELOW the height field
					// on purpose. Conforming to them would sculpt a trench around
					// every kerb, so only the roughly horizontal top surface is a
					// valid target for the terrain.
					if (bSkirtToGround && FMath::Abs(TM.GetTriNormal(tid).Z) < 0.5) { continue; }
					const UE::Geometry::FIndex3i T = TM.GetTriangle(tid);
					TC.Verts.Add((FVector)TM.GetVertex(T.A));
					TC.Verts.Add((FVector)TM.GetVertex(T.B));
					TC.Verts.Add((FVector)TM.GetVertex(T.C));
				}
			}
		}
	}

	if (Tris == 0) { return 0; }
	// Normals are set from the height-field gradient inside AppendSurfaceMesh
	// (smooth, grade-following) — recomputing would overwrite them with the
	// jittery per-triangle average that caused the facet blotches.

	// Push each cell's accumulated mesh onto its tile actor's layer component.
	for (TPair<FIntPoint, TUniquePtr<UE::Geometry::FDynamicMesh3>>& KV : TileMeshes)
	{
		if (!KV.Value || KV.Value->TriangleCount() == 0) { continue; }
		ARoadNetTileActor* Tile = GetOrCreateTile(KV.Key);
		if (!Tile) { continue; }
		if (UDynamicMeshComponent* Comp =
			Tile->GetOrCreateMeshLayer(LayerName, Material, Color, bBakeLaneColors && bShowLaneRibbons))
		{
			if (LayerIsPaint(LayerName)) { Comp->SetCastShadow(false); }
			Comp->SetMesh(MoveTemp(*KV.Value));
			Comp->NotifyMeshUpdated();
		}
	}
	return Tris;
}

void URoadNetwork::BuildLaneRibbons(FRoadNetRebuildContext& Ctx) const
{
	const int32 NumZones = Ctx.Zones.Num();
	Ctx.ZoneLaneEvenPolys.Reset(); Ctx.ZoneLaneEvenPolys.SetNum(NumZones);
	Ctx.ZoneLaneOddPolys.Reset();  Ctx.ZoneLaneOddPolys.SetNum(NumZones);
	Ctx.ZoneLaneBikePolys.Reset(); Ctx.ZoneLaneBikePolys.SetNum(NumZones);
	Ctx.ZoneLaneParkPolys.Reset(); Ctx.ZoneLaneParkPolys.SetNum(NumZones);
	// Typed-lane overlays (bike/parking) are built regardless of the even/odd
	// shading ribbons, so only bail on an empty network.
	if (NumZones == 0) { return; }

	// Small per-side inset so adjacent lanes read as separate strips (a visible
	// seam) rather than one continuous slab.
	constexpr double kLaneGapCm = 8.0;
	int32 Ribbons = 0;
	const bool bHaveClip = (Ctx.ZoneJunctionClip.Num() == NumZones);
	for (int32 z = 0; z < NumZones; ++z)
	{
		for (int32 RoadIdx : Ctx.Zones[z])
		{
			const FRoadCurves* C = Ctx.Curves.Find(RoadIdx);
			if (!C || C->Sampled.Num() < 2 || !Roads.IsValidIndex(RoadIdx)) { continue; }

			const TArray<FRoadNetLane> Lanes = Roads[RoadIdx].Lanes.ResolveLanes(bDriveOnLeft);
			for (int32 i = 0; i < Lanes.Num(); ++i)
			{
				const FRoadNetLane& Ln = Lanes[i];

				// Thicken the lane centreline through Clipper (round joins, butt
				// ends) instead of looping two raw miter offsets — this dissolves
				// the inner-edge folds that produced spiky/dark triangles on bends.
				TArray<FVector> CL;
				RoadNetLanes::BuildLaneCenterline(C->Sampled, Ln, CL);

				// Even/odd shading ribbons (opt-in): contrasting asphalt banks.
				const double HalfInner = 0.5 * (double)Ln.Width - kLaneGapCm;
				if (bShowLaneRibbons && HalfInner >= 5.0)
				{
					TArray<UE::Geometry::FGeneralPolygon2d> Strip;
					if (RoadNetSurface::BuildPathRibbon(CL, HalfInner, Strip))
					{
						TArray<UE::Geometry::FGeneralPolygon2d>& Dst =
							((i % 2) == 0 ? Ctx.ZoneLaneEvenPolys : Ctx.ZoneLaneOddPolys)[z];
						for (UE::Geometry::FGeneralPolygon2d& GP : Strip)
						{
							Dst.Add(MoveTemp(GP));
							++Ribbons;
						}
					}
				}

				// Typed-lane overlay: bike / parking lanes get a fuller-coverage
				// strip in their own bank so they can be skinned distinctly.
				const bool bBike = (Ln.Type == ERoadNetLaneType::Bicycle);
				const bool bPark = (Ln.Type == ERoadNetLaneType::Parking);
				const double HalfTyped = 0.5 * (double)Ln.Width - 4.0;
				if ((bBike || bPark) && HalfTyped >= 5.0)
				{
					TArray<UE::Geometry::FGeneralPolygon2d> Strip;
					if (RoadNetSurface::BuildPathRibbon(CL, HalfTyped, Strip))
					{
						TArray<UE::Geometry::FGeneralPolygon2d>& Dst =
							(bBike ? Ctx.ZoneLaneBikePolys : Ctx.ZoneLaneParkPolys)[z];
						for (UE::Geometry::FGeneralPolygon2d& GP : Strip)
						{
							Dst.Add(MoveTemp(GP));
							++Ribbons;
						}
					}
				}
			}
		}

		// Lane strips stop at junctions (same §2.1 rule as markings): subtract the
		// true edge-line junction region so ribbons never run into a junction.
		if (bHaveClip && Ctx.ZoneJunctionClip[z].Num() > 0)
		{
			auto ClipBank = [&](TArray<UE::Geometry::FGeneralPolygon2d>& Bank)
			{
				if (Bank.Num() == 0) { return; }
				TArray<UE::Geometry::FGeneralPolygon2d> Clipped;
				if (RoadNetSurface::Difference(Bank, Ctx.ZoneJunctionClip[z], Clipped))
				{
					Bank = MoveTemp(Clipped);
				}
			};
			ClipBank(Ctx.ZoneLaneEvenPolys[z]);
			ClipBank(Ctx.ZoneLaneOddPolys[z]);
			ClipBank(Ctx.ZoneLaneBikePolys[z]);
			ClipBank(Ctx.ZoneLaneParkPolys[z]);
		}
	}
	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] BuildLaneRibbons: %d lane ribbons across %d zones."), Ribbons, NumZones);
}

void URoadNetwork::CommitGeometry(FRoadNetRebuildContext& Ctx)
{
	if (!WorldPtr.IsValid())
	{
		UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet] CommitGeometry: no world bound; skipping spawn."));
		return;
	}

	// ---- topological tiling scope (§ topo tiles) --------------------------
	// On a full rebuild every tile is recreated, so reset the id maps. Build this
	// pass's junction/segment accelerators (assigns junction ids). For a windowed
	// edit, re-express the commit SCOPE from grid cells to the segment + junction
	// tile keys actually touched — the grid corridor logic in DeterminePendingRoads
	// still chose WHICH roads are pending; here we only translate that to tiles.
	// A city-scale commit spends minutes inside these sub-stages with no output,
	// which is indistinguishable from a freeze. Trace each one as it finishes.
	const bool bTraceCommit = Roads.Num() > 50;
	double tC = FPlatformTime::Seconds();
	auto CTrace = [&](const TCHAR* Name)
	{
		const double Now = FPlatformTime::Seconds();
		if (bTraceCommit)
		{
			UE_LOG(LogRoadNet, Log, TEXT("[RoadNet]   commit %-13s %8.1f ms"), Name, (Now - tC) * 1000.0);
		}
		tC = Now;
	};

	if (Ctx.bFullCommit)
	{
		SegKeyOf.Reset(); JunKeyOf.Reset(); SegAlias.Reset();
		NextSegId = 0; NextJunId = 0;
		LastRoadJunctions.Reset();
	}
	BuildTopoAccel(Ctx);
	CTrace(TEXT("topoaccel"));
	// Divided-road pairing must run BEFORE any SegTileKey call this pass (the
	// windowed dirty-tile block below, plus every commit) so paired carriageways
	// resolve to the same tile everywhere.
	BuildDividedPairs(Ctx);
	CTrace(TEXT("dividedpairs"));
	// § tiling v2: generate each segment tile's surface + sidewalks from its own
	// arm run (ownership by construction) — CommitLayer consumes these buckets.
	BuildTilePartition(Ctx);
	CTrace(TEXT("tilepartition"));
	// Parcel access spurs go in HERE and nowhere earlier: the buckets they append to only exist
	// after the partition, and appending to Ctx.ZoneSidewalkPolys instead would hand them to the
	// kerb generator, the cycle-track carve and the furniture guard as well. See
	// RoadNetParcelAccess.cpp for why that separation is the point.
	BuildParcelAccessPaths(Ctx);
	CTrace(TEXT("parcelaccess"));
	if (!Ctx.bFullCommit)
	{
		TSet<FIntPoint> Topo;
		for (int32 r : Ctx.Pending)
		{
			if (!Roads.IsValidIndex(r) || !Roads[r].Id.IsValid()) { continue; }
			// A road can own several arm tiles (one per inter-junction stretch);
			// dirty every arm it currently has so the whole road is rewritten.
			TSet<int32> Arms;
			if (const TArray<int32>* A = Ctx.RoadSampleArm.Find(r))
			{
				for (int32 a : *A) { if (a >= 0) { Arms.Add(a); } }
			}
			if (Arms.Num() == 0) { Arms.Add(0); }
			for (int32 a : Arms) { Topo.Add(SegTileKey(Roads[r].Id, a)); }
		}
		for (const FRoadNetRebuildContext::FTopoJunctionRegion& JR : Ctx.TopoJunctions) { Topo.Add(JR.Key); }
		// Also dirty (clear + maybe retire) any junction a modified road touched
		// LAST rebuild but may have moved away from this time.
		for (int32 r : Ctx.Modified)
		{
			if (!Roads.IsValidIndex(r)) { continue; }
			if (const TArray<FIntPoint>* Old = LastRoadJunctions.Find(Roads[r].Id))
			{
				for (const FIntPoint& K : *Old) { Topo.Add(K); }
			}
		}
		Ctx.DirtyTiles = MoveTemp(Topo);
	}

	// Clear the tiles we're allowed to rewrite before repopulating (full rebuild
	// = all tiles; windowed = only the dirty segment/junction tiles).
	PrepareTilesForCommit(Ctx);
	CTrace(TEXT("preparetiles"));

	// Drop the terrain-conform cache for the cells we're about to rebuild (ground
	// surface layers repopulate them in CommitLayer); clean cells keep theirs.
	// The whole-network ConformVerts/ConformTris are reassembled from the cache
	// at the end of this function.
	if (Ctx.bFullCommit) { ConformCache.Reset(); }
	else { for (const FIntPoint& C : Ctx.DirtyTiles) { ConformCache.Remove(C); } }

	// Road carriageway (dark asphalt) and sidewalk band (light concrete, raised
	// one curb height above the road so the kerb reads correctly). Lane shading
	// is BAKED into the carriageway's vertex colours (bBakeLaneColors=true) so
	// lanes no longer need a separate lifted overlay that dove in/out of the road.
	const int32 RoadTris = CommitLayer(TEXT("Surface"),
		Ctx.ZoneSurfacePolys, /*ExtraLift*/0.0, FColor(38, 38, 42), RoadMaterial, Ctx,
		/*bBakeLaneColors*/true, /*bWorldUVs*/false, /*bConformSurface*/true);
	CTrace(TEXT("layer:surface"));
	const int32 WalkTris = CommitLayer(TEXT("Sidewalks"),
		Ctx.ZoneSidewalkPolys, /*ExtraLift*/15.0, FColor(165, 162, 155), SidewalkMaterial, Ctx,
		/*bBakeLaneColors*/false, /*bWorldUVs*/false, /*bConformSurface*/true,
		/*bSkirtToGround*/true);
	// Outboard cycle track: sits at footway level (not road level) because it is
	// on the far side of the kerb, so it gets the same lift and the same skirt.
	const int32 TrackTris = CommitLayer(TEXT("BikePath"),
		Ctx.ZoneBikePathPolys, /*ExtraLift*/15.0, FColor(64, 132, 88), BikeLaneMaterial, Ctx,
		/*bBakeLaneColors*/false, /*bWorldUVs*/false, /*bConformSurface*/true,
		/*bSkirtToGround*/true);
	CTrace(TEXT("layer:walks"));
	// Paint sits kMarkingLiftCm (1 cm) proud of the asphalt, not the 4 cm it used to:
	// both are draped off the same centrelines, so 4 cm was pure step, visible at
	// human height as markings hovering over the road.
	//
	// Markings are EITHER meshed ribbons or projected decals, never both — the
	// two would z-fight over the same centimetre of road.
	int32 WhiteTris = 0, YellowTris = 0;
	if (bMarkingsAsDecals)
	{
		CommitMarkingDecals(TEXT("MarkingsWhite"), Ctx.ZoneMarkingWhitePolys,
			MarkingWhiteDecal.LoadSynchronous(), Ctx);
		CommitMarkingDecals(TEXT("MarkingsYellow"), Ctx.ZoneMarkingYellowPolys,
			MarkingYellowDecal.LoadSynchronous(), Ctx);
	}
	else
	{
		WhiteTris = CommitLayer(TEXT("MarkingsWhite"),
			Ctx.ZoneMarkingWhitePolys, kMarkingLiftCm, FColor(232, 232, 226), MarkingWhiteMaterial, Ctx);
		YellowTris = CommitLayer(TEXT("MarkingsYellow"),
			Ctx.ZoneMarkingYellowPolys, kMarkingLiftCm, FColor(240, 190, 30), MarkingYellowMaterial, Ctx);
	}
	CTrace(TEXT("layer:markings"));

	// Typed-lane overlays: bike paths (green) + parking bays (amber) as thin
	// surfaces lifted a hair above the carriageway, skinned with their Assets-tab
	// material when set (else the tint fallback). Not part of the terrain conform
	// (they ride the road surface). Empty layers stay empty (cleared above), so
	// removing the last bike/parking lane clears its surface.
	//
	// These sit BELOW the paint (kLaneOverlayLiftCm against kMarkingLiftCm). They
	// used to be lifted above it, which buried the very lines that make them
	// readable: a parking bay's stall dividers are generated into the white bank by
	// BuildStandardParkingBays and were then covered by the bay surface drawn on
	// top of them, so the bays rendered as blank slabs. Paint goes ON a surface.
	// The two tiers dropped together when the paint came down to 1 cm — keeping the
	// overlay at 2 cm would have re-buried the dividers.
	const int32 BikeTris = CommitLayer(TEXT("LanesBike"),
		Ctx.ZoneLaneBikePolys, kLaneOverlayLiftCm, FColor(60, 170, 90), BikeLaneMaterial, Ctx,
		/*bBakeLaneColors*/false, /*bWorldUVs*/false, /*bConformSurface*/false);
	const int32 ParkTris = CommitLayer(TEXT("LanesParking"),
		Ctx.ZoneLaneParkPolys, kLaneOverlayLiftCm, FColor(200, 165, 45), ParkingMaterial, Ctx,
		/*bBakeLaneColors*/false, /*bWorldUVs*/false, /*bConformSurface*/false);

	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet] CommitGeometry: road %d tris (lane shading baked), sidewalk %d tris, cycle track %d tris, markings %d white + %d yellow tris, bike lane %d + parking %d tris."),
		RoadTris, WalkTris, TrackTris, WhiteTris, YellowTris, BikeTris, ParkTris);

	// Kerb line rides on the same merged surface + sidewalk polys, so it must be
	// committed AFTER the sidewalk band exists (it reads Ctx.ZoneSidewalkPolys).
	CommitCurbs(Ctx);
	CTrace(TEXT("curbs"));
	CommitFurniture(Ctx);         // § street furniture (HISM instances / spawned Blueprint actors)
	CommitJunctionSignals(Ctx);   // § traffic-signal placeholders at signalized junctions
	CommitLaneMarks(Ctx);         // turn arrows from LaneConnections + role filter
	CommitPlacedMarks(Ctx);       // hand-placed road marks (independent of the above)
	CommitMedian(Ctx);            // § raised median strip + centre planting splines
	CommitPerimeters(Ctx);
	CommitLaneGraph(Ctx);
	CommitSegmentSplines(Ctx);    // § per-segment editable centre + edge splines
	PublishZoneGraphRuns(Ctx);    // § trimmed centrelines for the ZoneGraph builder
	CTrace(TEXT("furniture+etc"));

	// Retire any cell this pass emptied out (all its layers/instances/splines
	// gone). Clean tiles outside the commit scope are left untouched.
	RetireEmptyTiles(Ctx);

	// Record which junctions each PENDING road contributed to this pass, so the
	// next windowed edit of that road can dirty (and retire) those junction tiles
	// if it moves away. Only pending roads are refreshed; untouched roads keep
	// their record.
	for (int32 r : Ctx.Pending)
	{
		if (Roads.IsValidIndex(r) && Roads[r].Id.IsValid()) { LastRoadJunctions.Remove(Roads[r].Id); }
	}
	for (const FRoadNetRebuildContext::FTopoJunctionRegion& JR : Ctx.TopoJunctions)
	{
		if (!Ctx.Zones.IsValidIndex(JR.Zone)) { continue; }
		for (int32 r : Ctx.Zones[JR.Zone])
		{
			if (Roads.IsValidIndex(r) && Roads[r].Id.IsValid())
			{
				LastRoadJunctions.FindOrAdd(Roads[r].Id).AddUnique(JR.Key);
			}
		}
	}

	// Reassemble the whole-network terrain-conform soup from the per-cell cache
	// (clean cells + this pass's dirty cells), dropping any retired cell, so the
	// landscape sculpt (OSMOverpassRoadImport GetConformTris) always receives the
	// complete ground surface — never just the edited window.
	for (auto It = ConformCache.CreateIterator(); It; ++It)
	{
		if (!TileActors.Contains(It.Key())) { It.RemoveCurrent(); }
	}
	ConformVerts.Reset();
	ConformTris.Reset();
	for (const TPair<FIntPoint, FRoadNetTileConform>& KV : ConformCache)
	{
		const int32 Base = ConformVerts.Num();
		ConformVerts.Append(KV.Value.Verts);
		for (int32 i = 0; i < KV.Value.Verts.Num(); ++i) { ConformTris.Add(Base + i); }
	}
}

// Close a kerb ring in place, if it is not closed already.
//
// The copy is the whole point. A TArray that was just copy-constructed has
// capacity EXACTLY equal to its length, so closing it always reallocates — and
// TArray::Add asserts (rightly) when handed an element of the array it is about
// to move out from under itself. Passing Ring[0] straight in reads freed memory.
// Both kerb rings below close a loop, and only one of them used to copy first,
// so an island whose ring exactly filled its allocation crashed the editor.
static void CloseRingInPlace(TArray<FVector>& Ring, double TolCm = 1.0)
{
	if (Ring.Num() < 2 || Ring[0].Equals(Ring.Last(), TolCm)) { return; }
	const FVector First = Ring[0];
	Ring.Add(First);
}

void URoadNetwork::CommitCurbs(FRoadNetRebuildContext& Ctx)
{
	if (!WorldPtr.IsValid()) { return; }

	// Default to the PCG kerb kit mesh; fall back to an engine cube only if that
	// asset can't be found, so the kerb line is always visible.
	UStaticMesh* Mesh = CurbMesh ? CurbMesh.Get() : nullptr;
	const bool bUserMesh = (Mesh != nullptr);
	if (!Mesh)
	{
		Mesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Game/PCG/Assets/Meshes/SM_Curb2.SM_Curb2"));
	}
	if (!Mesh)
	{
		Mesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	}

	// (Tiles were cleared in PrepareTilesForCommit, so an early return simply
	// leaves the dirty cells with no kerb HISM this pass.)
	if (!bBuildCurbs || !Mesh) { return; }

	// Gather kerb placements from every zone (each zone samples its own heights
	// so overpasses keep their elevation).
	TArray<RoadNetCurbs::FCurbInstance> Insts;
	for (int32 z = 0; z < Ctx.Zones.Num(); ++z)
	{
		if (!Ctx.ZoneSurfacePolys.IsValidIndex(z) || !Ctx.ZoneSidewalkPolys.IsValidIndex(z)) { continue; }
		if (Ctx.ZoneSurfacePolys[z].Num() == 0 || Ctx.ZoneSidewalkPolys[z].Num() == 0)       { continue; }

		TArray<const TArray<FVector>*> CenterLines;
		for (int32 RoadIdx : Ctx.Zones[z])
		{
			if (const FRoadCurves* C = Ctx.Curves.Find(RoadIdx)) { CenterLines.Add(&C->Sampled); }
		}
		RoadNetMesh::FCenterlineHeightField Field;
		Field.Build(CenterLines);

		RoadNetCurbs::BuildCurbInstancesForZone(
			Ctx.ZoneSurfacePolys[z], Ctx.ZoneSidewalkPolys[z], Field,
			CurbSpacingCm, kRoadZLiftCm, Insts);
	}

	// Median-island kerbs: a constant-width kerb wrapping ONLY the OUTER perimeter
	// of each island (the carriageway side). We union the grass + concrete-band
	// polygons per zone and trace just the union's outer ring, so there is never
	// an inner kerb between the grass and its sidewalk band — the grass meets the
	// walk flush. The outline is already junction-clipped and round-capped, so the
	// kerb follows the nose and stops with the strip. Walking the ring counter-
	// clockwise puts the carriageway on the kerb's right (what the builder wants).
	TArray<UE::Geometry::FGeneralPolygon2d> IslandPathCuts;
	for (const FRoadNetIsland& Isl : PlacedIslands)
	{
		for (const FRoadNetIslandPath& Path : Isl.Paths)
		{
			TArray<FVector> CL;
			CL.Add(FVector(Path.A.X, Path.A.Y, 0.0));
			CL.Add(FVector(Path.B.X, Path.B.Y, 0.0));
			TArray<UE::Geometry::FGeneralPolygon2d> Ribbon;
			if (RoadNetSurface::BuildPathRibbon(CL, 0.5 * (double)Path.WidthCm, Ribbon, 16.0, true))
			{
				IslandPathCuts.Append(Ribbon);
			}
		}
	}

	for (int32 z = 0; z < Ctx.Zones.Num(); ++z)
	{
		const bool bHasSoil = Ctx.ZoneMedianPolys.IsValidIndex(z)     && Ctx.ZoneMedianPolys[z].Num()     > 0;
		const bool bHasWalk = Ctx.ZoneMedianWalkPolys.IsValidIndex(z) && Ctx.ZoneMedianWalkPolys[z].Num() > 0;
		if (!bHasSoil && !bHasWalk) { continue; }

		TArray<const TArray<FVector>*> CenterLines;
		for (int32 RoadIdx : Ctx.Zones[z])
		{
			if (const FRoadCurves* C = Ctx.Curves.Find(RoadIdx)) { CenterLines.Add(&C->Sampled); }
		}
		RoadNetMesh::FCenterlineHeightField Field;
		Field.Build(CenterLines);

		// Merge grass + walk band into the full island footprint(s).
		TArray<UE::Geometry::FGeneralPolygon2d> Island;
		{
			TArray<UE::Geometry::FGeneralPolygon2d> All;
			if (bHasSoil) { All.Append(Ctx.ZoneMedianPolys[z]); }
			if (bHasWalk) { All.Append(Ctx.ZoneMedianWalkPolys[z]); }
			if (!RoadNetSurface::Union(All, Island)) { Island = MoveTemp(All); }
			if (IslandPathCuts.Num() > 0)
			{
				TArray<UE::Geometry::FGeneralPolygon2d> Cut;
				if (RoadNetSurface::Difference(Island, IslandPathCuts, Cut) && Cut.Num() > 0)
				{
					Island = MoveTemp(Cut);
				}
			}
		}

		for (const UE::Geometry::FGeneralPolygon2d& Poly : Island)
		{
			const UE::Geometry::FPolygon2d& Outer = Poly.GetOuter();
			const TArray<FVector2d>& V = Outer.GetVertices();
			const int32 N = V.Num();
			if (N < 3) { continue; }
			TArray<FVector> Ring;
			Ring.Reserve(N + 1);
			// Ensure CCW so the kerb face turns outward toward the carriageway.
			const bool bCCW = (Outer.SignedArea() > 0.0);
			for (int32 v = 0; v < N; ++v)
			{
				const FVector2d& P = V[bCCW ? v : (N - 1 - v)];
				Ring.Add(FVector(P.X, P.Y, 0.0));
			}
			CloseRingInPlace(Ring);
			RoadNetCurbs::BuildCurbInstancesAlongLine(Ring, Field, CurbSpacingCm, kRoadZLiftCm, Insts);
		}
	}

	if (Insts.Num() == 0) { return; }

	// Per-cell zebra kerb HISMs (piece i → A / B), routed by each piece's world
	// position into its tile actor. Each cell gets its own CurbA/CurbB pair
	// (materials 0/1 respectively), created on first use and reused thereafter.
	auto TileHISM = [&](const FIntPoint& Coord, bool bA) -> UHierarchicalInstancedStaticMeshComponent*
	{
		ARoadNetTileActor* Tile = GetOrCreateTile(Coord);
		if (!Tile) { return nullptr; }
		return Tile->GetOrCreateHISM(bA ? FName(TEXT("CurbA")) : FName(TEXT("CurbB")),
			Mesh, bA ? CurbMaterial0.Get() : CurbMaterial1.Get());
	};

	// ---- fit SM_Curb2 onto the kerb line ----------------------------------
	// SM_Curb2 needs a 180° yaw so its raised face turns toward the road; we
	// also scale it to a target section (height/width) and seat its bottom-centre
	// on the kerb line, nudged toward the sidewalk so the road face meets the
	// carriageway edge. Tunables are local so this stays a Live-Coding change.
	constexpr double kCurbTargetHeightCm = 15.0;   // matches the +15 sidewalk lift
	constexpr double kCurbTargetWidthCm  = 18.0;
	constexpr double kCurbYawOffsetDeg   = 180.0;  // "rotated 180" to face the road
	// Seat the kerb a bit INWARD of the carriageway edge (toward the road) so it
	// reads as the road edge instead of sitting out on the sidewalk. +LeftN is
	// the sidewalk side, so a net-negative offset shifts the piece onto the road.
	constexpr double kCurbInwardCm       = 15.0;   // how far inward from the edge
	const double kCurbLateralNudgeCm     = 0.5 * kCurbTargetWidthCm - kCurbInwardCm;

	const FBox Box      = Mesh->GetBoundingBox();
	const FVector Size  = Box.GetSize();
	const FVector Ctr   = Box.GetCenter();
	const bool bLongIsX = Size.X >= Size.Y;
	const double MeshLong = FMath::Max(1.0, bLongIsX ? Size.X : Size.Y);
	const double MeshWide = FMath::Max(1.0, bLongIsX ? Size.Y : Size.X);
	const double MeshTall = FMath::Max(1.0, Size.Z);

	int32 iCurb = 0, nA = 0, nB = 0;
	// Batch instances per HISM: HISM rebuilds its cluster tree on every single
	// AddInstance (O(n^2) for a big city). AddInstances() builds the tree ONCE.
	TMap<UHierarchicalInstancedStaticMeshComponent*, TArray<FTransform>> CurbBatches;
	for (const RoadNetCurbs::FCurbInstance& CI : Insts)
	{
		const double sLong = (double)CI.LengthCm / MeshLong;
		const double sWide = kCurbTargetWidthCm  / MeshWide;
		const double sTall = kCurbTargetHeightCm / MeshTall;
		const FVector Scale = bLongIsX ? FVector(sLong, sWide, sTall) : FVector(sWide, sLong, sTall);
		const float Yaw = CI.YawDeg + (bLongIsX ? 0.f : 90.f) + (float)kCurbYawOffsetDeg;

		// Tilt the piece to follow the longitudinal grade. Applied in WORLD space
		// about the horizontal axis perpendicular to travel, so the whole oriented
		// kerb (whatever its local long axis) rotates as one — its ends then land
		// on the sloped ground instead of the run staircasing. Composed AFTER the
		// in-plane yaw (Q = pitch * yaw).
		const double PlaceYawRad = FMath::DegreesToRadians((double)CI.YawDeg);
		const FVector PitchAxis(FMath::Sin(PlaceYawRad), -FMath::Cos(PlaceYawRad), 0.0);
		const FQuat QPitch(PitchAxis, FMath::DegreesToRadians((double)CI.PitchDeg));
		const FQuat QRot = QPitch * FRotator(0.f, Yaw, 0.f).Quaternion();

		// Seat the mesh's bottom-centre (in local space) onto the target world
		// point: TransformVector applies scale+rotation only (no translation).
		FTransform Inst(QRot, FVector::ZeroVector, Scale);
		const FVector AnchorWorld = Inst.TransformVector(FVector(Ctr.X, Ctr.Y, Box.Min.Z));

		const FVector LeftN(-FMath::Sin(PlaceYawRad), FMath::Cos(PlaceYawRad), 0.0); // sidewalk side
		const FVector Target = CI.Location + LeftN * kCurbLateralNudgeCm;
		Inst.SetTranslation(Target - AnchorWorld);

		const FIntPoint Coord = TopoKeyOf(CI.Location, Ctx);
		if (Coord.X == INDEX_NONE || !IsTileInCommitScope(Coord, Ctx)) { ++iCurb; continue; }

		ERoadNetCurbPaintType Paint;
		UHierarchicalInstancedStaticMeshComponent* H = nullptr;
		if (LookupCurbPaint(CI.Location, Paint))
		{
			UMaterialInterface* PaintMat =
				(Paint == ERoadNetCurbPaintType::WhiteBlack) ? (CurbPaintWhiteBlack ? CurbPaintWhiteBlack.Get() : MarkingWhiteMaterial.Get()) :
				(Paint == ERoadNetCurbPaintType::WhiteRed)   ? (CurbPaintWhiteRed ? CurbPaintWhiteRed.Get() : MarkingYellowMaterial.Get()) :
				(Paint == ERoadNetCurbPaintType::WhiteBlue)  ? CurbPaintWhiteBlue.Get() :
				(CurbPaintGray ? CurbPaintGray.Get() : CurbMaterial1.Get());
			if (!PaintMat)
			{
				PaintMat = LoadObject<UMaterialInterface>(nullptr,
					TEXT("/Engine/EngineMaterials/DefaultWhiteGrid.DefaultWhiteGrid"));
			}
			ARoadNetTileActor* Tile = GetOrCreateTile(Coord);
			H = Tile ? Tile->GetOrCreateHISM(CurbPaintHISMName(Paint), Mesh, PaintMat) : nullptr;
			if (H && PaintMat)
			{
				const int32 Slots = Mesh->GetStaticMaterials().Num();
				for (int32 s = 0; s < FMath::Max(1, Slots); ++s) { H->SetMaterial(s, PaintMat); }
			}
		}
		else
		{
			const bool bA = ((iCurb++ & 1) == 0);
			H = TileHISM(Coord, bA);
			if (H) { if (bA) { ++nA; } else { ++nB; } }
		}
		if (H) { CurbBatches.FindOrAdd(H).Add(Inst); }
	}
	for (TPair<UHierarchicalInstancedStaticMeshComponent*, TArray<FTransform>>& KV : CurbBatches)
	{
		KV.Key->AddInstances(KV.Value, /*bShouldReturnIndices*/false, /*bWorldSpace*/true);
	}

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] CommitCurbs: %d kerb instances (zebra %d/%d)%s."),
		Insts.Num(), nA, nB, bUserMesh ? TEXT("") : TEXT(" [SM_Curb2 default]"));
}

// ---- junction markings (§2 junctions) -------------------------------------

namespace
{
	// Advance/reverse a preset through the cycle order.
	ERoadNetJunctionPreset CyclePreset(ERoadNetJunctionPreset P, int32 Dir)
	{
		constexpr int32 N = 5; // None..GiveWay
		int32 v = (int32)P + (Dir >= 0 ? 1 : -1);
		v = ((v % N) + N) % N;
		return (ERoadNetJunctionPreset)v;
	}

	bool PresetHasCrosswalk(ERoadNetJunctionPreset P)
	{
		return P == ERoadNetJunctionPreset::StopAndCrosswalk
			|| P == ERoadNetJunctionPreset::Signalized;
	}

	// A CCW rectangle centred at C, ±HalfU along unit axis U, ±HalfV across.
	UE::Geometry::FGeneralPolygon2d MakeRectPoly(const FVector2D& C, const FVector2D& U, double HalfU, double HalfV)
	{
		const FVector2D V(-U.Y, U.X);
		TArray<FVector2d> Loop;
		Loop.Reserve(4);
		Loop.Emplace(C.X - U.X * HalfU - V.X * HalfV, C.Y - U.Y * HalfU - V.Y * HalfV);
		Loop.Emplace(C.X + U.X * HalfU - V.X * HalfV, C.Y + U.Y * HalfU - V.Y * HalfV);
		Loop.Emplace(C.X + U.X * HalfU + V.X * HalfV, C.Y + U.Y * HalfU + V.Y * HalfV);
		Loop.Emplace(C.X - U.X * HalfU + V.X * HalfV, C.Y - U.Y * HalfU + V.Y * HalfV);
		UE::Geometry::FPolygon2d P(Loop);
		if (P.IsClockwise()) { P.Reverse(); }
		UE::Geometry::FGeneralPolygon2d G;
		G.SetOuter(P);
		return G;
	}
}

ERoadNetJunctionPreset URoadNetwork::ResolveJunctionPresetNear(const FVector2D& Loc) const
{
	double BestD2 = FMath::Square(kJunctionMatchCm);
	ERoadNetJunctionPreset Best = ERoadNetJunctionPreset::None;
	for (const FRoadNetJunctionConfig& Cfg : JunctionConfigs)
	{
		const double D2 = FVector2D::DistSquared(Cfg.Location, Loc);
		if (D2 < BestD2) { BestD2 = D2; Best = Cfg.Preset; }
	}
	return Best;
}

ERoadNetJunctionPreset URoadNetwork::CycleJunctionPresetNear(const FVector2D& Loc, int32 Dir)
{
	Modify();
	int32 BestIdx = INDEX_NONE;
	double BestD2 = FMath::Square(kJunctionMatchCm);
	for (int32 i = 0; i < JunctionConfigs.Num(); ++i)
	{
		const double D2 = FVector2D::DistSquared(JunctionConfigs[i].Location, Loc);
		if (D2 < BestD2) { BestD2 = D2; BestIdx = i; }
	}
	if (BestIdx == INDEX_NONE)
	{
		FRoadNetJunctionConfig Cfg;
		Cfg.Location = Loc;
		Cfg.Preset = ERoadNetJunctionPreset::None;
		BestIdx = JunctionConfigs.Add(Cfg);
	}
	FRoadNetJunctionConfig& C = JunctionConfigs[BestIdx];
	C.Location = Loc; // re-anchor to the live junction position
	C.Preset = CyclePreset(C.Preset, Dir);
	return C.Preset;
}

bool URoadNetwork::ResolveJunctionIslandsNear(const FVector2D& Loc) const
{
	double BestD2 = FMath::Square(kJunctionMatchCm);
	bool bBest = false;
	for (const FRoadNetJunctionConfig& Cfg : JunctionConfigs)
	{
		const double D2 = FVector2D::DistSquared(Cfg.Location, Loc);
		if (D2 < BestD2) { BestD2 = D2; bBest = Cfg.bCornerIslands; }
	}
	return bBest;
}

bool URoadNetwork::ToggleJunctionIslandsNear(const FVector2D& Loc)
{
	Modify();
	int32 BestIdx = INDEX_NONE;
	double BestD2 = FMath::Square(kJunctionMatchCm);
	for (int32 i = 0; i < JunctionConfigs.Num(); ++i)
	{
		const double D2 = FVector2D::DistSquared(JunctionConfigs[i].Location, Loc);
		if (D2 < BestD2) { BestD2 = D2; BestIdx = i; }
	}
	if (BestIdx == INDEX_NONE)
	{
		FRoadNetJunctionConfig Cfg;
		Cfg.Location = Loc;
		BestIdx = JunctionConfigs.Add(Cfg);
	}
	FRoadNetJunctionConfig& C = JunctionConfigs[BestIdx];
	C.Location = Loc; // re-anchor to the live junction position
	C.bCornerIslands = !C.bCornerIslands;
	return C.bCornerIslands;
}

double URoadNetwork::ResolveJunctionSmoothingNear(const FVector2D& Loc) const
{
	double BestD2 = FMath::Square(kJunctionMatchCm);
	double Best = JunctionSmoothingCm;   // network default
	for (const FRoadNetJunctionConfig& Cfg : JunctionConfigs)
	{
		if (Cfg.SmoothingCm < 0.f) { continue; }   // no override here
		const double D2 = FVector2D::DistSquared(Cfg.Location, Loc);
		if (D2 < BestD2) { BestD2 = D2; Best = Cfg.SmoothingCm; }
	}
	return Best;
}

double URoadNetwork::AdjustJunctionSmoothingNear(const FVector2D& Loc, double DeltaCm, FVector2D& OutJunctionLoc)
{
	Modify();
	int32 BestIdx = INDEX_NONE;
	double BestD2 = FMath::Square(kJunctionMatchCm);
	for (int32 i = 0; i < JunctionConfigs.Num(); ++i)
	{
		const double D2 = FVector2D::DistSquared(JunctionConfigs[i].Location, Loc);
		if (D2 < BestD2) { BestD2 = D2; BestIdx = i; }
	}
	if (BestIdx == INDEX_NONE)
	{
		FRoadNetJunctionConfig Cfg;
		Cfg.Location = Loc;
		BestIdx = JunctionConfigs.Add(Cfg);
	}
	FRoadNetJunctionConfig& C = JunctionConfigs[BestIdx];
	C.Location = Loc; // re-anchor to the live junction position
	// Seed the override from the current effective value on first touch so the
	// first nudge steps relative to what the junction already shows.
	const double Cur = (C.SmoothingCm >= 0.f) ? (double)C.SmoothingCm : JunctionSmoothingCm;
	C.SmoothingCm = (float)FMath::Clamp(Cur + DeltaCm, 0.0, 300.0);
	OutJunctionLoc = C.Location;
	return C.SmoothingCm;
}

void URoadNetwork::BuildJunctionIslands(FRoadNetRebuildContext& Ctx) const
{
	using namespace UE::Geometry;

	// A channelizing island is the junction PAVEMENT between two angularly
	// adjacent arms, minus the arm corridors themselves, eroded inward so
	// turning traffic passes around it. Built per junction (one clip polygon)
	// and appended to the median layer so it inherits the grass mesh + kerb ring
	// + world-planar UVs automatically. Only junctions with the toggle set emit.
	const double Inset = FMath::Max(10.0, JunctionIslandInsetCm);
	constexpr double kMinIslandAreaCm2 = 3.0e4; // ~3 m² — drop slivers

	for (int32 z = 0; z < Ctx.ZoneJunctionClip.Num(); ++z)
	{
		if (!Ctx.Zones.IsValidIndex(z) || !Ctx.ZoneSurfacePolys.IsValidIndex(z)) { continue; }
		if (Ctx.ZoneJunctionClip[z].Num() == 0 || Ctx.ZoneSurfacePolys[z].Num() == 0) { continue; }
		if (!Ctx.ZoneMedianPolys.IsValidIndex(z)) { continue; }

		// Arm corridors for this zone (each road's straight carriageway outline).
		TArray<FGeneralPolygon2d> Arms;
		for (int32 r : Ctx.Zones[z])
		{
			const FRoadCurves* C = Ctx.Curves.Find(r);
			if (!C) { continue; }
			FGeneralPolygon2d O;
			if (RoadNetSurface::BuildRoadOutline(*C, O)) { Arms.Add(MoveTemp(O)); }
		}
		if (Arms.Num() == 0) { continue; }
		TArray<FGeneralPolygon2d> ArmsU;
		if (!RoadNetSurface::Union(Arms, ArmsU)) { ArmsU = MoveTemp(Arms); }

		for (const FGeneralPolygon2d& GP : Ctx.ZoneJunctionClip[z])
		{
			const TArray<FVector2d>& OV = GP.GetOuter().GetVertices();
			if (OV.Num() < 3) { continue; }
			FVector2D Centre(0, 0);
			for (const FVector2d& V : OV) { Centre += FVector2D(V.X, V.Y); }
			Centre /= (double)OV.Num();
			if (!ResolveJunctionIslandsNear(Centre))
			{
				const FRoadNetRoundaboutConfig* Rb = FindRoundaboutNear(Centre);
				if (!Rb || !Rb->bSplitterIslands) { continue; }
			}

			// Pavement inside this junction = merged surface ∩ clip polygon.
			const TArray<FGeneralPolygon2d> GPArr = { GP };
			TArray<FGeneralPolygon2d> Paved;
			if (!PolygonsIntersection(Ctx.ZoneSurfacePolys[z], GPArr, Paved) || Paved.Num() == 0)
			{
				continue;
			}

			// Corner negative space = junction pavement MINUS the arm corridors.
			TArray<FGeneralPolygon2d> Corners;
			if (!RoadNetSurface::Difference(Paved, ArmsU, Corners) || Corners.Num() == 0)
			{
				continue;
			}

			// Erode for the kerb/grass setback so the island sits off the lanes.
			TArray<FGeneralPolygon2d> Islands;
			if (!PolygonsOffset(-Inset, Corners, Islands, /*bCopyInputOnFailure*/false,
					/*MiterLimit*/2.0, EPolygonOffsetJoinType::Round,
					EPolygonOffsetEndType::Polygon, /*MaxStepsPerRadian*/16.0,
					/*DefaultStepsPerRadianScale*/1.0e-3))
			{
				continue;
			}

			for (FGeneralPolygon2d& Isl : Islands)
			{
				if (FMath::Abs(Isl.GetOuter().SignedArea()) >= kMinIslandAreaCm2)
				{
					Ctx.ZoneMedianPolys[z].Add(MoveTemp(Isl));
				}
			}
		}
	}

	for (const FRoadNetRoundaboutConfig& Cfg : RoundaboutConfigs)
	{
		const double Half = FMath::Max(50.0, (double)Cfg.CirculatoryWidthCm * 0.5);
		const double InnerKerb = FMath::Max(80.0, (double)Cfg.InscribedRadiusCm - Half);
		const double Apron = FMath::Max(0.0, (double)Cfg.ApronWidthCm);
		const double IslandR = FMath::Max(50.0, InnerKerb - Apron);

		int32 Zone = INDEX_NONE;
		const FVector2d C(Cfg.Location.X, Cfg.Location.Y);
		for (int32 z = 0; z < Ctx.ZoneSurfacePolys.Num() && Zone == INDEX_NONE; ++z)
		{
			for (const FGeneralPolygon2d& SP : Ctx.ZoneSurfacePolys[z])
			{
				if (PointInRing2(SP.GetOuter().GetVertices(), C)) { Zone = z; break; }
			}
		}
		if (Zone == INDEX_NONE)
		{
			double BestD2 = TNumericLimits<double>::Max();
			for (int32 z = 0; z < Ctx.ZoneSurfacePolys.Num(); ++z)
			{
				for (const FGeneralPolygon2d& SP : Ctx.ZoneSurfacePolys[z])
				{
					for (const FVector2d& V : SP.GetOuter().GetVertices())
					{
						const double D2 = (V - C).SizeSquared();
						if (D2 < BestD2) { BestD2 = D2; Zone = z; }
					}
				}
			}
		}
		if (Zone == INDEX_NONE || !Ctx.ZoneMedianPolys.IsValidIndex(Zone)) { continue; }

		FGeneralPolygon2d Island;
		RoadNetSurface::MakeDisc(Cfg.Location, IslandR, 48, Island);
		Ctx.ZoneMedianPolys[Zone].Add(MoveTemp(Island));

		if (Apron > 10.0 && Ctx.ZoneMedianWalkPolys.IsValidIndex(Zone))
		{
			FGeneralPolygon2d Outer, Inner;
			RoadNetSurface::MakeDisc(Cfg.Location, InnerKerb, 48, Outer);
			RoadNetSurface::MakeDisc(Cfg.Location, IslandR, 48, Inner);
			TArray<FGeneralPolygon2d> Subj, Clip, Ring;
			Subj.Add(MoveTemp(Outer));
			Clip.Add(MoveTemp(Inner));
			if (RoadNetSurface::Difference(Subj, Clip, Ring) && Ring.Num() > 0)
			{
				Ctx.ZoneMedianWalkPolys[Zone].Append(MoveTemp(Ring));
			}
		}
	}

	// Authored pedestrian islands: fold into the median layer so they inherit
	// grass + the existing kerb wrap. Smooth, clip excess off existing medians,
	// and cut pedestrian paths from attached crosswalks.
	//
	// EVERY drawn ring is filled this way. There used to be an escape hatch here
	// that skipped the fill when an IslandMesh was set and instanced that prefab
	// at the ring's centroid instead — so drawing an island produced an outline
	// with an unrelated mesh dropped in the middle. A drawn shape has to become
	// that shape.
	for (const FRoadNetIsland& Isl : PlacedIslands)
	{
		if (Isl.Ring.Num() < 3) { continue; }
		if (RingAreaCm2(Isl.Ring) < RoadNetStandards::MinIslandAreaCm2()) { continue; }

		FVector2d C(0, 0);
		for (const FVector& P : Isl.Ring) { C.X += P.X; C.Y += P.Y; }
		C /= (double)Isl.Ring.Num();

		int32 Zone = INDEX_NONE;
		for (int32 z = 0; z < Ctx.ZoneSurfacePolys.Num() && Zone == INDEX_NONE; ++z)
		{
			for (const FGeneralPolygon2d& SP : Ctx.ZoneSurfacePolys[z])
			{
				if (PointInRing2(SP.GetOuter().GetVertices(), C)) { Zone = z; break; }
			}
		}
		if (Zone == INDEX_NONE)
		{
			// An island drawn off the carriageway (a wide splitter nose, a refuge
			// that overhangs the kerb) still belongs to the zone it is nearest to.
			// This used to take zone 0 unconditionally, which on a multi-zone city
			// dumped the island into whichever zone happened to be built first —
			// usually kilometres away, where nothing is drawn at all.
			double BestD2 = TNumericLimits<double>::Max();
			for (int32 z = 0; z < Ctx.ZoneSurfacePolys.Num(); ++z)
			{
				for (const FGeneralPolygon2d& SP : Ctx.ZoneSurfacePolys[z])
				{
					for (const FVector2d& V : SP.GetOuter().GetVertices())
					{
						const double D2 = (V - C).SizeSquared();
						if (D2 < BestD2) { BestD2 = D2; Zone = z; }
					}
				}
			}
		}
		if (Zone == INDEX_NONE || !Ctx.ZoneMedianPolys.IsValidIndex(Zone))
		{
			UE_LOG(LogRoadNet, Warning,
				TEXT("[RoadNet] Island at (%.0f, %.0f) has no zone to live in — not built."), C.X, C.Y);
			continue;
		}

		TArray<FVector2d> Loop;
		Loop.Reserve(Isl.Ring.Num());
		for (const FVector& P : Isl.Ring) { Loop.Emplace(P.X, P.Y); }
		FPolygon2d Poly(Loop);
		if (Poly.VertexCount() < 3) { continue; }
		if (Poly.IsClockwise()) { Poly.Reverse(); }
		FGeneralPolygon2d GP;
		GP.SetOuter(Poly);

		if (Isl.SmoothCm > 1.f)
		{
			// Morphological OPEN (erode then dilate) rounds convex corners.
			// Close would keep a sharp triangle sharp. If SmoothCm is larger
			// than the inradius, try half before giving up.
			auto TryOpen = [&](double R) -> bool
			{
				TArray<FGeneralPolygon2d> In = { GP }, Ero, Sm;
				if (!PolygonsOffset(-R, In, Ero, /*bCopyInputOnFailure*/false,
						/*MiterLimit*/2.0, EPolygonOffsetJoinType::Round,
						EPolygonOffsetEndType::Polygon, /*MaxStepsPerRadian*/16.0,
						/*DefaultStepsPerRadianScale*/1.0e-3)
					|| Ero.Num() == 0)
				{
					return false;
				}
				if (!PolygonsOffset(R, Ero, Sm, false,
						2.0, EPolygonOffsetJoinType::Round,
						EPolygonOffsetEndType::Polygon, 16.0, 1.0e-3)
					|| Sm.Num() == 0)
				{
					return false;
				}
				GP = MoveTemp(Sm[0]);
				return true;
			};
			if (!TryOpen((double)Isl.SmoothCm)) { TryOpen(0.5 * (double)Isl.SmoothCm); }
		}

		TArray<FGeneralPolygon2d> Pieces;
		Pieces.Add(MoveTemp(GP));

		if (Ctx.ZoneMedianPolys[Zone].Num() > 0)
		{
			TArray<FGeneralPolygon2d> Cut;
			if (RoadNetSurface::Difference(Pieces, Ctx.ZoneMedianPolys[Zone], Cut) && Cut.Num() > 0)
			{
				// Keep EVERY fragment worth keeping. This used to keep only the
				// largest, so an island a median runs through lost the smaller half
				// and left a gap the user drew a shape to fill.
				const double MinA = RoadNetStandards::MinIslandAreaCm2();
				Pieces.Reset();
				for (FGeneralPolygon2d& Frag : Cut)
				{
					if (FMath::Abs(Frag.GetOuter().SignedArea()) >= MinA) { Pieces.Add(MoveTemp(Frag)); }
				}
				if (Pieces.Num() == 0) { continue; }   // entirely swallowed by the median
			}

			auto ClosestOnMedians = [&](const FVector2d& Pt, FVector2d& OutQ) -> double
			{
				double Best = TNumericLimits<double>::Max();
				OutQ = Pt;
				for (const FGeneralPolygon2d& Med : Ctx.ZoneMedianPolys[Zone])
				{
					const TArray<FVector2d>& MV = Med.GetOuter().GetVertices();
					for (int32 i = 0; i < MV.Num(); ++i)
					{
						const FVector2d A = MV[i], B = MV[(i + 1) % MV.Num()];
						const FVector2d AB = B - A;
						const double Len2 = AB.SizeSquared();
						const double T = (Len2 > 1.0) ? FMath::Clamp(FVector2d::DotProduct(AB, Pt - A) / Len2, 0.0, 1.0) : 0.0;
						const FVector2d Q = A + AB * T;
						const double D2 = (Pt - Q).SizeSquared();
						if (D2 < Best) { Best = D2; OutQ = Q; }
					}
				}
				return Best;
			};
			// Weld each piece's boundary onto the median it abuts, so the island
			// reads as one continuous kerb instead of two shapes a hair apart.
			constexpr double kSnapR2 = 120.0 * 120.0;
			for (FGeneralPolygon2d& Piece : Pieces)
			{
				TArray<FVector2d> V = Piece.GetOuter().GetVertices();
				if (V.Num() < 3) { continue; }
				FVector2d Cent(0, 0);
				for (const FVector2d& P : V) { Cent += P; }
				Cent /= (double)V.Num();

				FVector2d Ignored;
				const double CentD2 = ClosestOnMedians(Cent, Ignored);
				for (FVector2d& Pt : V)
				{
					FVector2d Snap;
					const double D2 = ClosestOnMedians(Pt, Snap);
					if (D2 < kSnapR2 && D2 + 1.0 < CentD2) { Pt = Snap; }
				}
				FPolygon2d NP(V);
				if (NP.VertexCount() >= 3)
				{
					if (NP.IsClockwise()) { NP.Reverse(); }
					Piece.SetOuter(NP);
				}
			}
		}

		TArray<FGeneralPolygon2d> Grass = MoveTemp(Pieces);
		for (const FRoadNetIslandPath& Path : Isl.Paths)
		{
			TArray<FVector> CL;
			CL.Add(FVector(Path.A.X, Path.A.Y, 0.0));
			CL.Add(FVector(Path.B.X, Path.B.Y, 0.0));
			TArray<FGeneralPolygon2d> Ribbon;
			if (!RoadNetSurface::BuildPathRibbon(CL, 0.5 * (double)Path.WidthCm, Ribbon, 16.0, true)
				|| Ribbon.Num() == 0)
			{
				continue;
			}
			TArray<FGeneralPolygon2d> Walk, CutGrass;
			if (PolygonsIntersection(Grass, Ribbon, Walk) && Walk.Num() > 0
				&& Ctx.ZoneMedianWalkPolys.IsValidIndex(Zone))
			{
				Ctx.ZoneMedianWalkPolys[Zone].Append(Walk);
			}
			if (RoadNetSurface::Difference(Grass, Ribbon, CutGrass) && CutGrass.Num() > 0)
			{
				Grass = MoveTemp(CutGrass);
			}
		}
		Ctx.ZoneMedianPolys[Zone].Append(Grass);
	}
}

void URoadNetwork::BuildJunctionMarkings(FRoadNetRebuildContext& Ctx)
{
	using namespace UE::Geometry;
	JunctionViews.Reset();
	Ctx.Signals.Reset();
	if (!bBuildJunctionMarkings) { return; }

	auto HalfWidth = [this](int32 RoadIdx) -> double
	{
		return Roads.IsValidIndex(RoadIdx) ? FMath::Max(50.0, (double)Roads[RoadIdx].Lanes.HalfWidthCm()) : 300.0;
	};

	// Every junction (T / X / Y / roundabout / multi-arm) is already captured by
	// the per-zone clip region (built from 3+ arm joints AND crossings), so we
	// treat EACH clip polygon as one junction node and derive its approaches
	// from where the zone's centrelines cross that polygon's boundary.
	for (int32 z = 0; z < Ctx.ZoneJunctionClip.Num(); ++z)
	{
		if (!Ctx.ZoneMarkingWhitePolys.IsValidIndex(z) || !Ctx.Zones.IsValidIndex(z)) { continue; }
		const TArray<int32>& ZoneRoads = Ctx.Zones[z];

		// Accumulated cut-back rectangles that push the YELLOW centre line back
		// behind the crosswalk on each crosswalk approach (applied after the
		// junction loop). Only crosswalk presets contribute.
		TArray<FGeneralPolygon2d> YellowCut;

		for (const FGeneralPolygon2d& GP : Ctx.ZoneJunctionClip[z])
		{
			const TArray<FVector2d>& OV = GP.GetOuter().GetVertices();
			if (OV.Num() < 3) { continue; }

			// Junction centre + a representative height (centroid of outer ring).
			FVector2D Centre(0, 0);
			for (const FVector2d& V : OV) { Centre += FVector2D(V.X, V.Y); }
			Centre /= (double)OV.Num();

			auto InClip = [&GP](const FVector2D& P) { return GP.Contains(FVector2d(P.X, P.Y)); };

			// Unit direction of the junction-boundary edge nearest a point (used to
			// gauge how skewed an approach meets the junction).
			auto NearestEdgeDir = [&OV](const FVector2D& P) -> FVector2D
			{
				double Best = TNumericLimits<double>::Max();
				FVector2D Dir(1, 0);
				const int32 N = OV.Num();
				for (int32 i = 0; i < N; ++i)
				{
					const FVector2D E0(OV[i].X, OV[i].Y);
					const FVector2D E1(OV[(i + 1) % N].X, OV[(i + 1) % N].Y);
					const FVector2D Seg = E1 - E0;
					const double L2 = Seg.SizeSquared();
					if (L2 < 1.0) { continue; }
					const double T = FMath::Clamp(FVector2D::DotProduct(P - E0, Seg) / L2, 0.0, 1.0);
					const FVector2D Proj = E0 + Seg * T;
					const double D = FVector2D::DistSquared(P, Proj);
					if (D < Best) { Best = D; Dir = Seg / FMath::Sqrt(L2); }
				}
				return Dir;
			};

			// Approaches: each place a zone road's centreline crosses this
			// polygon's boundary is one approach (stop-line pos + outward dir).
			TArray<RoadNetJunctionMarks::FApproach> Approaches;
			double SumZ = 0.0; int32 ZN = 0;
			for (int32 r : ZoneRoads)
			{
				const FRoadCurves* C = Ctx.Curves.Find(r);
				if (!C || C->Sampled.Num() < 2) { continue; }
				const double Half = HalfWidth(r);
				const TArray<FVector>& S = C->Sampled;
				bool bPrevIn = InClip(FVector2D(S[0].X, S[0].Y));
				for (int32 i = 1; i < S.Num(); ++i)
				{
					const FVector2D A(S[i - 1].X, S[i - 1].Y);
					const FVector2D B(S[i].X, S[i].Y);
					const bool bIn = InClip(B);
					if (bIn != bPrevIn)
					{
						// Bisect for the boundary point between A (bPrevIn) and B (bIn).
						FVector2D Lo = A, Hi = B; bool bLoIn = bPrevIn;
						for (int32 it = 0; it < 12; ++it)
						{
							const FVector2D Mid = 0.5 * (Lo + Hi);
							if (InClip(Mid) == bLoIn) { Lo = Mid; } else { Hi = Mid; }
						}
						const FVector2D Boundary = 0.5 * (Lo + Hi);
						// Outward = from inside toward outside.
						FVector2D Outward = bPrevIn ? (B - A) : (A - B);
						FVector2D OutN = Outward;
						if (!OutN.Normalize()) { OutN = FVector2D(1, 0); }

						// Skew setback: on a tilted approach the boundary is not
						// perpendicular to travel, so a stop bar / crosswalk drawn
						// square to the road would clip into the junction on the
						// acute side. Push the stop position outward by the amount
						// the worst lateral corner (±Half) overhangs the boundary:
						//   extra = Half * |sin θ| / |cos θ|,  cos θ = Out·N.
						const FVector2D EdgeDir = NearestEdgeDir(Boundary);
						const FVector2D EdgeN(-EdgeDir.Y, EdgeDir.X);     // boundary normal
						const double CosT = FMath::Abs(FVector2D::DotProduct(OutN, EdgeN));
						double Extra = 30.0;                              // base clearance
						if (CosT > 0.15)
						{
							const double SinT = FMath::Sqrt(FMath::Max(0.0, 1.0 - CosT * CosT));
							Extra += Half * (SinT / CosT);
						}
						else { Extra += 1.5 * Half; }                    // near-grazing: cap
						Extra = FMath::Min(Extra, 1.75 * Half + 30.0);

						RoadNetJunctionMarks::FApproach Ap;
						Ap.StopPos = Boundary + OutN * Extra;
						Ap.Outward = Outward;
						Ap.HalfWidthCm = Half;

						// Which of this arm's lanes ENTER the junction, and where they
						// sit. Without this the arm is only "a place a centreline
						// crosses the boundary", so every arm got a bar — including a
						// one-way arm whose traffic only leaves.
						//
						// Outward was taken as (B-A) when the road was heading OUT of
						// the clip at this crossing, so forward travel is inbound
						// exactly when it was heading in.
						const bool bForwardEnters = !bPrevIn;

						// Lane offsets are signed +right of the road's FORWARD
						// direction. The approach frame is +right of INBOUND, so the
						// two agree when forward is inbound and negate when it is not.
						const double LatSign = bForwardEnters ? 1.0 : -1.0;

						bool bAnyEnters = false;
						double EnterLo = 0.0, EnterHi = 0.0;
						for (const FRoadNetLane& Ln : GetLanesLeftToRight(r))
						{
							if (!Ln.bDrivable()) { continue; }
							const bool bEnters = bForwardEnters
								? Ln.bTravelsForward(bDriveOnLeft)
								: Ln.bTravelsBackward(bDriveOnLeft);
							if (!bEnters) { continue; }
							const double E0 = LatSign * (Ln.CenterOffset - 0.5 * (double)Ln.Width);
							const double E1 = LatSign * (Ln.CenterOffset + 0.5 * (double)Ln.Width);
							const double A0 = FMath::Min(E0, E1), A1 = FMath::Max(E0, E1);
							EnterLo = bAnyEnters ? FMath::Min(EnterLo, A0) : A0;
							EnterHi = bAnyEnters ? FMath::Max(EnterHi, A1) : A1;
							bAnyEnters = true;
						}
						Ap.bHasEnteringTraffic = bAnyEnters;
						if (bAnyEnters) { Ap.EnterLoCm = EnterLo; Ap.EnterHiCm = EnterHi; }

						Approaches.Add(Ap);
						SumZ += FMath::Lerp(S[i - 1].Z, S[i].Z, 0.5); ++ZN;
					}
					bPrevIn = bIn;
				}
			}

			const double CentreZ = (ZN > 0) ? (SumZ / (double)ZN) : 0.0;
			ERoadNetJunctionPreset Preset = ResolveJunctionPresetNear(Centre);
			if (const FRoadNetRoundaboutConfig* Rb = FindRoundaboutNear(Centre))
			{
				if (Rb->EntryGiveWay && Preset == ERoadNetJunctionPreset::None)
				{
					Preset = ERoadNetJunctionPreset::GiveWay;
				}
			}

			FRoadNetJunctionView View;
			View.Location = FVector(Centre.X, Centre.Y, CentreZ);
			View.Preset = Preset;
			View.ArmCount = Approaches.Num();
			JunctionViews.Add(View);

			if (Preset == ERoadNetJunctionPreset::None || Approaches.Num() == 0) { continue; }

			TArray<RoadNetJunctionMarks::FSignal> Signals;
			RoadNetJunctionMarks::BuildJoint(
				Centre, CentreZ, Approaches, Preset, bDriveOnLeft, Ctx.ZoneMarkingWhitePolys[z], Signals);

			for (const RoadNetJunctionMarks::FSignal& Sg : Signals)
			{
				Ctx.Signals.Emplace(Sg.Location, Sg.YawDeg);
			}

			// Cut the yellow centre line back to BEHIND the approach paint: a full-
			// width rectangle from the junction boundary out past the stop bar. The
			// reach comes from BuildJoint's own layout so it follows
			// roadnet.CrosswalkLengthCm instead of being a second copy of it.
			if (PresetHasCrosswalk(Preset))
			{
				const double kCwFarCm = RoadNetJunctionMarks::ApproachPaintReachCm() + 40.0;
				for (const RoadNetJunctionMarks::FApproach& Ap : Approaches)
				{
					FVector2D Out = Ap.Outward;
					if (!Out.Normalize()) { continue; }
					const double HalfU = 0.5 * (kCwFarCm + 10.0);
					const FVector2D C = Ap.StopPos + Out * (HalfU - 10.0);
					YellowCut.Add(MakeRectPoly(C, Out, HalfU, Ap.HalfWidthCm + 25.0));
				}
			}
		}

		// Apply the yellow cut-backs for this zone (if any crosswalk junctions).
		if (YellowCut.Num() > 0 && Ctx.ZoneMarkingYellowPolys.IsValidIndex(z)
			&& Ctx.ZoneMarkingYellowPolys[z].Num() > 0)
		{
			TArray<FGeneralPolygon2d> Trimmed;
			if (RoadNetSurface::Difference(Ctx.ZoneMarkingYellowPolys[z], YellowCut, Trimmed))
			{
				Ctx.ZoneMarkingYellowPolys[z] = MoveTemp(Trimmed);
			}
		}
	}
}

void URoadNetwork::CommitJunctionSignals(FRoadNetRebuildContext& Ctx)
{
	if (!WorldPtr.IsValid()) { return; }

	if (!bBuildJunctionMarkings || Ctx.Signals.Num() == 0) { return; }

	// Resolution order: Blueprint/actor class > Static Mesh override > cylinder.
	// A Blueprint carries its own pole, heads and pivot, so it is spawned at the
	// approach point unscaled rather than fitted like the placeholder.
	if (UClass* BPClass = SignalBlueprint.IsNull() ? nullptr : SignalBlueprint.LoadSynchronous())
	{
		UWorld* World = WorldPtr.Get();
		int32 Spawned = 0;
		for (const TPair<FVector, float>& SP : Ctx.Signals)
		{
			const FIntPoint Coord = TopoKeyOf(SP.Key, Ctx);
			if (Coord.X == INDEX_NONE || !IsTileInCommitScope(Coord, Ctx)) { continue; }
			ARoadNetTileActor* Tile = GetOrCreateTile(Coord);
			if (!Tile) { continue; }

			FActorSpawnParameters Params;
			Params.ObjectFlags |= RF_Transient;
			Params.Owner = Tile;
			Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
			const FTransform Xf(FRotator(0.f, SP.Value, 0.f), SP.Key, FVector::OneVector);
			if (AActor* Child = World->SpawnActor<AActor>(BPClass, Xf, Params))
			{
				Child->AttachToActor(Tile, FAttachmentTransformRules::KeepWorldTransform);
				Tile->TrackChildActor(Child);
				++Spawned;
			}
		}
		UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] CommitJunctionSignals: %d signal actors [%s]."),
			Spawned, *BPClass->GetName());
		return;
	}

	UStaticMesh* Mesh = SignalMesh ? SignalMesh.Get() : nullptr;
	if (!Mesh) { Mesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder")); }
	if (!Mesh) { return; }

	// Fit the mesh into a thin, ~3.5 m tall "pole" placeholder seated on the
	// ground point, unless the user supplied a real signal mesh (then place it
	// upright, un-stretched, seated at the base).
	const bool bUserMesh = (SignalMesh != nullptr);
	const FBox Box = Mesh->GetBoundingBox();
	const FVector Size = Box.GetSize();
	const FVector Ctr = Box.GetCenter();
	constexpr double kPoleTallCm = 350.0;
	constexpr double kPoleWideCm = 25.0;
	const double sTall = bUserMesh ? 1.0 : kPoleTallCm / FMath::Max(1.0, Size.Z);
	const double sWide = bUserMesh ? 1.0 : kPoleWideCm / FMath::Max(1.0, FMath::Max(Size.X, Size.Y));

	int32 Placed = 0;
	TMap<UHierarchicalInstancedStaticMeshComponent*, TArray<FTransform>> SignalBatches;
	for (const TPair<FVector, float>& SP : Ctx.Signals)
	{
		const FIntPoint Coord = TopoKeyOf(SP.Key, Ctx);
		if (Coord.X == INDEX_NONE || !IsTileInCommitScope(Coord, Ctx)) { continue; }
		ARoadNetTileActor* Tile = GetOrCreateTile(Coord);
		if (!Tile) { continue; }
		UHierarchicalInstancedStaticMeshComponent* H = Tile->GetOrCreateHISM(FName(TEXT("Signals")), Mesh);
		if (!H) { continue; }

		const FVector Scale(sWide, sWide, sTall);
		FTransform Inst(FRotator(0.f, SP.Value, 0.f), FVector::ZeroVector, Scale);
		const FVector AnchorWorld = Inst.TransformVector(FVector(Ctr.X, Ctr.Y, Box.Min.Z)); // bottom-centre
		Inst.SetTranslation(SP.Key - AnchorWorld);
		SignalBatches.FindOrAdd(H).Add(Inst);
		++Placed;
	}
	for (TPair<UHierarchicalInstancedStaticMeshComponent*, TArray<FTransform>>& KV : SignalBatches)
	{
		KV.Key->AddInstances(KV.Value, /*bShouldReturnIndices*/false, /*bWorldSpace*/true);
	}

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] CommitJunctionSignals: %d signal placeholders%s."),
		Placed, bUserMesh ? TEXT("") : TEXT(" [cylinder default]"));
}

// Seat an arrow mesh flat on the paint tier at At, pointing along Yaw. Shared by
// the automatic and the hand-placed path so both sit at the same height and are
// anchored the same way (bottom-centre, not pivot — a triangulated spline mesh
// keeps whatever pivot the source spline had).
static FTransform MakeMarkInstance(UStaticMesh* Mesh, const FVector& At, float YawDeg,
	bool bUserMesh, double LiftCm)
{
	const FBox Box = Mesh->GetBoundingBox();
	const FVector Size = Box.GetSize();
	// A placeholder cube has to be squashed into an arrow-sized slab; a real
	// arrow asset is already the size the author wanted.
	const double sXY = bUserMesh ? 1.0 : 120.0 / FMath::Max(1.0, FMath::Max(Size.X, Size.Y));
	const double sZ  = bUserMesh ? 1.0 : 4.0 / FMath::Max(1.0, Size.Z);
	FTransform Inst(FRotator(0.f, YawDeg, 0.f), FVector::ZeroVector, FVector(sXY, sXY, sZ));
	const FVector Anchor = Inst.TransformVector(FVector(Box.GetCenter().X, Box.GetCenter().Y, Box.Min.Z));
	Inst.SetTranslation(At + FVector(0, 0, LiftCm) - Anchor);
	return Inst;
}

UStaticMesh* URoadNetwork::ResolveMarkMesh(ERoadNetMarkKind Kind, TMap<FName, UStaticMesh*>& Cache,
	UMaterialInterface*& OutMat, FName& OutKey) const
{
	// Row order IS ERoadNetMarkKind order. Stencil is the /Game/OSM/Stencils
	// asset bUseOSMStencils swaps in; LeftRight has none because Arrow_Left_Turn
	// is the odd one out in that folder and could be either a left turn or a
	// hook, so the assigned mesh always wins there. Fill it in if you decide.
	struct FSlot { const TCHAR* Key; const TCHAR* Stencil; };
	static const FSlot kSlots[] = {
		{ TEXT("ArrowThrough"),      TEXT("/Game/OSM/Stencils/Arrow_Straight.Arrow_Straight") },
		{ TEXT("ArrowLeft"),         TEXT("/Game/OSM/Stencils/Arrow_Left.Arrow_Left") },
		{ TEXT("ArrowRight"),        TEXT("/Game/OSM/Stencils/Arrow_Right.Arrow_Right") },
		{ TEXT("ArrowUTurn"),        TEXT("/Game/OSM/Stencils/Arrow_Turn.Arrow_Turn") },
		{ TEXT("ArrowThroughLeft"),  TEXT("/Game/OSM/Stencils/Arrow_Straight_Left.Arrow_Straight_Left") },
		{ TEXT("ArrowThroughRight"), TEXT("/Game/OSM/Stencils/Arrow_Straight_Right.Arrow_Straight_Right") },
		{ TEXT("ArrowLeftRight"),    nullptr },
	};
	static_assert(UE_ARRAY_COUNT(kSlots) == (int32)ERoadNetMarkKind::LeftRight + 1,
		"kSlots must have one row per ERoadNetMarkKind");

	const int32 i = (int32)Kind;
	if (i < 0 || i >= UE_ARRAY_COUNT(kSlots)) { return nullptr; }
	OutKey = FName(kSlots[i].Key);

	const TObjectPtr<UStaticMesh> MeshSlots[] = {
		ArrowThroughMesh, ArrowLeftMesh, ArrowRightMesh, ArrowUTurnMesh,
		ArrowThroughLeftMesh, ArrowThroughRightMesh, ArrowLeftRightMesh };
	const TObjectPtr<UMaterialInterface> MatSlots[] = {
		ArrowThroughMaterial, ArrowLeftMaterial, ArrowRightMaterial, ArrowUTurnMaterial,
		ArrowThroughLeftMaterial, ArrowThroughRightMaterial, ArrowLeftRightMaterial };

	// With bUseOSMStencils the stencil WINS over whatever is assigned — that is
	// the point of a switch — but a stencil that will not load falls back to the
	// assigned mesh, so a half-populated folder gives a mix instead of a hole.
	UStaticMesh* Mesh = nullptr;
	if (bUseOSMStencils && kSlots[i].Stencil)
	{
		if (UStaticMesh** Hit = Cache.Find(OutKey))
		{
			Mesh = *Hit;
		}
		else
		{
			Mesh = LoadObject<UStaticMesh>(nullptr, kSlots[i].Stencil);
			Cache.Add(OutKey, Mesh);
			if (!Mesh)
			{
				UE_LOG(LogRoadNet, Warning,
					TEXT("[RoadNet] bUseOSMStencils: %s not found, using the assigned mesh."),
					kSlots[i].Stencil);
			}
		}
	}
	if (!Mesh) { Mesh = MeshSlots[i].Get(); }

	// An arrow IS white road paint, so an unset arrow-material slot should reach
	// for the same material the painted lines already use rather than leaving the
	// mesh's authored material to decide. Triangulated meshes carry whatever
	// material the source spline happened to have.
	OutMat = MatSlots[i] ? MatSlots[i].Get() : MarkingWhiteMaterial.Get();
	return Mesh;
}

void URoadNetwork::CommitLaneMarks(FRoadNetRebuildContext& Ctx)
{
	if (!WorldPtr.IsValid() || Ctx.LaneConnections.Num() == 0) { return; }

	// Derivation is opt-in (roadnet.AutoTurnArrows): it sets arrows back a fixed
	// distance from the joint rather than from the stop line, which puts them in
	// the wrong place on most approaches. Hand-placed marks commit regardless.
	static IConsoleVariable* AutoCVar = nullptr;
	if (!AutoCVar) { AutoCVar = IConsoleManager::Get().FindConsoleVariable(TEXT("roadnet.AutoTurnArrows")); }
	if (!AutoCVar || AutoCVar->GetInt() == 0) { return; }

	UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	TMap<FName, UStaticMesh*> StencilCache;

	const float Setback = FMath::Max(400.f, ArrowSetbackCm);
	int32 Placed = 0;
	TMap<UHierarchicalInstancedStaticMeshComponent*, TArray<FTransform>> Batches;

	// ---- movement set per approach lane ----------------------------------
	// One arrow per LANE, not per connection. A shared left+through lane emits
	// two connections from the same lane at the same joint, and both resolve to
	// the same setback point, so placing per connection stacked two arrows in
	// one spot. Gather the movements first, then draw the one arrow that says
	// what the lane actually does.
	// Keyed by (road, lane, JOINT), not just (road, lane): a lane running between
	// two junctions feeds a movement set at EACH end, and merging those would both
	// mask the arrow with the far junction's movements and collapse two arrows
	// into one. Each approach is its own entry.
	struct FApproach
	{
		FVector Entry = FVector::ZeroVector;
		uint8 Mask = 0;
	};
	auto BitOf = [](ERoadNetTurnRole K) -> uint8
	{
		switch (K)
		{
		case ERoadNetTurnRole::Through: return 1 << 0;
		case ERoadNetTurnRole::Left:    return 1 << 1;
		case ERoadNetTurnRole::Right:   return 1 << 2;
		case ERoadNetTurnRole::UTurn:   return 1 << 3;
		default:                        return 0;
		}
	};

	TMap<FIntVector, FApproach> Approaches; // (road, lane, joint) -> movements out of it
	for (const FRoadNetLaneConnection& Cn : Ctx.LaneConnections)
	{
		if (!Ctx.Joints.IsValidIndex(Cn.Joint) || Ctx.Joints[Cn.Joint].Arms.Num() < 3) { continue; }
		if (Cn.Kind == ERoadNetTurnRole::Auto || Cn.Kind == ERoadNetTurnRole::None) { continue; }
		if (!Roads.IsValidIndex(Cn.From.Road)) { continue; }

		const TArray<FRoadNetLane> Lanes = Roads[Cn.From.Road].Lanes.ResolveLanes(bDriveOnLeft);
		if (!Lanes.IsValidIndex(Cn.From.Lane)) { continue; }
		const FRoadNetLane& Ln = Lanes[Cn.From.Lane];
		if (!Ln.bDrivable()) { continue; }
		if (Ln.Type == ERoadNetLaneType::Parking || Ln.Type == ERoadNetLaneType::Shoulder
			|| Ln.Type == ERoadNetLaneType::Median) { continue; }
		if (!RoleAllowsKind(Ln.TurnRole, Cn.Kind)) { continue; }

		FApproach& A = Approaches.FindOrAdd(FIntVector(Cn.From.Road, Cn.From.Lane, Cn.Joint));
		A.Entry = Cn.Entry;
		A.Mask |= BitOf(Cn.Kind);
	}

	for (const TPair<FIntVector, FApproach>& AP : Approaches)
	{
		const int32 RoadIdx = AP.Key.X;
		const int32 LaneIdx = AP.Key.Y;
		const FApproach& Ap = AP.Value;
		if (Ap.Mask == 0) { continue; }

		const TArray<FRoadNetLane> Lanes = Roads[RoadIdx].Lanes.ResolveLanes(bDriveOnLeft);
		if (!Lanes.IsValidIndex(LaneIdx)) { continue; }
		const FRoadNetLane& Ln = Lanes[LaneIdx];

		const FRoadCurves* C = Ctx.Curves.Find(RoadIdx);
		if (!C || C->Sampled.Num() < 2) { continue; }

		TArray<FVector> CL;
		RoadNetLanes::BuildLaneCenterline(C->Sampled, Ln, CL);
		if (CL.Num() < 2) { continue; }

		int32 Near = 0;
		double BestD2 = TNumericLimits<double>::Max();
		for (int32 i = 0; i < CL.Num(); ++i)
		{
			const double D2 = FVector::DistSquaredXY(CL[i], Ap.Entry);
			if (D2 < BestD2) { BestD2 = D2; Near = i; }
		}
		const bool bAtStart = FVector::DistSquaredXY(Ap.Entry, CL[0])
			< FVector::DistSquaredXY(Ap.Entry, CL.Last());
		const int32 Step = bAtStart ? 1 : -1;
		double Walked = 0.0;
		int32 Idx = Near;
		FVector Mark = CL[Near];
		while (Walked < (double)Setback)
		{
			const int32 Next = Idx + Step;
			if (!CL.IsValidIndex(Next)) { break; }
			Walked += FVector::Dist2D(CL[Idx], CL[Next]);
			Idx = Next;
			Mark = CL[Idx];
		}

		FVector Travel = Ap.Entry - Mark;
		Travel.Z = 0.0;
		if (!Travel.Normalize())
		{
			Travel = bAtStart ? (CL[0] - CL[1]) : (CL.Last() - CL[CL.Num() - 2]);
			Travel.Z = 0.0;
			Travel.Normalize();
		}
		const float Yaw = FMath::RadiansToDegrees(FMath::Atan2(Travel.Y, Travel.X));

		// Exact combination first, then degrade to the dominant single movement,
		// so an unfilled combo slot still paints something truthful rather than
		// nothing. Through outranks the turns because a lane that can go straight
		// reads as a through lane to a driver.
		UStaticMesh* Mesh = nullptr;
		UMaterialInterface* Mat = nullptr;
		FName Key;
		const uint8 M = Ap.Mask;
		auto Pick = [&](ERoadNetMarkKind Kind) -> bool
		{
			UMaterialInterface* M2 = nullptr;
			FName K2;
			UStaticMesh* Resolved = ResolveMarkMesh(Kind, StencilCache, M2, K2);
			if (!Resolved) { return false; }
			Mesh = Resolved; Mat = M2; Key = K2;
			return true;
		};

		const bool bT = (M & (1 << 0)) != 0, bL = (M & (1 << 1)) != 0;
		const bool bR = (M & (1 << 2)) != 0, bU = (M & (1 << 3)) != 0;

		if      (bT && bL && Pick(ERoadNetMarkKind::ThroughLeft))  {}
		else if (bT && bR && Pick(ERoadNetMarkKind::ThroughRight)) {}
		else if (bL && bR && Pick(ERoadNetMarkKind::LeftRight))    {}
		else if (bT && Pick(ERoadNetMarkKind::Through)) {}
		else if (bL && Pick(ERoadNetMarkKind::Left))    {}
		else if (bR && Pick(ERoadNetMarkKind::Right))   {}
		else if (bU && Pick(ERoadNetMarkKind::UTurn))   {}

		if (!Mesh) { Mesh = Cube; Key = FName(TEXT("ArrowThrough")); }
		if (!Mesh) { continue; }

		const FIntPoint Coord = TopoKeyOf(Mark, Ctx);
		if (Coord.X == INDEX_NONE || !IsTileInCommitScope(Coord, Ctx)) { continue; }
		ARoadNetTileActor* Tile = GetOrCreateTile(Coord);
		if (!Tile) { continue; }
		UHierarchicalInstancedStaticMeshComponent* H = Tile->GetOrCreateHISM(Key, Mesh, Mat);
		if (!H) { continue; }

		// Mark comes off the raw lane centreline, which is the LANDSCAPE line: the
		// road slab sits kRoadZLiftCm above it and the painted markings kMarkingLiftCm
		// above that. An arrow lifted less than kRoadZLiftCm is inside the asphalt and
		// invisible, which is exactly how "97 arrows placed, none on screen" happens.
		// Sit half a centimetre proud of the paint tier.
		Batches.FindOrAdd(H).Add(
			MakeMarkInstance(Mesh, Mark, Yaw, /*bUserMesh*/Mesh != Cube,
				kRoadZLiftCm + kMarkingLiftCm + 0.5));
		++Placed;
	}

	for (TPair<UHierarchicalInstancedStaticMeshComponent*, TArray<FTransform>>& KV : Batches)
	{
		KV.Key->AddInstances(KV.Value, /*bShouldReturnIndices*/false, /*bWorldSpace*/true);
	}
	// Report the approach count too: "0 arrows from 0 approaches" means no lane
	// reaches a 3+ arm junction (the usual cause), while "0 from N" would mean the
	// meshes failed to resolve. Those are different problems and used to look alike.
	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] CommitLaneMarks: %d turn arrows from %d approach lanes%s."),
		Placed, Approaches.Num(), bUseOSMStencils ? TEXT(" [OSM stencils]") : TEXT(""));
}

// How far a click may be from a road's reference line and still count as "on it".
// Generous enough to cover the far kerb of a dual carriageway.
static constexpr double kMarkBindRadiusCm = 3000.0;

void URoadNetwork::CommitPlacedMarks(FRoadNetRebuildContext& Ctx)
{
	if (!WorldPtr.IsValid() || PlacedMarks.Num() == 0) { return; }

	UStaticMesh* Cube = nullptr;   // loaded lazily: only an empty slot needs it
	TMap<FName, UStaticMesh*> StencilCache;
	int32 Placed = 0, NoMesh = 0, Migrated = 0, Unbound = 0;
	TMap<UHierarchicalInstancedStaticMeshComponent*, TArray<FTransform>> Batches;

	// Migration for saves written before marks carried a road binding: resolve each
	// once, here, and write the keys back so it only ever happens on the first
	// rebuild after load. A mark that resolves to nothing is NOT dropped — it stays
	// world-anchored, which is what it has always been.
	for (FRoadNetPlacedMark& Mk : PlacedMarks)
	{
		if (Mk.IsBoundToRoad()) { continue; }
		if (BindPlacedMarkToRoad(Mk, kMarkBindRadiusCm)) { ++Migrated; }
		else { ++Unbound; }
	}
	if (Migrated > 0) { Modify(); }

	for (const FRoadNetPlacedMark& Mk : PlacedMarks)
	{
		UMaterialInterface* Mat = nullptr;
		FName Key;
		UStaticMesh* Mesh = ResolveMarkMesh(Mk.Kind, StencilCache, Mat, Key);
		if (!Mesh)
		{
			// Placeholder rather than nothing: a mark the user deliberately
			// clicked must show up somewhere, or assigning the mesh later is
			// guesswork about whether the click even registered.
			if (!Cube) { Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")); }
			Mesh = Cube;
			++NoMesh;
		}
		if (!Mesh) { continue; }

		// Where the mark goes NOW: a bound mark is derived from its lane, so it has
		// already followed any median restack or centreline edit since it was clicked.
		FVector At; float Yaw = 0.f;
		const bool bBound = ResolvePlacedMark(Mk, At, Yaw);

		const FIntPoint Coord = TopoKeyOf(At, Ctx);
		if (Coord.X == INDEX_NONE || !IsTileInCommitScope(Coord, Ctx)) { continue; }
		ARoadNetTileActor* Tile = GetOrCreateTile(Coord);
		if (!Tile) { continue; }
		UHierarchicalInstancedStaticMeshComponent* H = Tile->GetOrCreateHISM(Key, Mesh, Mat);
		if (!H) { continue; }

		// A bound mark's Z is the raw landscape line, so it needs the full slab lift
		// plus the paint clearance. An unbound one kept the traced Z of the surface
		// under the cursor and only needs to clear the marking tier.
		const double Lift = bBound ? (kRoadZLiftCm + kMarkingLiftCm + 0.5) : (kMarkingLiftCm + 0.5);
		Batches.FindOrAdd(H).Add(
			MakeMarkInstance(Mesh, At, Yaw, /*bUserMesh*/Mesh != Cube, Lift));
		++Placed;
	}

	for (TPair<UHierarchicalInstancedStaticMeshComponent*, TArray<FTransform>>& KV : Batches)
	{
		KV.Key->AddInstances(KV.Value, /*bShouldReturnIndices*/false, /*bWorldSpace*/true);
	}
	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet] CommitPlacedMarks: %d hand-placed mark(s) of %d%s; %d bound to a road this run, %d left world-anchored."),
		Placed, PlacedMarks.Num(),
		NoMesh > 0 ? *FString::Printf(TEXT(" (%d on a placeholder cube — assign the arrow mesh in Assets | Markings)"), NoMesh)
		           : TEXT(""),
		Migrated, Unbound);
}

bool URoadNetwork::BindPlacedMarkToRoad(FRoadNetPlacedMark& Mk, double RadiusCm) const
{
	int32   BestRoad = INDEX_NONE;
	double  BestD    = RadiusCm;
	RoadNetMath::FProjectResult BestP;

	const FVector2D Q(Mk.Location.X, Mk.Location.Y);
	for (int32 i = 0; i < Roads.Num(); ++i)
	{
		if (Roads[i].Ref.Num() < 2) { continue; }
		const RoadNetMath::FProjectResult P = RoadNetMath::ProjectToPolyline(Roads[i].Ref, Q);
		if (P.Distance >= BestD || P.Segment < 0 || !Roads[i].Ref.IsValidIndex(P.Segment + 1)) { continue; }
		BestD = P.Distance; BestRoad = i; BestP = P;
	}
	if (BestRoad == INDEX_NONE) { return false; }

	Mk.Road       = BestRoad;
	Mk.DistanceCm = (float)BestP.AlongDist;

	// Which lane the offset lands in, and where inside it. Nearest lane centre
	// rather than a containment test, so a click on a lane boundary still binds.
	const TArray<FRoadNetLane> Lanes = GetLanesLeftToRight(BestRoad);
	Mk.LaneIndex    = INDEX_NONE;
	Mk.LaneOffsetCm = (float)BestP.Offset;
	double BestLaneD = TNumericLimits<double>::Max();
	for (int32 i = 0; i < Lanes.Num(); ++i)
	{
		const double D = FMath::Abs(BestP.Offset - Lanes[i].CenterOffset);
		if (D < BestLaneD) { BestLaneD = D; Mk.LaneIndex = i; Mk.LaneOffsetCm = (float)(BestP.Offset - Lanes[i].CenterOffset); }
	}

	// Yaw is stored against the road's tangent so a reshaped centreline re-aims
	// the arrow instead of leaving it pointing at the old heading.
	const FVector& A = Roads[BestRoad].Ref[BestP.Segment];
	const FVector& B = Roads[BestRoad].Ref[BestP.Segment + 1];
	const float TangentYaw = (float)FMath::RadiansToDegrees(FMath::Atan2(B.Y - A.Y, B.X - A.X));
	Mk.LaneYawDeg = FRotator::NormalizeAxis(Mk.YawDeg - TangentYaw);
	return true;
}

bool URoadNetwork::ResolvePlacedMark(const FRoadNetPlacedMark& Mk, FVector& OutLoc, float& OutYawDeg) const
{
	// Unbound: the click point is all there is. Its Z came off a collision trace,
	// so it is already on whatever surface was under the cursor.
	if (!Mk.IsBoundToRoad() || !Roads.IsValidIndex(Mk.Road) || Roads[Mk.Road].Ref.Num() < 2)
	{
		OutLoc = Mk.Location;
		OutYawDeg = Mk.YawDeg;
		return false;
	}

	const TArray<FVector>& Ref = Roads[Mk.Road].Ref;
	TArray<double> CL;
	RoadNetMath::CumulativeLength(Ref, CL);
	const double Len = CL.Last();
	const double S = FMath::Clamp((double)Mk.DistanceCm, 0.0, Len);

	int32 Seg = 0;
	while (Seg + 2 < Ref.Num() && CL[Seg + 1] < S) { ++Seg; }
	const double SegLen = FMath::Max(1e-3, CL[Seg + 1] - CL[Seg]);
	const double t = FMath::Clamp((S - CL[Seg]) / SegLen, 0.0, 1.0);
	const FVector Base = FMath::Lerp(Ref[Seg], Ref[Seg + 1], t);

	FVector2D Fwd(Ref[Seg + 1].X - Ref[Seg].X, Ref[Seg + 1].Y - Ref[Seg].Y);
	if (!Fwd.Normalize()) { Fwd = FVector2D(1.0, 0.0); }
	const FVector2D Right(Fwd.Y, -Fwd.X);   // +right of travel, matching FProjectResult::Offset

	// The lane is where the offset comes from: a median restack moves the lane
	// centre, so the mark moves with it and never ends up under the median.
	const TArray<FRoadNetLane> Lanes = GetLanesLeftToRight(Mk.Road);
	const double Offset = Lanes.IsValidIndex(Mk.LaneIndex)
		? Lanes[Mk.LaneIndex].CenterOffset + (double)Mk.LaneOffsetCm
		: (double)Mk.LaneOffsetCm;

	// Z from the road model, not a trace: Ref is the landscape line the slab is
	// built on, so the slab top is exactly kRoadZLiftCm above it. (Tracing here
	// would hit the landscape, because the road slab is deliberately NoCollision.)
	OutLoc = FVector(Base.X + Right.X * Offset, Base.Y + Right.Y * Offset, Base.Z + kRoadZLiftCm);
	OutYawDeg = FRotator::NormalizeAxis(
		(float)FMath::RadiansToDegrees(FMath::Atan2(Fwd.Y, Fwd.X)) + Mk.LaneYawDeg);
	return true;
}

int32 URoadNetwork::AddPlacedMark(const FVector& WorldLoc, float YawDeg, ERoadNetMarkKind Kind)
{
	Modify();
	FRoadNetPlacedMark Mk;
	Mk.Id = FGuid::NewGuid();
	Mk.Location = WorldLoc;
	Mk.YawDeg = YawDeg;
	Mk.Kind = Kind;
	BindPlacedMarkToRoad(Mk, kMarkBindRadiusCm);
	return PlacedMarks.Add(Mk);
}

int32 URoadNetwork::RemovePlacedMarkNear(const FVector& WorldLoc, double RadiusCm)
{
	int32 Best = INDEX_NONE;
	double BestD2 = RadiusCm * RadiusCm;
	for (int32 i = 0; i < PlacedMarks.Num(); ++i)
	{
		// Compare against where the mark is DRAWN, not where it was clicked — a
		// bound mark has moved with its lane since then.
		FVector At; float Yaw = 0.f;
		ResolvePlacedMark(PlacedMarks[i], At, Yaw);
		const double D2 = FVector::DistSquaredXY(At, WorldLoc);
		if (D2 <= BestD2) { BestD2 = D2; Best = i; }
	}
	if (Best == INDEX_NONE) { return INDEX_NONE; }
	Modify();
	PlacedMarks.RemoveAt(Best);
	return Best;
}

bool URoadNetwork::RemovePlacedById(ERoadNetPlacedKind Kind, FGuid Id)
{
	if (!Id.IsValid()) { return false; }
	Modify();
	auto Drop = [Id](auto& Arr) -> bool
	{
		for (int32 i = 0; i < Arr.Num(); ++i)
		{
			if (Arr[i].Id == Id) { Arr.RemoveAt(i); return true; }
		}
		return false;
	};
	switch (Kind)
	{
	case ERoadNetPlacedKind::Mark:         return Drop(PlacedMarks);
	case ERoadNetPlacedKind::Island:       return Drop(PlacedIslands);
	case ERoadNetPlacedKind::BikeCrossing: return Drop(BikeCrossings);
	case ERoadNetPlacedKind::CurbPaint:    return Drop(CurbPaints);
	default: return false;
	}
}

bool URoadNetwork::HeadingOfNearestRoad(const FVector& WorldLoc, double RadiusCm, float& OutYawDeg) const
{
	const FVector2D Q(WorldLoc.X, WorldLoc.Y);
	double BestD = RadiusCm;
	FVector2D BestDir = FVector2D::ZeroVector;

	for (const FRoadDef& R : Roads)
	{
		if (R.Ref.Num() < 2) { continue; }
		const RoadNetMath::FProjectResult P = RoadNetMath::ProjectToPolyline(R.Ref, Q);
		// Segment stays INDEX_NONE on a degenerate polyline, and -1 + 1 == 0 is a
		// valid index, so the bounds check alone would read Ref[-1].
		if (P.Distance >= BestD || P.Segment < 0 || !R.Ref.IsValidIndex(P.Segment + 1)) { continue; }
		const FVector& A = R.Ref[P.Segment];
		const FVector& B = R.Ref[P.Segment + 1];
		FVector2D Dir(B.X - A.X, B.Y - A.Y);
		if (!Dir.Normalize()) { continue; }
		BestD = P.Distance;
		BestDir = Dir;
	}

	if (BestDir.IsZero()) { return false; }
	OutYawDeg = (float)FMath::RadiansToDegrees(FMath::Atan2(BestDir.Y, BestDir.X));
	return true;
}

int32 URoadNetwork::AddSplineSplitByTile(const TArray<FVector>& Points, bool bClosed,
	bool bCurved, const TArray<FName>& Tags, FRoadNetRebuildContext& Ctx)
{
	const int32 N = Points.Num();
	if (N < 2) { return 0; }

	const ESplinePointType::Type PtType = bCurved ? ESplinePointType::Curve
	                                              : ESplinePointType::Linear;

	// Per-point tile key. This is the cut: the key flips at a junction/merge, and
	// that flip is exactly where the user wants the spline (and the road) severed.
	TArray<FIntPoint> Keys;
	Keys.SetNumUninitialized(N);
	for (int32 i = 0; i < N; ++i) { Keys[i] = TopoKeyOf(Points[i], Ctx); }

	auto EmitRun = [&](const FIntPoint& Key, int32 A, int32 B, bool bLoop) -> bool
	{
		if (Key.X == INDEX_NONE || !IsTileInCommitScope(Key, Ctx)) { return false; }
		if (B - A + 1 < 2) { return false; }               // need >=2 pts for a spline
		ARoadNetTileActor* Tile = GetOrCreateTile(Key);
		if (!Tile) { return false; }
		USplineComponent* Sp = Tile->AddSpline();
		if (!Sp) { return false; }
		Sp->ClearSplinePoints(false);
		TArray<FVector> Run;
		Run.Reserve(B - A + 1);
		for (int32 i = A; i <= B; ++i) { Run.Add(Points[i]); }
		Sp->SetSplinePoints(Run, ESplineCoordinateSpace::World, false);
		for (int32 i = 0; i < Sp->GetNumberOfSplinePoints(); ++i) { Sp->SetSplinePointType(i, PtType, false); }
		Sp->SetClosedLoop(bLoop, false);
		Sp->UpdateSpline();
		for (const FName& T : Tags) { Sp->ComponentTags.Add(T); }
		return true;
	};

	// Whole polyline lives in one tile → keep it intact (a closed loop stays
	// closed; nothing to cut).
	bool bSingleKey = true;
	for (int32 i = 1; i < N; ++i) { if (Keys[i] != Keys[0]) { bSingleKey = false; break; } }
	if (bSingleKey)
	{
		return EmitRun(Keys[0], 0, N - 1, bClosed) ? 1 : 0;
	}

	// Rotate a closed loop so index 0 sits on a key boundary; then the wrap-around
	// arc isn't split across the seam and every run is a clean open arc.
	TArray<FVector> P = Points;
	TArray<FIntPoint> K = Keys;
	if (bClosed)
	{
		int32 Rot = INDEX_NONE;
		for (int32 i = 0; i < N; ++i) { if (K[i] != K[(i + N - 1) % N]) { Rot = i; break; } }
		if (Rot > 0)
		{
			TArray<FVector> P2; TArray<FIntPoint> K2;
			P2.Reserve(N); K2.Reserve(N);
			for (int32 i = 0; i < N; ++i) { P2.Add(P[(i + Rot) % N]); K2.Add(K[(i + Rot) % N]); }
			P = MoveTemp(P2); K = MoveTemp(K2);
		}
	}

	// Emit one open arc per maximal same-key run, overlapping each neighbour by a
	// point so arcs meet at the cut. Uses P/K (rotated for loops).
	auto EmitFromPK = [&](const FIntPoint& Key, int32 A, int32 B) -> bool
	{
		if (Key.X == INDEX_NONE || !IsTileInCommitScope(Key, Ctx) || (B - A + 1) < 2) { return false; }
		ARoadNetTileActor* Tile = GetOrCreateTile(Key);
		if (!Tile) { return false; }
		USplineComponent* Sp = Tile->AddSpline();
		if (!Sp) { return false; }
		Sp->ClearSplinePoints(false);
		TArray<FVector> Run;
		Run.Reserve(B - A + 1);
		for (int32 i = A; i <= B; ++i) { Run.Add(P[i]); }
		Sp->SetSplinePoints(Run, ESplineCoordinateSpace::World, false);
		for (int32 i = 0; i < Sp->GetNumberOfSplinePoints(); ++i) { Sp->SetSplinePointType(i, PtType, false); }
		Sp->SetClosedLoop(false, false);
		Sp->UpdateSpline();
		for (const FName& T : Tags) { Sp->ComponentTags.Add(T); }
		return true;
	};

	int32 Arcs = 0;
	int32 a = 0;
	while (a < N)
	{
		int32 b = a;
		while (b + 1 < N && K[b + 1] == K[a]) { ++b; }
		const int32 Lo = FMath::Max(0, a - 1);       // share boundary with prev arc
		const int32 Hi = FMath::Min(N - 1, b + 1);   // share boundary with next arc
		if (EmitFromPK(K[a], Lo, Hi)) { ++Arcs; }
		a = b + 1;
	}
	return Arcs;
}

void URoadNetwork::CommitMedian(FRoadNetRebuildContext& Ctx)
{
	if (!WorldPtr.IsValid()) { return; }

	// Mesh the raised median strips into the tile actors. Reuse CommitLayer so
	// the strip drapes on terrain with a curb-height lift. Two layers: soil
	// (green, plantable) and walkable (concrete → sidewalk material). Empty
	// input just commits nothing (tiles were cleared in PrepareTilesForCommit).
	const int32 SoilTris = CommitLayer(TEXT("Median"), Ctx.ZoneMedianPolys,
		/*ExtraLift*/15.0, FColor(70, 110, 60), MedianMaterial, Ctx,
		/*bBakeLaneColors*/false, /*bWorldUVs*/true);
	const int32 WalkTris = CommitLayer(TEXT("MedianWalk"), Ctx.ZoneMedianWalkPolys,
		/*ExtraLift*/15.0, FColor(165, 162, 155), SidewalkMaterial, Ctx,
		/*bBakeLaneColors*/false, /*bWorldUVs*/true);
	const bool bAnyMedian = (SoilTris + WalkTris) > 0;

	// Centre planting splines (one open spline per median road) for PCG tree
	// scatter. Tagged for discovery; lifted to the median top. Routed to the
	// tile containing each road's midpoint (a spline loads with that cell).
	int32 SplineCount = 0;
	for (int32 r = 0; r < Roads.Num(); ++r)
	{
		if (!Roads[r].Lanes.bMedian) { continue; }
		const FRoadCurves* C = Ctx.Curves.Find(r);
		if (!C || C->Sampled.Num() < 2) { continue; }

		TArray<FVector> Pts = C->Sampled;
		for (FVector& P : Pts) { P.Z += kRoadZLiftCm + 15.0; } // sit on the median top

		// Cut the centre spline at junction boundaries and drop each arc on its
		// own segment tile (nothing lost — arcs cover the whole centreline).
		SplineCount += AddSplineSplitByTile(Pts, /*bClosed*/false, /*bCurved*/true,
			{ FName(TEXT("RoadNetMedianCenter")) }, Ctx);
	}

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] CommitMedian: median strips=%s, %d centre spline(s)."),
		bAnyMedian ? TEXT("yes") : TEXT("no"), SplineCount);
}

void URoadNetwork::GetJunctionEnds(int32 RoadIdx, bool& bOutAtStart, bool& bOutAtEnd) const
{
	bOutAtStart = bOutAtEnd = false;
	if (!Roads.IsValidIndex(RoadIdx) || !Roads[RoadIdx].IsValid()) { return; }

	// The joint topology itself is rebuild-scoped (FRoadNetRebuildContext::Joints)
	// and there is no serialized copy, so callers that run OUTSIDE a rebuild — a
	// parking bay added from the panel — have to re-derive it. This repeats the
	// same spatial endpoint weld BuildEndpointJoints uses, at the same radius, so
	// the two cannot disagree about where a junction is.
	//
	// ponytail: O(roads) per call, which is nothing for a per-click action but
	// would not do inside a loop over roads. It also counts a plain two-road
	// continuation seam as a junction, so a bay is set back from one — the
	// conservative direction. The upgrade path is publishing the joint arms from
	// the rebuild the way ZoneGraphRuns are published.
	const FVector2D S(Roads[RoadIdx].Ref[0]);
	const FVector2D E(Roads[RoadIdx].Ref.Last());
	const double R2 = FMath::Square(kEndpointWeldCm);
	for (int32 i = 0; i < Roads.Num(); ++i)
	{
		if (i == RoadIdx || !Roads[i].IsValid()) { continue; }
		const FVector2D Ends[2] = { FVector2D(Roads[i].Ref[0]), FVector2D(Roads[i].Ref.Last()) };
		for (const FVector2D& Q : Ends)
		{
			if (FVector2D::DistSquared(Q, S) <= R2) { bOutAtStart = true; }
			if (FVector2D::DistSquared(Q, E) <= R2) { bOutAtEnd = true; }
		}
		if (bOutAtStart && bOutAtEnd) { return; }
	}
}

int32 URoadNetwork::SetZoneGraphOnRoads(TArrayView<const int32> RoadIndices, bool bOn)
{
	int32 Changed = 0;
	for (int32 r : RoadIndices)
	{
		if (!Roads.IsValidIndex(r) || Roads[r].bZoneGraph == bOn) { continue; }
		Roads[r].bZoneGraph = bOn;
		++Changed;
	}
	return Changed;
}

void URoadNetwork::PublishZoneGraphRuns(FRoadNetRebuildContext& Ctx)
{
	// Master gate off: nothing is published, and the builder's next sync destroys
	// whatever shapes are standing. Per-road flags are left alone, so the feature
	// can be parked and resumed without re-picking every segment.
	if (!bBuildZoneGraph)
	{
		ZoneGraphRuns.Reset();
		return;
	}

	// How far the junction reaches down each arm. A joint's own arms carry the
	// half-widths that produced its mouth, so the widest of them is the right
	// scale to pull back by — a lane-width guess would leave a spline hanging in
	// the middle of a wide intersection.
	TMap<FGuid, TPair<double, double>> Trim;   // road id -> (at start, at end)
	for (const FRoadNetJoint& J : Ctx.Joints)
	{
		if (J.Arms.Num() < 2) { continue; }
		double Reach = 0.0;
		for (const FRoadNetJointArm& Arm : J.Arms) { Reach = FMath::Max(Reach, (double)Arm.HalfWidthCm); }
		// Past the mouth, not level with it: a lane that ends exactly on the
		// junction edge still overlaps the crossing traffic's turning path.
		Reach += 150.0;
		for (const FRoadNetJointArm& Arm : J.Arms)
		{
			if (!Roads.IsValidIndex(Arm.Road)) { continue; }
			TPair<double, double>& T = Trim.FindOrAdd(Roads[Arm.Road].Id, TPair<double, double>(0.0, 0.0));
			double& Side = Arm.bAtStart ? T.Key : T.Value;
			Side = FMath::Max(Side, Reach);
		}
	}

	// Per-road replacement keyed on the stable id, NOT a wholesale reset: a
	// windowed rebuild only has curves for the roads in its scope, and clearing
	// the array would silently drop every other road's shape.
	for (const TPair<int32, FRoadCurves>& Pair : Ctx.Curves)
	{
		const int32 r = Pair.Key;
		if (!Roads.IsValidIndex(r)) { continue; }
		const FRoadDef& R = Roads[r];
		ZoneGraphRuns.RemoveAll([&R](const FRoadNetZoneRun& Run) { return Run.RoadId == R.Id; });
		if (!R.bZoneGraph || Pair.Value.Sampled.Num() < 2) { continue; }

		const TPair<double, double>* T = Trim.Find(R.Id);
		TArray<FVector> Pts = Pair.Value.Sampled;
		if (T)
		{
			TrimPolylineEnd(Pts, /*bFromStart*/true,  T->Key);
			TrimPolylineEnd(Pts, /*bFromStart*/false, T->Value);
		}
		// Wholly inside its own junctions: a stub here is worse than nothing,
		// because ZoneGraph would run it straight into the crossing traffic.
		if (Pts.Num() < 2 || RoadNetMath::TotalLength(Pts) < 500.0) { continue; }

		FRoadNetZoneRun Run;
		Run.RoadId = R.Id;
		Run.Points = MoveTemp(Pts);
		Run.LanesForward = 0;
		Run.LanesBackward = 0;
		Run.LaneWidthCm = 0.f;
		for (const FRoadNetLane& L : GetLanesLeftToRight(r))
		{
			if (L.Type == ERoadNetLaneType::Bicycle) { Run.bHasBikeLane = true; }
			if (!L.bDrivable()) { continue; }
			(L.bTravelsForward(bDriveOnLeft) ? Run.LanesForward : Run.LanesBackward) += 1;
			Run.LaneWidthCm = FMath::Max(Run.LaneWidthCm, (float)L.Width);
		}
		if (Run.LanesForward + Run.LanesBackward == 0) { continue; }
		ZoneGraphRuns.Add(MoveTemp(Run));
	}

	// Drop runs whose road was deleted or merged away. Cheap, and it is the only
	// thing standing between a delete and a shape that outlives its road.
	TSet<FGuid> Live;
	for (const FRoadDef& R : Roads) { if (R.bZoneGraph) { Live.Add(R.Id); } }
	ZoneGraphRuns.RemoveAll([&Live](const FRoadNetZoneRun& Run) { return !Live.Contains(Run.RoadId); });

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] PublishZoneGraphRuns: %d run(s) for the ZoneGraph builder."),
		ZoneGraphRuns.Num());
}

void URoadNetwork::CommitPerimeters(FRoadNetRebuildContext& Ctx)
{
	if (!WorldPtr.IsValid()) { return; }

	// Perimeter loops become spline components — the seam a PCG graph samples for
	// road edges / blocks. Each loop is CUT at junction/merge boundaries so its
	// arcs land on the segment tiles they border (a loop wholly inside one tile
	// stays a single closed spline). Tags preserved verbatim so tag-based PCG
	// discovery keeps working across the split.
	int32 LoopCount = 0;
	for (const FRoadNetLoop& Loop : Ctx.PerimeterLoops)
	{
		if (Loop.Points.Num() < 3) { continue; }
		LoopCount += AddSplineSplitByTile(Loop.Points, /*bClosed*/true, /*bCurved*/false,
			{ FName(TEXT("RoadNetPerimeter")),
			  Loop.bOuter ? FName(TEXT("RoadNetPerimeterOuter")) : FName(TEXT("RoadNetPerimeterHole")) },
			Ctx);
	}

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] CommitPerimeters: %d perimeter arc(s) for PCG (cut at junctions)."), LoopCount);
}

void URoadNetwork::CommitLaneGraph(FRoadNetRebuildContext& Ctx)
{
	if (!WorldPtr.IsValid()) { return; }

	// Lane-connectivity movements become open spline components (one per
	// connection), curving Entry → joint centre → Exit so a PCG graph / traffic
	// system can sample turn paths. A movement lives AT the junction, so route it
	// to the tile holding the joint centre (the junction tile) rather than its
	// entry — splitting a turn path into slivers would be wrong. Tags preserved.
	int32 Made = 0;
	for (const FRoadNetLaneConnection& Cn : Ctx.LaneConnections)
	{
		const FVector RoutePt = Ctx.Joints.IsValidIndex(Cn.Joint)
			? FVector(Ctx.Joints[Cn.Joint].Location.X, Ctx.Joints[Cn.Joint].Location.Y, Ctx.Joints[Cn.Joint].Z)
			: (Cn.Entry + Cn.Exit) * 0.5;
		const FIntPoint Coord = TopoKeyOf(RoutePt, Ctx);
		if (Coord.X == INDEX_NONE || !IsTileInCommitScope(Coord, Ctx)) { continue; }
		ARoadNetTileActor* Tile = GetOrCreateTile(Coord);
		if (!Tile) { continue; }
		USplineComponent* Sp = Tile->AddSpline();
		if (!Sp) { continue; }
		Sp->ClearSplinePoints(false);

		TArray<FVector> Pts;
		Pts.Add(Cn.Entry);
		// Curve turns through the joint centre; a straight-through seam stays
		// linear (Entry → Exit) so it doesn't wobble.
		if (!Cn.bThrough && Ctx.Joints.IsValidIndex(Cn.Joint))
		{
			const FRoadNetJoint& J = Ctx.Joints[Cn.Joint];
			Pts.Add(FVector(J.Location.X, J.Location.Y, J.Z));
		}
		Pts.Add(Cn.Exit);
		Sp->SetSplinePoints(Pts, ESplineCoordinateSpace::World, false);
		const ESplinePointType::Type PtType = Cn.bThrough ? ESplinePointType::Linear
		                                                   : ESplinePointType::Curve;
		for (int32 i = 0; i < Sp->GetNumberOfSplinePoints(); ++i)
		{
			Sp->SetSplinePointType(i, PtType, false);
		}
		Sp->SetClosedLoop(false, false);
		Sp->UpdateSpline();
		Sp->ComponentTags.Add(FName(TEXT("RoadNetLaneGraph")));
		Sp->ComponentTags.Add(Cn.bThrough ? FName(TEXT("RoadNetLaneThrough"))
		                                   : FName(TEXT("RoadNetLaneTurn")));
		++Made;
	}

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] CommitLaneGraph: %d lane-connection splines for PCG."), Made);
}

void URoadNetwork::CommitSegmentSplines(FRoadNetRebuildContext& Ctx)
{
	if (!WorldPtr.IsValid()) { return; }

	// One editable set per segment tile: the arm's own centreline plus its two
	// outer edges. Points are the dense sampled centreline/edges (already the
	// control points a user drags); the whole arm is one spline on its ONE tile
	// (SegTileKey is alias-resolved, so a divided pair's two carriageways drop
	// their centre/edge splines on the shared tile).
	int32 Centre = 0, Edge = 0;
	auto AddSeg = [&](const FIntPoint& Key, const TArray<FVector>& Pts, const TArray<FName>& Tags) -> bool
	{
		if (Key.X == INDEX_NONE || !IsTileInCommitScope(Key, Ctx) || Pts.Num() < 2) { return false; }
		ARoadNetTileActor* Tile = GetOrCreateTile(Key);
		if (!Tile) { return false; }
		USplineComponent* Sp = Tile->AddSpline();
		if (!Sp) { return false; }
		Sp->ClearSplinePoints(false);
		Sp->SetSplinePoints(Pts, ESplineCoordinateSpace::World, false);
		for (int32 i = 0; i < Sp->GetNumberOfSplinePoints(); ++i) { Sp->SetSplinePointType(i, ESplinePointType::Curve, false); }
		Sp->SetClosedLoop(false, false);
		Sp->UpdateSpline();
		for (const FName& T : Tags) { Sp->ComponentTags.Add(T); }
		return true;
	};

	for (const TPair<int32, TArray<TPair<int32, int32>>>& KV : Ctx.RoadArmRuns)
	{
		const int32 r = KV.Key;
		if (!Roads.IsValidIndex(r) || !Roads[r].Id.IsValid()) { continue; }
		const FRoadCurves* C = Ctx.Curves.Find(r);
		if (!C || C->Sampled.Num() < 2) { continue; }
		const bool bHaveEdges = (C->LeftEdge.Num() == C->Sampled.Num() && C->RightEdge.Num() == C->Sampled.Num());

		auto Slice = [&](const TArray<FVector>& Src, int32 Lo, int32 Hi) -> TArray<FVector>
		{
			TArray<FVector> Out;
			if (Src.Num() != C->Sampled.Num()) { return Out; }
			Out.Reserve(Hi - Lo + 1);
			for (int32 i = Lo; i <= Hi; ++i) { FVector P = Src[i]; P.Z += kRoadZLiftCm; Out.Add(P); }
			return Out;
		};

		for (int32 av = 0; av < KV.Value.Num(); ++av)
		{
			const TPair<int32, int32>& Run = KV.Value[av];
			if (Run.Key < 0 || Run.Value <= Run.Key || !C->Sampled.IsValidIndex(Run.Value)) { continue; }
			const FIntPoint Key = SegTileKey(Roads[r].Id, av);
			if (Key.X == INDEX_NONE || !IsTileInCommitScope(Key, Ctx)) { continue; }
			if (AddSeg(Key, Slice(C->Sampled, Run.Key, Run.Value), { FName(TEXT("RoadNetSegmentCenter")) })) { ++Centre; }
			if (bHaveEdges)
			{
				if (AddSeg(Key, Slice(C->LeftEdge,  Run.Key, Run.Value), { FName(TEXT("RoadNetSegmentEdge")), FName(TEXT("RoadNetSegmentEdgeLeft"))  })) { ++Edge; }
				if (AddSeg(Key, Slice(C->RightEdge, Run.Key, Run.Value), { FName(TEXT("RoadNetSegmentEdge")), FName(TEXT("RoadNetSegmentEdgeRight")) })) { ++Edge; }
			}
		}
	}
	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] CommitSegmentSplines: %d centre + %d edge spline(s)."), Centre, Edge);
}
