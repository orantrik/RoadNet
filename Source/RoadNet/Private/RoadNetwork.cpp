// RoadNetwork.cpp — orchestration + early rebuild stages (§10.18).
#include "RoadNetwork.h"
#include "RoadNetMath.h"
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

// Pipeline tunables (§2.6). Kept local until a settings object is added.
namespace
{
	// §2.6 PolylineDensity is now a per-network property (URoadNetwork::PolylineDensityCm).
	constexpr double kAdaptiveTurnRad   = 0.0873;  // ~5° knot-preserve threshold
	constexpr double kRoadZLiftCm       = 12.0;    // lift above landscape (anti z-fight)
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
		for (const TPair<int32, bool>& A : J.Arms)
		{
			const int32* zz = RoadZone.Find(A.Key);
			if (!zz) { continue; }
			if (z == INDEX_NONE) { z = *zz; }
			if (*zz == z) { MaxHalf = FMath::Max(MaxHalf, HalfW(A.Key)); }
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
			TArray<FGeneralPolygon2d> Piece;
			if (Merged.Num() > 0 && PolygonsIntersection(Merged, CPArr, Piece) && Piece.Num() > 0)
			{
				Surf[z].FindOrAdd(JKey).Append(MoveTemp(Piece));
			}
			TArray<FGeneralPolygon2d> WPiece;
			if (Band.Num() > 0 && PolygonsIntersection(Band, CPArr, WPiece) && WPiece.Num() > 0)
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
			const double SwOut = Half + SwW + 60.0;

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
						TArray<FGeneralPolygon2d> Cut;
						if (RoadNetSurface::Difference(Piece, Carve, Cut)) { Piece = MoveTemp(Cut); }
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
						if (!PolygonsIntersection(Band, RArr, Piece) || Piece.Num() == 0) { return; }
						if (Carve.Num() > 0)
						{
							TArray<FGeneralPolygon2d> Cut;
							if (RoadNetSurface::Difference(Piece, Carve, Cut)) { Piece = MoveTemp(Cut); }
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
	int32 TotalLanes = 0;
	bool  bSwL = false, bSwR = false;
	float SwW = 0.f;
	for (int32 r : Idx)
	{
		TotalLanes += FMath::Max(1, Roads[r].Lanes.EffectiveLaneCount());
		bSwL |= Roads[r].Lanes.bSidewalkLeft;
		bSwR |= Roads[r].Lanes.bSidewalkRight;
		SwW = FMath::Max(SwW, Roads[r].Lanes.SidewalkWidth);
	}

	FRoadDef Merged = Roads[Primary];   // inherit class / source / name / grade
	Merged.Id = FGuid::NewGuid();
	Merged.Ref = MoveTemp(Mid);
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
			case ERoadNetLaneType::Bicycle: return 150.0; // 1.5 m
			case ERoadNetLaneType::Parking: return 250.0; // 2.5 m
			default:                        return 350.0; // 3.5 m driving lane
		}
	}

	// Recompute every authored lane's CenterOffset + Side by stacking widths
	// left→right, centred on the reference line. A central median (MedianHalf>0)
	// opens a gap in the middle. Lanes are assumed already ordered left→right.
	void RelayoutLanes(TArray<FRoadNetLane>& Lanes, double MedianHalfCm)
	{
		double Sum = 0.0;
		for (const FRoadNetLane& Ln : Lanes) { Sum += FMath::Max(1.f, Ln.Width); }
		const double Total = Sum + 2.0 * FMath::Max(0.0, MedianHalfCm);
		double Cursor = -0.5 * Total;
		bool bGapDone = (MedianHalfCm <= 0.0);
		for (FRoadNetLane& Ln : Lanes)
		{
			const double W = FMath::Max(1.f, Ln.Width);
			// Insert the median gap once, before the first lane whose centre would
			// cross the reference line.
			if (!bGapDone && (Cursor + 0.5 * W) >= 0.0) { Cursor += 2.0 * MedianHalfCm; bGapDone = true; }
			Ln.CenterOffset = Cursor + 0.5 * W;
			Ln.Side = (Ln.CenterOffset < 0.0) ? ERoadNetSide::Left : ERoadNetSide::Right;
			Cursor += W;
		}
	}

	// Materialise a road's lanes as authored DetailedLanes ordered left→right,
	// so per-lane edits (insert/type) have concrete entities to act on. Idempotent
	// once authored.
	void EnsureDetailedLanes(FRoadNetLaneSpec& L)
	{
		if (!L.HasDetailedLanes()) { L.DetailedLanes = L.ResolveLanes(); }
		L.DetailedLanes.Sort([](const FRoadNetLane& A, const FRoadNetLane& B)
			{ return A.CenterOffset < B.CenterOffset; });
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
		RelayoutLanes(L.DetailedLanes, (double)L.MedianHalfCm());
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
		RelayoutLanes(L.DetailedLanes, (double)L.MedianHalfCm());
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
	TArray<FRoadNetLane> Lanes = Roads[RoadIdx].Lanes.ResolveLanes();
	Lanes.Sort([](const FRoadNetLane& A, const FRoadNetLane& B)
		{ return A.CenterOffset < B.CenterOffset; });
	return Lanes;
}

bool URoadNetwork::MaterializeLanes(int32 RoadIdx)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	EnsureDetailedLanes(Roads[RoadIdx].Lanes);
	return true;
}

int32 URoadNetwork::InsertLaneRelative(int32 RoadIdx, int32 LaneLtoR, bool bRightSide)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return INDEX_NONE; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	EnsureDetailedLanes(L);
	if (!L.DetailedLanes.IsValidIndex(LaneLtoR)) { return INDEX_NONE; }

	FRoadNetLane NL;
	NL.LaneId = FGuid::NewGuid();
	NL.Type   = ERoadNetLaneType::Normal;
	NL.Width  = (float)LaneTypeDefaultWidthCm(ERoadNetLaneType::Normal);

	const int32 Pos = bRightSide ? (LaneLtoR + 1) : LaneLtoR;
	L.DetailedLanes.Insert(NL, Pos);
	RelayoutLanes(L.DetailedLanes, (double)L.MedianHalfCm());

	// New left→right index of the originally selected lane (so the caller keeps
	// its highlight): unchanged when we inserted to its right, +1 to its left.
	return bRightSide ? LaneLtoR : (LaneLtoR + 1);
}

ERoadNetLaneType URoadNetwork::CycleLaneType(int32 RoadIdx, int32 LaneLtoR, int32 Dir)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return ERoadNetLaneType::Normal; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	EnsureDetailedLanes(L);
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
	RelayoutLanes(L.DetailedLanes, (double)L.MedianHalfCm());
	return Ln.Type;
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
	L.bMedian = !L.bMedian;
	return L.bMedian;
}

ERoadNetMedianEdge URoadNetwork::CycleMedianEdge(int32 RoadIdx, int32 Dir)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return ERoadNetMedianEdge::Plantable; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	L.bMedian = true;   // cycling edge implies a median
	constexpr int32 N = 4; // Plantable, CurbOnly, SidewalkAndCurb, PlantableWalkCurb
	int32 V = ((int32)L.MedianEdge + (Dir >= 0 ? 1 : N - 1)) % N;
	L.MedianEdge = (ERoadNetMedianEdge)V;
	return L.MedianEdge;
}

float URoadNetwork::AdjustMedianWidth(int32 RoadIdx, float DeltaCm)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return 0.f; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	L.MedianWidth = FMath::Clamp(L.MedianWidth + DeltaCm, 30.f, 2000.f);
	return L.MedianWidth;
}

bool URoadNetwork::IsMedian(int32 RoadIdx) const
{
	return Roads.IsValidIndex(RoadIdx) && Roads[RoadIdx].Lanes.bMedian;
}

float URoadNetwork::GetMedianWidth(int32 RoadIdx) const
{
	return Roads.IsValidIndex(RoadIdx) ? Roads[RoadIdx].Lanes.MedianWidth : 0.f;
}

double URoadNetwork::AdjustJunctionSmoothing(double DeltaCm)
{
	JunctionSmoothingCm = FMath::Clamp(JunctionSmoothingCm + DeltaCm, 0.0, 300.0);
	return JunctionSmoothingCm;
}

URoadNetwork::URoadNetwork()
{
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
	return Changed;
}

float URoadNetwork::AdjustSidewalkWidth(int32 RoadIdx, float DeltaCm)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return 0.f; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	L.SidewalkWidth = FMath::Clamp(L.SidewalkWidth + DeltaCm, 0.f, 2000.f);
	// First widening from nothing turns the sidewalk on (both sides) so the
	// nudge is immediately visible.
	if (L.SidewalkWidth > 0.f && !L.bSidewalkLeft && !L.bSidewalkRight)
	{
		L.bSidewalkLeft  = true;
		L.bSidewalkRight = true;
	}
	return L.SidewalkWidth;
}

int32 URoadNetwork::AddStandardParkingBay(int32 RoadIdx, ERoadNetSide Side, ERoadNetParkingLayout Layout)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return INDEX_NONE; }
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
	Bay.StartArcCm = 0.f;
	Bay.LengthCm   = 0.f; // whole road
	return Roads[RoadIdx].ParkingBays.Add(Bay);
}

int32 URoadNetwork::ClearParkingBays(int32 RoadIdx)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return 0; }
	const int32 N = Roads[RoadIdx].ParkingBays.Num();
	Roads[RoadIdx].ParkingBays.Reset();
	return N;
}

#if WITH_EDITOR
void URoadNetwork::PostEditUndo()
{
	Super::PostEditUndo();

	// The transaction restored the reflected authoring state (Roads / junction
	// overrides); rebind the world from our owning actor (the weak WorldPtr may
	// have gone stale across the undo) and regenerate the disposable geometry so
	// the viewport reflects the undone/redone edit.
	if (const AActor* Owner = GetTypedOuter<AActor>())
	{
		if (UWorld* World = Owner->GetWorld()) { WorldPtr = World; }
	}
	Rebuild();
}
#endif

void URoadNetwork::Rebuild(TArrayView<const int32> Modified, const FBox2D& DirtyRegionWorld)
{
	const double T0 = FPlatformTime::Seconds();

	FRoadNetRebuildContext Ctx;
	Ctx.ExplicitDirtyBox = DirtyRegionWorld;
	if (Modified.IsEmpty())
	{
		Ctx.Modified.SetNumUninitialized(Roads.Num());
		for (int32 i = 0; i < Roads.Num(); ++i) { Ctx.Modified[i] = i; }
	}
	else
	{
		Ctx.Modified = TArray<int32>(Modified.GetData(), Modified.Num());
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
	BuildCurves(Ctx);             // §10.2–§10.4 reference line + outer edges
	const double tCurves = Now();  Trace(TEXT("curves"), tCurves - tA);

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
			Cor.FlatHalfCm = Half + Walk;
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
	BuildCrossings(Ctx);          // §10.12 grid broadphase (shared by zones+surface)
	const double tCross = Now();   Trace(TEXT("crossings"), tCross - tCurves);
	BuildEndpointJoints(Ctx);     // §10.7 topology from shared node ids
	const double tJoints = Now();  Trace(TEXT("joints"), tJoints - tCross);
	BuildZones(Ctx);              // §10.12 grade-separation layering
	const double tZones = Now();   Trace(TEXT("zones"), tZones - tJoints);
	BuildSurfaceUnion(Ctx);       // §10.9 Clipper2 boolean-union per zone
	const double tSurface = Now(); Trace(TEXT("surface"), tSurface - tZones);
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
		TEXT("[RoadNet] Rebuild stages (ms): curves %.1f, crossings %.1f, joints %.1f, zones %.1f, surface %.1f, perimeters %.1f, lanegraph %.1f, commit %.1f."),
		(tCurves - tA) * 1000.0, (tCross - tCurves) * 1000.0, (tJoints - tCross) * 1000.0,
		(tZones - tJoints) * 1000.0, (tSurface - tZones) * 1000.0, (tPerim - tSurface) * 1000.0,
		(tGraph - tPerim) * 1000.0, (tCommit - tRibbon) * 1000.0);
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
		// Round the piecewise-linear reference with a G2 (curvature-continuous)
		// cubic spline through the source knots, THEN resample at uniform spacing.
		// G2 (vs C1 Catmull-Rom) means curvature no longer jumps at knots, so the
		// offset edges and meshed surface stop faceting on curves.
		TArray<FVector> Smooth;
		RoadNetMath::SmoothG2Spline(R.Ref, DensityCm, Smooth);
		RoadNetMath::ResampleByArcLength(Smooth, DensityCm, C.Sampled, kAdaptiveTurnRad);
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

		if (R.OuterEdgeRight.Num() > 0 || R.OuterEdgeLeft.Num() > 0)
		{
			TArray<double> S; ArcLengths(C.Sampled, S);
			TArray<double> OffR, OffL;
			OffR.SetNumUninitialized(C.Sampled.Num());
			OffL.SetNumUninitialized(C.Sampled.Num());
			for (int32 i = 0; i < C.Sampled.Num(); ++i)
			{
				OffR[i] = SampleProfile(R.OuterEdgeRight, S[i], +Half); // +side
				OffL[i] = SampleProfile(R.OuterEdgeLeft,  S[i], -Half); // −side
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
		Joints[JIdx].Arms.Emplace(RoadIdx, bStart);
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
		// meeting at a node leave a wedge/notch that a disc at the node fills. The
		// disc is centred on the centroid of the participating arm endpoints and
		// grown to span the gap between them, so the fill reaches both road ends.
		auto ArmEndpoint = [&](const TPair<int32, bool>& Arm) -> FVector2D
		{
			if (const FRoadCurves* C = Ctx.Curves.Find(Arm.Key))
			{
				const FVector& P = Arm.Value ? C->Sampled[0] : C->Sampled.Last();
				return FVector2D(P.X, P.Y);
			}
			const FVector& P = Arm.Value ? Roads[Arm.Key].Ref[0] : Roads[Arm.Key].Ref.Last();
			return FVector2D(P.X, P.Y);
		};

		for (const FRoadNetJoint& J : Ctx.Joints)
		{
			// Terminal (dead-end) joints have a single arm — nothing to bridge.
			if (J.Arms.Num() < 2) { continue; }

			double MaxHalf = 0.0;
			FVector2D Centroid(0, 0);
			int32 InZoneArms = 0;
			for (const TPair<int32, bool>& Arm : J.Arms)
			{
				if (!ZoneSet.Contains(Arm.Key)) { continue; }
				Centroid += ArmEndpoint(Arm);
				MaxHalf = FMath::Max(MaxHalf, HalfWidth(Arm.Key));
				++InZoneArms;
			}
			if (InZoneArms < 2 || MaxHalf <= 0.0) { continue; }
			Centroid /= (double)InZoneArms;

			// Radius must reach the farthest arm endpoint (so the gap is covered)
			// plus that road's half-width (so the disc meets its edges cleanly).
			double R = MaxHalf;
			for (const TPair<int32, bool>& Arm : J.Arms)
			{
				if (!ZoneSet.Contains(Arm.Key)) { continue; }
				const double D = FVector2D::Distance(Centroid, ArmEndpoint(Arm));
				R = FMath::Max(R, D + HalfWidth(Arm.Key));
			}
			AddJPoint(Centroid, R);
		}

		// (b) centerline crossings between roads in this zone (X and touching-T).
		// Uses the precomputed shared crossings (grid broadphase) instead of a
		// per-zone O(N^2) segment sweep.
		for (const FRoadNetCrossing& X : Ctx.Crossings)
		{
			if (!ZoneSet.Contains(X.RoadA) || !ZoneSet.Contains(X.RoadB)) { continue; }
			const double R = FMath::Max(HalfWidth(X.RoadA), HalfWidth(X.RoadB));
			AddJPoint(X.Point, R);
		}

		TArray<UE::Geometry::FGeneralPolygon2d> Discs;         // fill discs (radius R)
		Discs.Reserve(JPts.Num());
		for (const TPair<FVector2D, double>& E : JPts)
		{
			UE::Geometry::FGeneralPolygon2d Disc;
			// Higher segment count → smoother junction fill (fewer facet corners).
			RoadNetSurface::MakeDisc(E.Key, E.Value, /*Segments*/48, Disc);
			Discs.Add(MoveTemp(Disc));
		}

		// JunctionSmoothingCm drives the morphological close (round corners / gap
		// bridging). Keep a small floor so abutting arms always weld.
		const double CloseCm = FMath::Max(2.0, JunctionSmoothingCm);

		// Per-junction smoothing: if ANY junction touched by this zone carries an
		// override, switch the merge to per-junction local closes; otherwise use
		// the single global close (identical to the original output). Cheap: for a
		// windowed junction edit JPts holds only the edited junction.
		TArray<RoadNetSurface::FJunctionClose> JClose;
		bool bAnyOverride = false;
		{
			// Match a JPt to a stored override the same way the interactive picker
			// does (nearest config within tolerance).
			constexpr double kJunctionMatchCm = 600.0; // 6 m
			JClose.Reserve(JPts.Num());
			for (const TPair<FVector2D, double>& E : JPts)
			{
				double e = JunctionSmoothingCm;
				double BestD2 = FMath::Square(kJunctionMatchCm);
				for (const FRoadNetJunctionConfig& Cfg : JunctionConfigs)
				{
					if (Cfg.SmoothingCm < 0.f) { continue; }
					const double D2 = FVector2D::DistSquared(Cfg.Location, E.Key);
					if (D2 < BestD2) { BestD2 = D2; e = Cfg.SmoothingCm; bAnyOverride = true; }
				}
				RoadNetSurface::FJunctionClose J;
				J.Center = E.Key;
				J.FillRadiusCm = E.Value;
				J.CloseCm = FMath::Max(0.0, e);
				JClose.Add(J);
			}
		}

		if (bAnyOverride)
		{
			RoadNetSurface::BuildMergedSurface(Ptrs, Ctx.ZoneSurfacePolys[z], CloseCm, &Discs, &JClose);
		}
		else
		{
			RoadNetSurface::BuildMergedSurface(Ptrs, Ctx.ZoneSurfacePolys[z], CloseCm, &Discs);
		}
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
						const int32 A = J.Arms[i].Key, B = J.Arms[j].Key;
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
				TArray<FGeneralPolygon2d> RegionU;
				if (!PolygonsUnion(Overlaps, RegionU, /*bCopyInputOnFailure*/true)) { RegionU = MoveTemp(Overlaps); }

				const double Setback = FMath::Max(0.0, JunctionClearanceCm);
				if (Setback > 1.0 && RegionU.Num() > 0)
				{
					TArray<FGeneralPolygon2d> Dilated;
					if (PolygonsOffset(Setback, RegionU, Dilated, /*bCopyInputOnFailure*/true,
							/*MiterLimit*/2.0, EPolygonOffsetJoinType::Round,
							EPolygonOffsetEndType::Polygon, /*MaxStepsPerRadian*/16.0,
							/*DefaultStepsPerRadianScale*/1.0e-3))
					{
						Clip = MoveTemp(Dilated);
					}
					else { Clip = MoveTemp(RegionU); }
				}
				else { Clip = MoveTemp(RegionU); }
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

			double MaxW = 0.0;
			bool bAnySide = false, bAnyDisabled = false;
			for (int32 RoadIdx : ZoneRoads)
			{
				if (!Roads.IsValidIndex(RoadIdx)) { continue; }
				const FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
				const double W = (double)L.SidewalkWidth;
				if (W <= 0.0 || (!L.bSidewalkLeft && !L.bSidewalkRight)) { continue; }
				bAnySide = true;
				MaxW = FMath::Max(MaxW, W);
				if (!L.bSidewalkLeft || !L.bSidewalkRight) { bAnyDisabled = true; }
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
					if (bAnyDisabled)
					{
						// Mask the band to only the requested sides. Ribbons are
						// intentionally generous (overlap the road edge and extend
						// past the band) so the intersection edge is the band's.
						TArray<FGeneralPolygon2d> Masks;
						for (int32 RoadIdx : ZoneRoads)
						{
							const FRoadCurves* C = Ctx.Curves.Find(RoadIdx);
							if (!C || !Roads.IsValidIndex(RoadIdx)) { continue; }
							const FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
							const double W = (double)L.SidewalkWidth;
							if (W <= 0.0) { continue; }
							const double Half = HalfWidth(RoadIdx);
							const double In  = FMath::Max(1.0, Half - 30.0);
							const double Out = Half + W + 60.0;
							FGeneralPolygon2d Mask;
							if (L.bSidewalkLeft  && RoadNetSurface::BuildSideRibbon(C->Sampled, +In, +Out, Mask)) { Masks.Add(Mask); }
							if (L.bSidewalkRight && RoadNetSurface::BuildSideRibbon(C->Sampled, -In, -Out, Mask)) { Masks.Add(Mask); }
						}
						TArray<FGeneralPolygon2d> MaskU;
						if (Masks.Num() > 0 && PolygonsUnion(Masks, MaskU, /*bCopyInputOnFailure*/true)
							&& PolygonsIntersection(Band, MaskU, Ctx.ZoneSidewalkPolys[z]))
						{
							// masked band stored
						}
						else
						{
							Ctx.ZoneSidewalkPolys[z] = MoveTemp(Band);
						}
					}
					else
					{
						Ctx.ZoneSidewalkPolys[z] = MoveTemp(Band);
					}
					Ctx.SidewalkPolys.Append(Ctx.ZoneSidewalkPolys[z]);
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
		TArray<UE::Geometry::FGeneralPolygon2d> White, Yellow;
		for (int32 RoadIdx : ZoneRoads)
		{
			const FRoadCurves* C = Ctx.Curves.Find(RoadIdx);
			if (C && Roads.IsValidIndex(RoadIdx))
			{
				RoadNetMarkings::BuildRoadMarkings(Roads[RoadIdx], *C, White, Yellow);
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
	// §12.2 — RoadBLD ships no routing graph, so we build our own: at every
	// welded joint, connect each drivable lane ENTERING the joint to each
	// drivable lane LEAVING it on the other arms (all turn movements), plus the
	// straight-through pairing at a 2-arm seam. Direction is side-based
	// (ROADBLD_FEATURES.md §4): Right lanes travel +arc, Left lanes travel −arc.
	Ctx.LaneConnections.Reset();

	// Per-arm lane end at a joint: world position + whether it feeds INTO the
	// joint (incoming) and/or leaves it (outgoing), plus its lane index.
	struct FLaneEnd
	{
		int32   Road = INDEX_NONE;
		int32   Lane = INDEX_NONE;
		FVector Pos = FVector::ZeroVector;
		bool    bIn = false;   // travels into the joint
		bool    bOut = false;  // travels out of the joint
	};

	// Cache resolved lanes + their two centreline endpoints per road so a road
	// touching two joints is only offset once.
	struct FRoadLaneCache { TArray<FRoadNetLane> Lanes; TArray<FVector> StartPt, EndPt; };
	TMap<int32, FRoadLaneCache> Cache;

	auto GetCache = [&](int32 RoadIdx) -> const FRoadLaneCache*
	{
		if (const FRoadLaneCache* Hit = Cache.Find(RoadIdx)) { return Hit; }
		const FRoadCurves* C = Ctx.Curves.Find(RoadIdx);
		if (!C || C->Sampled.Num() < 2 || !Roads.IsValidIndex(RoadIdx)) { return nullptr; }

		FRoadLaneCache New;
		New.Lanes = Roads[RoadIdx].Lanes.ResolveLanes();
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

	for (int32 ji = 0; ji < Ctx.Joints.Num(); ++ji)
	{
		const FRoadNetJoint& J = Ctx.Joints[ji];
		if (J.Arms.Num() < 2) { continue; } // a terminal end connects to nothing

		// Gather every arm's lane ends at this joint.
		TArray<FLaneEnd> Ends;
		for (const TPair<int32, bool>& Arm : J.Arms)
		{
			const int32 RoadIdx  = Arm.Key;
			const bool  bAtStart = Arm.Value;
			const FRoadLaneCache* RC = GetCache(RoadIdx);
			if (!RC) { continue; }

			for (int32 li = 0; li < RC->Lanes.Num(); ++li)
			{
				const FRoadNetLane& L = RC->Lanes[li];
				if (!L.bDrivable()) { continue; }

				// Right = travels +arc (start→end); Left = travels −arc.
				// Center (turn lane) is treated as bidirectional.
				bool bIn = false, bOut = false;
				const bool bRight  = (L.Side == ERoadNetSide::Right);
				const bool bLeft   = (L.Side == ERoadNetSide::Left);
				const bool bCenter = (L.Side == ERoadNetSide::Center);
				if (bAtStart)
				{
					// At the road's start: forward(right) leaves, backward(left) arrives.
					bOut = bRight || bCenter;
					bIn  = bLeft  || bCenter;
				}
				else
				{
					bIn  = bRight || bCenter;
					bOut = bLeft  || bCenter;
				}
				if (!bIn && !bOut) { continue; }

				FLaneEnd E;
				E.Road = RoadIdx;
				E.Lane = li;
				E.Pos  = bAtStart ? RC->StartPt[li] : RC->EndPt[li];
				E.bIn  = bIn;
				E.bOut = bOut;
				Ends.Add(E);
			}
		}

		const bool bSeam = (J.Arms.Num() == 2);
		// Connect every incoming lane to every outgoing lane on a DIFFERENT road.
		for (const FLaneEnd& In : Ends)
		{
			if (!In.bIn) { continue; }
			for (const FLaneEnd& Out : Ends)
			{
				if (!Out.bOut) { continue; }
				if (Out.Road == In.Road) { continue; } // no U-turn onto the same arm
				FRoadNetLaneConnection Cn;
				Cn.From.Road = In.Road;  Cn.From.Lane = In.Lane;
				Cn.To.Road   = Out.Road; Cn.To.Lane   = Out.Lane;
				Cn.Joint  = ji;
				Cn.Entry  = In.Pos;
				Cn.Exit   = Out.Pos;
				Cn.bThrough = bSeam;
				Ctx.LaneConnections.Add(Cn);
			}
		}
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

int32 URoadNetwork::CommitLayer(
	FName LayerName,
	const TArray<TArray<UE::Geometry::FGeneralPolygon2d>>& ZonePolys,
	double ExtraLiftCm, FColor Color, UMaterialInterface* Material, FRoadNetRebuildContext& Ctx,
	bool bBakeLaneColors, bool bWorldUVs, bool bConformSurface)
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
		TFunction<FVector3f(double, double)> ShadeFn =
			[this, &Ctx, &ZoneRoads, BaseCol, EvenCol, OddCol](double X, double Y) -> FVector3f
		{
			const FVector2D Q(X, Y);
			int32 BestRoad = INDEX_NONE;
			double BestDist = TNumericLimits<double>::Max();
			double BestOffset = 0.0;
			for (int32 r : ZoneRoads)
			{
				const FRoadCurves* C = Ctx.Curves.Find(r);
				if (!C || C->Sampled.Num() < 2) { continue; }
				const RoadNetMath::FProjectResult PR = RoadNetMath::ProjectToPolyline(C->Sampled, Q);
				if (PR.Distance < BestDist) { BestDist = PR.Distance; BestRoad = r; BestOffset = PR.Offset; }
			}
			if (BestRoad == INDEX_NONE || !Roads.IsValidIndex(BestRoad)) { return BaseCol; }
			const TArray<FRoadNetLane> Lanes = Roads[BestRoad].Lanes.ResolveLanes();
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
				/*bWorldUVs*/bWorldUVs);

			// Terrain-conform soup (world cm; tiles are spawned at origin/identity),
			// cached PER CELL so a windowed rebuild can reassemble the whole-network
			// conform arrays from clean cells + this pass's dirty cells.
			if (bZoneGround)
			{
				FRoadNetTileConform& TC = ConformCache.FindOrAdd(Coord);
				for (int32 tid = TID0; tid < TM.MaxTriangleID(); ++tid)
				{
					if (!TM.IsTriangle(tid)) { continue; }
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

			const TArray<FRoadNetLane> Lanes = Roads[RoadIdx].Lanes.ResolveLanes();
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
	if (Ctx.bFullCommit)
	{
		SegKeyOf.Reset(); JunKeyOf.Reset(); SegAlias.Reset();
		NextSegId = 0; NextJunId = 0;
		LastRoadJunctions.Reset();
	}
	BuildTopoAccel(Ctx);
	// Divided-road pairing must run BEFORE any SegTileKey call this pass (the
	// windowed dirty-tile block below, plus every commit) so paired carriageways
	// resolve to the same tile everywhere.
	BuildDividedPairs(Ctx);
	// § tiling v2: generate each segment tile's surface + sidewalks from its own
	// arm run (ownership by construction) — CommitLayer consumes these buckets.
	BuildTilePartition(Ctx);
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
	const int32 WalkTris = CommitLayer(TEXT("Sidewalks"),
		Ctx.ZoneSidewalkPolys, /*ExtraLift*/15.0, FColor(165, 162, 155), SidewalkMaterial, Ctx,
		/*bBakeLaneColors*/false, /*bWorldUVs*/false, /*bConformSurface*/true);
	const int32 WhiteTris = CommitLayer(TEXT("MarkingsWhite"),
		Ctx.ZoneMarkingWhitePolys, /*ExtraLift*/4.0, FColor(232, 232, 226), MarkingWhiteMaterial, Ctx);
	const int32 YellowTris = CommitLayer(TEXT("MarkingsYellow"),
		Ctx.ZoneMarkingYellowPolys, /*ExtraLift*/4.0, FColor(240, 190, 30), MarkingYellowMaterial, Ctx);

	// Typed-lane overlays: bike paths (green) + parking bays (amber) as thin
	// surfaces lifted a hair above the carriageway and the paint, skinned with
	// their Assets-tab material when set (else the tint fallback). Not part of
	// the terrain conform (they ride the road surface). Empty layers stay empty
	// (cleared above), so removing the last bike/parking lane clears its surface.
	const int32 BikeTris = CommitLayer(TEXT("LanesBike"),
		Ctx.ZoneLaneBikePolys, /*ExtraLift*/6.0, FColor(60, 170, 90), BikeLaneMaterial, Ctx,
		/*bBakeLaneColors*/false, /*bWorldUVs*/false, /*bConformSurface*/false);
	const int32 ParkTris = CommitLayer(TEXT("LanesParking"),
		Ctx.ZoneLaneParkPolys, /*ExtraLift*/6.0, FColor(200, 165, 45), ParkingMaterial, Ctx,
		/*bBakeLaneColors*/false, /*bWorldUVs*/false, /*bConformSurface*/false);

	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet] CommitGeometry: road %d tris (lane shading baked), sidewalk %d tris, markings %d white + %d yellow tris, bike %d + parking %d tris."),
		RoadTris, WalkTris, WhiteTris, YellowTris, BikeTris, ParkTris);

	// Kerb line rides on the same merged surface + sidewalk polys, so it must be
	// committed AFTER the sidewalk band exists (it reads Ctx.ZoneSidewalkPolys).
	CommitCurbs(Ctx);
	CommitFurniture(Ctx);         // § street furniture (HISM instances / spawned Blueprint actors)
	CommitJunctionSignals(Ctx);   // § traffic-signal placeholders at signalized junctions
	CommitMedian(Ctx);            // § raised median strip + centre planting splines
	CommitPerimeters(Ctx);
	CommitLaneGraph(Ctx);
	CommitSegmentSplines(Ctx);    // § per-segment editable centre + edge splines

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
			const FVector First = Ring[0];   // copy out: Add() may reallocate
			Ring.Add(First);                 // close the loop
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
		const bool bA = ((iCurb++ & 1) == 0);
		if (UHierarchicalInstancedStaticMeshComponent* H = TileHISM(Coord, bA))
		{
			CurbBatches.FindOrAdd(H).Add(Inst);
			if (bA) { ++nA; } else { ++nB; }
		}
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
	// Junctions are matched to persistent overrides by proximity (locations
	// wobble slightly between rebuilds as centrelines re-smooth).
	constexpr double kJunctionMatchCm = 600.0;

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
			if (!ResolveJunctionIslandsNear(Centre)) { continue; }

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
						Approaches.Add(Ap);
						SumZ += FMath::Lerp(S[i - 1].Z, S[i].Z, 0.5); ++ZN;
					}
					bPrevIn = bIn;
				}
			}

			const double CentreZ = (ZN > 0) ? (SumZ / (double)ZN) : 0.0;
			const ERoadNetJunctionPreset Preset = ResolveJunctionPresetNear(Centre);

			FRoadNetJunctionView View;
			View.Location = FVector(Centre.X, Centre.Y, CentreZ);
			View.Preset = Preset;
			View.ArmCount = Approaches.Num();
			JunctionViews.Add(View);

			if (Preset == ERoadNetJunctionPreset::None || Approaches.Num() == 0) { continue; }

			TArray<RoadNetJunctionMarks::FSignal> Signals;
			RoadNetJunctionMarks::BuildJoint(
				Centre, CentreZ, Approaches, Preset, Ctx.ZoneMarkingWhitePolys[z], Signals);

			for (const RoadNetJunctionMarks::FSignal& Sg : Signals)
			{
				Ctx.Signals.Emplace(Sg.Location, Sg.YawDeg);
			}

			// Cut the yellow centre line back to BEHIND the crosswalk: a full-
			// width rectangle from the junction boundary out past the crosswalk's
			// far edge (matches the crosswalk band placement in BuildJoint).
			if (PresetHasCrosswalk(Preset))
			{
				constexpr double kCwFarCm = 560.0;  // just beyond the zebra band
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
