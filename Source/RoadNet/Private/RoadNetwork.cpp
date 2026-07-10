// RoadNetwork.cpp — orchestration + early rebuild stages (§10.18).
#include "RoadNetwork.h"
#include "RoadNetMath.h"
#include "RoadNetSurface.h"
#include "RoadNetMesh.h"
#include "RoadNetCurbs.h"
#include "RoadNetJunctionMarks.h"
#include "RoadNetZones.h"
#include "Polygon2.h"
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

// Pipeline tunables (§2.6). Kept local until a settings object is added.
namespace
{
	// §2.6 PolylineDensity is now a per-network property (URoadNetwork::PolylineDensityCm).
	constexpr double kAdaptiveTurnRad   = 0.0873;  // ~5° knot-preserve threshold
	constexpr double kRoadZLiftCm       = 12.0;    // lift above landscape (anti z-fight)
	constexpr double kMaxZGapCm         = 350.0;   // §10.12 at-grade crossing threshold
	constexpr double kEndpointWeldCm    = 400.0;   // §10.7 spatial endpoint weld radius
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

	R.Ref.RemoveAt(PointIdx);
	if (R.Elev.IsValidIndex(PointIdx)) { R.Elev.RemoveAt(PointIdx); }
	if (R.NodeIds.IsValidIndex(PointIdx)) { R.NodeIds.RemoveAt(PointIdx); }

	if (R.Ref.Num() < 2)
	{
		Roads.RemoveAt(RoadIdx);
		bOutRoadRemoved = true;
	}
	return true;
}

bool URoadNetwork::RemoveRoad(int32 RoadIdx)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	Roads.RemoveAt(RoadIdx);
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

bool URoadNetwork::AddLane(int32 RoadIdx, ERoadNetSide Side)
{
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	FRoadNetLaneSpec& L = Roads[RoadIdx].Lanes;
	// Authored per-lane roads aren't handled by this coarse count-model editor.
	if (L.HasDetailedLanes()) { return false; }

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
	if (L.HasDetailedLanes()) { return false; }

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

double URoadNetwork::AdjustJunctionSmoothing(double DeltaCm)
{
	JunctionSmoothingCm = FMath::Clamp(JunctionSmoothingCm + DeltaCm, 0.0, 300.0);
	return JunctionSmoothingCm;
}

void URoadNetwork::Rebuild(TArrayView<const int32> Modified)
{
	const double T0 = FPlatformTime::Seconds();

	FRoadNetRebuildContext Ctx;
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
	BuildCrossings(Ctx);          // §10.12 grid broadphase (shared by zones+surface)
	const double tCross = Now();   Trace(TEXT("crossings"), tCross - tCurves);
	BuildEndpointJoints(Ctx);     // §10.7 topology from shared node ids
	const double tJoints = Now();  Trace(TEXT("joints"), tJoints - tCross);
	BuildZones(Ctx);              // §10.12 grade-separation layering
	const double tZones = Now();   Trace(TEXT("zones"), tZones - tJoints);
	BuildSurfaceUnion(Ctx);       // §10.9 Clipper2 boolean-union per zone
	const double tSurface = Now(); Trace(TEXT("surface"), tSurface - tZones);
	BuildJunctionMarkings(Ctx);   // §2 junction paint (stop/crosswalk) + signals
	const double tJunc = Now();    Trace(TEXT("junctionmarks"), tJunc - tSurface);
	BuildPerimeterLoops(Ctx);     // §10.11 perimeter loops (PCG export)
	const double tPerim = Now();   Trace(TEXT("perimeters"), tPerim - tJunc);
	BuildLaneGraph(Ctx);          // §12.2 lane connectivity across joints
	const double tGraph = Now();   Trace(TEXT("lanegraph"), tGraph - tPerim);
	BuildLaneRibbons(Ctx);        // §12.1 per-lane ribbon polys
	const double tRibbon = Now();  Trace(TEXT("laneribbon"), tRibbon - tGraph);
	CommitGeometry(Ctx);          // §10.15 triangulate + spawn surface actor
	const double tCommit = Now();  Trace(TEXT("commit"), tCommit - tRibbon);

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
	// Phase-0 scoping: pending = modified; test-against = all. Endpoint-link and
	// octree broadphase expansion (§10.17) come with the incremental phase.
	Ctx.Pending = Ctx.Modified;
	Ctx.TestAgainst.SetNumUninitialized(Roads.Num());
	for (int32 i = 0; i < Roads.Num(); ++i) { Ctx.TestAgainst[i] = i; }
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

		const double Half = FMath::Max(50.0, (double)R.Lanes.HalfWidthCm());
		RoadNetMath::OffsetPolyline(C.Sampled, +Half, C.LeftEdge);
		RoadNetMath::OffsetPolyline(C.Sampled, -Half, C.RightEdge);
		C.Length = RoadNetMath::TotalLength(C.Sampled);

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
		RoadNetSurface::BuildMergedSurface(Ptrs, Ctx.ZoneSurfacePolys[z], CloseCm, &Discs);
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

int32 URoadNetwork::CommitLayer(
	TWeakObjectPtr<AActor>& ActorPtr, const TCHAR* Label,
	const TArray<TArray<UE::Geometry::FGeneralPolygon2d>>& ZonePolys,
	double ExtraLiftCm, FColor Color, UMaterialInterface* Material, FRoadNetRebuildContext& Ctx,
	bool bBakeLaneColors)
{
	UWorld* World = WorldPtr.Get();
	if (!World) { return 0; }

	// Baked per-lane shading colours (linear). Alternating asphalt banks plus a
	// base for junction fill / off-lane areas — same palette the old lifted
	// ribbon overlay used, now folded into the ONE carriageway mesh so nothing
	// z-intersects the road.
	const FVector3f BaseCol(FLinearColor(FColor(38, 38, 42)).R, FLinearColor(FColor(38, 38, 42)).G, FLinearColor(FColor(38, 38, 42)).B);
	const FVector3f EvenCol(FLinearColor(FColor(58, 58, 64)).R, FLinearColor(FColor(58, 58, 64)).G, FLinearColor(FColor(58, 58, 64)).B);
	const FVector3f OddCol (FLinearColor(FColor(44, 44, 49)).R, FLinearColor(FColor(44, 44, 49)).G, FLinearColor(FColor(44, 44, 49)).B);
	const bool bBake = bBakeLaneColors && bShowLaneRibbons;

	// Mesh each grade zone with ONLY its own centerline heights, so overpasses
	// keep their elevation instead of snapping to whatever is below them.
	UE::Geometry::FDynamicMesh3 Mesh;
	int32 Tris = 0;
	for (int32 z = 0; z < ZonePolys.Num(); ++z)
	{
		if (ZonePolys[z].Num() == 0) { continue; }

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

		Tris += RoadNetMesh::AppendSurfaceMesh(
			ZonePolys[z], CenterLines, kRoadZLiftCm + ExtraLiftCm, Mesh,
			bBake ? &ShadeFn : nullptr,
			/*bComputeUVs*/true, /*UVUnitCm*/100.0, /*bGradientNormals*/true);
	}

	if (Tris == 0) { return 0; }
	// Normals are set from the height-field gradient inside AppendSurfaceMesh
	// (smooth, grade-following) — recomputing here would overwrite them with the
	// jittery per-triangle average that caused the facet blotches.

	bool bNewActor = false;
	ADynamicMeshActor* Actor = Cast<ADynamicMeshActor>(ActorPtr.Get());
	if (!Actor)
	{
		FActorSpawnParameters Params;
		Params.ObjectFlags |= RF_Transient;
		Actor = World->SpawnActor<ADynamicMeshActor>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
		if (!Actor) { return 0; }
#if WITH_EDITOR
		Actor->SetActorLabel(Label);
#endif
		ActorPtr = Actor;
		bNewActor = true;
	}

	if (UDynamicMeshComponent* Comp = Actor->GetDynamicMeshComponent())
	{
		Comp->SetMesh(MoveTemp(Mesh));

		// Keep the assigned material in sync every rebuild (cheap, idempotent).
		if (Material) { Comp->SetMaterial(0, Material); }

		// Choose the display mode ONCE, when the actor is first created. On later
		// rebuilds (e.g. after adding a lane) we deliberately leave it alone so a
		// manual Color Override chosen in the details panel isn't reverted.
		if (bNewActor)
		{
			if (Material)
			{
				// Real material assigned: show it directly (no colour tint).
				Comp->SetColorOverrideMode(EDynamicMeshComponentColorOverrideMode::None);
			}
			else if (bBake)
			{
				// Baked per-lane shading lives in the mesh vertex colours.
				Comp->SetColorOverrideMode(EDynamicMeshComponentColorOverrideMode::VertexColors);
			}
			else
			{
				// No material: flat colour so the layer is visible.
				Comp->SetColorOverrideMode(EDynamicMeshComponentColorOverrideMode::Constant);
				Comp->SetConstantOverrideColor(Color);
			}
		}
		Comp->NotifyMeshUpdated();
	}
	return Tris;
}

void URoadNetwork::BuildLaneRibbons(FRoadNetRebuildContext& Ctx) const
{
	const int32 NumZones = Ctx.Zones.Num();
	Ctx.ZoneLaneEvenPolys.Reset(); Ctx.ZoneLaneEvenPolys.SetNum(NumZones);
	Ctx.ZoneLaneOddPolys.Reset();  Ctx.ZoneLaneOddPolys.SetNum(NumZones);
	if (!bShowLaneRibbons || NumZones == 0) { return; }

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
				const double HalfInner = 0.5 * (double)Ln.Width - kLaneGapCm;
				if (HalfInner < 5.0) { continue; } // too thin after the seam inset

				// Thicken the lane centreline through Clipper (round joins, butt
				// ends) instead of looping two raw miter offsets — this dissolves
				// the inner-edge folds that produced spiky/dark triangles on bends.
				TArray<FVector> CL;
				RoadNetLanes::BuildLaneCenterline(C->Sampled, Ln, CL);

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

	// Road carriageway (dark asphalt) and sidewalk band (light concrete, raised
	// one curb height above the road so the kerb reads correctly). Lane shading
	// is BAKED into the carriageway's vertex colours (bBakeLaneColors=true) so
	// lanes no longer need a separate lifted overlay that dove in/out of the road.
	const int32 RoadTris = CommitLayer(GeoActor, TEXT("RoadNet_Surface"),
		Ctx.ZoneSurfacePolys, /*ExtraLift*/0.0, FColor(38, 38, 42), RoadMaterial, Ctx,
		/*bBakeLaneColors*/true);
	const int32 WalkTris = CommitLayer(GeoSidewalkActor, TEXT("RoadNet_Sidewalks"),
		Ctx.ZoneSidewalkPolys, /*ExtraLift*/15.0, FColor(165, 162, 155), SidewalkMaterial, Ctx);
	const int32 WhiteTris = CommitLayer(GeoMarkingWhiteActor, TEXT("RoadNet_Markings_White"),
		Ctx.ZoneMarkingWhitePolys, /*ExtraLift*/4.0, FColor(232, 232, 226), MarkingWhiteMaterial, Ctx);
	const int32 YellowTris = CommitLayer(GeoMarkingYellowActor, TEXT("RoadNet_Markings_Yellow"),
		Ctx.ZoneMarkingYellowPolys, /*ExtraLift*/4.0, FColor(240, 190, 30), MarkingYellowMaterial, Ctx);

	// Retire the old separate lane-ribbon actors (their lifted meshes were the
	// source of the "dive in/out of the road" artifact — now baked into surface).
	auto RetireActor = [](TWeakObjectPtr<AActor>& Ptr)
	{
		if (AActor* A = Ptr.Get()) { A->Destroy(); }
		Ptr = nullptr;
	};
	RetireActor(GeoLaneEvenActor);
	RetireActor(GeoLaneOddActor);

	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet] CommitGeometry: road %d tris (lane shading baked), sidewalk %d tris, markings %d white + %d yellow tris."),
		RoadTris, WalkTris, WhiteTris, YellowTris);

	// Kerb line rides on the same merged surface + sidewalk polys, so it must be
	// committed AFTER the sidewalk band exists (it reads Ctx.ZoneSidewalkPolys).
	CommitCurbs(Ctx);
	CommitJunctionSignals(Ctx);   // § traffic-signal placeholders at signalized junctions
	CommitPerimeters(Ctx);
	CommitLaneGraph(Ctx);
}

void URoadNetwork::CommitCurbs(FRoadNetRebuildContext& Ctx)
{
	UWorld* World = WorldPtr.Get();
	if (!World) { return; }

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

	auto Retire = [this]()
	{
		if (AActor* A = GeoCurbActor.Get()) { A->Destroy(); }
		GeoCurbActor = nullptr;
	};

	if (!bBuildCurbs || !Mesh)
	{
		Retire();
		return;
	}

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

	if (Insts.Num() == 0) { Retire(); return; }

	// Spawn/reuse the kerb container actor (kept across rebuilds).
	AActor* Actor = GeoCurbActor.Get();
	if (!Actor)
	{
		FActorSpawnParameters Params;
		Params.ObjectFlags |= RF_Transient;
		Actor = World->SpawnActor<AActor>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
		if (!Actor) { return; }
		USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("Root"));
		Actor->SetRootComponent(Root);
		Root->RegisterComponent();
#if WITH_EDITOR
		Actor->SetActorLabel(TEXT("RoadNet_Curbs"));
#endif
		GeoCurbActor = Actor;
	}

	// Two alternating kerb HISMs → a zebra pattern (piece i → A / B). Each HISM
	// carries one of the two override materials, so alternate pieces read in
	// contrasting colours. Found/created by tag so material↔HISM stays stable
	// across rebuilds (GetComponents order is not guaranteed).
	auto GetOrMakeHISM = [&](FName Tag) -> UHierarchicalInstancedStaticMeshComponent*
	{
		TArray<UHierarchicalInstancedStaticMeshComponent*> Existing;
		Actor->GetComponents<UHierarchicalInstancedStaticMeshComponent>(Existing);
		for (UHierarchicalInstancedStaticMeshComponent* C : Existing)
		{
			if (C && C->ComponentHasTag(Tag)) { return C; }
		}
		UHierarchicalInstancedStaticMeshComponent* H = NewObject<UHierarchicalInstancedStaticMeshComponent>(Actor);
		H->SetupAttachment(Actor->GetRootComponent());
		H->SetMobility(EComponentMobility::Static);
		H->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		H->SetCanEverAffectNavigation(false);
		H->ComponentTags.Add(Tag);
		H->RegisterComponent();
		Actor->AddInstanceComponent(H);
		return H;
	};
	UHierarchicalInstancedStaticMeshComponent* HA = GetOrMakeHISM(TEXT("CurbA"));
	UHierarchicalInstancedStaticMeshComponent* HB = GetOrMakeHISM(TEXT("CurbB"));
	if (!HA || !HB) { return; }

	for (UHierarchicalInstancedStaticMeshComponent* H : { HA, HB })
	{
		H->ClearInstances();
		H->SetStaticMesh(Mesh);
	}
	if (CurbMaterial0) { HA->SetMaterial(0, CurbMaterial0); }
	if (CurbMaterial1) { HB->SetMaterial(0, CurbMaterial1); }

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
	for (const RoadNetCurbs::FCurbInstance& CI : Insts)
	{
		const double sLong = (double)CI.LengthCm / MeshLong;
		const double sWide = kCurbTargetWidthCm  / MeshWide;
		const double sTall = kCurbTargetHeightCm / MeshTall;
		const FVector Scale = bLongIsX ? FVector(sLong, sWide, sTall) : FVector(sWide, sLong, sTall);
		const float Yaw = CI.YawDeg + (bLongIsX ? 0.f : 90.f) + (float)kCurbYawOffsetDeg;

		// Seat the mesh's bottom-centre (in local space) onto the target world
		// point: TransformVector applies scale+rotation only (no translation).
		FTransform Inst(FRotator(0.f, Yaw, 0.f), FVector::ZeroVector, Scale);
		const FVector AnchorWorld = Inst.TransformVector(FVector(Ctr.X, Ctr.Y, Box.Min.Z));

		const double PlaceYawRad = FMath::DegreesToRadians((double)CI.YawDeg);
		const FVector LeftN(-FMath::Sin(PlaceYawRad), FMath::Cos(PlaceYawRad), 0.0); // sidewalk side
		const FVector Target = CI.Location + LeftN * kCurbLateralNudgeCm;
		Inst.SetTranslation(Target - AnchorWorld);

		if ((iCurb++ & 1) == 0) { HA->AddInstance(Inst, /*bWorldSpace*/true); ++nA; }
		else                    { HB->AddInstance(Inst, /*bWorldSpace*/true); ++nB; }
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
						RoadNetJunctionMarks::FApproach Ap;
						Ap.StopPos = Boundary;
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
	UWorld* World = WorldPtr.Get();
	if (!World) { return; }

	auto Retire = [this]()
	{
		if (AActor* A = GeoSignalActor.Get()) { A->Destroy(); }
		GeoSignalActor = nullptr;
	};

	if (!bBuildJunctionMarkings || Ctx.Signals.Num() == 0) { Retire(); return; }

	UStaticMesh* Mesh = SignalMesh ? SignalMesh.Get() : nullptr;
	if (!Mesh) { Mesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder")); }
	if (!Mesh) { Retire(); return; }

	AActor* Actor = GeoSignalActor.Get();
	if (!Actor)
	{
		FActorSpawnParameters Params;
		Params.ObjectFlags |= RF_Transient;
		Actor = World->SpawnActor<AActor>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
		if (!Actor) { return; }
		USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("Root"));
		Actor->SetRootComponent(Root);
		Root->RegisterComponent();
#if WITH_EDITOR
		Actor->SetActorLabel(TEXT("RoadNet_Signals"));
#endif
		GeoSignalActor = Actor;
	}

	UHierarchicalInstancedStaticMeshComponent* H = nullptr;
	{
		TArray<UHierarchicalInstancedStaticMeshComponent*> Existing;
		Actor->GetComponents<UHierarchicalInstancedStaticMeshComponent>(Existing);
		H = Existing.Num() ? Existing[0] : nullptr;
		if (!H)
		{
			H = NewObject<UHierarchicalInstancedStaticMeshComponent>(Actor);
			H->SetupAttachment(Actor->GetRootComponent());
			H->SetMobility(EComponentMobility::Static);
			H->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			H->SetCanEverAffectNavigation(false);
			H->RegisterComponent();
			Actor->AddInstanceComponent(H);
		}
	}
	if (!H) { return; }
	H->ClearInstances();
	H->SetStaticMesh(Mesh);

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

	for (const TPair<FVector, float>& SP : Ctx.Signals)
	{
		const FVector Scale(sWide, sWide, sTall);
		FTransform Inst(FRotator(0.f, SP.Value, 0.f), FVector::ZeroVector, Scale);
		const FVector AnchorWorld = Inst.TransformVector(FVector(Ctr.X, Ctr.Y, Box.Min.Z)); // bottom-centre
		Inst.SetTranslation(SP.Key - AnchorWorld);
		H->AddInstance(Inst, /*bWorldSpace*/true);
	}

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] CommitJunctionSignals: %d signal placeholders%s."),
		Ctx.Signals.Num(), bUserMesh ? TEXT("") : TEXT(" [cylinder default]"));
}

void URoadNetwork::CommitPerimeters(FRoadNetRebuildContext& Ctx)
{
	UWorld* World = WorldPtr.Get();
	if (!World) { return; }

	// Spawn/reuse a dedicated actor that hosts the perimeter loops as closed
	// spline components — the seam a PCG graph samples for road edges / blocks.
	AActor* Actor = GeoPerimeterActor.Get();
	if (!Actor)
	{
		FActorSpawnParameters Params;
		Params.ObjectFlags |= RF_Transient;
		Actor = World->SpawnActor<AActor>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
		if (!Actor) { return; }
		USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("Root"));
		Actor->SetRootComponent(Root);
		Root->RegisterComponent();
#if WITH_EDITOR
		Actor->SetActorLabel(TEXT("RoadNet_Perimeters"));
#endif
		GeoPerimeterActor = Actor;
	}

	// Clear the previous rebuild's spline components.
	{
		TArray<USplineComponent*> Existing;
		Actor->GetComponents<USplineComponent>(Existing);
		for (USplineComponent* Sp : Existing) { if (Sp) { Sp->DestroyComponent(); } }
	}

	USceneComponent* Root = Actor->GetRootComponent();
	int32 LoopCount = 0;
	for (const FRoadNetLoop& Loop : Ctx.PerimeterLoops)
	{
		if (Loop.Points.Num() < 3) { continue; }

		USplineComponent* Sp = NewObject<USplineComponent>(Actor);
		if (!Sp) { continue; }
		Sp->SetMobility(EComponentMobility::Movable);
		Sp->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
		Sp->RegisterComponent();
		Sp->ClearSplinePoints(false);
		Sp->SetSplinePoints(Loop.Points, ESplineCoordinateSpace::World, false);
		for (int32 i = 0; i < Sp->GetNumberOfSplinePoints(); ++i)
		{
			Sp->SetSplinePointType(i, ESplinePointType::Linear, false);
		}
		Sp->SetClosedLoop(true, false);
		Sp->UpdateSpline();
		Sp->ComponentTags.Add(FName(TEXT("RoadNetPerimeter")));
		Sp->ComponentTags.Add(Loop.bOuter ? FName(TEXT("RoadNetPerimeterOuter"))
		                                   : FName(TEXT("RoadNetPerimeterHole")));
		++LoopCount;
	}

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] CommitPerimeters: %d spline loops for PCG."), LoopCount);
}

void URoadNetwork::CommitLaneGraph(FRoadNetRebuildContext& Ctx)
{
	UWorld* World = WorldPtr.Get();
	if (!World) { return; }

	// Host the lane-connectivity movements as open spline components (one per
	// connection), curving Entry → joint centre → Exit so a PCG graph / traffic
	// system can sample turn paths. Tagged for discovery.
	AActor* Actor = GeoLaneGraphActor.Get();
	if (!Actor)
	{
		FActorSpawnParameters Params;
		Params.ObjectFlags |= RF_Transient;
		Actor = World->SpawnActor<AActor>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
		if (!Actor) { return; }
		USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("Root"));
		Actor->SetRootComponent(Root);
		Root->RegisterComponent();
#if WITH_EDITOR
		Actor->SetActorLabel(TEXT("RoadNet_LaneGraph"));
#endif
		GeoLaneGraphActor = Actor;
	}

	// Clear the previous rebuild's spline components.
	{
		TArray<USplineComponent*> Existing;
		Actor->GetComponents<USplineComponent>(Existing);
		for (USplineComponent* Sp : Existing) { if (Sp) { Sp->DestroyComponent(); } }
	}

	USceneComponent* Root = Actor->GetRootComponent();
	int32 Made = 0;
	for (const FRoadNetLaneConnection& Cn : Ctx.LaneConnections)
	{
		USplineComponent* Sp = NewObject<USplineComponent>(Actor);
		if (!Sp) { continue; }
		Sp->SetMobility(EComponentMobility::Movable);
		Sp->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
		Sp->RegisterComponent();
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
