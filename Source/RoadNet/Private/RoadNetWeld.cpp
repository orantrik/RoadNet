// RoadNetWeld.cpp — every junction welded into ONE shared point.
//
// RoadNet stores no junction node: each rebuild re-derives joints from road ENDS
// that share a node id or lie within the joint weld radius, and never moves them.
// So a junction whose arms end 0.5-3 m apart (a sloppy source, a side road pushed
// past a through-road, a lane merge that shifted an end) stays a knot of separate
// points, hidden under the junction hull but visible in the Points tool and felt
// as a sloppy connection. WeldJunctionPoints makes it one point:
//
//   Phase A  a road end that sits ON another road (within the radius of its
//            centreline, not at its end) splits that road there, so the through
//            road gets ends at the junction too. An interior node id shared by
//            two roads (a crossing mapped through one node) splits both. A cut
//            within the joint weld radius of an end trims that overshoot instead.
//   Phase B  ends within the radius of each other collapse onto one coordinate
//            and one node id.
//
// Roads are tracked by FRoadDef::Id, never by index across a mutation, and the
// network is Modify()'d only when something actually changes, so a welded
// network is a no-op (no empty undo step, no rebuild churn).
#include "RoadNetwork.h"
#include "RoadNetMath.h"
#include "RoadNetLog.h"
#include "HAL/IConsoleManager.h"

namespace
{
	// = kEndpointWeldCm in RoadNetwork.cpp. A weld radius past it would fold the
	// two junctions of a divided road (or neighbouring roundabout entries) into one.
	constexpr double kWeldMaxRadiusCm = 400.0;

	// Two cut requests closer than this along one road are one junction: the second
	// cut would land inside the first cut's weld radius and SplitRoadAt refuses it.
	constexpr double kMergeCutsCm = kWeldMaxRadiusCm;

	TAutoConsoleVariable<float> CVarJunctionWeldRadiusCm(
		TEXT("roadnet.JunctionWeldRadiusCm"),
		300.f,
		TEXT("Road points within this radius (cm) of a road end are welded into one shared junction point "
			 "(import, draw, point drag, Rebuild RoadNet / Build Street). Clamped to 400. 0 = off."),
		ECVF_Default);

	bool SameGrade(const FRoadDef& A, const FRoadDef& B)
	{
		return A.Layer == B.Layer && A.bBridge == B.bBridge && A.bTunnel == B.bTunnel;
	}

	double EffectiveZ(const FRoadDef& R, int32 i)
	{
		return R.Elev.IsValidIndex(i) ? R.Elev[i] : R.Ref[i].Z;
	}

	// Point + elevation at arc length S along R, interpolated the same way SplitRoadAt does.
	void SampleAtArc(const FRoadDef& R, const TArray<double>& CL, double S, FVector& OutP, double& OutElev)
	{
		int32 Seg = 0;
		while (Seg + 2 < CL.Num() && CL[Seg + 1] <= S) { ++Seg; }
		const double SegLen = FMath::Max(1e-6, CL[Seg + 1] - CL[Seg]);
		const double T = FMath::Clamp((S - CL[Seg]) / SegLen, 0.0, 1.0);
		OutP = FMath::Lerp(R.Ref[Seg], R.Ref[Seg + 1], T);
		OutElev = R.Elev.Num() == R.Ref.Num() ? FMath::Lerp(R.Elev[Seg], R.Elev[Seg + 1], T) : OutP.Z;
	}

	// Cut the overshoot off one end of R so it ends exactly at arc S. Arc-parameterised
	// authored data (edge / sidewalk knots, bays, crossings) is rebased or dropped the
	// way SplitRoadAt treats the side it throws away. Returns false when too little
	// road would survive.
	bool TrimRoadToArc(FRoadDef& R, bool bStart, double S)
	{
		TArray<double> CL;
		RoadNetMath::CumulativeLength(R.Ref, CL);
		const double Len = CL.Last();
		if (S <= 1.0 || S >= Len - 1.0) { return false; }

		FVector P; double PElev = 0.0;
		SampleAtArc(R, CL, S, P, PElev);
		const bool bElev = R.Elev.Num() == R.Ref.Num();
		const bool bIds  = R.NodeIds.Num() == R.Ref.Num();

		TArray<FVector> Ref; TArray<double> Elev; TArray<int64> Ids;
		auto Keep = [&](int32 i) { Ref.Add(R.Ref[i]); if (bElev) { Elev.Add(R.Elev[i]); } if (bIds) { Ids.Add(R.NodeIds[i]); } };
		auto AddCut = [&]() { Ref.Add(P); if (bElev) { Elev.Add(PElev); } if (bIds) { Ids.Add(-1); } };
		if (bStart)
		{
			AddCut();
			for (int32 i = 0; i < R.Ref.Num(); ++i) { if (CL[i] > S + 0.5) { Keep(i); } }
		}
		else
		{
			for (int32 i = 0; i < R.Ref.Num(); ++i) { if (CL[i] < S - 0.5) { Keep(i); } }
			AddCut();
		}
		if (Ref.Num() < 2) { return false; }

		const double Shift = bStart ? S : 0.0;
		const double NewLen = bStart ? Len - S : S;
		auto Rebase = [&](TArray<FRoadNetEdgeKnot>& Knots)
		{
			TArray<FRoadNetEdgeKnot> Out;
			for (FRoadNetEdgeKnot K : Knots)
			{
				K.Distance -= Shift;
				if (K.Distance >= 0.0 && K.Distance <= NewLen) { Out.Add(K); }
			}
			Knots = MoveTemp(Out);
		};
		Rebase(R.OuterEdgeRight);
		Rebase(R.OuterEdgeLeft);
		Rebase(R.Lanes.SidewalkKnotsRight);
		Rebase(R.Lanes.SidewalkKnotsLeft);

		TArray<FRoadNetParkingBay> Bays;
		for (FRoadNetParkingBay Bay : R.ParkingBays)
		{
			const double S0 = Bay.StartArcCm - Shift;
			const double S1 = (Bay.LengthCm > 0.f) ? (S0 + Bay.LengthCm) : NewLen;
			if (S0 >= 0.0 && S1 <= NewLen) { Bay.StartArcCm = (float)S0; Bays.Add(Bay); }
		}
		R.ParkingBays = MoveTemp(Bays);

		TArray<FRoadNetCrossingMark> Xs;
		for (FRoadNetCrossingMark X : R.Crossings)
		{
			X.DistanceCm -= (float)Shift;
			if (X.DistanceCm >= 0.f && X.DistanceCm <= NewLen) { Xs.Add(X); }
		}
		R.Crossings = MoveTemp(Xs);

		R.Ref = MoveTemp(Ref);
		R.Elev = bElev ? MoveTemp(Elev) : TArray<double>();
		R.NodeIds = bIds ? MoveTemp(Ids) : TArray<int64>();
		return true;
	}

	// Closest point of polyline P to Q in plan, as (distance, arc length) along it.
	void ClosestOnPolyline2D(const TArray<FVector>& P, const TArray<double>& CL, const FVector2D& Q,
		double& OutDist, double& OutArc)
	{
		OutDist = TNumericLimits<double>::Max();
		OutArc = 0.0;
		for (int32 k = 0; k + 1 < P.Num(); ++k)
		{
			const FVector2D A(P[k]), B(P[k + 1]);
			const FVector2D AB = B - A;
			const double L2 = AB.SizeSquared();
			const double T = L2 > 1e-6 ? FMath::Clamp(FVector2D::DotProduct(Q - A, AB) / L2, 0.0, 1.0) : 0.0;
			const double D = FVector2D::Distance(Q, A + AB * T);
			if (D < OutDist) { OutDist = D; OutArc = CL[k] + T * (CL[k + 1] - CL[k]); }
		}
	}
}

double URoadNetwork::JunctionWeldRadiusCm()
{
	return FMath::Clamp((double)CVarJunctionWeldRadiusCm.GetValueOnGameThread(), 0.0, kWeldMaxRadiusCm);
}

int32 URoadNetwork::WeldJunctionPoints(double RadiusCm, const TArray<FGuid>* OnlyRoads, double MaxZGapCm)
{
	const double Radius = FMath::Clamp(RadiusCm, 0.0, kWeldMaxRadiusCm);
	if (Radius < 1.0 || Roads.Num() < 2) { return 0; }

	bool bModified = false;
	auto EnsureModified = [this, &bModified]() { if (!bModified) { Modify(); bModified = true; } };

	TSet<FGuid> Seeds;
	if (OnlyRoads) { Seeds.Append(*OnlyRoads); }
	auto IsSeed = [&](const FRoadDef& R) { return !OnlyRoads || Seeds.Contains(R.Id); };

	auto IndexOf = [this](const FGuid& Id) -> int32
	{
		return Roads.IndexOfByPredicate([&Id](const FRoadDef& R) { return R.Id == Id; });
	};

	// ---- Phase A: junction points on a road's interior become ends -----------
	TMap<FGuid, TArray<double>> CutArcs;
	{
		TArray<TArray<double>> CLs;
		TArray<FBox2D> Boxes;
		CLs.SetNum(Roads.Num());
		Boxes.Init(FBox2D(ForceInit), Roads.Num());
		for (int32 r = 0; r < Roads.Num(); ++r)
		{
			if (!Roads[r].IsValid()) { continue; }
			RoadNetMath::CumulativeLength(Roads[r].Ref, CLs[r]);
			FBox2D B(ForceInit);
			for (const FVector& P : Roads[r].Ref) { B += FVector2D(P); }
			Boxes[r] = B.ExpandBy(Radius);
		}

		// An end lying on another road's centreline, away from that road's ends.
		for (int32 a = 0; a < Roads.Num(); ++a)
		{
			const FRoadDef& A = Roads[a];
			if (!A.IsValid()) { continue; }
			for (int32 End = 0; End < 2; ++End)
			{
				const int32 Ei = End == 0 ? 0 : A.Ref.Num() - 1;
				const FVector2D Q(A.Ref[Ei]);
				for (int32 b = 0; b < Roads.Num(); ++b)
				{
					const FRoadDef& B = Roads[b];
					if (b == a || !B.IsValid() || !Boxes[b].IsInside(Q)) { continue; }
					if (!IsSeed(A) && !IsSeed(B)) { continue; }
					if (!SameGrade(A, B)) { continue; }
					// Two ring segments meet end to end; a ring is never cut for another ring.
					if (A.bRoundabout && B.bRoundabout) { continue; }

					double Dist, Arc;
					ClosestOnPolyline2D(B.Ref, CLs[b], Q, Dist, Arc);
					if (Dist > Radius) { continue; }
					const double Len = CLs[b].Last();
					// At B's own end this is an end-to-end weld, Phase B's job.
					if (Arc <= 1.0 || Arc >= Len - 1.0) { continue; }

					FVector OnB; double ElevB;
					SampleAtArc(B, CLs[b], Arc, OnB, ElevB);
					if (FMath::Abs(ElevB - EffectiveZ(A, Ei)) > MaxZGapCm) { continue; }
					CutArcs.FindOrAdd(B.Id).Add(Arc);
				}
			}
		}

		// A crossing mapped through one shared interior node.
		TMap<int64, TArray<FIntPoint>> ByNode;
		for (int32 r = 0; r < Roads.Num(); ++r)
		{
			const FRoadDef& R = Roads[r];
			if (!R.IsValid() || R.NodeIds.Num() != R.Ref.Num()) { continue; }
			for (int32 i = 0; i < R.NodeIds.Num(); ++i)
			{
				if (R.NodeIds[i] >= 0) { ByNode.FindOrAdd(R.NodeIds[i]).Add(FIntPoint(r, i)); }
			}
		}
		for (const TPair<int64, TArray<FIntPoint>>& KV : ByNode)
		{
			const TArray<FIntPoint>& Occ = KV.Value;
			bool bMultiRoad = false, bAnySeed = false;
			for (const FIntPoint& O : Occ)
			{
				bMultiRoad = bMultiRoad || O.X != Occ[0].X;
				bAnySeed = bAnySeed || IsSeed(Roads[O.X]);
			}
			if (!bMultiRoad || !bAnySeed) { continue; }
			for (const FIntPoint& O : Occ)
			{
				const FRoadDef& R = Roads[O.X];
				if (O.Y > 0 && O.Y < R.Ref.Num() - 1) { CutArcs.FindOrAdd(R.Id).Add(CLs[O.X][O.Y]); }
			}
		}
	}

	int32 Splits = 0, Trims = 0;
	for (TPair<FGuid, TArray<double>>& KV : CutArcs)
	{
		int32 Idx = IndexOf(KV.Key);
		if (Idx == INDEX_NONE) { continue; }
		const bool bSeed = IsSeed(Roads[Idx]);

		TArray<double> Arcs = KV.Value;
		Arcs.Sort();
		TArray<double> Merged;
		for (double S : Arcs)
		{
			if (Merged.Num() > 0 && S - Merged.Last() < kMergeCutsCm) { continue; }
			Merged.Add(S);
		}

		const double Len = RoadNetMath::TotalLength(Roads[Idx].Ref);
		double StartTrim = -1.0, EndTrim = -1.0;
		TArray<double> Middle;
		for (double S : Merged)
		{
			if (S <= kWeldMaxRadiusCm) { StartTrim = S; }
			else if (S >= Len - kWeldMaxRadiusCm) { EndTrim = S; }
			else { Middle.Add(S); }
		}
		if (StartTrim >= 0.0 && EndTrim >= 0.0 && EndTrim - StartTrim < kMergeCutsCm) { StartTrim = EndTrim = -1.0; }

		// Far end first (no arc shifts), then cuts from the far end inward (the head keeps
		// its slot and every remaining smaller arc), then the near end (shifts arcs, last).
		if (EndTrim >= 0.0)
		{
			EnsureModified();
			if (TrimRoadToArc(Roads[Idx], /*bStart*/false, EndTrim)) { ++Trims; }
		}
		for (int32 m = Middle.Num() - 1; m >= 0; --m)
		{
			EnsureModified();
			const int32 Tail = SplitRoadAt(Idx, Middle[m]);
			if (Tail == INDEX_NONE) { continue; }
			++Splits;
			// Cut exactly on a vertex, the head ends on that vertex twice (as MergeRoadPoints handles).
			FRoadDef& Head = Roads[Idx];
			const int32 N = Head.Ref.Num();
			if (N >= 3 && Head.Ref[N - 1].Equals(Head.Ref[N - 2], 0.01))
			{
				Head.Ref.RemoveAt(N - 2);
				if (Head.Elev.Num() == N) { Head.Elev.RemoveAt(N - 2); }
			}
			if (bSeed) { Seeds.Add(Roads[Tail].Id); }
		}
		if (StartTrim >= 0.0)
		{
			EnsureModified();
			if (TrimRoadToArc(Roads[Idx], /*bStart*/true, StartTrim)) { ++Trims; }
		}
	}

	// ---- Phase B: ends within the radius become one point, one node id -------
	struct FEnd { int32 Road; bool bStart; FVector2D XY; double Z; };
	TArray<FEnd> Ends;
	for (int32 r = 0; r < Roads.Num(); ++r)
	{
		const FRoadDef& R = Roads[r];
		if (!R.IsValid()) { continue; }
		Ends.Add({ r, true, FVector2D(R.Ref[0]), EffectiveZ(R, 0) });
		Ends.Add({ r, false, FVector2D(R.Ref.Last()), EffectiveZ(R, R.Ref.Num() - 1) });
	}
	// By side, not by stored index: welding one end can drop a point and shift the far one.
	auto PtOf = [this, &Ends](int32 e) { return Ends[e].bStart ? 0 : Roads[Ends[e].Road].Ref.Num() - 1; };

	TArray<int32> Parent;
	Parent.SetNum(Ends.Num());
	for (int32 i = 0; i < Parent.Num(); ++i) { Parent[i] = i; }
	auto Find = [&Parent](int32 x) { while (Parent[x] != x) { Parent[x] = Parent[Parent[x]]; x = Parent[x]; } return x; };

	TMap<FIntPoint, TArray<int32>> Grid;
	auto CellOf = [Radius](const FVector2D& P) { return FIntPoint(FMath::FloorToInt(P.X / Radius), FMath::FloorToInt(P.Y / Radius)); };
	for (int32 e = 0; e < Ends.Num(); ++e) { Grid.FindOrAdd(CellOf(Ends[e].XY)).Add(e); }
	for (int32 e = 0; e < Ends.Num(); ++e)
	{
		const FIntPoint C = CellOf(Ends[e].XY);
		for (int32 dy = -1; dy <= 1; ++dy)
		for (int32 dx = -1; dx <= 1; ++dx)
		{
			const TArray<int32>* Bucket = Grid.Find(FIntPoint(C.X + dx, C.Y + dy));
			if (!Bucket) { continue; }
			for (int32 f : *Bucket)
			{
				// A road's own two ends never weld to each other here: that would fold a short
				// road to nothing. A loop already shares its point and needs no weld.
				if (f <= e || Ends[f].Road == Ends[e].Road) { continue; }
				if (FVector2D::Distance(Ends[e].XY, Ends[f].XY) > Radius) { continue; }
				if (!SameGrade(Roads[Ends[e].Road], Roads[Ends[f].Road])) { continue; }
				if (FMath::Abs(Ends[e].Z - Ends[f].Z) > MaxZGapCm) { continue; }
				const int32 Ra = Find(e), Rb = Find(f);
				if (Ra != Rb) { Parent[Ra] = Rb; }
			}
		}
	}

	TMap<int32, TArray<int32>> Clusters;
	for (int32 e = 0; e < Ends.Num(); ++e) { Clusters.FindOrAdd(Find(e)).Add(e); }

	int64 NextId = 0;
	for (const FRoadDef& R : Roads)
	{
		for (int64 Id : R.NodeIds) { NextId = FMath::Max(NextId, Id); }
	}
	++NextId;

	int32 Welded = 0, Moved = 0, Skipped = 0;
	TSet<int32> MovedRoads;
	for (const TPair<int32, TArray<int32>>& KV : Clusters)
	{
		// A road with both ends in one cluster (a stub shorter than the radius, reached
		// through other roads) would collapse to a point; leave its ends where they are.
		TArray<int32> M;
		{
			TMap<int32, int32> PerRoad;
			for (int32 e : KV.Value) { ++PerRoad.FindOrAdd(Ends[e].Road); }
			for (int32 e : KV.Value) { if (PerRoad[Ends[e].Road] == 1) { M.Add(e); } }
		}
		if (M.Num() < 2) { continue; }

		bool bAnySeed = false;
		for (int32 e : M) { bAnySeed = bAnySeed || IsSeed(Roads[Ends[e].Road]); }
		if (!bAnySeed) { continue; }

		// A chain of ends each within the radius of the next can span a whole street.
		double Span = 0.0;
		for (int32 i = 0; i < M.Num(); ++i)
			for (int32 j = i + 1; j < M.Num(); ++j)
				Span = FMath::Max(Span, FVector2D::Distance(Ends[M[i]].XY, Ends[M[j]].XY));
		if (Span > 2.0 * Radius)
		{
			++Skipped;
			UE_LOG(LogRoadNet, Warning,
				TEXT("[RoadNet] WeldJunctionPoints: %d road ends near (%.0f, %.0f) chain over %.0f cm, more than twice the %.0f cm radius; left as they are."),
				M.Num(), Ends[M[0]].XY.X, Ends[M[0]].XY.Y, Span, Radius);
			continue;
		}

		// Target: the node an OSM import already shares, else the most major road's end
		// (a through road just split there), else the centroid. "Highest Z is king":
		// a shared junction point is never lowered.
		int64 SharedId = -1;
		{
			TMap<int64, int32> Count;
			for (int32 e : M)
			{
				const FRoadDef& R = Roads[Ends[e].Road];
				if (R.NodeIds.IsValidIndex(PtOf(e)) && R.NodeIds[PtOf(e)] >= 0) { ++Count.FindOrAdd(R.NodeIds[PtOf(e)]); }
			}
			int32 Best = 1;
			for (const TPair<int64, int32>& C : Count) { if (C.Value > Best) { Best = C.Value; SharedId = C.Key; } }
		}
		ERoadNetClass BestClass = ERoadNetClass::Unknown;
		for (int32 e : M) { BestClass = FMath::Min(BestClass, Roads[Ends[e].Road].Class); }

		FVector2D Sum(0.0, 0.0);
		int32 N = 0;
		double MaxZ = -TNumericLimits<double>::Max();
		for (int32 e : M)
		{
			const FRoadDef& R = Roads[Ends[e].Road];
			MaxZ = FMath::Max(MaxZ, Ends[e].Z);
			const bool bShared = SharedId >= 0 && R.NodeIds.IsValidIndex(PtOf(e)) && R.NodeIds[PtOf(e)] == SharedId;
			const bool bUse = SharedId >= 0 ? bShared : R.Class == BestClass;
			if (bUse) { Sum += Ends[e].XY; ++N; }
		}
		const FVector2D XY = Sum / double(FMath::Max(1, N));
		const FVector Target(XY.X, XY.Y, MaxZ);
		const int64 NodeId = SharedId >= 0 ? SharedId : NextId;

		bool bChanged = false;
		for (int32 e : M)
		{
			const FRoadDef& R = Roads[Ends[e].Road];
			const bool bSameId = R.NodeIds.IsValidIndex(PtOf(e)) && R.NodeIds[PtOf(e)] == NodeId;
			if (!bSameId || !FVector(R.Ref[PtOf(e)]).Equals(Target, 0.5)) { bChanged = true; break; }
		}
		if (!bChanged) { continue; }

		EnsureModified();
		if (SharedId < 0) { ++NextId; }
		for (int32 e : M)
		{
			FRoadDef& R = Roads[Ends[e].Road];
			if (!FVector(R.Ref[PtOf(e)]).Equals(Target, 0.5)) { ++Moved; MovedRoads.Add(Ends[e].Road); }
			MoveRoadPoint(Ends[e].Road, PtOf(e), Target);
			if (R.NodeIds.Num() != R.Ref.Num()) { R.NodeIds.Init(-1, R.Ref.Num()); }
			R.NodeIds[PtOf(e)] = NodeId;
			// An end pulled onto its own neighbour (a road that ran across the junction and
			// had a point on it) would leave a zero-length segment: drop the neighbour. The
			// end's index stays put at the start; at the far end it moves down by one.
			if (R.Ref.Num() > 2)
			{
				const int32 Nb = Ends[e].bStart ? 1 : R.Ref.Num() - 2;
				if (FVector::DistXY(R.Ref[Nb], Target) < 1.0)
				{
					R.Ref.RemoveAt(Nb);
					if (R.Elev.IsValidIndex(Nb)) { R.Elev.RemoveAt(Nb); }
					if (R.NodeIds.IsValidIndex(Nb)) { R.NodeIds.RemoveAt(Nb); }
				}
			}
			// Same as MergeRoadPoints' Reattach: links named the old ends.
			(Ends[e].bStart ? R.StartLinks : R.EndLinks).Reset();
		}
		++Welded;
	}

	// No StraightenJunctionApproaches here: it reshapes EVERY road's approaches, which a weld
	// limited to one freshly drawn road must not do. The OSM bridge straightens after its weld.

	if (bModified)
	{
		UE_LOG(LogRoadNet, Log,
			TEXT("[RoadNet] WeldJunctionPoints (radius %.0f cm): welded %d junction(s), moved %d end(s) on %d road(s), split %d road(s), trimmed %d overshoot(s)%s."),
			Radius, Welded, Moved, MovedRoads.Num(), Splits, Trims,
			Skipped > 0 ? *FString::Printf(TEXT(", %d chained cluster(s) skipped"), Skipped) : TEXT(""));
	}
	return Moved;
}
