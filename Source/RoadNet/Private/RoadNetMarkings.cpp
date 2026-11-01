// RoadNetMarkings.cpp — baked lane-marking ribbons (§8.10, תקו"ה 2020 ch. 8).
#include "RoadNetMarkings.h"
#include "RoadNetwork.h"    // FRoadCurves
#include "RoadNetTypes.h"   // FRoadDef
#include "RoadNetMath.h"
#include "RoadNetJunctionMarks.h"   // EmitZebra / EmitStopBar, shared with junctions
#include "RoadNetStandards.h"       // LOT"M paint specs
#include "Polygon2.h"

using namespace UE::Geometry;

namespace
{
	// Extract the sub-polyline of Path between arc-lengths [S, E] (inclusive),
	// interpolating the cut endpoints.
	void ExtractByArcLength(const TArray<FVector>& Path, double S, double E, TArray<FVector>& Out)
	{
		Out.Reset();
		if (Path.Num() < 2 || E <= S) { return; }
		double Acc = 0.0;
		for (int32 i = 0; i + 1 < Path.Num(); ++i)
		{
			const double SegLen = FVector::Dist2D(Path[i], Path[i + 1]);
			if (SegLen <= KINDA_SMALL_NUMBER) { continue; }
			const double SegStart = Acc, SegEnd = Acc + SegLen;
			// Overlap of [S,E] with [SegStart,SegEnd].
			const double A = FMath::Max(S, SegStart);
			const double B = FMath::Min(E, SegEnd);
			if (B > A)
			{
				const double ta = (A - SegStart) / SegLen;
				const double tb = (B - SegStart) / SegLen;
				const FVector Pa = FMath::Lerp(Path[i], Path[i + 1], ta);
				const FVector Pb = FMath::Lerp(Path[i], Path[i + 1], tb);
				if (Out.Num() == 0) { Out.Add(Pa); }
				Out.Add(Pb);
			}
			Acc = SegEnd;
		}
	}

	// Sub-polyline of Path between REFERENCE arc lengths [S, E]. RefAcc holds the
	// reference line's arc length at each Path vertex, so every line offset from
	// the same reference is cut at the same stations — dashes on parallel lanes
	// sit side by side (801 rule 3ד) and every line ends on the same stop line.
	void ExtractByRef(const TArray<FVector>& Path, const TArray<double>& RefAcc, double S, double E,
		TArray<FVector>& Out)
	{
		Out.Reset();
		const int32 N = FMath::Min(Path.Num(), RefAcc.Num());
		if (N < 2 || E <= S) { return; }
		for (int32 i = 0; i + 1 < N; ++i)
		{
			const double A0 = RefAcc[i], A1 = RefAcc[i + 1];
			if (A1 <= A0) { continue; }
			const double Lo = FMath::Max(S, A0), Hi = FMath::Min(E, A1);
			if (Hi <= Lo) { continue; }
			const FVector Pa = FMath::Lerp(Path[i], Path[i + 1], (Lo - A0) / (A1 - A0));
			const FVector Pb = FMath::Lerp(Path[i], Path[i + 1], (Hi - A0) / (A1 - A0));
			if (Out.Num() == 0) { Out.Add(Pa); }
			Out.Add(Pb);
		}
	}

	// Position and unit forward direction at arc length S along Path. False when
	// the path is degenerate.
	bool FrameAtArc(const TArray<FVector>& Path, double S, FVector& OutPos, FVector& OutFwd)
	{
		if (Path.Num() < 2) { return false; }
		double Acc = 0.0;
		for (int32 i = 0; i + 1 < Path.Num(); ++i)
		{
			const double SegLen = FVector::Dist2D(Path[i], Path[i + 1]);
			if (SegLen <= KINDA_SMALL_NUMBER) { continue; }
			if (S <= Acc + SegLen)
			{
				OutPos = FMath::Lerp(Path[i], Path[i + 1], (S - Acc) / SegLen);
				OutFwd = (Path[i + 1] - Path[i]).GetSafeNormal2D();
				return !OutFwd.IsNearlyZero();
			}
			Acc += SegLen;
		}
		OutPos = Path.Last();
		OutFwd = (Path.Last() - Path[Path.Num() - 2]).GetSafeNormal2D();
		return !OutFwd.IsNearlyZero();
	}

	// Build a closed ribbon polygon centred on PathLine with total width 2*HalfW.
	bool MakeRibbon(const TArray<FVector>& PathLine, double HalfW, FGeneralPolygon2d& Out)
	{
		if (PathLine.Num() < 2) { return false; }
		TArray<FVector> L, R;
		RoadNetMath::OffsetPolyline(PathLine, +HalfW, L);
		RoadNetMath::OffsetPolyline(PathLine, -HalfW, R);
		if (L.Num() != R.Num() || L.Num() < 2) { return false; }

		TArray<FVector2d> Loop;
		Loop.Reserve(L.Num() * 2);
		for (int32 i = 0; i < L.Num(); ++i)      { Loop.Emplace(L[i].X, L[i].Y); }
		for (int32 i = R.Num() - 1; i >= 0; --i) { Loop.Emplace(R[i].X, R[i].Y); }

		FPolygon2d Poly(Loop);
		if (Poly.VertexCount() < 3 || FMath::Abs(Poly.SignedArea()) < 1.0) { return false; }
		if (Poly.IsClockwise()) { Poly.Reverse(); }
		Out.SetOuter(Poly);
		return true;
	}

	// Paint reference stations [S0, S1] of Path: solid when Dash <= 0, else dashes.
	// bFromEnd lays the first dash flush against S1 and works back, which is how a
	// line meets a stop line with paint rather than with a gap.
	void EmitRun(const TArray<FVector>& Path, const TArray<double>& RefAcc, double S0, double S1,
		double Dash, double Gap, double HalfW, bool bFromEnd, TArray<FGeneralPolygon2d>& Out)
	{
		if (S1 - S0 < 1.0 || HalfW <= 0.0) { return; }
		TArray<FVector> Piece;
		if (Dash <= 0.0)
		{
			ExtractByRef(Path, RefAcc, S0, S1, Piece);
			FGeneralPolygon2d Ribbon;
			if (MakeRibbon(Piece, HalfW, Ribbon)) { Out.Add(MoveTemp(Ribbon)); }
			return;
		}
		const double Stride = Dash + FMath::Max(0.0, Gap);
		if (bFromEnd)
		{
			for (double E = S1; E > S0; E -= Stride)
			{
				ExtractByRef(Path, RefAcc, FMath::Max(S0, E - Dash), E, Piece);
				FGeneralPolygon2d Ribbon;
				if (MakeRibbon(Piece, HalfW, Ribbon)) { Out.Add(MoveTemp(Ribbon)); }
			}
		}
		else
		{
			for (double S = S0; S < S1; S += Stride)
			{
				ExtractByRef(Path, RefAcc, S, FMath::Min(S + Dash, S1), Piece);
				FGeneralPolygon2d Ribbon;
				if (MakeRibbon(Piece, HalfW, Ribbon)) { Out.Add(MoveTemp(Ribbon)); }
			}
		}
	}

	void AddLocalPoly(const TArray<FVector2D>& Local, const FVector2D& Tail, const FVector2D& Dir,
		double Scale, TArray<FGeneralPolygon2d>& Out)
	{
		const FVector2D Rt = RoadNetMath::RightAxis(Dir);
		TArray<FVector2d> Loop;
		Loop.Reserve(Local.Num());
		for (const FVector2D& P : Local)
		{
			const FVector2D W = Tail + Dir * (P.X * Scale) + Rt * (P.Y * Scale);
			Loop.Emplace(W.X, W.Y);
		}
		FPolygon2d Poly(Loop);
		if (Poly.VertexCount() < 3 || FMath::Abs(Poly.SignedArea()) < 1.0) { return; }
		if (Poly.IsClockwise()) { Poly.Reverse(); }
		FGeneralPolygon2d G;
		G.SetOuter(Poly);
		Out.Add(MoveTemp(G));
	}

	// Fig. 6 straight arrow in the local frame (X along the arrow, Y to its right):
	// stem 25 wide, head 110 wide and 150 long, the head starting at StemLen.
	void ThroughGlyph(double StemLen, TArray<FVector2D>& Out)
	{
		Out = { { 0.0, -12.5 }, { StemLen, -12.5 }, { StemLen, -55.0 }, { StemLen + 150.0, 0.0 },
			{ StemLen, 55.0 }, { StemLen, 12.5 }, { 0.0, 12.5 } };
	}

	// Fig. 6 turning arrow: a 37.5 band that runs straight for Straight, then bends
	// SweepDeg toward Hand (+1 right, -1 left) on a centre radius Rc, ending in a
	// triangular head. One simple polygon: left edge out, head, right edge back.
	void TurnGlyph(double Hand, double Straight, double Rc, double SweepDeg, double HeadLen, double HeadHalf,
		TArray<FVector2D>& Out)
	{
		constexpr double kHalfBand = 18.75;
		TArray<FVector2D> C, H;   // centreline points and headings
		C.Add({ 0.0, 0.0 }); H.Add({ 1.0, 0.0 });
		const int32 Steps = FMath::Max(4, FMath::CeilToInt(SweepDeg / 10.0));
		for (int32 k = 0; k <= Steps; ++k)
		{
			const double A = FMath::DegreesToRadians(SweepDeg) * (double)k / (double)Steps;
			C.Add({ Straight + Rc * FMath::Sin(A), Hand * Rc * (1.0 - FMath::Cos(A)) });
			H.Add({ FMath::Cos(A), Hand * FMath::Sin(A) });
		}
		auto Right = [](const FVector2D& D) { return FVector2D(-D.Y, D.X); };
		Out.Reset();
		for (int32 i = 0; i < C.Num(); ++i) { Out.Add(C[i] - Right(H[i]) * kHalfBand); }
		const FVector2D End = C.Last(), Hd = H.Last();
		Out.Add(End - Right(Hd) * HeadHalf);
		Out.Add(End + Hd * HeadLen);
		Out.Add(End + Right(Hd) * HeadHalf);
		for (int32 i = C.Num() - 1; i >= 0; --i) { Out.Add(C[i] + Right(H[i]) * kHalfBand); }
	}
}

namespace RoadNetMarkings
{
	void EmitArrowGlyph(uint8 Mask, const FVector2D& Tail, const FVector2D& DirIn,
		double Scale, bool bUTurnLeft, TArray<FGeneralPolygon2d>& Out)
	{
		FVector2D Dir = DirIn;
		if (!Dir.Normalize() || Mask == 0) { return; }
		const bool bT = (Mask & 1) != 0, bL = (Mask & 2) != 0, bR = (Mask & 4) != 0, bU = (Mask & 8) != 0;
		const bool bTurn = bL || bR || bU;
		TArray<FVector2D> P;
		if (bT)
		{
			// Alone it is the 240 arrow; beside a turn it is fig. 6's 360 combined arrow.
			ThroughGlyph(bTurn ? 210.0 : 90.0, P);
			AddLocalPoly(P, Tail, Dir, Scale, Out);
		}
		if (bL) { TurnGlyph(-1.0, 30.0, 148.75, 60.0, 100.0, 73.75, P); AddLocalPoly(P, Tail, Dir, Scale, Out); }
		if (bR) { TurnGlyph(+1.0, 30.0, 148.75, 60.0, 100.0, 73.75, P); AddLocalPoly(P, Tail, Dir, Scale, Out); }
		if (bU && !(bUTurnLeft ? bL : bR))
		{
			TurnGlyph(bUTurnLeft ? -1.0 : 1.0, 120.0, 60.0, 180.0, 80.0, 50.0, P);
			AddLocalPoly(P, Tail, Dir, Scale, Out);
		}
	}

	void ReplaceApproachLine(const FRoadCurves& C, double OffsetCm, const FMarkingStop& Stop,
		double LengthCm, double Dash, double Gap, double HalfW,
		TArray<FGeneralPolygon2d>& OutCut, TArray<FGeneralPolygon2d>& OutPaint)
	{
		if (C.Sampled.Num() < 2 || LengthCm <= 1.0) { return; }
		TArray<FVector> Dense;
		RoadNetMath::SmoothG2Spline(C.Sampled, 40.0, Dense);
		if (Dense.Num() < 2) { Dense = C.Sampled; }
		TArray<double> Acc;
		RoadNetMath::CumulativeLength(Dense, Acc);
		const double Total = Acc.Last();
		const double S = RoadNetMath::ProjectToPolyline(Dense, Stop.Pos).AlongDist;
		const double SStop = FMath::Clamp(Stop.bAhead ? S - Stop.ExtraCm : S + Stop.ExtraCm, 0.0, Total);
		const double S0 = Stop.bAhead ? FMath::Max(0.0, SStop - LengthCm) : SStop;
		const double S1 = Stop.bAhead ? SStop : FMath::Min(Total, SStop + LengthCm);
		TArray<FVector> Path;
		RoadNetMath::OffsetPolyline(Dense, OffsetCm, Path);
		EmitRun(Path, Acc, S0, S1, 0.0, 0.0, 20.0, false, OutCut);
		EmitRun(Path, Acc, S0, S1, Dash, Gap, HalfW, Stop.bAhead, OutPaint);
	}

	void BuildRoadMarkings(const FRoadDef& Road, const FRoadCurves& C, bool bDriveOnLeft,
		bool bJointAtStart, bool bJointAtEnd,
		TArray<FGeneralPolygon2d>& OutWhite, TArray<FGeneralPolygon2d>& OutYellow,
		const FRoadMarkingPlan* Plan)
	{
		using RoadNetStandards::EMark;
		using RoadNetStandards::FPaintSpec;
		if (C.Sampled.Num() < 2) { return; }

		// Every interior line is derived from where two REAL lanes meet, never
		// from dividing the carriageway into equal shares. That is what makes a
		// road with a 2.2 m parking lane beside 3.5 m driving lanes paint its
		// lines where the lanes actually are, and it is also why a single-lane
		// road gets no centre line: with one lane there is no boundary to paint.
		TArray<FRoadNetLane> Lanes = Road.Lanes.ResolveLanes(bDriveOnLeft);
		Lanes.Sort([](const FRoadNetLane& A, const FRoadNetLane& B)
			{ return A.CenterOffset < B.CenterOffset; });

		// Densify the (already G2-smoothed) centreline so every offset line reads
		// as a smooth curve around corners — matching the finely-sampled kerb
		// instead of faceting at the ~2 m resample spacing.
		TArray<FVector> Dense;
		RoadNetMath::SmoothG2Spline(C.Sampled, 40.0, Dense);
		if (Dense.Num() < 2) { Dense = C.Sampled; }
		TArray<double> Acc;
		RoadNetMath::CumulativeLength(Dense, Acc);
		const double Total = Acc.Num() > 0 ? Acc.Last() : 0.0;
		if (Total < 1.0) { return; }

		const FPaintSpec P801  = RoadNetStandards::Paint(EMark::LaneDivider);
		const FPaintSpec P801J = RoadNetStandards::Paint(EMark::LaneDividerJunction);
		const FPaintSpec P805  = RoadNetStandards::Paint(EMark::WarningDash);
		const FPaintSpec PShort = RoadNetStandards::Paint(EMark::ShortRects);
		const FPaintSpec PLong  = RoadNetStandards::Paint(EMark::LongRects);

		const int32 Kph = Plan ? Plan->DesignKph : 50;
		TArray<double> Tails;
		RoadNetStandards::ArrowTailDistancesCm(Kph, Tails);
		// A junction approach carries 803 beside its 813 arrows (813 3ב), to just past
		// the farthest one; the continuing lanes densify to 100:100 over the same stretch.
		const double Approach = Tails.Last() + 500.0;
		// 803 rule ב: 50 m is the minimum length of an 803 that is not at a junction.
		constexpr double kCrossingSolidCm = 5000.0;
		const double Warn = RoadNetStandards::WarningLineLengthCm(Kph);

		// ---- where the paint stops ------------------------------------------
		// Junction approaches and exits come from the plan; mid-block crossings
		// stop the lines on both sides of their stripes and bars.
		struct FStopS { double S; bool bAhead; bool bJunction; };
		TArray<FStopS> Stops;
		if (Plan)
		{
			for (const FMarkingStop& St : Plan->Stops)
			{
				const double S = RoadNetMath::ProjectToPolyline(Dense, St.Pos).AlongDist;
				Stops.Add({ FMath::Clamp(St.bAhead ? S - St.ExtraCm : S + St.ExtraCm, 0.0, Total), St.bAhead, true });
			}
		}
		for (const FRoadNetCrossingMark& X : Road.Crossings)
		{
			const double Sx = FMath::Clamp((double)X.DistanceCm, 0.0, Total);
			const double Cut = 0.5 * (double)X.DepthCm + (X.bStopBar ? 75.0 : 30.0);
			Stops.Add({ Sx - Cut, true, false });
			Stops.Add({ Sx + Cut, false, false });
		}
		Stops.Sort([](const FStopS& A, const FStopS& B) { return A.S < B.S; });

		// Paintable stretches between stops. End kinds: 0 open road, 1 crossing, 2 junction.
		struct FSpan { double A; double B; int32 EndA; int32 EndB; };
		TArray<FSpan> Spans;
		{
			bool bOpen = true;
			double Start = 0.0;
			int32 StartKind = 0;
			for (const FStopS& St : Stops)
			{
				const int32 Kind = St.bJunction ? 2 : 1;
				if (St.bAhead)
				{
					if (bOpen && St.S - Start > 1.0) { Spans.Add({ Start, St.S, StartKind, Kind }); }
					bOpen = false;
				}
				else
				{
					Start = St.S;
					StartKind = Kind;
					bOpen = true;
				}
			}
			if (bOpen && Total - Start > 1.0) { Spans.Add({ Start, Total, StartKind, 0 }); }
		}

		// ---- junction widenings ---------------------------------------------
		const double RefLen = (Plan && Plan->RoadLengthCm > 1.0) ? Plan->RoadLengthCm : Total;
		const double ToRef = RefLen / Total;
		// The lane edge a widening opens beside, on its own side: the entering lanes'
		// edge for a bay, the leaving lanes' edge for a carried-out lane.
		auto EdgeFor = [&](const FMarkingWidening& W, double& OutEdge) -> bool
		{
			const bool bWantForward = W.bDrop ? W.bAtStart : !W.bAtStart;
			bool bAny = false;
			for (const FRoadNetLane& L : Lanes)
			{
				if (!L.bDrivable() || L.Type == ERoadNetLaneType::CenterTurn) { continue; }
				if (!(bWantForward ? L.bTravelsForward(bDriveOnLeft) : L.bTravelsBackward(bDriveOnLeft))) { continue; }
				const double E = L.CenterOffset + (double)W.Side * 0.5 * (double)L.Width;
				if (!bAny || (double)W.Side * E > (double)W.Side * OutEdge) { OutEdge = E; }
				bAny = true;
			}
			return bAny;
		};
		struct FWide { const FMarkingWidening* W; double Edge; };
		TArray<FWide> Wides;
		if (Plan)
		{
			for (const FMarkingWidening& W : Plan->Widenings)
			{
				double Edge = 0.0;
				if (!EdgeFor(W, Edge)) { continue; }
				Wides.Add({ &W, Edge });
			}
		}
		// Every line follows the road's lateral moves: a closing median at a seam,
		// and the opposing half moved over beside a far-side bay (the bay takes the
		// gap). bSqueezeOnly is for a far bay's own 808, which marks that gap.
		auto MovedAt = [&](double Off, double S, bool bSqueezeOnly)
		{
			return FRoadNetLateralMove::ApplyAll(C.Moves, Off, S * ToRef, RefLen, bSqueezeOnly);
		};
		auto PathAt = [&](double Off, TArray<FVector>& Out, bool bSqueezeOnly = false)
		{
			bool bVar = false;
			TArray<double> PerV;
			PerV.SetNumUninitialized(Dense.Num());
			for (int32 i = 0; i < Dense.Num(); ++i)
			{
				PerV[i] = MovedAt(Off, Acc[i], bSqueezeOnly);
				bVar |= FMath::Abs(PerV[i] - Off) > 0.5;
			}
			if (bVar) { RoadNetMath::OffsetPolylineVariable(Dense, PerV, Out); }
			else      { RoadNetMath::OffsetPolyline(Dense, Off, Out); }
		};
		auto Run = [&](const TArray<FVector>& Path, double S0, double S1, const FPaintSpec& Sp, bool bFromEnd)
		{
			EmitRun(Path, Acc, S0, S1, Sp.DashLenCm, Sp.GapLenCm, 0.5 * Sp.WidthCm, bFromEnd, OutWhite);
		};

		// ---- lane boundaries -------------------------------------------------
		// Walk adjacent lane pairs left to right and paint the boundary between
		// each. Walking pairs rather than collecting edges means the outer
		// carriageway edges are never candidates, so there is nothing to trim.
		// A raised median is kerbed on both sides, so it takes no line (803 rule 2ג).
		for (int32 i = 0; i + 1 < Lanes.Num(); ++i)
		{
			const FRoadNetLane& A = Lanes[i];
			const FRoadNetLane& B = Lanes[i + 1];
			if (A.Type == ERoadNetLaneType::Median || B.Type == ERoadNetLaneType::Median) { continue; }
			if (A.bOutboard() || B.bOutboard()) { continue; }

			const double EdgeA = A.CenterOffset + 0.5 * (double)A.Width;
			const double EdgeB = B.CenterOffset - 0.5 * (double)B.Width;
			if (EdgeB - EdgeA > 1.0) { continue; }
			const double Boundary = 0.5 * (EdgeA + EdgeB);

			const bool bDrivenA = A.bTravelsForward(bDriveOnLeft) || A.bTravelsBackward(bDriveOnLeft);
			const bool bDrivenB = B.bTravelsForward(bDriveOnLeft) || B.bTravelsBackward(bDriveOnLeft);
			const bool bOpposed =
				   A.bTravelsForward(bDriveOnLeft)  != B.bTravelsForward(bDriveOnLeft)
				|| A.bTravelsBackward(bDriveOnLeft) != B.bTravelsBackward(bDriveOnLeft);

			TArray<FVector> Path;
			PathAt(Boundary, Path);

			if (!bOpposed)
			{
				// 801 between same-direction lanes: 100:200 in the section, 100:100 on
				// the stretch approaching the junction those lanes ENTER (table 10).
				const bool bFwd = A.bTravelsForward(bDriveOnLeft);
				for (const FSpan& Sp : Spans)
				{
					const double Zone = FMath::Min(Approach, 0.5 * (Sp.B - Sp.A));
					if (bFwd && Sp.EndB == 2)
					{
						Run(Path, Sp.B - Zone, Sp.B, P801J, true);
						Run(Path, Sp.A, Sp.B - Zone, P801, true);
					}
					else if (!bFwd && Sp.EndA == 2)
					{
						Run(Path, Sp.A, Sp.A + Zone, P801J, false);
						Run(Path, Sp.A + Zone, Sp.B, P801, false);
					}
					else
					{
						Run(Path, Sp.A, Sp.B, P801, /*bFromEnd*/bFwd);
					}
				}
			}
			else if (bDrivenA && bDrivenB)
			{
				// Opposing traffic: 803 on every junction and crossing approach, 805
				// before it, 801 in between; two 803s closer than 150 m are joined
				// (803 rule ג). 803 is never painted between same-direction lanes.
				TArray<FVector> L0, L1;
				PathAt(Boundary - 10.0, L0);
				PathAt(Boundary + 10.0, L1);
				auto Double = [&](double S0, double S1)
				{
					EmitRun(L0, Acc, S0, S1, 0.0, 0.0, 5.0, false, OutWhite);
					EmitRun(L1, Acc, S0, S1, 0.0, 0.0, 5.0, false, OutWhite);
				};
				auto SolidLen = [&](int32 End) { return End == 2 ? Approach : (End == 1 ? kCrossingSolidCm : 0.0); };
				for (const FSpan& Sp : Spans)
				{
					const double L = Sp.B - Sp.A;
					const double EA = SolidLen(Sp.EndA), EB = SolidLen(Sp.EndB);
					if (EA + EB <= 0.0) { Run(Path, Sp.A, Sp.B, P801, false); continue; }
					const double WA = EA > 0.0 ? Warn : 0.0, WB = EB > 0.0 ? Warn : 0.0;
					if (L - EA - EB < RoadNetStandards::DoubleSolidJoinGapCm()) { Double(Sp.A, Sp.B); continue; }
					if (EA > 0.0) { Double(Sp.A, Sp.A + EA); Run(Path, Sp.A + EA, Sp.A + EA + WA, P805, false); }
					if (EB > 0.0) { Double(Sp.B - EB, Sp.B); Run(Path, Sp.B - EB - WB, Sp.B - EB, P805, true); }
					Run(Path, Sp.A + EA + WA, Sp.B - EB - WB, P801, false);
				}
			}
			else
			{
				// One side is not driven at all (a parking lane, a shoulder): a solid
				// white lane edge, not a centre line.
				for (const FSpan& Sp : Spans) { EmitRun(Path, Acc, Sp.A, Sp.B, 0.0, 0.0, 5.0, false, OutWhite); }
			}
		}

		// ---- bays and carried-out lanes --------------------------------------
		// 808 short rectangles separate a turn bay from the through lane (808 rule ד),
		// long rectangles a merge lane from the continuing lane (808 rule ג). Painted
		// where the widening is at least half a lane wide, on the original lane edge.
		for (const FWide& Wd : Wides)
		{
			const FMarkingWidening& W = *Wd.W;
			const double LaneW = W.WidthCm / (double)FMath::Max(1, W.Lanes);
			const double OpenFrac = 1.0 - FMath::Min(1.0, 0.5 * LaneW / FMath::Max(1.0, W.WidthCm));
			const bool bCarried = W.bDrop && W.TaperCm <= 0.0;
			const double SLen = FMath::Min(Total, (W.HoldCm + W.TaperCm * OpenFrac) / ToRef);
			const double S0 = W.bAtStart ? 0.0 : Total - SLen;
			const double S1 = W.bAtStart ? SLen : Total;
			TArray<FVector> Path;
			PathAt(Wd.Edge, Path, /*bSqueezeOnly*/W.bFarSide);
			const FPaintSpec& R = (W.bDrop || W.bSpeedChange) ? PLong : (W.bGain ? P801 : PShort);
			// A lane carried to a far junction turns into its kerb turn lane: short
			// rectangles over the last 50 m before it (808 rule ד, fig. 24).
			const bool bFarJunction = W.bAtStart ? bJointAtEnd : bJointAtStart;
			const double ShortLen = (bCarried && bFarJunction) ? FMath::Min(SLen, RoadNetStandards::RectSectionLengthCm()) : 0.0;
			const double L0 = W.bAtStart ? S0 : S0 + ShortLen;
			const double L1 = W.bAtStart ? S1 - ShortLen : S1;
			for (const FSpan& Sp : Spans)
			{
				Run(Path, FMath::Max(L0, Sp.A), FMath::Min(L1, Sp.B), R, /*bFromEnd*/!W.bAtStart);
				if (ShortLen > 0.0)
				{
					const double A = W.bAtStart ? L1 : S0, B = W.bAtStart ? S1 : L0;
					Run(Path, FMath::Max(A, Sp.A), FMath::Min(B, Sp.B), PShort, /*bFromEnd*/W.bAtStart);
				}
			}

			if (!W.bDrop || !W.bArrows || bCarried) { continue; }
			// 814 (813-814 rule 3ג, fig. 5): three clusters of three diagonal arrows
			// on the dropped lane's full-width run, the first 15 m before its taper
			// starts, 15 m between clusters, 3 m between arrows, pointing across into
			// the continuing lane; 75% size in a lane narrower than 3.20 m (rule 4א).
			const double Sign = W.bAtStart ? 1.0 : -1.0;       // leaving traffic runs away from the joint
			const double Scale = LaneW >= 320.0 ? 1.0 : 0.75;
			const double End = W.HoldCm / ToRef;               // where the taper starts
			for (int32 c = 0; c < 3; ++c)
			{
				for (int32 k = 0; k < 3; ++k)
				{
					const double D = End - (1500.0 + c * (900.0 + 1500.0) + 300.0 * k);
					const double S = W.bAtStart ? D : Total - D;
					bool bPainted = false;
					for (const FSpan& Sp : Spans) { bPainted |= S > Sp.A && S < Sp.B; }
					if (D <= 0.0 || !bPainted) { continue; }
					FVector Pos, Fwd;
					if (!FrameAtArc(Dense, S, Pos, Fwd)) { continue; }
					const FVector2D Tf(Fwd.X, Fwd.Y);
					const FVector2D Rf = RoadNetMath::RightAxis(Tf);
					const FVector2D Travel = Tf * Sign;
					const FVector2D Dir = (Travel - Rf * (double)W.Side).GetSafeNormal();
					for (int32 j = 0; j < W.Lanes; ++j)
					{
						const FVector2D Centre = FVector2D(Pos.X, Pos.Y) + Rf * MovedAt(Wd.Edge + (double)W.Side * (j + 0.5) * LaneW, S, false);
						EmitArrowGlyph(1, Centre - Dir * (120.0 * Scale), Dir, Scale, !bDriveOnLeft, OutWhite);
					}
				}
			}
		}

		// 814 in a lane that ends past this road's end (813-814 rule 3ג, fig. 5):
		// three clusters of three arrows, the first 15 m before the lane stops
		// being full width, 15 m between clusters, 3 m between arrows; 75% size in
		// a lane narrower than 3.20 m (rule 4א).
		if (Plan)
		{
			for (const FMarkingLaneEnd& LE : Plan->LaneEnds)
			{
				const double Scale = LE.WidthCm >= 320.0 ? 1.0 : 0.75;
				for (int32 c = 0; c < 3; ++c)
				{
					for (int32 k = 0; k < 3; ++k)
					{
						const double D = 1500.0 + c * (900.0 + 1500.0) + 300.0 * k;
						const double S = LE.bAtStart ? D : Total - D;
						bool bPainted = false;
						for (const FSpan& Sp : Spans) { bPainted |= S > Sp.A && S < Sp.B; }
						if (!bPainted) { continue; }
						FVector Pos, Fwd;
						if (!FrameAtArc(Dense, S, Pos, Fwd)) { continue; }
						const FVector2D Tf(Fwd.X, Fwd.Y);
						const FVector2D Rf = RoadNetMath::RightAxis(Tf);
						const FVector2D Travel = LE.bAtStart ? -Tf : Tf;
						const FVector2D Dir = (Travel + Rf * (double)LE.Toward).GetSafeNormal();
						const FVector2D Centre = FVector2D(Pos.X, Pos.Y) + Rf * MovedAt(LE.OffsetCm, S, false);
						EmitArrowGlyph(1, Centre - Dir * (120.0 * Scale), Dir, Scale, !bDriveOnLeft, OutWhite);
					}
				}
			}
		}

		// Mid-block pedestrian crossings. Painted from the same emitter the
		// junction presets use, so a crossing outside a school reads exactly like
		// one at a junction mouth.
		if (Road.Crossings.Num() > 0)
		{
			const double Half = FMath::Max(50.0, (double)Road.Lanes.HalfWidthCm());
			for (const FRoadNetCrossingMark& X : Road.Crossings)
			{
				FVector Pos, Fwd;
				if (!FrameAtArc(Dense, FMath::Clamp((double)X.DistanceCm, 0.0, Total), Pos, Fwd)) { continue; }
				const FVector2D P(Pos.X, Pos.Y), T(Fwd.X, Fwd.Y);
				RoadNetJunctionMarks::EmitZebra(P, T, Half, (double)X.DepthCm, OutWhite);

				if (X.bStopBar)
				{
					// A bar belongs to a DIRECTION OF TRAVEL, so it needs lanes going
					// that way. The old code emitted both unconditionally, which put a
					// bar facing no traffic at all on every one-way road.
					const FVector2D Lateral = RoadNetMath::RightAxis(T);   // +right of forward travel
					const double Back = 0.5 * (double)X.DepthCm + 60.0;

					bool bAnyFwd = false, bAnyBwd = false;
					double FwdLo = 0.0, FwdHi = 0.0, BwdLo = 0.0, BwdHi = 0.0;
					for (const FRoadNetLane& L : Lanes)
					{
						if (!L.bDrivable()) { continue; }
						const double E0 = L.CenterOffset - 0.5 * (double)L.Width;
						const double E1 = L.CenterOffset + 0.5 * (double)L.Width;
						if (L.bTravelsForward(bDriveOnLeft))
						{
							FwdLo = bAnyFwd ? FMath::Min(FwdLo, E0) : E0;
							FwdHi = bAnyFwd ? FMath::Max(FwdHi, E1) : E1;
							bAnyFwd = true;
						}
						if (L.bTravelsBackward(bDriveOnLeft))
						{
							BwdLo = bAnyBwd ? FMath::Min(BwdLo, E0) : E0;
							BwdHi = bAnyBwd ? FMath::Max(BwdHi, E1) : E1;
							bAnyBwd = true;
						}
					}

					// At a junction mouth the arm's own approach bar already stops the
					// traffic coming IN. Traffic going OUT of that joint has just been
					// released by it, so a second bar a few metres later is a double
					// stop line. Skip the outgoing direction on the junction end only —
					// the far side of the crossing keeps its bar.
					constexpr double kJunctionMouthCm = 1500.0;   // 15 m
					const bool bNearStart = bJointAtStart && (double)X.DistanceCm < kJunctionMouthCm;
					const bool bNearEnd   = bJointAtEnd   && (Total - (double)X.DistanceCm) < kJunctionMouthCm;
					// Leaving the joint at the START means travelling forward.
					if (bNearStart) { bAnyFwd = false; }
					// Leaving the joint at the END means travelling backward.
					if (bNearEnd)   { bAnyBwd = false; }

					// Travel only sets the bar's thin axis, which is symmetric, so both
					// bars can be measured in the forward lane frame.
					if (bAnyFwd)
					{
						RoadNetJunctionMarks::EmitStopBar(P - T * Back, T, Lateral, FwdLo, FwdHi, OutWhite);
					}
					if (bAnyBwd)
					{
						RoadNetJunctionMarks::EmitStopBar(P + T * Back, T, Lateral, BwdLo, BwdHi, OutWhite);
					}
				}
			}
		}

		// 807 (yellow, no kerb only) is stroked from the merged carriageway boundary
		// in BuildSurfaceUnion so it curves around junction corners.
	}

	void DashPolyline(const TArray<FVector>& Path, double DashLenCm, double GapLenCm,
		double HalfWCm, TArray<FGeneralPolygon2d>& Out)
	{
		// A non-positive stride would loop forever; a non-positive dash would
		// emit nothing but still walk the path. Both are caller bugs, and both
		// are cheaper to refuse here than to debug in a hung editor.
		if (Path.Num() < 2 || DashLenCm <= 0.0 || HalfWCm <= 0.0) { return; }
		const double Stride = DashLenCm + FMath::Max(0.0, GapLenCm);
		if (Stride <= 0.0) { return; }

		const double Total = RoadNetMath::TotalLength(Path);
		for (double S = 0.0; S < Total; S += Stride)
		{
			TArray<FVector> Dash;
			ExtractByArcLength(Path, S, FMath::Min(S + DashLenCm, Total), Dash);
			FGeneralPolygon2d Ribbon;
			if (MakeRibbon(Dash, HalfWCm, Ribbon)) { Out.Add(MoveTemp(Ribbon)); }
		}
	}

	bool DashSelfCheck(FString& OutReport)
	{
		// A 100 m straight run dashed at the junction turn guide's own spec.
		const RoadNetStandards::FPaintSpec Spec =
			RoadNetStandards::Paint(RoadNetStandards::EMark::JunctionTurn);
		constexpr double kTotalCm = 10000.0;
		TArray<FVector> Line = { FVector(0, 0, 0), FVector(kTotalCm, 0, 0) };

		TArray<FGeneralPolygon2d> Out;
		DashPolyline(Line, Spec.DashLenCm, Spec.GapLenCm, 0.5 * Spec.WidthCm, Out);

		// One dash per stride, the last one truncated by the end of the line.
		const double Stride = Spec.DashLenCm + Spec.GapLenCm;
		const int32 Expect = FMath::CeilToInt(kTotalCm / Stride);
		const bool bCount = (Out.Num() == Expect);

		// Painted area is the duty cycle of the strip, give or take the
		// truncated last dash and the ribbon's square ends.
		double Area = 0.0;
		for (const FGeneralPolygon2d& P : Out) { Area += FMath::Abs(P.GetOuter().SignedArea()); }
		const double Ideal = kTotalCm * Spec.WidthCm * (Spec.DashLenCm / Stride);
		const bool bArea = Area > 0.85 * Ideal && Area < 1.15 * Ideal;

		// Degenerate inputs REFUSE rather than hang. A zero or negative stride is
		// the one that matters: `for (S = 0; S < Total; S += Stride)` never
		// terminates, and it would hang the editor mid-rebuild with no log line.
		TArray<FGeneralPolygon2d> Junk;
		DashPolyline(Line, 0.0,   500.0, 6.0, Junk);      // no dash
		DashPolyline(Line, -100.0, 0.0,  6.0, Junk);      // negative dash => stride <= 0
		DashPolyline(Line, 100.0, 150.0, 0.0, Junk);      // no width
		DashPolyline({ FVector::ZeroVector }, 100.0, 150.0, 6.0, Junk);  // no path
		const bool bRefuse = (Junk.Num() == 0);

		// A solid-equivalent request (no gap) covers the whole strip.
		TArray<FGeneralPolygon2d> Solid;
		DashPolyline(Line, 500.0, 0.0, 6.0, Solid);
		double SolidArea = 0.0;
		for (const FGeneralPolygon2d& P : Solid) { SolidArea += FMath::Abs(P.GetOuter().SignedArea()); }
		const bool bSolid = SolidArea > 0.95 * kTotalCm * 12.0;

		// Reference-station cutting: two parallel lines dashed from the same
		// reference put their dashes at the same stations (801 rule 3ד).
		TArray<double> Ref = { 0.0, kTotalCm };
		TArray<FVector> Near = { FVector(0, 0, 0), FVector(kTotalCm, 0, 0) };
		TArray<FVector> Far  = { FVector(0, 350, 0), FVector(kTotalCm, 350, 0) };
		TArray<FGeneralPolygon2d> DN, DF;
		EmitRun(Near, Ref, 0.0, kTotalCm, 100.0, 200.0, 5.0, true, DN);
		EmitRun(Far,  Ref, 0.0, kTotalCm, 100.0, 200.0, 5.0, true, DF);
		bool bAligned = DN.Num() == DF.Num() && DN.Num() > 0;
		for (int32 i = 0; bAligned && i < DN.Num(); ++i)
		{
			const FAxisAlignedBox2d BN = DN[i].GetOuter().Bounds(), BF = DF[i].GetOuter().Bounds();
			bAligned = FMath::IsNearlyEqual(BN.Min.X, BF.Min.X, 1.0) && FMath::IsNearlyEqual(BN.Max.X, BF.Max.X, 1.0);
		}
		// Laid from the end, the run's last dash finishes ON the stop station.
		const bool bFlush = DN.Num() > 0 && FMath::IsNearlyEqual(DN[0].GetOuter().Bounds().Max.X, kTotalCm, 1.0);

		// Arrow glyphs come out as closed, non-degenerate paint.
		TArray<FGeneralPolygon2d> Glyphs;
		EmitArrowGlyph(1 | 4, FVector2D(0, 0), FVector2D(1, 0), 1.0, true, Glyphs);
		EmitArrowGlyph(2, FVector2D(0, 0), FVector2D(0, 1), 0.75, true, Glyphs);
		bool bGlyph = Glyphs.Num() == 3;
		for (const FGeneralPolygon2d& G : Glyphs) { bGlyph &= FMath::Abs(G.GetOuter().SignedArea()) > 1000.0; }

		const bool bOk = bCount && bArea && bRefuse && bSolid && bAligned && bFlush && bGlyph;
		OutReport = FString::Printf(
			TEXT("DashSelfCheck: %s | count %d/%d %s | area %.0f vs %.0f %s | degenerate refused %s | solid %s | aligned %s | flush %s | glyphs %s"),
			bOk ? TEXT("PASS") : TEXT("FAIL"),
			Out.Num(), Expect, bCount   ? TEXT("ok") : TEXT("FAILED"),
			Area, Ideal,       bArea    ? TEXT("ok") : TEXT("FAILED"),
			bRefuse ? TEXT("ok") : TEXT("FAILED"),
			bSolid  ? TEXT("ok") : TEXT("FAILED"),
			bAligned ? TEXT("ok") : TEXT("FAILED"),
			bFlush  ? TEXT("ok") : TEXT("FAILED"),
			bGlyph  ? TEXT("ok") : TEXT("FAILED"));
		return bOk;
	}
}
