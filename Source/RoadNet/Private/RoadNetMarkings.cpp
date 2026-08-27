// RoadNetMarkings.cpp — baked lane-marking ribbons (§8.10).
#include "RoadNetMarkings.h"
#include "RoadNetwork.h"    // FRoadCurves
#include "RoadNetTypes.h"   // FRoadDef
#include "RoadNetMath.h"
#include "RoadNetJunctionMarks.h"   // EmitZebra / EmitStopBar, shared with junctions
#include "Polygon2.h"

using namespace UE::Geometry;

namespace
{
	constexpr double kDashLenCm     = 300.0;  // 3 m dash
	constexpr double kGapLenCm      = 500.0;  // 5 m gap
	// NOTE: the white EDGE line now comes from the eroded carriageway boundary in
	// URoadNetwork::BuildSurfaceUnion (so it curves around corners); the inset +
	// half-width used there mirror kEdgeInsetCm=38 / SolidHalfFor(3.5 m) here.

	// Painted lines run wider on wider lanes — a motorway line is not the width
	// of a lane line on a back street — but they never leave the 10-15 cm band
	// the standards allow. A 3.5 m lane lands on 14 cm solid / 12 cm dashed,
	// which is what these were fixed at before they became width-aware.
	double SolidHalfFor(double LaneWidthCm)
	{
		return 0.5 * FMath::Clamp(0.040 * LaneWidthCm, 10.0, 15.0);
	}

	double DashHalfFor(double LaneWidthCm)
	{
		return 0.5 * FMath::Clamp(0.034 * LaneWidthCm, 10.0, 15.0);
	}

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
}

namespace RoadNetMarkings
{
	void BuildRoadMarkings(const FRoadDef& Road, const FRoadCurves& C, bool bDriveOnLeft,
		TArray<FGeneralPolygon2d>& OutWhite, TArray<FGeneralPolygon2d>& OutYellow)
	{
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

		// Solid line centred at LateralOffset, routed to a specific colour set.
		auto EmitSolid = [&](double LateralOffset, double HalfW, TArray<FGeneralPolygon2d>& Dst)
		{
			TArray<FVector> Path;
			RoadNetMath::OffsetPolyline(Dense, LateralOffset, Path);
			FGeneralPolygon2d Ribbon;
			if (MakeRibbon(Path, HalfW, Ribbon)) { Dst.Add(MoveTemp(Ribbon)); }
		};

		// Dashed line centred at LateralOffset (lane divider) — always white.
		auto EmitDashed = [&](double LateralOffset, double HalfW)
		{
			TArray<FVector> Path;
			RoadNetMath::OffsetPolyline(Dense, LateralOffset, Path);
			const double Total = RoadNetMath::TotalLength(Path);
			const double Stride = kDashLenCm + kGapLenCm;
			for (double S = 0.0; S < Total; S += Stride)
			{
				TArray<FVector> Dash;
				ExtractByArcLength(Path, S, FMath::Min(S + kDashLenCm, Total), Dash);
				FGeneralPolygon2d Ribbon;
				if (MakeRibbon(Dash, HalfW, Ribbon)) { OutWhite.Add(MoveTemp(Ribbon)); }
			}
		};

		if (Road.Lanes.bMedian)
		{
			// Divided road: the raised median replaces the centre line. A solid
			// white line sits just outside the slab on each side so it stays on
			// the carriageway rather than hiding under the median top.
			const double MedianHalf = (double)Road.Lanes.MedianHalfCm();
			constexpr double kMedianEdgeLineInsetCm = 14.0;
			const double InnerW = Lanes.Num() > 0 ? (double)Lanes[0].Width
			                                      : (double)Road.Lanes.LaneWidthDefault;
			EmitSolid(+(MedianHalf + kMedianEdgeLineInsetCm), SolidHalfFor(InnerW), OutWhite);
			EmitSolid(-(MedianHalf + kMedianEdgeLineInsetCm), SolidHalfFor(InnerW), OutWhite);
		}

		// Walk adjacent lane pairs left to right and paint the boundary between
		// each. Walking pairs rather than collecting edges means the outer
		// carriageway edges are never candidates, so there is nothing to trim.
		for (int32 i = 0; i + 1 < Lanes.Num(); ++i)
		{
			const FRoadNetLane& A = Lanes[i];
			const FRoadNetLane& B = Lanes[i + 1];

			const double EdgeA = A.CenterOffset + 0.5 * (double)A.Width;
			const double EdgeB = B.CenterOffset - 0.5 * (double)B.Width;

			// A gap between two lanes is a physical divider (the median), and a
			// divider carries no paint down its middle.
			if (EdgeB - EdgeA > 1.0) { continue; }

			const double Boundary = 0.5 * (EdgeA + EdgeB);
			const double AvgW     = 0.5 * ((double)A.Width + (double)B.Width);

			const bool bDrivenA = A.bTravelsForward(bDriveOnLeft) || A.bTravelsBackward(bDriveOnLeft);
			const bool bDrivenB = B.bTravelsForward(bDriveOnLeft) || B.bTravelsBackward(bDriveOnLeft);
			const bool bOpposed =
				   A.bTravelsForward(bDriveOnLeft)  != B.bTravelsForward(bDriveOnLeft)
				|| A.bTravelsBackward(bDriveOnLeft) != B.bTravelsBackward(bDriveOnLeft);

			if (!bOpposed)
			{
				EmitDashed(Boundary, DashHalfFor(AvgW));
			}
			else if (bDrivenA && bDrivenB)
			{
				// The centre line, dividing opposing traffic. Right-hand-traffic
				// countries paint it yellow; the UK paints it white.
				EmitSolid(Boundary, SolidHalfFor(AvgW), bDriveOnLeft ? OutWhite : OutYellow);
			}
			else
			{
				// One side is not driven at all (a painted median strip, a
				// shoulder): a solid white boundary, not a centre line.
				EmitSolid(Boundary, SolidHalfFor(AvgW), OutWhite);
			}
		}

		// Mid-block pedestrian crossings. Painted from the same emitter the
		// junction presets use, so a crossing outside a school reads exactly like
		// one at a junction mouth.
		if (Road.Crossings.Num() > 0)
		{
			const double Half = FMath::Max(50.0, (double)Road.Lanes.HalfWidthCm());
			const double Total = RoadNetMath::TotalLength(Dense);
			for (const FRoadNetCrossingMark& X : Road.Crossings)
			{
				FVector Pos, Fwd;
				if (!FrameAtArc(Dense, FMath::Clamp((double)X.DistanceCm, 0.0, Total), Pos, Fwd)) { continue; }
				const FVector2D P(Pos.X, Pos.Y), T(Fwd.X, Fwd.Y);
				RoadNetJunctionMarks::EmitZebra(P, T, Half, (double)X.DepthCm, OutWhite);

				if (X.bStopBar)
				{
					// One bar per direction, each on the half its traffic enters
					// on, set back from the band by half its depth plus a margin.
					const FVector2D Lateral(T.Y, -T.X);
					const double Back = 0.5 * (double)X.DepthCm + 60.0;
					const FVector2D EnterFwd = bDriveOnLeft ? FVector2D(-Lateral.X, -Lateral.Y) : Lateral;
					RoadNetJunctionMarks::EmitStopBar(P - T * Back, T, EnterFwd, Half, OutWhite);
					RoadNetJunctionMarks::EmitStopBar(P + T * Back, FVector2D(-T.X, -T.Y),
						FVector2D(-EnterFwd.X, -EnterFwd.Y), Half, OutWhite);
				}
			}
		}

		// Edge lines (white) are NOT emitted per-road here anymore: they are
		// stroked from the merged carriageway boundary in BuildSurfaceUnion so
		// they curve around junction/sidewalk corners (a straight per-road offset
		// can't turn a corner). kEdgeInsetCm is kept in sync there.
	}
}
