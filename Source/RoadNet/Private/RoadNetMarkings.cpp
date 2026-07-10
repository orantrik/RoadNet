// RoadNetMarkings.cpp — baked lane-marking ribbons (§8.10).
#include "RoadNetMarkings.h"
#include "RoadNetwork.h"    // FRoadCurves
#include "RoadNetTypes.h"   // FRoadDef
#include "RoadNetMath.h"
#include "Polygon2.h"

using namespace UE::Geometry;

namespace
{
	constexpr double kStripeHalfCm  = 7.0;    // 14 cm solid line (centre/edge)
	constexpr double kDashHalfCm    = 6.0;    // 12 cm dashed lane divider
	constexpr double kDashLenCm     = 300.0;  // 3 m dash
	constexpr double kGapLenCm      = 500.0;  // 5 m gap
	// NOTE: the white EDGE line now comes from the eroded carriageway boundary in
	// URoadNetwork::BuildSurfaceUnion (so it curves around corners); the inset +
	// half-width used there mirror kEdgeInsetCm=38 / kStripeHalfCm here.

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
	void BuildRoadMarkings(const FRoadDef& Road, const FRoadCurves& C,
		TArray<FGeneralPolygon2d>& OutWhite, TArray<FGeneralPolygon2d>& OutYellow)
	{
		if (C.Sampled.Num() < 2) { return; }

		const double Half = FMath::Max(50.0, (double)Road.Lanes.HalfWidthCm());

		// Densify the (already G2-smoothed) centreline so every offset line reads
		// as a smooth curve around corners — matching the finely-sampled kerb
		// instead of faceting at the ~2 m resample spacing.
		TArray<FVector> Dense;
		RoadNetMath::SmoothG2Spline(C.Sampled, 40.0, Dense);
		if (Dense.Num() < 2) { Dense = C.Sampled; }

		// Solid line centred at LateralOffset, routed to a specific colour set.
		auto EmitSolid = [&](double LateralOffset, TArray<FGeneralPolygon2d>& Dst)
		{
			TArray<FVector> Path;
			RoadNetMath::OffsetPolyline(Dense, LateralOffset, Path);
			FGeneralPolygon2d Ribbon;
			if (MakeRibbon(Path, kStripeHalfCm, Ribbon)) { Dst.Add(MoveTemp(Ribbon)); }
		};

		// Dashed line centred at LateralOffset (lane divider) — always white.
		auto EmitDashed = [&](double LateralOffset)
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
				if (MakeRibbon(Dash, kDashHalfCm, Ribbon)) { OutWhite.Add(MoveTemp(Ribbon)); }
			}
		};

		// Centre line on two-way roads only (yellow, dividing opposing traffic).
		if (!Road.Lanes.bOneway)
		{
			EmitSolid(0.0, OutYellow);
		}

		// Dashed dividers at interior lane boundaries (white).
		const int32 NLanes = FMath::Max(1, Road.Lanes.EffectiveLaneCount());
		if (NLanes >= 2)
		{
			const double LaneW = (2.0 * Half) / NLanes;
			for (int32 k = 1; k < NLanes; ++k)
			{
				const double Offset = -Half + k * LaneW;
				if (!Road.Lanes.bOneway && FMath::Abs(Offset) < 1.0) { continue; } // centre already solid
				EmitDashed(Offset);
			}
		}

		// Edge lines (white) are NOT emitted per-road here anymore: they are
		// stroked from the merged carriageway boundary in BuildSurfaceUnion so
		// they curve around junction/sidewalk corners (a straight per-road offset
		// can't turn a corner). kEdgeInsetCm/kStripeHalfCm are kept in sync there.
	}
}
