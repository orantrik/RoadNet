#include "RoadNetLanes.h"
#include "RoadNetMath.h"

namespace
{
	// Linear-interpolate a sorted-by-Distance edge knot list at DistCm. Empty →
	// returns false. Clamps outside the knot range.
	bool EdgeOffsetAt(const TArray<FRoadNetEdgeKnot>& Edge, double DistCm, double& Out)
	{
		const int32 N = Edge.Num();
		if (N == 0) return false;
		if (N == 1) { Out = Edge[0].Offset; return true; }
		if (DistCm <= Edge[0].Distance)         { Out = Edge[0].Offset;     return true; }
		if (DistCm >= Edge[N - 1].Distance)     { Out = Edge[N - 1].Offset; return true; }
		for (int32 i = 0; i + 1 < N; ++i)
		{
			const FRoadNetEdgeKnot& A = Edge[i];
			const FRoadNetEdgeKnot& B = Edge[i + 1];
			if (DistCm >= A.Distance && DistCm <= B.Distance)
			{
				const double span = B.Distance - A.Distance;
				const double t = (span > KINDA_SMALL_NUMBER) ? (DistCm - A.Distance) / span : 0.0;
				Out = FMath::Lerp(A.Offset, B.Offset, t);
				return true;
			}
		}
		Out = Edge[N - 1].Offset;
		return true;
	}
}

namespace RoadNetLanes
{
	double LaneOffsetAt(const FRoadNetLane& Lane, double DistCm)
	{
		double L, R;
		const bool bL = EdgeOffsetAt(Lane.LeftEdge,  DistCm, L);
		const bool bR = EdgeOffsetAt(Lane.RightEdge, DistCm, R);
		if (bL && bR) return 0.5 * (L + R);
		if (bL)       return L;
		if (bR)       return R;
		return Lane.CenterOffset; // uniform lane
	}

	void BuildLaneCenterline(
		const TArray<FVector>& SampledRef, const FRoadNetLane& Lane, TArray<FVector>& Out)
	{
		Out.Reset();
		const int32 N = SampledRef.Num();
		if (N < 2) { Out = SampledRef; return; }

		TArray<double> Len;
		RoadNetMath::CumulativeLength(SampledRef, Len);

		Out.Reserve(N);
		for (int32 i = 0; i < N; ++i)
		{
			const double d = Len.IsValidIndex(i) ? Len[i] : 0.0;
			const double off = LaneOffsetAt(Lane, d);
			const FVector2D T = RoadNetMath::TangentAt(SampledRef, i);
			const FVector2D Rt = RoadNetMath::RightAxis(T);
			FVector P = SampledRef[i];
			P.X += Rt.X * off;
			P.Y += Rt.Y * off;
			Out.Add(P);
		}
	}
}
