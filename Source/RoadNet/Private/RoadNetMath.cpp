// RoadNetMath.cpp — implementation of the pure road-geometry math (§10.1–§10.8).
#include "RoadNetMath.h"

namespace RoadNetMath
{
	static FORCEINLINE FVector2D XY(const FVector& V) { return FVector2D(V.X, V.Y); }

	FVector2D TangentAt(const TArray<FVector>& Poly, int32 Index)
	{
		const int32 N = Poly.Num();
		if (N < 2) { return FVector2D(1.0, 0.0); }

		Index = FMath::Clamp(Index, 0, N - 1);
		FVector2D T(0, 0);
		if (Index > 0)     { T += (XY(Poly[Index]) - XY(Poly[Index - 1])).GetSafeNormal(); }
		if (Index < N - 1) { T += (XY(Poly[Index + 1]) - XY(Poly[Index])).GetSafeNormal(); }

		const FVector2D N2 = T.GetSafeNormal();
		return N2.IsNearlyZero() ? FVector2D(1.0, 0.0) : N2;
	}

	FVector2D RightAxis(const FVector2D& Tangent)
	{
		// R = T x Z with Z up in a left-handed frame → (T.Y, -T.X).
		return FVector2D(Tangent.Y, -Tangent.X);
	}

	void CumulativeLength(const TArray<FVector>& Poly, TArray<double>& OutLen)
	{
		OutLen.Reset();
		OutLen.SetNumUninitialized(Poly.Num());
		double Acc = 0.0;
		for (int32 i = 0; i < Poly.Num(); ++i)
		{
			if (i > 0) { Acc += FVector::Dist2D(Poly[i - 1], Poly[i]); }
			OutLen[i] = Acc;
		}
	}

	double TotalLength(const TArray<FVector>& Poly)
	{
		double Acc = 0.0;
		for (int32 i = 1; i < Poly.Num(); ++i) { Acc += FVector::Dist2D(Poly[i - 1], Poly[i]); }
		return Acc;
	}

	void ResampleByArcLength(const TArray<FVector>& In, double Spacing, TArray<FVector>& Out, double MaxTurnRad)
	{
		Out.Reset();
		const int32 N = In.Num();
		if (N == 0) { return; }
		if (N == 1 || Spacing <= KINDA_SMALL_NUMBER) { Out = In; return; }

		TArray<double> Len;
		CumulativeLength(In, Len);
		const double Total = Len.Last();
		if (Total <= KINDA_SMALL_NUMBER) { Out.Add(In[0]); return; }

		// Build the set of arc-length positions to sample: the two ends, every
		// multiple of Spacing, plus any ORIGINAL knot whose heading changes more
		// sharply than MaxTurnRad (so tight bends keep fidelity). We evaluate all
		// of them in strict arc-length order and de-duplicate, which avoids the
		// self-crossing/duplicate-vertex artifacts of sort-by-distance merging.
		TArray<double> Samples;
		Samples.Reserve((int32)(Total / Spacing) + N + 2);
		Samples.Add(0.0);
		for (double S = Spacing; S < Total - KINDA_SMALL_NUMBER; S += Spacing) { Samples.Add(S); }
		Samples.Add(Total);

		if (MaxTurnRad > 0.0)
		{
			for (int32 i = 1; i < N - 1; ++i)
			{
				const FVector2D A = (XY(In[i]) - XY(In[i - 1])).GetSafeNormal();
				const FVector2D B = (XY(In[i + 1]) - XY(In[i])).GetSafeNormal();
				const double Ang = FMath::Acos(FMath::Clamp(FVector2D::DotProduct(A, B), -1.0, 1.0));
				if (Ang > MaxTurnRad) { Samples.Add(Len[i]); }
			}
		}

		Samples.Sort();

		// Emit points by interpolating each arc-length position along In. Skip
		// positions coincident with the previous one (within a small epsilon).
		const double DedupeCm = FMath::Min(1.0, Spacing * 0.01);
		int32 Seg = 0;
		double Prev = -TNumericLimits<double>::Max();
		for (double S : Samples)
		{
			const double Sc = FMath::Clamp(S, 0.0, Total);
			if (Sc - Prev < DedupeCm) { continue; }
			Prev = Sc;
			while (Seg < N - 2 && Len[Seg + 1] < Sc) { ++Seg; }
			const double SegLen = Len[Seg + 1] - Len[Seg];
			const double T = SegLen > KINDA_SMALL_NUMBER
				? FMath::Clamp((Sc - Len[Seg]) / SegLen, 0.0, 1.0) : 0.0;
			Out.Add(FMath::Lerp(In[Seg], In[Seg + 1], T));
		}

		// Guarantee a usable polyline even in pathological input.
		if (Out.Num() < 2) { Out.Reset(); Out.Add(In[0]); Out.Add(In.Last()); }
	}

	void OffsetPolyline(const TArray<FVector>& In, double SignedOffset, TArray<FVector>& Out, double MiterLimit)
	{
		Out.Reset();
		const int32 N = In.Num();
		if (N < 2) { Out = In; return; }
		Out.SetNumUninitialized(N);

		for (int32 i = 0; i < N; ++i)
		{
			const FVector2D T = TangentAt(In, i);
			FVector2D R = RightAxis(T);

			// At interior vertices, use the miter of the two adjacent right axes.
			if (i > 0 && i < N - 1)
			{
				const FVector2D Ta = (XY(In[i]) - XY(In[i - 1])).GetSafeNormal();
				const FVector2D Tb = (XY(In[i + 1]) - XY(In[i])).GetSafeNormal();
				const FVector2D Ra = RightAxis(Ta);
				const FVector2D Rb = RightAxis(Tb);
				// If a neighbouring segment is degenerate (coincident vertices),
				// fall back to the valid side's axis instead of mitering, which
				// would otherwise blow up to MiterLimit and spike the offset.
				if (Ra.IsNearlyZero() || Rb.IsNearlyZero())
				{
					R = Ra.IsNearlyZero() ? Rb : Ra;
				}
				else
				{
				FVector2D M = (Ra + Rb);
				const double MLen = M.Size();
				if (MLen > KINDA_SMALL_NUMBER)
				{
					M /= MLen;
					// miter length factor = 1 / cos(phi/2); cos(phi/2) = |M·Ra|.
					const double CosHalf = FMath::Abs(FVector2D::DotProduct(M, Ra));
					const double Scale = CosHalf > KINDA_SMALL_NUMBER ? (1.0 / CosHalf) : MiterLimit;
					if (Scale <= MiterLimit) { R = M * Scale; }
					else { R = M * MiterLimit; } // clamp; round-join refinement is TODO (§10.4)
				}
				}
			}

			const FVector2D P = XY(In[i]) + R * SignedOffset;
			Out[i] = FVector(P.X, P.Y, In[i].Z);
		}
	}

	FVector2D ClosestOnSegment(const FVector2D& A, const FVector2D& B, const FVector2D& Q, double& OutT)
	{
		const FVector2D AB = B - A;
		const double Len2 = AB.SizeSquared();
		OutT = (Len2 > KINDA_SMALL_NUMBER)
			? FMath::Clamp(FVector2D::DotProduct(Q - A, AB) / Len2, 0.0, 1.0)
			: 0.0;
		return A + AB * OutT;
	}

	FProjectResult ProjectToPolyline(const TArray<FVector>& Poly, const FVector2D& Q)
	{
		FProjectResult Best;
		const int32 N = Poly.Num();
		if (N == 0) { return Best; }
		if (N == 1)
		{
			Best.Point = XY(Poly[0]);
			Best.Distance = FVector2D::Distance(Q, Best.Point);
			Best.Segment = 0;
			return Best;
		}

		double AccLen = 0.0;
		for (int32 i = 0; i < N - 1; ++i)
		{
			const FVector2D A = XY(Poly[i]);
			const FVector2D B = XY(Poly[i + 1]);
			double T;
			const FVector2D C = ClosestOnSegment(A, B, Q, T);
			const double D = FVector2D::Distance(Q, C);
			const double Along = AccLen + T * FVector2D::Distance(A, B);

			// Seam-safe: prefer strictly smaller distance; break ties by smaller
			// AlongDist so we don't flip between two segments sharing a vertex.
			if (D < Best.Distance - 1e-4 ||
				(FMath::Abs(D - Best.Distance) <= 1e-4 && Along < Best.AlongDist))
			{
				const FVector2D AB = (B - A);
				const double SegLen = AB.Size();
				const double Signed = SegLen > KINDA_SMALL_NUMBER
					? Cross2D(AB / SegLen, Q - A) : 0.0;
				Best.Distance  = D;
				Best.AlongDist = Along;
				Best.Offset    = Signed;
				Best.Segment   = i;
				Best.Point     = C;
			}
			AccLen += FVector2D::Distance(A, B);
		}
		return Best;
	}

	bool SegmentIntersect2D(
		const FVector2D& P1, const FVector2D& P2,
		const FVector2D& P3, const FVector2D& P4,
		FVector2D& OutPoint, double& OutT, double& OutU, double Eps)
	{
		const FVector2D R = P2 - P1;
		const FVector2D S = P4 - P3;
		const double D = Cross2D(R, S);
		if (FMath::Abs(D) < Eps) { return false; } // parallel / degenerate

		const FVector2D QP = P3 - P1;
		OutT = Cross2D(QP, S) / D;
		OutU = Cross2D(QP, R) / D;
		if (OutT < 0.0 || OutT > 1.0 || OutU < 0.0 || OutU > 1.0) { return false; }

		OutPoint = P1 + R * OutT;
		return true;
	}

	bool CornerFillet(
		const FVector2D& Apex, const FVector2D& DirA, const FVector2D& DirB,
		double Radius, double& OutSetback, FVector2D& OutArcCenter, double& OutTurnAngleRad)
	{
		const FVector2D A = DirA.GetSafeNormal();
		const FVector2D B = DirB.GetSafeNormal();
		if (A.IsNearlyZero() || B.IsNearlyZero()) { return false; }

		const double Cos = FMath::Clamp(FVector2D::DotProduct(A, B), -1.0, 1.0);
		const double Phi = FMath::Acos(Cos);         // angle between arms
		OutTurnAngleRad = Phi;
		if (Phi < KINDA_SMALL_NUMBER || (PI - Phi) < KINDA_SMALL_NUMBER)
		{
			return false; // (anti)parallel — no finite fillet
		}

		const double HalfTan = FMath::Tan(Phi * 0.5);
		if (FMath::Abs(HalfTan) < KINDA_SMALL_NUMBER) { return false; }

		OutSetback = Radius / HalfTan;                // t = r / tan(phi/2)

		// Arc centre lies along the interior bisector at distance r / sin(phi/2).
		FVector2D Bis = (A + B).GetSafeNormal();
		const double SinHalf = FMath::Sin(Phi * 0.5);
		const double CenterDist = (SinHalf > KINDA_SMALL_NUMBER) ? (Radius / SinHalf) : Radius;
		OutArcCenter = Apex + Bis * CenterDist;
		return true;
	}

	double SignedArea(const TArray<FVector2D>& Loop)
	{
		const int32 N = Loop.Num();
		if (N < 3) { return 0.0; }
		double A = 0.0;
		for (int32 i = 0; i < N; ++i)
		{
			const FVector2D& P = Loop[i];
			const FVector2D& Q = Loop[(i + 1) % N];
			A += (P.X * Q.Y - Q.X * P.Y);
		}
		return 0.5 * A;
	}
}
