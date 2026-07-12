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

	void SmoothProfileZ(TArray<FVector>& Poly, double HalfWindowCm)
	{
		const int32 N = Poly.Num();
		if (N < 3 || HalfWindowCm <= KINDA_SMALL_NUMBER) { return; }

		// Arc length along the (planar) polyline — the profile's independent var.
		TArray<double> S; CumulativeLength(Poly, S);
		if (S.Last() <= KINDA_SMALL_NUMBER) { return; }

		// Fit z = a + b·(s - s0) locally at each vertex; keep only the intercept a
		// (the fitted height AT that vertex). Endpoints get a one-sided window, so
		// they stay anchored near their original height (junction ends barely move).
		TArray<double> Out; Out.SetNumUninitialized(N);
		for (int32 i = 0; i < N; ++i)
		{
			const double s0 = S[i];
			double sw = 0, ss = 0, sz = 0, sss = 0, ssz = 0;
			for (int32 j = 0; j < N; ++j)
			{
				const double ds = S[j] - s0;
				const double ad = FMath::Abs(ds);
				if (ad > HalfWindowCm) { continue; }
				const double wj = 1.0 - ad / HalfWindowCm; // triangular weight in [0,1]
				const double z  = Poly[j].Z;
				sw  += wj;
				ss  += wj * ds;
				sz  += wj * z;
				sss += wj * ds * ds;
				ssz += wj * ds * z;
			}
			const double denom = sw * sss - ss * ss;
			if (FMath::Abs(denom) > 1e-6)
			{
				Out[i] = (sz * sss - ss * ssz) / denom; // intercept a = fit at ds = 0
			}
			else
			{
				Out[i] = (sw > 0.0) ? (sz / sw) : Poly[i].Z; // window collapsed → mean
			}
		}
		for (int32 i = 0; i < N; ++i) { Poly[i].Z = Out[i]; }
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

	void SmoothCatmullRom(const TArray<FVector>& In, double MaxChordCm, TArray<FVector>& Out)
	{
		Out.Reset();
		const int32 N = In.Num();
		if (N < 3 || MaxChordCm <= KINDA_SMALL_NUMBER) { Out = In; return; }

		auto Dist = [](const FVector& A, const FVector& B) { return FMath::Max(1.0e-4, FVector::Dist(A, B)); };
		// Interpolate on the (t)-parametrised segment [A@ta, B@tb] at parameter t.
		auto RemapLerp = [](const FVector& A, const FVector& B, double ta, double tb, double t)
		{
			const double d = tb - ta;
			const double w = (FMath::Abs(d) > KINDA_SMALL_NUMBER) ? (t - ta) / d : 0.0;
			return A + (B - A) * w;
		};

		Out.Reserve(N * 4);
		Out.Add(In[0]);
		for (int32 i = 0; i < N - 1; ++i)
		{
			// Centripetal Catmull-Rom (alpha = 0.5) is cusp/overshoot-free. End
			// spans mirror the neighbour to synthesise a natural end tangent.
			const FVector P0 = (i > 0)     ? In[i - 1] : (In[i] + (In[i] - In[i + 1]));
			const FVector P1 = In[i];
			const FVector P2 = In[i + 1];
			const FVector P3 = (i + 2 < N) ? In[i + 2] : (In[i + 1] + (In[i + 1] - In[i]));

			const double t0 = 0.0;
			const double t1 = t0 + FMath::Sqrt(Dist(P0, P1));
			const double t2 = t1 + FMath::Sqrt(Dist(P1, P2));
			const double t3 = t2 + FMath::Sqrt(Dist(P2, P3));

			const int32 Sub = FMath::Clamp((int32)FMath::CeilToInt(Dist(P1, P2) / MaxChordCm), 1, 64);
			for (int32 k = 1; k <= Sub; ++k)
			{
				const double t = FMath::Lerp(t1, t2, (double)k / (double)Sub);
				const FVector A1 = RemapLerp(P0, P1, t0, t1, t);
				const FVector A2 = RemapLerp(P1, P2, t1, t2, t);
				const FVector A3 = RemapLerp(P2, P3, t2, t3, t);
				const FVector B1 = RemapLerp(A1, A2, t0, t2, t);
				const FVector B2 = RemapLerp(A2, A3, t1, t3, t);
				Out.Add(RemapLerp(B1, B2, t1, t2, t)); // k==Sub lands exactly on P2 (knot preserved)
			}
		}

		if (Out.Num() < 2) { Out.Reset(); Out.Add(In[0]); Out.Add(In.Last()); }
	}

	// Natural cubic spline second derivatives (M) for samples Y at parameters T.
	// Tridiagonal (Thomas) solve with natural end conditions M[0]=M[n-1]=0.
	static void SolveNaturalCubic(const TArray<double>& T, const TArray<double>& Y, TArray<double>& M)
	{
		const int32 n = T.Num();
		M.Init(0.0, n);
		const int32 m = n - 2;                 // interior unknowns M[1..n-2]
		if (m <= 0) { return; }

		TArray<double> h; h.SetNum(n - 1);
		for (int32 i = 0; i < n - 1; ++i) { h[i] = FMath::Max(1.0e-4, T[i + 1] - T[i]); }

		TArray<double> b, c, d;
		b.SetNum(m); c.SetNum(m); d.SetNum(m);
		for (int32 k = 0; k < m; ++k)
		{
			const int32 i = k + 1;
			b[k] = 2.0 * (h[i - 1] + h[i]);
			c[k] = h[i];                       // super-diagonal
			d[k] = 6.0 * ((Y[i + 1] - Y[i]) / h[i] - (Y[i] - Y[i - 1]) / h[i - 1]);
		}
		// Forward sweep (sub-diagonal a[k] = h[i-1] = h[k]).
		for (int32 k = 1; k < m; ++k)
		{
			const double w = h[k] / b[k - 1];
			b[k] -= w * c[k - 1];
			d[k] -= w * d[k - 1];
		}
		// Back substitution.
		TArray<double> x; x.SetNum(m);
		x[m - 1] = d[m - 1] / b[m - 1];
		for (int32 k = m - 2; k >= 0; --k) { x[k] = (d[k] - c[k] * x[k + 1]) / b[k]; }
		for (int32 k = 0; k < m; ++k) { M[k + 1] = x[k]; }
	}

	void SmoothG2Spline(const TArray<FVector>& In, double MaxChordCm, TArray<FVector>& Out)
	{
		Out.Reset();
		const int32 N = In.Num();
		if (N < 3 || MaxChordCm <= KINDA_SMALL_NUMBER) { Out = In; return; }

		// Chord-length parametrisation (3D distance).
		TArray<double> T; T.SetNum(N); T[0] = 0.0;
		for (int32 i = 1; i < N; ++i) { T[i] = T[i - 1] + FMath::Max(1.0e-3, FVector::Dist(In[i - 1], In[i])); }

		TArray<double> X, Y, Z; X.SetNum(N); Y.SetNum(N); Z.SetNum(N);
		for (int32 i = 0; i < N; ++i) { X[i] = In[i].X; Y[i] = In[i].Y; Z[i] = In[i].Z; }

		TArray<double> Mx, My, Mz;
		SolveNaturalCubic(T, X, Mx);
		SolveNaturalCubic(T, Y, My);
		SolveNaturalCubic(T, Z, Mz);

		auto Eval = [&](int32 i, double t) -> FVector
		{
			const double h = FMath::Max(1.0e-4, T[i + 1] - T[i]);
			const double A = (T[i + 1] - t) / h;
			const double B = (t - T[i]) / h;
			auto F = [&](const TArray<double>& yv, const TArray<double>& Mv)
			{
				return A * yv[i] + B * yv[i + 1]
					+ ((A * A * A - A) * Mv[i] + (B * B * B - B) * Mv[i + 1]) * h * h / 6.0;
			};
			return FVector(F(X, Mx), F(Y, My), F(Z, Mz));
		};

		Out.Reserve(N * 4);
		Out.Add(In[0]);
		for (int32 i = 0; i < N - 1; ++i)
		{
			const double Seg = T[i + 1] - T[i];
			const int32 Sub = FMath::Clamp((int32)FMath::CeilToInt(Seg / MaxChordCm), 1, 64);
			for (int32 k = 1; k <= Sub; ++k)
			{
				const double t = FMath::Lerp(T[i], T[i + 1], (double)k / (double)Sub);
				Out.Add(Eval(i, t)); // k==Sub lands exactly on In[i+1] (knot preserved)
			}
		}

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
