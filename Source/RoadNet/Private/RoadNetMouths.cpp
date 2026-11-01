#include "RoadNetMouths.h"
#include "Curve/GeneralPolygon2.h"

void RoadNetMouths::ClipCrossings(const TArray<FVector>& S, const UE::Geometry::FGeneralPolygon2d& GP,
	double HalfWidthCm, TArray<FCrossing>& Out)
{
	using namespace UE::Geometry;
	if (S.Num() < 2) { return; }
	const TArray<FVector2d>& OV = GP.GetOuter().GetVertices();
	if (OV.Num() < 3) { return; }

	auto InClip = [&GP](const FVector2D& P) { return GP.Contains(FVector2d(P.X, P.Y)); };
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

	const double Half = FMath::Max(50.0, HalfWidthCm);
	bool bPrevIn = InClip(FVector2D(S[0].X, S[0].Y));
	for (int32 i = 1; i < S.Num(); ++i)
	{
		const FVector2D A(S[i - 1].X, S[i - 1].Y);
		const FVector2D B(S[i].X, S[i].Y);
		const bool bIn = InClip(B);
		if (bIn == bPrevIn) { bPrevIn = bIn; continue; }

		FVector2D Lo = A, Hi = B;
		bool bLoIn = bPrevIn;
		for (int32 it = 0; it < 12; ++it)
		{
			const FVector2D Mid = 0.5 * (Lo + Hi);
			if (InClip(Mid) == bLoIn) { Lo = Mid; } else { Hi = Mid; }
		}
		const FVector2D Boundary = 0.5 * (Lo + Hi);
		FVector2D Outward = bPrevIn ? (B - A) : (A - B);
		FVector2D OutN = Outward;
		if (!OutN.Normalize()) { OutN = FVector2D(1, 0); }

		const FVector2D EdgeDir = NearestEdgeDir(Boundary);
		const FVector2D EdgeN(-EdgeDir.Y, EdgeDir.X);
		const double CosT = FMath::Abs(FVector2D::DotProduct(OutN, EdgeN));
		double Extra = 30.0;
		if (CosT > 0.15)
		{
			const double SinT = FMath::Sqrt(FMath::Max(0.0, 1.0 - CosT * CosT));
			Extra += Half * (SinT / CosT);
		}
		else { Extra += 1.5 * Half; }
		Extra = FMath::Min(Extra, 1.75 * Half + 30.0);

		FCrossing C;
		C.Boundary = Boundary;
		C.Stop = Boundary + OutN * Extra;
		C.Outward = Outward;
		C.OutN = OutN;
		C.Z = FMath::Lerp(S[i - 1].Z, S[i].Z, 0.5);
		C.bForwardEnters = !bPrevIn;
		Out.Add(C);
		bPrevIn = bIn;
	}
}
