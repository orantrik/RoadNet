// RoadNetZones.cpp — grade-separation layering (§10.12).
#include "RoadNetZones.h"
#include "RoadNetwork.h"   // FRoadCurves, FRoadNetJoint
#include "RoadNetTypes.h"  // FRoadDef
#include "RoadNetMath.h"

namespace
{
	// Minimal union-find over a compact [0..N) index space.
	struct FUnionFind
	{
		TArray<int32> Parent;
		void Init(int32 N) { Parent.SetNumUninitialized(N); for (int32 i = 0; i < N; ++i) { Parent[i] = i; } }
		int32 Find(int32 X) { while (Parent[X] != X) { Parent[X] = Parent[Parent[X]]; X = Parent[X]; } return X; }
		void Union(int32 A, int32 B) { A = Find(A); B = Find(B); if (A != B) { Parent[A] = B; } }
	};

	FBox2D CurveBounds2D(const FRoadCurves& C, double MarginCm)
	{
		FBox2D B(ForceInit);
		for (const FVector& P : C.Sampled) { B += FVector2D(P.X, P.Y); }
		if (B.bIsValid) { B = B.ExpandBy(MarginCm); }
		return B;
	}

	// Z on a sampled polyline at segment Seg, param T in [0,1].
	double SegZ(const TArray<FVector>& Poly, int32 Seg, double T)
	{
		if (!Poly.IsValidIndex(Seg) || !Poly.IsValidIndex(Seg + 1)) { return Poly.Num() ? Poly[0].Z : 0.0; }
		return FMath::Lerp(Poly[Seg].Z, Poly[Seg + 1].Z, T);
	}

	// True if the two sampled centerlines cross in 2-D; fills the Z on each.
	bool CenterlinesCross(const TArray<FVector>& A, const TArray<FVector>& B, double& OutZa, double& OutZb)
	{
		for (int32 i = 0; i + 1 < A.Num(); ++i)
		{
			const FVector2D A0(A[i].X, A[i].Y), A1(A[i + 1].X, A[i + 1].Y);
			for (int32 j = 0; j + 1 < B.Num(); ++j)
			{
				const FVector2D B0(B[j].X, B[j].Y), B1(B[j + 1].X, B[j + 1].Y);
				FVector2D Hit; double Ta, Tb;
				if (RoadNetMath::SegmentIntersect2D(A0, A1, B0, B1, Hit, Ta, Tb))
				{
					OutZa = SegZ(A, i, Ta);
					OutZb = SegZ(B, j, Tb);
					return true;
				}
			}
		}
		return false;
	}
}

namespace RoadNetZones
{
	void PartitionLayers(
		const TArray<int32>& RoadIndices,
		const TMap<int32, FRoadCurves>& Curves,
		const TArray<FRoadDef>& Roads,
		const TArray<FRoadNetJoint>& Joints,
		double MaxZGapCm,
		TArray<TArray<int32>>& OutGroups)
	{
		OutGroups.Reset();
		const int32 N = RoadIndices.Num();
		if (N == 0) { return; }

		// Map global road index -> compact local index.
		TMap<int32, int32> Local;
		Local.Reserve(N);
		for (int32 i = 0; i < N; ++i) { Local.Add(RoadIndices[i], i); }

		FUnionFind UF;
		UF.Init(N);

		// (1) Shared-node connectivity — arms meeting at a joint are same-layer.
		for (const FRoadNetJoint& J : Joints)
		{
			int32 First = INDEX_NONE;
			for (const TPair<int32, bool>& Arm : J.Arms)
			{
				const int32* L = Local.Find(Arm.Key);
				if (!L) { continue; }
				if (First == INDEX_NONE) { First = *L; }
				else { UF.Union(First, *L); }
			}
		}

		// Precompute broadphase bounds.
		TArray<FBox2D> Bounds;
		Bounds.SetNum(N);
		for (int32 i = 0; i < N; ++i)
		{
			const FRoadCurves* C = Curves.Find(RoadIndices[i]);
			Bounds[i] = C ? CurveBounds2D(*C, 500.0) : FBox2D(ForceInit);
		}

		// (2) At-grade crossings without a shared node — union only if truly at-grade.
		for (int32 a = 0; a < N; ++a)
		{
			const FRoadCurves* Ca = Curves.Find(RoadIndices[a]);
			if (!Ca) { continue; }
			const FRoadDef& Ra = Roads[RoadIndices[a]];
			for (int32 b = a + 1; b < N; ++b)
			{
				if (UF.Find(a) == UF.Find(b)) { continue; } // already same zone
				if (!Bounds[a].bIsValid || !Bounds[b].bIsValid || !Bounds[a].Intersect(Bounds[b])) { continue; }

				const FRoadCurves* Cb = Curves.Find(RoadIndices[b]);
				if (!Cb) { continue; }
				const FRoadDef& Rb = Roads[RoadIndices[b]];

				double Za, Zb;
				if (!CenterlinesCross(Ca->Sampled, Cb->Sampled, Za, Zb)) { continue; }

				const bool bGradeSep =
					(Ra.Layer != Rb.Layer) ||
					(Ra.bBridge != Rb.bBridge) ||
					(Ra.bTunnel != Rb.bTunnel) ||
					(FMath::Abs(Za - Zb) > MaxZGapCm);

				if (!bGradeSep) { UF.Union(a, b); }
			}
		}

		// Collect components.
		TMap<int32, int32> RootToGroup;
		for (int32 i = 0; i < N; ++i)
		{
			const int32 Root = UF.Find(i);
			int32* G = RootToGroup.Find(Root);
			if (!G) { G = &RootToGroup.Add(Root, OutGroups.AddDefaulted()); }
			OutGroups[*G].Add(RoadIndices[i]);
		}
	}
}
