// RoadNetZones.cpp — grade-separation layering (§10.12).
#include "RoadNetZones.h"
#include "RoadNetwork.h"   // FRoadCurves, FRoadNetJoint, FRoadNetCrossing
#include "RoadNetTypes.h"  // FRoadDef

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
}

namespace RoadNetZones
{
	void PartitionLayers(
		const TArray<int32>& RoadIndices,
		const TMap<int32, FRoadCurves>& Curves,
		const TArray<FRoadDef>& Roads,
		const TArray<FRoadNetJoint>& Joints,
		const TArray<FRoadNetCrossing>& Crossings,
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

		// (2) At-grade crossings without a shared node — union only if truly
		// at-grade. Crossings were precomputed with a spatial-grid broadphase
		// (shared with the surface stage), so this is now linear in crossings.
		for (const FRoadNetCrossing& X : Crossings)
		{
			const int32* La = Local.Find(X.RoadA);
			const int32* Lb = Local.Find(X.RoadB);
			if (!La || !Lb) { continue; }
			if (UF.Find(*La) == UF.Find(*Lb)) { continue; } // already same zone
			if (!Roads.IsValidIndex(X.RoadA) || !Roads.IsValidIndex(X.RoadB)) { continue; }

			const FRoadDef& Ra = Roads[X.RoadA];
			const FRoadDef& Rb = Roads[X.RoadB];
			const bool bGradeSep =
				(Ra.Layer != Rb.Layer) ||
				(Ra.bBridge != Rb.bBridge) ||
				(Ra.bTunnel != Rb.bTunnel) ||
				(FMath::Abs(X.Za - X.Zb) > MaxZGapCm);

			if (!bGradeSep) { UF.Union(*La, *Lb); }
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
