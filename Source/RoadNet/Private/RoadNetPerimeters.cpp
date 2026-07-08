// RoadNetPerimeters.cpp — lift merged-surface rings into world loops (§10.11/§8.4).
#include "RoadNetPerimeters.h"
#include "RoadNetwork.h"   // FRoadCurves
#include "RoadNetMesh.h"   // SampleHeight / FirstCenterlineZ
#include "Polygon2.h"

using namespace UE::Geometry;

namespace RoadNetPerimeters
{
	static void RingToLoop(const FPolygon2d& Ring, const TArray<const TArray<FVector>*>& CenterLines,
		double FallbackZ, double ZLiftCm, bool bOuter, int32 Zone, TArray<FRoadNetLoop>& OutLoops)
	{
		const TArray<FVector2d>& V = Ring.GetVertices();
		if (V.Num() < 3) { return; }

		FRoadNetLoop Loop;
		Loop.bOuter = bOuter;
		Loop.Zone = Zone;
		Loop.Points.Reserve(V.Num());
		for (const FVector2d& P : V)
		{
			const double Z = RoadNetMesh::SampleHeight(CenterLines, P.X, P.Y, FallbackZ) + ZLiftCm;
			Loop.Points.Emplace(P.X, P.Y, Z);
		}
		OutLoops.Add(MoveTemp(Loop));
	}

	void ExtractLoops(
		const TArray<TArray<FGeneralPolygon2d>>& ZonePolys,
		const TArray<TArray<int32>>& Zones,
		const TMap<int32, FRoadCurves>& Curves,
		double ZLiftCm,
		TArray<FRoadNetLoop>& OutLoops)
	{
		for (int32 z = 0; z < ZonePolys.Num(); ++z)
		{
			if (ZonePolys[z].Num() == 0) { continue; }

			// Gather this zone's centerlines for height sampling (mirrors CommitLayer).
			TArray<const TArray<FVector>*> CenterLines;
			if (Zones.IsValidIndex(z))
			{
				for (int32 RoadIdx : Zones[z])
				{
					if (const FRoadCurves* C = Curves.Find(RoadIdx)) { CenterLines.Add(&C->Sampled); }
				}
			}
			const double FallbackZ = RoadNetMesh::FirstCenterlineZ(CenterLines);

			for (const FGeneralPolygon2d& GP : ZonePolys[z])
			{
				RingToLoop(GP.GetOuter(), CenterLines, FallbackZ, ZLiftCm, /*bOuter*/true, z, OutLoops);
				for (const FPolygon2d& Hole : GP.GetHoles())
				{
					RingToLoop(Hole, CenterLines, FallbackZ, ZLiftCm, /*bOuter*/false, z, OutLoops);
				}
			}
		}
	}
}
