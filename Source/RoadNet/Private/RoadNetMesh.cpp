// RoadNetMesh.cpp — surface triangulation → FDynamicMesh3 (§10.15).
#include "RoadNetMesh.h"
#include "RoadNetMath.h"
#include "RoadNetLog.h"
#include "ConstrainedDelaunay2.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/MeshNormals.h"

using namespace UE::Geometry;

namespace RoadNetMesh
{
	// Interpolated elevation: project (X,Y) onto every centerline, keep the
	// nearest, and lerp Z along that segment (§10.10). Continuous over slopes,
	// unlike nearest-vertex. Linear scan — fine for moderate zones.
	double SampleHeight(const TArray<const TArray<FVector>*>& CenterLines, double X, double Y, double Fallback)
	{
		const FVector2D Q(X, Y);
		double BestD2 = TNumericLimits<double>::Max();
		double BestZ  = Fallback;

		for (const TArray<FVector>* CLPtr : CenterLines)
		{
			if (!CLPtr) { continue; }
			const TArray<FVector>& CL = *CLPtr;
			if (CL.Num() == 1)
			{
				const double dx = CL[0].X - X, dy = CL[0].Y - Y;
				const double D2 = dx * dx + dy * dy;
				if (D2 < BestD2) { BestD2 = D2; BestZ = CL[0].Z; }
				continue;
			}
			for (int32 i = 0; i + 1 < CL.Num(); ++i)
			{
				const FVector2D A(CL[i].X, CL[i].Y);
				const FVector2D B(CL[i + 1].X, CL[i + 1].Y);
				double T;
				const FVector2D C = RoadNetMath::ClosestOnSegment(A, B, Q, T);
				const double D2 = FVector2D::DistSquared(Q, C);
				if (D2 < BestD2)
				{
					BestD2 = D2;
					BestZ  = FMath::Lerp(CL[i].Z, CL[i + 1].Z, T);
				}
			}
		}
		return BestZ;
	}

	double FirstCenterlineZ(const TArray<const TArray<FVector>*>& CenterLines)
	{
		for (const TArray<FVector>* CLPtr : CenterLines)
		{
			if (CLPtr && CLPtr->Num() > 0) { return (*CLPtr)[0].Z; }
		}
		return 0.0;
	}

	int32 AppendSurfaceMesh(
		const TArray<FGeneralPolygon2d>& Polys,
		const TArray<const TArray<FVector>*>& CenterLines,
		double ZLiftCm,
		FDynamicMesh3& OutMesh)
	{
		const double FallbackZ = FirstCenterlineZ(CenterLines);
		int32 TriCount = 0;

		for (const FGeneralPolygon2d& GP : Polys)
		{
			TArray<FVector2d> Verts2D;
			const TArray<FIndex3i> Tris = ConstrainedDelaunayTriangulateWithVertices<double>(GP, Verts2D);
			if (Tris.Num() == 0 || Verts2D.Num() < 3) { continue; }

			// Append this polygon's vertices with re-projected elevation.
			TArray<int32> VMap;
			VMap.SetNumUninitialized(Verts2D.Num());
			for (int32 i = 0; i < Verts2D.Num(); ++i)
			{
				const FVector2d& P = Verts2D[i];
				const double Z = SampleHeight(CenterLines, P.X, P.Y, FallbackZ) + ZLiftCm;
				VMap[i] = OutMesh.AppendVertex(FVector3d(P.X, P.Y, Z));
			}

			// Append triangles, forcing an upward (+Z) facing winding.
			for (const FIndex3i& T : Tris)
			{
				const FVector2d& A = Verts2D[T.A];
				const FVector2d& B = Verts2D[T.B];
				const FVector2d& C = Verts2D[T.C];
				const double Cross = (B.X - A.X) * (C.Y - A.Y) - (B.Y - A.Y) * (C.X - A.X);
				const FIndex3i Tri = (Cross >= 0.0)
					? FIndex3i(VMap[T.A], VMap[T.B], VMap[T.C])
					: FIndex3i(VMap[T.A], VMap[T.C], VMap[T.B]);
				if (OutMesh.AppendTriangle(Tri) >= 0) { ++TriCount; }
			}
		}

		return TriCount;
	}

	void FinalizeNormals(FDynamicMesh3& Mesh)
	{
		if (Mesh.TriangleCount() == 0) { return; }
		Mesh.EnableAttributes();
		FMeshNormals::QuickComputeVertexNormals(Mesh);
	}
}
