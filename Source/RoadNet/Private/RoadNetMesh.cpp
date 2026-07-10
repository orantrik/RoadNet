// RoadNetMesh.cpp — surface triangulation → FDynamicMesh3 (§10.15).
#include "RoadNetMesh.h"
#include "RoadNetMath.h"
#include "RoadNetLog.h"
#include "ConstrainedDelaunay2.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "DynamicMesh/MeshNormals.h"

using namespace UE::Geometry;

namespace RoadNetMesh
{
	void FCenterlineHeightField::Build(const TArray<const TArray<FVector>*>& CenterLines, double InCellCm)
	{
		Segs.Reset();
		Grid.Reset();
		CellCm = FMath::Max(1.0, InCellCm);
		FallbackZ = FirstCenterlineZ(CenterLines);

		int32 Count = 0;
		for (const TArray<FVector>* CLPtr : CenterLines)
		{
			if (CLPtr) { Count += FMath::Max(0, CLPtr->Num() - 1); }
		}
		Segs.Reserve(Count);
		for (const TArray<FVector>* CLPtr : CenterLines)
		{
			if (!CLPtr) { continue; }
			const TArray<FVector>& CL = *CLPtr;
			for (int32 i = 0; i + 1 < CL.Num(); ++i)
			{
				Segs.Add({ FVector2D(CL[i].X, CL[i].Y), FVector2D(CL[i + 1].X, CL[i + 1].Y),
					CL[i].Z, CL[i + 1].Z });
			}
		}
		if (Segs.Num() == 0) { return; }

		Grid.Reserve(Segs.Num() * 2);
		auto Floor = [this](double V) { return (int32)FMath::FloorToInt(V / CellCm); };
		for (int32 s = 0; s < Segs.Num(); ++s)
		{
			const FSeg& G = Segs[s];
			const int32 X0 = Floor(FMath::Min(G.A.X, G.B.X)), X1 = Floor(FMath::Max(G.A.X, G.B.X));
			const int32 Y0 = Floor(FMath::Min(G.A.Y, G.B.Y)), Y1 = Floor(FMath::Max(G.A.Y, G.B.Y));
			for (int32 cx = X0; cx <= X1; ++cx)
			{
				for (int32 cy = Y0; cy <= Y1; ++cy) { Grid.Add(FIntPoint(cx, cy), s); }
			}
		}
	}

	double FCenterlineHeightField::SampleHeight(double X, double Y, double Fallback) const
	{
		if (Segs.Num() == 0) { return Fallback; }
		const FVector2D Q(X, Y);
		const int32 CX = (int32)FMath::FloorToInt(X / CellCm);
		const int32 CY = (int32)FMath::FloorToInt(Y / CellCm);

		// Smooth inverse-distance blend of nearby centerlines (§8.5 plane/blend).
		// The old code snapped Z to the single NEAREST centerline, so where two
		// roads meet at a junction at different heights the surface stepped along
		// the "which road is closest" seam and thin marking polygons scattered.
		// Blending nearby segments makes the junction transition smoothly; on a
		// lone road its own segments dominate so grade is preserved.
		constexpr double SoftCm  = 120.0;              // softening (near-field flat weight)
		constexpr double Soft2   = SoftCm * SoftCm;
		constexpr double BlendCm = 700.0;              // only blend roads within this
		constexpr double Blend2  = BlendCm * BlendCm;

		double SumW = 0.0, SumWZ = 0.0;
		double BestD2 = TNumericLimits<double>::Max();
		double BestZ  = Fallback;

		TArray<int32> Bucket;
		auto TestCell = [&](int32 cx, int32 cy)
		{
			Bucket.Reset();
			Grid.MultiFind(FIntPoint(cx, cy), Bucket);
			for (int32 s : Bucket)
			{
				const FSeg& G = Segs[s];
				double T;
				const FVector2D C = RoadNetMath::ClosestOnSegment(G.A, G.B, Q, T);
				const double D2 = FVector2D::DistSquared(Q, C);
				const double Z  = FMath::Lerp(G.Za, G.Zb, T);
				if (D2 < BestD2) { BestD2 = D2; BestZ = Z; }
				if (D2 <= Blend2)
				{
					const double W = 1.0 / (D2 + Soft2);
					SumW += W; SumWZ += W * Z;
				}
			}
		};

		// Cover every cell that can hold a segment within BlendCm of Q.
		const int32 Rings = (int32)FMath::CeilToInt(BlendCm / CellCm) + 1;
		for (int32 R = 0; R <= Rings; ++R)
		{
			if (R == 0) { TestCell(CX, CY); }
			else
			{
				for (int32 dx = -R; dx <= R; ++dx)
				{
					TestCell(CX + dx, CY - R);
					TestCell(CX + dx, CY + R);
				}
				for (int32 dy = -R + 1; dy <= R - 1; ++dy)
				{
					TestCell(CX - R, CY + dy);
					TestCell(CX + R, CY + dy);
				}
			}
		}

		// Blended height when anything was within range, else the nearest hit.
		return (SumW > 0.0) ? (SumWZ / SumW) : BestZ;
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
		FDynamicMesh3& OutMesh,
		const TFunction<FVector3f(double, double)>* VertexColorFn,
		bool bComputeUVs,
		double UVUnitCm,
		bool bGradientNormals,
		bool bWorldUVs)
	{
		FCenterlineHeightField Field;
		Field.Build(CenterLines);
		const double FallbackZ = Field.FirstZ();
		int32 TriCount = 0;

		if (VertexColorFn && !OutMesh.HasVertexColors())
		{
			OutMesh.EnableVertexColors(FVector3f(0.15f, 0.15f, 0.16f));
		}

		FDynamicMeshUVOverlay* UVo = nullptr;
		FDynamicMeshNormalOverlay* No = nullptr;
		if (bComputeUVs || bGradientNormals)
		{
			OutMesh.EnableAttributes();
			if (bComputeUVs)       { UVo = OutMesh.Attributes()->PrimaryUV(); }
			if (bGradientNormals)  { No  = OutMesh.Attributes()->PrimaryNormals(); }
		}
		const double UVScale = (UVUnitCm > KINDA_SMALL_NUMBER) ? (1.0 / UVUnitCm) : 0.01;

		// Nearest-centreline projection (offset + arc length) for UVs.
		auto ProjectNearest = [&CenterLines](const FVector2D& Q) -> RoadNetMath::FProjectResult
		{
			RoadNetMath::FProjectResult Best;
			for (const TArray<FVector>* CLPtr : CenterLines)
			{
				if (!CLPtr || CLPtr->Num() < 2) { continue; }
				const RoadNetMath::FProjectResult PR = RoadNetMath::ProjectToPolyline(*CLPtr, Q);
				if (PR.Distance < Best.Distance) { Best = PR; }
			}
			return Best;
		};

		// Smooth normal from the blended height-field gradient (central diff).
		auto GradientNormal = [&Field, FallbackZ](double X, double Y) -> FVector3f
		{
			constexpr double D = 75.0;
			const double Zpx = Field.SampleHeight(X + D, Y, FallbackZ);
			const double Zmx = Field.SampleHeight(X - D, Y, FallbackZ);
			const double Zpy = Field.SampleHeight(X, Y + D, FallbackZ);
			const double Zmy = Field.SampleHeight(X, Y - D, FallbackZ);
			FVector3d N(-(Zpx - Zmx) / (2.0 * D), -(Zpy - Zmy) / (2.0 * D), 1.0);
			N.Normalize();
			return FVector3f((float)N.X, (float)N.Y, (float)N.Z);
		};

		for (const FGeneralPolygon2d& GP : Polys)
		{
			TArray<FVector2d> Verts2D;
			const TArray<FIndex3i> Tris = ConstrainedDelaunayTriangulateWithVertices<double>(GP, Verts2D);
			if (Tris.Num() == 0 || Verts2D.Num() < 3) { continue; }

			// Append this polygon's vertices with re-projected elevation, plus a
			// UV element and normal element per vertex (kept in parallel arrays).
			TArray<int32> VMap, UVMap, NMap;
			VMap.SetNumUninitialized(Verts2D.Num());
			if (UVo) { UVMap.SetNumUninitialized(Verts2D.Num()); }
			if (No)  { NMap.SetNumUninitialized(Verts2D.Num()); }

			for (int32 i = 0; i < Verts2D.Num(); ++i)
			{
				const FVector2d& P = Verts2D[i];
				const double Z = Field.SampleHeight(P.X, P.Y, FallbackZ) + ZLiftCm;
				const int32 Vid = OutMesh.AppendVertex(FVector3d(P.X, P.Y, Z));
				VMap[i] = Vid;
				if (VertexColorFn) { OutMesh.SetVertexColor(Vid, (*VertexColorFn)(P.X, P.Y)); }

				if (UVo)
				{
					if (bWorldUVs)
					{
						// World-planar tiling (grass islands): no centreline mirror seam.
						UVMap[i] = UVo->AppendElement(FVector2f(
							(float)(P.X * UVScale), (float)(P.Y * UVScale)));
					}
					else
					{
						const RoadNetMath::FProjectResult PR = ProjectNearest(FVector2D(P.X, P.Y));
						UVMap[i] = UVo->AppendElement(FVector2f(
							(float)(PR.Offset * UVScale), (float)(PR.AlongDist * UVScale)));
					}
				}
				if (No) { NMap[i] = No->AppendElement(GradientNormal(P.X, P.Y)); }
			}

			// Append triangles, forcing an upward (+Z) facing winding, and mirror
			// that winding into the UV/normal overlays so corners line up.
			for (const FIndex3i& T : Tris)
			{
				const FVector2d& A = Verts2D[T.A];
				const FVector2d& B = Verts2D[T.B];
				const FVector2d& C = Verts2D[T.C];
				const double Cross = (B.X - A.X) * (C.Y - A.Y) - (B.Y - A.Y) * (C.X - A.X);
				const bool bCCW = (Cross >= 0.0);
				const int32 I0 = T.A, I1 = bCCW ? T.B : T.C, I2 = bCCW ? T.C : T.B;
				const int32 Tid = OutMesh.AppendTriangle(FIndex3i(VMap[I0], VMap[I1], VMap[I2]));
				if (Tid >= 0)
				{
					++TriCount;
					if (UVo) { UVo->SetTriangle(Tid, FIndex3i(UVMap[I0], UVMap[I1], UVMap[I2])); }
					if (No)  { No->SetTriangle(Tid, FIndex3i(NMap[I0], NMap[I1], NMap[I2])); }
				}
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
