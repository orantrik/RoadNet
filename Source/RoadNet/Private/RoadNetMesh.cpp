// RoadNetMesh.cpp — surface triangulation → FDynamicMesh3 (§10.15).
#include "RoadNetMesh.h"
#include "RoadNetMath.h"
#include "RoadNetLog.h"
#include "ConstrainedDelaunay2.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "DynamicMesh/MeshNormals.h"
#include "HAL/IConsoleManager.h"

using namespace UE::Geometry;

// Junction height-blend temperature (cm). SampleHeight resolves overlapping
// roads with a distance-weighted SOFT-MAX of their heights, i.e. the HIGHEST
// road wins by default (so a connecting road rises/falls to meet the through
// road, and the through road is NOT dragged down). Tau controls how sharp that
// max is: small → a crisp crown at the junction (lower road excluded quickly);
// large → the old smooth average (both roads pulled together). 50 cm keeps
// through roads put while ramping the minor road over the blend radius.
static TAutoConsoleVariable<float> CVarRoadNetHeightTauCm(
	TEXT("roadnet.HeightBlendTauCm"),
	50.0f,
	TEXT("RoadNet junction height soft-max temperature (cm): highest road wins. Small = crisp crown (through road stays, connecting road ramps to it); large = smooth average of both. Default 50."),
	ECVF_Default);

// Traffic handedness for the whole network. Registered here in the always-loaded
// runtime module so the OSM Roads panel can push it without a module link; the
// authority is URoadNetwork::bDriveOnLeft and this only seeds it.
static TAutoConsoleVariable<int32> CVarRoadNetDriveOnLeft(
	TEXT("roadnet.DriveOnLeft"),
	0,
	TEXT("1 = drive on the left (UK/JP/AU), 0 = drive on the right. Moves forward traffic to the other side, swaps the stop-bar/zebra half at junctions, and paints the centre line white instead of yellow."),
	ECVF_Default);

// Active RoadNet Draw sub-tool (see ERoadNetDrawTool): 0=Draw 1=Points 2=Lanes
// 3=Junctions 4=Edge. Registered here in the always-loaded runtime module so the
// OSM Roads panel (which writes it) and the RoadNetEditor mode (which reads it)
// can both reach it by name without a module dependency between them.
static TAutoConsoleVariable<int32> CVarRoadNetDrawTool(
	TEXT("roadnet.DrawTool"),
	0,
	TEXT("Active RoadNet Draw sub-tool: 0=Draw, 1=Points, 2=Lanes, 3=Junctions, 4=Edge. Exactly one tool is live at a time so clicks/hotkeys are unambiguous."),
	ECVF_Default);

// Shape the Draw sub-tool lays down (see ERoadNetDrawShape): 0=Freehand
// (click-per-point polyline), 1=Roundabout (centre+radius closed circle),
// 2=Curve (start+end circular arc). Written by the OSM Roads panel, read by the
// RoadNetEditor mode — registered in the always-loaded runtime module so both
// reach it by name with no module link.
static TAutoConsoleVariable<int32> CVarRoadNetDrawShape(
	TEXT("roadnet.DrawShape"),
	0,
	TEXT("Draw sub-tool shape: 0=Freehand, 1=Roundabout (centre+radius), 2=Curve (start+end arc), 3=FreeCurve (origin+dest+apex bezier)."),
	ECVF_Default);

// Central angle (degrees) the Curve shape bends by, e.g. 90/45/25. A 90 arc
// between the two clicked endpoints is a quarter circle; smaller = shallower.
static TAutoConsoleVariable<int32> CVarRoadNetDrawAngleDeg(
	TEXT("roadnet.DrawAngleDeg"),
	90,
	TEXT("Curve-shape arc angle in degrees (how far the road bends between the two clicks). Typical: 90, 45, 25."),
	ECVF_Default);

// Standard parking-bay layout used by the Lanes-tool 'P' authoring hotkey (see
// ERoadNetParkingLayout): 0=Parallel 1=Perpendicular 2=Angled. Written by the
// OSM Roads panel, read by the RoadNetEditor mode — registered here in the
// always-loaded runtime module so both reach it by name with no module link.
static TAutoConsoleVariable<int32> CVarRoadNetParkingLayout(
	TEXT("roadnet.ParkingLayout"),
	0,
	TEXT("Standard parking-bay layout for the Lanes-tool 'P' hotkey: 0=Parallel, 1=Perpendicular, 2=Angled."),
	ECVF_Default);

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
		// RoadId = the centreline's index in this array. Two distinct centrelines
		// are two distinct roads, so SampleHeight can resolve them per-road (each
		// road keeps its own smooth grade; only genuine overlaps are blended).
		int32 RoadId = 0;
		for (const TArray<FVector>* CLPtr : CenterLines)
		{
			if (!CLPtr) { ++RoadId; continue; }
			const TArray<FVector>& CL = *CLPtr;
			double AccLen = 0.0;
			for (int32 i = 0; i + 1 < CL.Num(); ++i)
			{
				const FVector2D A(CL[i].X, CL[i].Y), B(CL[i + 1].X, CL[i + 1].Y);
				const double L = FVector2D::Distance(A, B);
				Segs.Add({ A, B, CL[i].Z, CL[i + 1].Z, RoadId, AccLen, L, i });
				AccLen += L;
			}
			++RoadId;
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

	RoadNetMath::FProjectResult FCenterlineHeightField::ProjectNearest(double X, double Y) const
	{
		RoadNetMath::FProjectResult Best;
		if (Segs.Num() == 0) { return Best; }
		const FVector2D Q(X, Y);
		const int32 CX = (int32)FMath::FloorToInt(X / CellCm);
		const int32 CY = (int32)FMath::FloorToInt(Y / CellCm);

		TArray<int32, TInlineAllocator<32>> Bucket;
		auto TestSeg = [&](int32 s)
		{
			const FSeg& G = Segs[s];
			double T;
			const FVector2D C = RoadNetMath::ClosestOnSegment(G.A, G.B, Q, T);
			const double D = FVector2D::Distance(Q, C);
			const double Along = G.AlongA + T * G.Len;
			// Same seam-safe tie-break as ProjectToPolyline so UVs don't flip
			// between two segments sharing a vertex.
			if (D < Best.Distance - 1e-4 ||
				(FMath::Abs(D - Best.Distance) <= 1e-4 && Along < Best.AlongDist))
			{
				Best.Distance  = D;
				Best.AlongDist = Along;
				Best.Offset    = (G.Len > KINDA_SMALL_NUMBER)
					? RoadNetMath::Cross2D((G.B - G.A) / G.Len, Q - G.A) : 0.0;
				Best.Segment   = G.Local;
				Best.Point     = C;
			}
		};

		// Grow the search ring until the best hit is closer than the ring edge:
		// anything still unvisited is at least Ring*CellCm away, so at that point
		// no farther cell can beat it. The cap bounds the walk over empty space.
		constexpr int32 MaxRing = 16;
		for (int32 Ring = 0; Ring <= MaxRing; ++Ring)
		{
			for (int32 dy = -Ring; dy <= Ring; ++dy)
			{
				for (int32 dx = -Ring; dx <= Ring; ++dx)
				{
					// Ring shell only; interior cells were scanned on earlier passes.
					if (Ring > 0 && FMath::Max(FMath::Abs(dx), FMath::Abs(dy)) != Ring) { continue; }
					Bucket.Reset();
					Grid.MultiFind(FIntPoint(CX + dx, CY + dy), Bucket);
					for (int32 s : Bucket) { TestSeg(s); }
				}
			}
			if (Best.Segment != INDEX_NONE && Best.Distance <= (double)Ring * CellCm) { break; }
		}
		// Nothing within the capped neighbourhood (isolated island far from any
		// centreline): fall back to the exhaustive scan so the answer is exact.
		if (Best.Segment == INDEX_NONE)
		{
			for (int32 s = 0; s < Segs.Num(); ++s) { TestSeg(s); }
		}
		return Best;
	}

	double FCenterlineHeightField::SampleHeight(double X, double Y, double Fallback) const
	{
		if (Segs.Num() == 0) { return Fallback; }
		const FVector2D Q(X, Y);
		const int32 CX = (int32)FMath::FloorToInt(X / CellCm);
		const int32 CY = (int32)FMath::FloorToInt(Y / CellCm);

		// Resolve height PER ROAD, then combine roads with a distance-weighted
		// SOFT-MAX (highest wins). Two goals:
		//  1) SMOOTH — on a lone road only its OWN nearest segment contributes, so
		//     the surface is exactly that road's already grade-smoothed centreline
		//     (no cross-segment averaging = no washboard waves).
		//  2) JUNCTIONS — where roads overlap, the HIGHEST road wins, so the
		//     through/original road keeps its height and the connecting road
		//     rises/falls to meet it (never a dip carved into the through road).
		constexpr double SoftCm  = 120.0;              // near-field softening (flat weight)
		constexpr double Soft2   = SoftCm * SoftCm;
		constexpr double BlendCm = 700.0;              // roads within this can blend
		constexpr double Blend2  = BlendCm * BlendCm;

		// Nearest hit PER road within the blend radius (few roads meet at a point,
		// so an inline array + linear scan avoids any per-sample heap allocation).
		struct FRoadHit { int32 Road; double D2; double Z; };
		TArray<FRoadHit, TInlineAllocator<8>> Hits;
		double BestD2 = TNumericLimits<double>::Max();
		double BestZ  = Fallback;

		TArray<int32, TInlineAllocator<32>> Bucket;
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
				if (D2 > Blend2) { continue; }

				// Keep only the closest segment of each road.
				bool bFound = false;
				for (FRoadHit& Hh : Hits)
				{
					if (Hh.Road == G.Road)
					{
						if (D2 < Hh.D2) { Hh.D2 = D2; Hh.Z = Z; }
						bFound = true;
						break;
					}
				}
				if (!bFound) { Hits.Add({ G.Road, D2, Z }); }
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

		if (Hits.Num() == 0) { return BestZ; }          // nothing in range → nearest
		if (Hits.Num() == 1) { return Hits[0].Z; }      // lone road → its own grade

		// Distance-weighted soft-max across roads: w = invDist · exp((Z-Zmax)/Tau).
		// exp() term makes the higher road dominate (highest wins); invDist term
		// hands each road its own turf away from the overlap, so the transition is
		// a smooth ramp confined to where the roads actually meet.
		double Zmax = -TNumericLimits<double>::Max();
		for (const FRoadHit& Hh : Hits) { Zmax = FMath::Max(Zmax, Hh.Z); }

		const double Tau = FMath::Max(1.0, (double)CVarRoadNetHeightTauCm.GetValueOnAnyThread());
		double SumW = 0.0, SumWZ = 0.0;
		for (const FRoadHit& Hh : Hits)
		{
			const double W = (1.0 / (Hh.D2 + Soft2)) * FMath::Exp((Hh.Z - Zmax) / Tau);
			SumW += W; SumWZ += W * Hh.Z;
		}
		return (SumW > 0.0) ? (SumWZ / SumW) : Zmax;
	}

	double FirstCenterlineZ(const TArray<const TArray<FVector>*>& CenterLines)
	{
		for (const TArray<FVector>* CLPtr : CenterLines)
		{
			if (CLPtr && CLPtr->Num() > 0) { return (*CLPtr)[0].Z; }
		}
		return 0.0;
	}

#if !UE_BUILD_SHIPPING
	// One-shot self-check: the grid-accelerated ProjectNearest must agree with the
	// exhaustive ProjectToPolyline scan it replaced, on distance, arc length and
	// signed offset. Probes cover a near hit, a point beyond the ring cap (which
	// must fall back to the full scan) and a point past a polyline's end.
	static bool VerifyProjectNearest()
	{
		TArray<FVector> A, B;
		for (int32 i = 0; i <= 40; ++i) { A.Add(FVector(i * 250.0, 0.0, 0.0)); }
		for (int32 i = 0; i <= 40; ++i) { B.Add(FVector(5000.0, (i - 20) * 250.0, 0.0)); }
		const TArray<const TArray<FVector>*> CLs = { &A, &B };
		FCenterlineHeightField F;
		F.Build(CLs);

		const FVector2D Probes[] = {
			FVector2D(1234.0, 300.0),   FVector2D(4800.0,   60.0),
			FVector2D(2500.0, -75.0),   FVector2D(9800.0,  120.0),
			FVector2D(-4000.0, -4000.0), FVector2D(60000.0, 60000.0),
		};
		bool bOk = true;
		for (const FVector2D& Q : Probes)
		{
			RoadNetMath::FProjectResult Want;
			for (const TArray<FVector>* CL : CLs)
			{
				const RoadNetMath::FProjectResult PR = RoadNetMath::ProjectToPolyline(*CL, Q);
				if (PR.Distance < Want.Distance) { Want = PR; }
			}
			const RoadNetMath::FProjectResult Got = F.ProjectNearest(Q.X, Q.Y);
			if (!FMath::IsNearlyEqual(Got.Distance,  Want.Distance,  0.1) ||
				!FMath::IsNearlyEqual(Got.AlongDist, Want.AlongDist, 0.1) ||
				!FMath::IsNearlyEqual(Got.Offset,    Want.Offset,    0.1))
			{
				UE_LOG(LogRoadNet, Warning,
					TEXT("[RoadNet][PROJCHK] (%.0f,%.0f): grid d=%.1f along=%.1f off=%.1f, exhaustive d=%.1f along=%.1f off=%.1f"),
					Q.X, Q.Y, Got.Distance, Got.AlongDist, Got.Offset,
					Want.Distance, Want.AlongDist, Want.Offset);
				bOk = false;
			}
		}
		return bOk;
	}
#endif

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
#if !UE_BUILD_SHIPPING
		static const bool bProjOk = VerifyProjectNearest();
		(void)bProjOk;
#endif
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

		// Nearest-centreline projection (offset + arc length) for UVs, answered
		// off Field's grid. Scanning every centreline here instead made this
		// O(vertices × roads × polyline-points) and stalled city-scale commits.
		auto ProjectNearest = [&Field](const FVector2D& Q) -> RoadNetMath::FProjectResult
		{
			return Field.ProjectNearest(Q.X, Q.Y);
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
