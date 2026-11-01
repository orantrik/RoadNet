// RoadNetMesh.cpp — surface triangulation → FDynamicMesh3 (§10.15).
#include "RoadNetMesh.h"
#include "RoadNetMath.h"
#include "RoadNetLog.h"
#include "ConstrainedDelaunay2.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "DynamicMesh/MeshNormals.h"
#include "HAL/IConsoleManager.h"
#include "HAL/ThreadSafeCounter.h"

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

// Active RoadNet Draw sub-tool (see ERoadNetDrawTool): 0=Draw .. 9=BikeCrossing.
// Registered here in the always-loaded runtime module so the OSM Roads panel
// (which writes it) and the RoadNetEditor mode (which reads it) can both reach
// it by name without a module dependency between them.
static TAutoConsoleVariable<int32> CVarRoadNetDrawTool(
	TEXT("roadnet.DrawTool"),
	0,
	TEXT("Active RoadNet Draw sub-tool: 0=Draw, 1=Points, 2=Lanes, 3=Junctions, 4=Edge, 5=Markings, 6=Crosswalk, 7=Island, 8=CurbBrush, 9=BikeCrossing. Exactly one tool is live at a time so clicks/hotkeys are unambiguous."),
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

// Backs the panel's "Drape on landscape" checkbox for drawn roads.
static TAutoConsoleVariable<int32> CVarRoadNetDrawSmooth(TEXT("roadnet.DrawSmooth"), 0, TEXT("Freehand draw: 0 = straight lines between the clicked points (default). 1 = a smooth curve (natural cubic spline) through every clicked point."), ECVF_Default);
static TAutoConsoleVariable<int32> CVarRoadNetDrawMultiPoint(TEXT("roadnet.DrawMultiPoint"), 1, TEXT("1 = every point of a road is a handle (orange, default). 0 = main points: only each road's start and end are handles (blue), and dragging one stretches the road behind it."), ECVF_Default);
static TAutoConsoleVariable<int32> CVarRoadNetDrawDrape(
	TEXT("roadnet.DrawDrape"),
	1,
	TEXT("1 = a drawn road's in-between points take the landscape height under them (default). 0 = heights run straight between the clicked points."),
	ECVF_Default);

// Automatic turn arrows off the lane graph. ON by default: arrows are now
// anchored on each approach's STOP LINE (URoadNetwork::CommitLaneMarks reads
// Ctx.ApproachStops), which is what LOT"M 813 asks for — the arrow marks the
// lane a driver enters the junction from. It was off while the derivation still
// measured back from the joint centre, which read wrong on most approaches.
// Switching it off changes nothing about hand-placed marks — the two sources are
// independent and both commit.
static TAutoConsoleVariable<int32> CVarRoadNetAutoTurnArrows(
	TEXT("roadnet.AutoTurnArrows"),
	1,
	TEXT("1 = derive turn arrows automatically from the lane graph at junctions (default). 0 = hand-placed marks only. Hand-placed marks are unaffected either way."),
	ECVF_Default);

// Dashed turn guides through a junction (LOT"M 809). ON by default; a switch
// because a junction dense with movements can carry a lot of short dashes, and
// a stylised or low-detail city may not want any of them.
static TAutoConsoleVariable<int32> CVarRoadNetTurnGuides(
	TEXT("roadnet.TurnGuides"),
	1,
	TEXT("1 = paint a dashed white guide line along each junction turn movement, LOT\"M 809 (default). 0 = none."),
	ECVF_Default);

// Which arrow the Markings tool places on click (see ERoadNetMarkKind), or -1
// for the tool's original turn-ROLE filter behaviour (click a lane, T cycles).
// Registered here in the always-loaded runtime module so the OSM Roads panel
// (which writes it) and the RoadNetEditor mode (which reads it) both reach it by
// name with no module link.
static TAutoConsoleVariable<int32> CVarRoadNetMarkKind(
	TEXT("roadnet.MarkKind"),
	-1,
	TEXT("Arrow the Markings tool places on click: 0=Through, 1=Left, 2=Right, 3=UTurn, 4=Through+Left, 5=Through+Right, 6=Left+Right. -1 = no placement, click selects a lane for the turn-role filter instead."),
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
		double T = 0.0;
		const int32 s = NearestSeg(X, Y, T);
		if (s == INDEX_NONE) { return Best; }
		const FSeg& G = Segs[s];
		const FVector2D Q(X, Y);
		Best.Point     = G.A + (G.B - G.A) * T;
		Best.Distance  = FVector2D::Distance(Q, Best.Point);
		Best.AlongDist = G.AlongA + T * G.Len;
		Best.Offset    = (G.Len > KINDA_SMALL_NUMBER)
			? RoadNetMath::Cross2D((G.B - G.A) / G.Len, Q - G.A) : 0.0;
		Best.Segment   = G.Local;
		return Best;
	}

	bool FCenterlineHeightField::NearestFrame(double X, double Y, FVector2D& OutOrigin, FVector2D& OutDir, double& OutAlongAtOrigin) const
	{
		double T = 0.0;
		const int32 s = NearestSeg(X, Y, T);
		if (s == INDEX_NONE || Segs[s].Len <= KINDA_SMALL_NUMBER) { return false; }
		const FSeg& G = Segs[s];
		OutOrigin = G.A;
		OutDir = (G.B - G.A) / G.Len;
		OutAlongAtOrigin = G.AlongA;
		return true;
	}

	int32 FCenterlineHeightField::NearestSeg(double X, double Y, double& OutT) const
	{
		OutT = 0.0;
		if (Segs.Num() == 0) { return INDEX_NONE; }
		const FVector2D Q(X, Y);
		const int32 CX = (int32)FMath::FloorToInt(X / CellCm);
		const int32 CY = (int32)FMath::FloorToInt(Y / CellCm);

		int32 BestSeg = INDEX_NONE;
		double BestD = TNumericLimits<double>::Max(), BestAlong = 0.0;
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
			if (D < BestD - 1e-4 || (FMath::Abs(D - BestD) <= 1e-4 && Along < BestAlong))
			{
				BestD = D; BestAlong = Along; BestSeg = s; OutT = T;
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
			if (BestSeg != INDEX_NONE && BestD <= (double)Ring * CellCm) { break; }
		}
		// Nothing within the capped neighbourhood (isolated island far from any
		// centreline): fall back to the exhaustive scan so the answer is exact.
		if (BestSeg == INDEX_NONE)
		{
			for (int32 s = 0; s < Segs.Num(); ++s) { TestSeg(s); }
		}
		return BestSeg;
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
		// Each road's say fades to zero at the blend radius. Without the fade a higher
		// road took over at full strength the moment it came within range: a step of
		// the whole height difference on a ring around every junction.
		double SumW = 0.0, SumWZ = 0.0, NearD2 = TNumericLimits<double>::Max(), NearZ = Zmax;
		for (const FRoadHit& Hh : Hits)
		{
			const double Fade = 1.0 - FMath::SmoothStep(0.5 * BlendCm, BlendCm, FMath::Sqrt(Hh.D2));
			const double W = Fade * (1.0 / (Hh.D2 + Soft2)) * FMath::Exp((Hh.Z - Zmax) / Tau);
			SumW += W; SumWZ += W * Hh.Z;
			if (Hh.D2 < NearD2) { NearD2 = Hh.D2; NearZ = Hh.Z; }
		}
		return (SumW > 0.0) ? (SumWZ / SumW) : NearZ;
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

	// The skirt's outward facing is derived from the loop winding, which is easy
	// to get backwards — and a backwards wall is invisible in the editor until
	// someone stands in the street and sees straight through the kerb. Extrude
	// one square band and assert every wall faces away from the middle.
	bool VerifySkirtWinding()
	{
		FPolygon2d Square;
		Square.AppendVertex(FVector2d(-100.0, -100.0));
		Square.AppendVertex(FVector2d( 100.0, -100.0));
		Square.AppendVertex(FVector2d( 100.0,  100.0));
		Square.AppendVertex(FVector2d(-100.0,  100.0));   // CCW

		TArray<FGeneralPolygon2d> Polys;
		Polys.Add(FGeneralPolygon2d(Square));

		const TArray<FVector> Line = { FVector(-500.0, 0.0, 0.0), FVector(500.0, 0.0, 0.0) };
		TArray<const TArray<FVector>*> CenterLines;
		CenterLines.Add(&Line);

		FDynamicMesh3 M;
		AppendSurfaceMesh(Polys, CenterLines, /*ZLiftCm*/20.0, M,
			/*VertexColorFn*/nullptr, /*bComputeUVs*/false, /*UVUnitCm*/100.0,
			/*bGradientNormals*/false, /*bWorldUVs*/false, /*bSkirtToGround*/true);

		int32 Walls = 0;
		bool bOk = true;
		for (int32 Tid : M.TriangleIndicesItr())
		{
			const FVector3d N = M.GetTriNormal(Tid);
			if (FMath::Abs(N.Z) > 0.5) { continue; }      // that one is the top face
			++Walls;
			// The square is centred on the origin, so "away from the middle" is
			// just a positive dot with the wall's own position.
			const FVector3d C = M.GetTriCentroid(Tid);
			if (N.X * C.X + N.Y * C.Y <= 0.0) { bOk = false; }
		}
		if (Walls != 8) { bOk = false; }                  // 4 edges x 2 triangles

		if (!bOk)
		{
			UE_LOG(LogRoadNet, Warning,
				TEXT("[RoadNet][SKIRTCHK] skirt walls face inward or are miscounted (%d walls, expected 8)."),
				Walls);
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
		bool bWorldUVs,
		bool bSkirtToGround,
		const FCenterlineHeightField* PrebuiltField)
	{
#if !UE_BUILD_SHIPPING
		static const bool bProjOk = VerifyProjectNearest();
		(void)bProjOk;
		// Not a static initialiser, because the check calls back into this
		// function and a static local would be re-entering its own guarded init.
		// Counter rather than a plain bool so concurrent callers (the commit
		// meshes tiles in parallel) cannot both see it unset: whoever increments
		// to 1 runs the check, the re-entrant inner call sees 2 and skips.
		static FThreadSafeCounter SkirtChecked;
		if (SkirtChecked.Increment() == 1) { VerifySkirtWinding(); }
#endif
		// Only build a field when the caller has not already got one.
		FCenterlineHeightField LocalField;
		if (!PrebuiltField) { LocalField.Build(CenterLines); }
		const FCenterlineHeightField& Field = PrebuiltField ? *PrebuiltField : LocalField;
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

		// Ring hygiene BEFORE triangulation (the latent-space rule), with a hard
		// safety bound: only DEVIATION-SAFE removals. Dropping duplicates (<1 cm)
		// and collinear knots (<1.5 cm off the line) moves the boundary by at
		// most 1.5 cm, so the cleaned outer ring can never cross a hole and the
		// constrained Delaunay stays legal. A spacing floor was tried here and
		// produced map-crossing spike triangles: cutting a 25 cm corner cluster
		// let rings intersect, and a constrained triangulation fed intersecting
		// constraints returns garbage. Packed-cluster removal belongs upstream,
		// on the latent polylines, where a removal can be checked against the
		// plan — never on triangulation input.
		// Heights are sampled only at ring vertices, and the collinear drop is
		// planar: a road straight in plan but graded over a crest would keep just
		// its end vertices and mesh as one flat chord under its kerbs. Re-insert
		// points on the cleaned edges (on the edge, so the boundary is unchanged).
		auto CleanRing = [](const FPolygon2d& In) -> FPolygon2d
		{
			constexpr double MaxEdgeCm = 400.0;
			TArray<FVector2D> V(In.GetVertices());
			RoadNetMath::CleanPolygonRing(V, /*MinSpacingCm*/1.0, /*CollinearTolCm*/1.5);
			TArray<FVector2D> Dense;
			Dense.Reserve(V.Num());
			for (int32 i = 0; i < V.Num(); ++i)
			{
				const FVector2D& A = V[i];
				const FVector2D& B = V[(i + 1) % V.Num()];
				Dense.Add(A);
				const int32 N = FMath::CeilToInt(FVector2D::Distance(A, B) / MaxEdgeCm);
				for (int32 k = 1; k < N; ++k) { Dense.Add(FMath::Lerp(A, B, (double)k / N)); }
			}
			return FPolygon2d(Dense);
		};

		for (const FGeneralPolygon2d& GPSrc : Polys)
		{
			FGeneralPolygon2d GP(CleanRing(GPSrc.GetOuter()));
			for (const FPolygon2d& Hole : GPSrc.GetHoles())
			{
				GP.AddHole(CleanRing(Hole), /*bCheckContainment*/false, /*bCheckOrientation*/false);
			}

			// Keep a triangle only if it lies inside GP. The winding flood the convenience
			// wrapper uses leaks through any constraint edge the triangulator failed to insert,
			// and every convex-hull triangle behind the leak came back as a spike across the map.
			// ponytail: Contains is O(ring size) per triangle; fine at tile-chopped polygon sizes.
			TConstrainedDelaunay2<double> CDT;
			CDT.Add(GP);
			// The points CleanRing re-inserts are collinear only to rounding, so the
			// triangulator can span three of them with a triangle of no plan area: its
			// normal lies flat at a random sign. Real triangles here are far larger,
			// since the ring keeps no knot within 1.5 cm of its neighbours' line.
			constexpr double kMinTwiceAreaCm2 = 0.01;
			CDT.Triangulate([&GP](const TArray<FVector2d>& V, const FIndex3i& T)
			{
				const FVector2d AB = V[T.B] - V[T.A], AC = V[T.C] - V[T.A];
				if (FMath::Abs(AB.X * AC.Y - AB.Y * AC.X) < kMinTwiceAreaCm2) { return false; }
				return GP.Contains((V[T.A] + V[T.B] + V[T.C]) / 3.0);
			});
			const TArray<FVector2d>& Verts2D = CDT.Vertices;
			const TArray<FIndex3i>& Tris = CDT.Triangles;
			if (Tris.Num() == 0 || Verts2D.Num() < 3) { continue; }

			// Append this polygon's vertices with re-projected elevation, plus a
			// UV element and normal element per vertex (kept in parallel arrays).
			TArray<int32> VMap, UVMap, NMap;
			TArray<FVector2f> UVVal;
			VMap.SetNumUninitialized(Verts2D.Num());
			if (UVo) { UVMap.SetNumUninitialized(Verts2D.Num()); UVVal.SetNumUninitialized(Verts2D.Num()); }
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
						UVVal[i] = FVector2f((float)(P.X * UVScale), (float)(P.Y * UVScale));
					}
					else
					{
						const RoadNetMath::FProjectResult PR = ProjectNearest(FVector2D(P.X, P.Y));
						UVVal[i] = FVector2f((float)(PR.Offset * UVScale), (float)(PR.AlongDist * UVScale));
					}
					UVMap[i] = UVo->AppendElement(UVVal[i]);
				}
				if (No) { NMap[i] = No->AppendElement(GradientNormal(P.X, P.Y)); }
			}

			// The nearest-centreline mapping is a near-isometry along a road, but it collapses
			// to a line past a centreline end (end caps, corner arcs, junction cores: every
			// corner clamps to the same station) and smears across a station jump (corners on
			// two different roads, or either side of a closed loop's 0/L seam). A collapsed
			// triangle has no tangent basis, which is the "nearly zero tangents/bi-normals" of
			// the baked mesh and the wrong normal-map shading of the live tile. Such a triangle
			// takes UVs from the straight frame of the segment nearest its centroid instead:
			// the same mapping extended linearly, exact for every triangle. Corners shared by
			// triangles on the same frame share one element, so a junction core is continuous
			// wherever one segment rules it.
			TMap<TTuple<int32, FVector2D, FVector2D>, int32> FrameElem;
			auto FramedUV = [&](int32 i, const FVector2D& O, const FVector2D& Dir, double AlongO) -> int32
			{
				const TTuple<int32, FVector2D, FVector2D> Key(i, O, Dir);
				if (const int32* E = FrameElem.Find(Key)) { return *E; }
				const FVector2D V(Verts2D[i].X - O.X, Verts2D[i].Y - O.Y);
				const int32 E = UVo->AppendElement(FVector2f(
					(float)(RoadNetMath::Cross2D(Dir, V) * UVScale),
					(float)((AlongO + FVector2D::DotProduct(Dir, V)) * UVScale)));
				FrameElem.Add(Key, E);
				return E;
			};
			// Mapped (Offset, Along) is a mirror of the plane: UV area = -XY area * UVScale^2.
			auto IsIsometric = [&](int32 I0, int32 I1, int32 I2) -> bool
			{
				const FVector2d& A = Verts2D[I0];
				const FVector2d& B = Verts2D[I1];
				const FVector2d& C = Verts2D[I2];
				const double XY = (B.X - A.X) * (C.Y - A.Y) - (B.Y - A.Y) * (C.X - A.X);
				const FVector2f UA = UVVal[I0], UB = UVVal[I1], UC = UVVal[I2];
				const double UV = (double)((UB.X - UA.X) * (UC.Y - UA.Y) - (UB.Y - UA.Y) * (UC.X - UA.X));
				const double S2 = UVScale * UVScale;
				if (FMath::Abs(XY) < KINDA_SMALL_NUMBER) { return true; }   // nothing better to give it
				const double Q = -UV / (XY * S2);
				if (Q < 0.5 || Q > 2.0) { return false; }
				const int32 Ix[3] = { I0, I1, I2 };
				for (int32 e = 0; e < 3; ++e)
				{
					const int32 P = Ix[e], R = Ix[(e + 1) % 3];
					const double LXY = FVector2d::Distance(Verts2D[P], Verts2D[R]) * UVScale;
					const double LUV = (double)FVector2f::Distance(UVVal[P], UVVal[R]);
					if (LXY > KINDA_SMALL_NUMBER && (LUV < 0.5 * LXY || LUV > 2.0 * LXY)) { return false; }
				}
				return true;
			};

			// Append triangles front-facing up, and mirror that winding into the
			// UV/normal overlays so corners line up. Unreal is left-handed: a
			// triangle faces +Z when it runs clockwise in plan (GetTriNormal).
			for (const FIndex3i& T : Tris)
			{
				const FVector2d& A = Verts2D[T.A];
				const FVector2d& B = Verts2D[T.B];
				const FVector2d& C = Verts2D[T.C];
				const double Cross = (B.X - A.X) * (C.Y - A.Y) - (B.Y - A.Y) * (C.X - A.X);
				const bool bCCW = (Cross >= 0.0);
				const int32 I0 = T.A, I1 = bCCW ? T.C : T.B, I2 = bCCW ? T.B : T.C;
				const int32 Tid = OutMesh.AppendTriangle(FIndex3i(VMap[I0], VMap[I1], VMap[I2]));
				if (Tid >= 0)
				{
					++TriCount;
					if (UVo)
					{
						FIndex3i UT(UVMap[I0], UVMap[I1], UVMap[I2]);
						FVector2D O, Dir; double AlongO = 0.0;
						const FVector2d Cen = (A + B + C) / 3.0;
						if (!bWorldUVs && !IsIsometric(I0, I1, I2) && Field.NearestFrame(Cen.X, Cen.Y, O, Dir, AlongO))
						{
							UT = FIndex3i(FramedUV(I0, O, Dir, AlongO), FramedUV(I1, O, Dir, AlongO), FramedUV(I2, O, Dir, AlongO));
						}
						UVo->SetTriangle(Tid, UT);
					}
					if (No)  { No->SetTriangle(Tid, FIndex3i(NMap[I0], NMap[I1], NMap[I2])); }
				}
			}

			if (!bSkirtToGround) { continue; }

			// The layer above is a draped SHELL. Raised a kerb height clear of the
			// road it reads as a floating sheet — at eye level there is daylight
			// under its outer edge. Drop a wall from each boundary down past the
			// height field so the band becomes a slab that meets terrain. Going
			// BELOW the field rather than exactly to it matters: the conform pass
			// leaves the ground a centimetre or two off in places, and a wall that
			// stops level with it would show that slack as a gap.
			//
			// ponytail: the underside is left open (no bottom cap). It is never
			// visible from above ground, and capping would roughly double the
			// skirt's triangles across a whole city. Cap here if these layers ever
			// need to cast shadows seen from below.
			constexpr double kSkirtBuryCm = 10.0;

			auto AppendSkirtLoop = [&](const FPolygon2d& Loop)
			{
				const TArray<FVector2d>& LV = Loop.GetVertices();
				const int32 N = LV.Num();
				if (N < 3) { return; }

				// Which way the wall must face depends on which way the loop runs:
				// the outer boundary is CCW and faces away from the band, a hole is
				// CW and faces inward. Reading it from the winding keeps both right
				// without the caller having to say which loop is which.
				const bool bCCW = (Loop.SignedArea() >= 0.0);
				double LoopAcc = 0.0;

				for (int32 i = 0; i < N; ++i)
				{
					const FVector2d& P0 = LV[i];
					const FVector2d& P1 = LV[(i + 1) % N];
					FVector2d E = P1 - P0;
					const double EdgeLen = E.Length();
					if (!E.Normalize()) { continue; } // duplicate vertex: no wall to build
					const double A0 = LoopAcc, A1 = LoopAcc + EdgeLen;
					LoopAcc = A1;

					const double G0 = Field.SampleHeight(P0.X, P0.Y, FallbackZ);
					const double G1 = Field.SampleHeight(P1.X, P1.Y, FallbackZ);
					const double Top0 = G0 + ZLiftCm, Bot0 = G0 - kSkirtBuryCm;
					const double Top1 = G1 + ZLiftCm, Bot1 = G1 - kSkirtBuryCm;

					const int32 T0 = OutMesh.AppendVertex(FVector3d(P0.X, P0.Y, Top0));
					const int32 T1 = OutMesh.AppendVertex(FVector3d(P1.X, P1.Y, Top1));
					const int32 B0 = OutMesh.AppendVertex(FVector3d(P0.X, P0.Y, Bot0));
					const int32 B1 = OutMesh.AppendVertex(FVector3d(P1.X, P1.Y, Bot1));

					if (VertexColorFn)
					{
						const FVector3f C0 = (*VertexColorFn)(P0.X, P0.Y);
						const FVector3f C1 = (*VertexColorFn)(P1.X, P1.Y);
						OutMesh.SetVertexColor(T0, C0); OutMesh.SetVertexColor(B0, C0);
						OutMesh.SetVertexColor(T1, C1); OutMesh.SetVertexColor(B1, C1);
					}

					// Wall UVs run along the loop and up its face, so a kerb texture tiles
					// by real length. U is the distance walked round the loop, never a
					// centreline station: a wall across an end cap or a corner arc has both
					// ends at the same station, and a quad with one U has no tangent basis.
					int32 UT0 = 0, UT1 = 0, UB0 = 0, UB1 = 0;
					if (UVo)
					{
						UT0 = UVo->AppendElement(FVector2f((float)(A0 * UVScale), (float)(Top0 * UVScale)));
						UT1 = UVo->AppendElement(FVector2f((float)(A1 * UVScale), (float)(Top1 * UVScale)));
						UB0 = UVo->AppendElement(FVector2f((float)(A0 * UVScale), (float)(Bot0 * UVScale)));
						UB1 = UVo->AppendElement(FVector2f((float)(A1 * UVScale), (float)(Bot1 * UVScale)));
					}

					// Flat horizontal normal per wall, so the top surface keeps its
					// smooth gradient shading and the kerb edge stays a hard crease.
					int32 NT0 = 0, NT1 = 0, NB0 = 0, NB1 = 0;
					if (No)
					{
						const FVector3f Out = bCCW
							? FVector3f((float)E.Y, (float)-E.X, 0.f)
							: FVector3f((float)-E.Y, (float)E.X, 0.f);
						NT0 = No->AppendElement(Out); NT1 = No->AppendElement(Out);
						NB0 = No->AppendElement(Out); NB1 = No->AppendElement(Out);
					}

					// Outward winding, derived above from the loop direction.
					const FIndex3i Q1 = bCCW ? FIndex3i(0, 3, 2) : FIndex3i(0, 2, 3); // T0,B1,B0
					const FIndex3i Q2 = bCCW ? FIndex3i(0, 1, 3) : FIndex3i(0, 3, 1); // T0,T1,B1
					const int32  V[4] = { T0, T1, B0, B1 };
					const int32  U[4] = { UT0, UT1, UB0, UB1 };
					const int32  Nn[4] = { NT0, NT1, NB0, NB1 };
					for (const FIndex3i& Q : { Q1, Q2 })
					{
						const int32 Tid = OutMesh.AppendTriangle(
							FIndex3i(V[Q.A], V[Q.B], V[Q.C]));
						if (Tid < 0) { continue; }
						++TriCount;
						if (UVo) { UVo->SetTriangle(Tid, FIndex3i(U[Q.A], U[Q.B], U[Q.C])); }
						if (No)  { No->SetTriangle(Tid, FIndex3i(Nn[Q.A], Nn[Q.B], Nn[Q.C])); }
					}
				}
			};

			AppendSkirtLoop(GP.GetOuter());
			for (const FPolygon2d& Hole : GP.GetHoles()) { AppendSkirtLoop(Hole); }
		}

		return TriCount;
	}

	void FinalizeNormals(FDynamicMesh3& Mesh)
	{
		if (Mesh.TriangleCount() == 0) { return; }
		Mesh.EnableAttributes();
		FMeshNormals::QuickComputeVertexNormals(Mesh);
	}

	int32 RepairNormals(FDynamicMesh3& Mesh)
	{
		if (Mesh.TriangleCount() == 0) { return 0; }
		if (!Mesh.HasAttributes()) { Mesh.EnableAttributes(); }
		FDynamicMeshNormalOverlay* Overlay = Mesh.Attributes()->PrimaryNormals();
		if (!Overlay) { return 0; }

		// Computed lazily: a healthy mesh never needs it, and this runs per tile.
		FMeshNormals VertexNormals(&Mesh);
		bool bComputed = false;
		auto VertexNormal = [&](int32 Vid) -> FVector3f
		{
			if (!bComputed)
			{
				VertexNormals.ComputeVertexNormals();
				bComputed = true;
			}
			// A vertex whose triangles are all degenerate averages to zero; writing that back
			// would leave the defect in place. Road layers face up, so up is the safe answer.
			const FVector3f N = Mesh.IsVertex(Vid) ? (FVector3f)VertexNormals[Vid] : FVector3f::UnitZ();
			return N.SquaredLength() >= 0.25f ? N : FVector3f::UnitZ();
		};

		int32 Fixed = 0;

		// 1. Garbage elements: zero-length or non-finite.
		for (const int32 Eid : Overlay->ElementIndicesItr())
		{
			const FVector3f N = Overlay->GetElement(Eid);
			if (!FMath::IsFinite(N.X) || !FMath::IsFinite(N.Y) || !FMath::IsFinite(N.Z)
				|| N.SquaredLength() < 0.25f)
			{
				Overlay->SetElement(Eid, VertexNormal(Overlay->GetParentVertex(Eid)));
				++Fixed;
			}
		}

		// 2. Unset triangles: give each corner a vertex-normal element.
		// ponytail: adjacent unset triangles get separate (identical-valued)
		// elements instead of shared ones — correct shading, a few extra
		// elements. Merge via a per-vertex element cache if it ever matters.
		for (const int32 Tid : Mesh.TriangleIndicesItr())
		{
			if (Overlay->IsSetTriangle(Tid)) { continue; }
			const FIndex3i Tri = Mesh.GetTriangle(Tid);
			const FIndex3i Elems(
				Overlay->AppendElement(VertexNormal(Tri.A)),
				Overlay->AppendElement(VertexNormal(Tri.B)),
				Overlay->AppendElement(VertexNormal(Tri.C)));
			Overlay->SetTriangle(Tid, Elems);
			++Fixed;
		}
		return Fixed;
	}

	int32 RepairWinding(FDynamicMesh3& Mesh)
	{
		FDynamicMeshNormalOverlay* Overlay =
			Mesh.HasAttributes() ? Mesh.Attributes()->PrimaryNormals() : nullptr;
		if (!Overlay) { return 0; }

		int32 Flipped = 0;
		for (const int32 Tid : Mesh.TriangleIndicesItr())
		{
			if (!Overlay->IsSetTriangle(Tid)) { continue; }

			FVector3d A, B, C;
			Mesh.GetTriVertices(Tid, A, B, C);
			// Left-handed, as GetTriNormal and FMeshNormals: this is the side the
			// static-mesh renderer treats as front. Un-normalised is fine: only the
			// sign of the dot below is used, and a degenerate triangle gives ~0.
			const FVector3d WindingN = (C - A).Cross(B - A);

			const FIndex3i El = Overlay->GetTriangle(Tid);
			const FVector3f AuthoredN = Overlay->GetElement(El.A)
				+ Overlay->GetElement(El.B) + Overlay->GetElement(El.C);

			if (WindingN.Dot(FVector3d(AuthoredN)) < 0.0)
			{
				Mesh.ReverseTriOrientation(Tid);
				++Flipped;
			}
		}
		return Flipped;
	}

	bool RepairSelfCheck(FString& OutReport)
	{
		// A flat quad in the XY plane; RepairNormals gives it normals matching its winding.
		FDynamicMesh3 M;
		M.EnableAttributes();
		const int32 V0 = M.AppendVertex(FVector3d(0,   0,   0));
		const int32 V1 = M.AppendVertex(FVector3d(100, 0,   0));
		const int32 V2 = M.AppendVertex(FVector3d(100, 100, 0));
		const int32 V3 = M.AppendVertex(FVector3d(0,   100, 0));
		const int32 T0 = M.AppendTriangle(V0, V1, V2);
		const int32 T1 = M.AppendTriangle(V0, V2, V3);

		FDynamicMeshNormalOverlay* Ov = M.Attributes()->PrimaryNormals();
		const bool bHaveOverlay = (Ov != nullptr) && (T0 >= 0) && (T1 >= 0);

		// Defect 2 — no normal elements at all. Both triangles start unset.
		const int32 FixedUnset = RepairNormals(M);
		const bool bUnsetFixed = bHaveOverlay && FixedUnset == 2
			&& Ov->IsSetTriangle(T0) && Ov->IsSetTriangle(T1);

		// Idempotent: a healthy mesh gives the repair nothing to do.
		const bool bNoopNormals = (RepairNormals(M) == 0);

		// Defect 1 — a zero-length element.
		bool bZeroFixed = false;
		if (bHaveOverlay)
		{
			const FIndex3i El = Ov->GetTriangle(T0);
			Ov->SetElement(El.A, FVector3f::ZeroVector);
			const int32 Fixed = RepairNormals(M);
			bZeroFixed = (Fixed == 1) && (Ov->GetElement(El.A).SquaredLength() > 0.25f);
		}

		// Defect 3 — winding against the authored normal. Healthy first...
		const bool bNoopWinding = (RepairWinding(M) == 0);
		// ...then flip one triangle and confirm exactly that one comes back.
		M.ReverseTriOrientation(T1);
		const int32 Flipped = RepairWinding(M);
		bool bWindingFixed = (Flipped == 1);
		if (bWindingFixed)
		{
			FVector3d A, B, C;
			M.GetTriVertices(T1, A, B, C);
			// Restored orientation must face the same way as the authored normal.
			bWindingFixed = ((B - A).Cross(C - A).Z > 0.0) && (RepairWinding(M) == 0);
		}

		const bool bOk = bHaveOverlay && bUnsetFixed && bNoopNormals && bZeroFixed
			&& bNoopWinding && bWindingFixed;

		OutReport = FString::Printf(
			TEXT("MeshRepairSelfCheck: %s | unset-triangles %s | normals-idempotent %s | zero-element %s | winding-idempotent %s | winding-flip %s"),
			bOk ? TEXT("PASS") : TEXT("FAIL"),
			bUnsetFixed   ? TEXT("ok") : TEXT("FAILED"),
			bNoopNormals  ? TEXT("ok") : TEXT("FAILED"),
			bZeroFixed    ? TEXT("ok") : TEXT("FAILED"),
			bNoopWinding  ? TEXT("ok") : TEXT("FAILED"),
			bWindingFixed ? TEXT("ok") : TEXT("FAILED"));
		return bOk;
	}
}
