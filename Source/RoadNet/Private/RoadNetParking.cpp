// RoadNetParking.cpp — standard parking-bay generation (street features).
//
// BuildStandardParkingBays turns each road's authored FRoadNetParkingBay into
// (a) an amber bay-surface polygon appended to the parking overlay bank
// (Ctx.ZoneLaneParkPolys, committed with ParkingMaterial) and (b) white painted
// stall-divider lines appended to the white-marking bank (Ctx.ZoneMarkingWhitePolys,
// committed with the existing marking meshing). Because bays live on FRoadDef and
// are rebuilt from the road centreline, they reshape with the road like the
// Edge-tool bulge. No new commit stage or actor is needed — the polygons ride the
// existing parking-overlay and white-marking layers.
#include "RoadNetwork.h"
#include "RoadNetSurface.h"
#include "RoadNetMath.h"
#include "RoadNetLog.h"
#include "Polygon2.h"

using namespace UE::Geometry;

namespace
{
	// Sub-polyline of P over arc window [S0,S1], interpolating the endpoints and
	// keeping every interior vertex (Z carried). CL = cumulative lengths of P.
	void ExtractArcWindow(const TArray<FVector>& P, const TArray<double>& CL,
		double S0, double S1, TArray<FVector>& Out)
	{
		Out.Reset();
		const double Len = CL.Last();
		S0 = FMath::Clamp(S0, 0.0, Len);
		S1 = FMath::Clamp(S1, S0, Len);
		if (S1 - S0 < 1.0) { return; }

		auto SampleAt = [&](double S) -> FVector
		{
			S = FMath::Clamp(S, 0.0, Len);
			int32 seg = 0;
			while (seg + 1 < CL.Num() - 1 && CL[seg + 1] < S) { ++seg; }
			const double segLen = FMath::Max(1e-3, CL[seg + 1] - CL[seg]);
			const double t = FMath::Clamp((S - CL[seg]) / segLen, 0.0, 1.0);
			return FMath::Lerp(P[seg], P[seg + 1], t);
		};

		Out.Add(SampleAt(S0));
		for (int32 i = 0; i < P.Num(); ++i)
		{
			if (CL[i] > S0 + 1e-3 && CL[i] < S1 - 1e-3) { Out.Add(P[i]); }
		}
		Out.Add(SampleAt(S1));
	}

	// Thin rectangle (a painted line) from A to B with half-width HW → CCW poly.
	bool MakeLineQuad(const FVector2D& A, const FVector2D& B, double HW, FGeneralPolygon2d& Out)
	{
		FVector2D Dir = B - A;
		if (!Dir.Normalize()) { return false; }
		const FVector2D Perp(Dir.Y, -Dir.X);
		TArray<FVector2d> Loop;
		Loop.Emplace(A.X + Perp.X * HW, A.Y + Perp.Y * HW);
		Loop.Emplace(B.X + Perp.X * HW, B.Y + Perp.Y * HW);
		Loop.Emplace(B.X - Perp.X * HW, B.Y - Perp.Y * HW);
		Loop.Emplace(A.X - Perp.X * HW, A.Y - Perp.Y * HW);
		FPolygon2d Poly(Loop);
		if (Poly.VertexCount() < 3 || FMath::Abs(Poly.SignedArea()) < 0.5) { return false; }
		if (Poly.IsClockwise()) { Poly.Reverse(); }
		Out.SetOuter(Poly);
		return true;
	}
}

void URoadNetwork::BuildStandardParkingBays(FRoadNetRebuildContext& Ctx) const
{
	const int32 NumZones = Ctx.Zones.Num();
	if (NumZones == 0) { return; }

	// Road → zone lookup so bay polys land in the right per-zone bucket.
	TMap<int32, int32> RoadZone;
	for (int32 z = 0; z < NumZones; ++z)
	{
		for (int32 RoadIdx : Ctx.Zones[z]) { RoadZone.Add(RoadIdx, z); }
	}

	// Ensure the destination banks exist and are per-zone sized.
	if (Ctx.ZoneLaneParkPolys.Num()     != NumZones) { Ctx.ZoneLaneParkPolys.SetNum(NumZones); }
	if (Ctx.ZoneMarkingWhitePolys.Num() != NumZones) { Ctx.ZoneMarkingWhitePolys.SetNum(NumZones); }

	constexpr double kLineHalfWidthCm = 6.0; // ~12 cm painted stall line
	int32 BayCount = 0, StallLines = 0;

	for (int32 RoadIdx = 0; RoadIdx < Roads.Num(); ++RoadIdx)
	{
		const FRoadDef& R = Roads[RoadIdx];
		if (R.ParkingBays.Num() == 0) { continue; }

		const FRoadCurves* C = Ctx.Curves.Find(RoadIdx);
		if (!C || C->Sampled.Num() < 2) { continue; }
		const int32* ZonePtr = RoadZone.Find(RoadIdx);
		if (!ZonePtr) { continue; }
		const int32 z = *ZonePtr;

		const TArray<FVector>& P = C->Sampled;
		TArray<double> CL;
		RoadNetMath::CumulativeLength(P, CL);
		const double Len = CL.Last();
		if (Len < 1.0) { continue; }

		const double Half = (double)R.Lanes.HalfWidthCm();

		for (const FRoadNetParkingBay& Bay : R.ParkingBays)
		{
			const double Depth  = FMath::Max(50.f, Bay.StallDepthCm);
			const double Stall  = FMath::Max(50.f, Bay.StallWidthCm);
			const double S0     = FMath::Clamp((double)Bay.StartArcCm, 0.0, Len);
			const double S1     = (Bay.LengthCm > 0.f) ? FMath::Min(Len, S0 + (double)Bay.LengthCm) : Len;
			if (S1 - S0 < 1.0) { continue; }

			const double Sign = (Bay.Side == ERoadNetSide::Left) ? -1.0 : +1.0;
			const double InnerOff = Sign * Half;
			const double OuterOff = Sign * (Half + Depth);

			// Windowed centreline for this bay.
			TArray<FVector> Win;
			ExtractArcWindow(P, CL, S0, S1, Win);
			if (Win.Num() < 2) { continue; }

			// (a) Bay surface: the ribbon between the carriageway edge and the
			// outer stall depth → parking overlay bank (amber / ParkingMaterial).
			FGeneralPolygon2d BayPoly;
			if (RoadNetSurface::BuildSideRibbon(Win, InnerOff, OuterOff, BayPoly))
			{
				Ctx.ZoneLaneParkPolys[z].Add(MoveTemp(BayPoly));
				++BayCount;
			}

			// (b) Stall divider lines. Angle to the kerb: 90° for parallel /
			// perpendicular, Bay.AngleDeg for angled. Along-kerb spacing widens as
			// the angle shrinks so stalls keep their measured width.
			const double ThetaDeg = (Bay.Layout == ERoadNetParkingLayout::Angled)
				? FMath::Clamp((double)Bay.AngleDeg, 30.0, 90.0) : 90.0;
			const double Theta = FMath::DegreesToRadians(ThetaDeg);
			const double SinT = FMath::Max(0.5, FMath::Sin(Theta));
			const double CosT = FMath::Cos(Theta);
			const double AlongSpacing = FMath::Max(60.0,
				(Bay.Layout == ERoadNetParkingLayout::Angled) ? (Stall / SinT) : Stall);
			const double DividerLen = Depth / SinT;

			TArray<double> WinCL;
			RoadNetMath::CumulativeLength(Win, WinCL);
			const double WinLen = WinCL.Last();

			auto SampleWin = [&](double S, FVector2D& OutP, FVector2D& OutTan)
			{
				S = FMath::Clamp(S, 0.0, WinLen);
				int32 seg = 0;
				while (seg + 1 < WinCL.Num() - 1 && WinCL[seg + 1] < S) { ++seg; }
				const double segLen = FMath::Max(1e-3, WinCL[seg + 1] - WinCL[seg]);
				const double t = FMath::Clamp((S - WinCL[seg]) / segLen, 0.0, 1.0);
				const FVector Pos = FMath::Lerp(Win[seg], Win[seg + 1], t);
				OutP = FVector2D(Pos.X, Pos.Y);
				FVector2D Tan(Win[seg + 1].X - Win[seg].X, Win[seg + 1].Y - Win[seg].Y);
				if (!Tan.Normalize()) { Tan = FVector2D(1.0, 0.0); }
				OutTan = Tan;
			};

			// One divider at each stall boundary across the whole window.
			for (double S = 0.0; S <= WinLen + 1e-3; S += AlongSpacing)
			{
				FVector2D Base, Tan;
				SampleWin(S, Base, Tan);
				const FVector2D Nout(Tan.Y * Sign, -Tan.X * Sign); // outward normal (side-aware)
				// Inner edge point (at the carriageway edge) and the divider dir.
				const FVector2D Inner = Base + Nout * Half;
				const FVector2D Fwd = Tan; // along-kerb (for the angled slant)
				const FVector2D Dir = (Nout * SinT + Fwd * CosT).GetSafeNormal();
				const FVector2D End = Inner + Dir * DividerLen;

				FGeneralPolygon2d Line;
				if (MakeLineQuad(Inner, End, kLineHalfWidthCm, Line))
				{
					Ctx.ZoneMarkingWhitePolys[z].Add(MoveTemp(Line));
					++StallLines;
				}
			}
		}
	}

	if (BayCount > 0)
	{
		UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] BuildStandardParkingBays: %d bays, %d stall lines."),
			BayCount, StallLines);
	}
}
