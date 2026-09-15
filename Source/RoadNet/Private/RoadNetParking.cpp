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
			const double Taper  = FMath::Max(0.0, (double)Bay.TaperCm);
			double S0 = 0.0, S1 = 0.0;
			if (!Bay.ResolveWindow(Len, S0, S1)) { continue; }

			const double Sign = (Bay.Side == ERoadNetSide::Left) ? -1.0 : +1.0;

			// Flat window centreline — the part with full stall depth.
			TArray<FVector> Win;
			ExtractArcWindow(P, CL, S0, S1, Win);
			if (Win.Num() < 2) { continue; }

			// (a) Bay surface → parking overlay bank (amber / ParkingMaterial).
			// The ribbon spans the tapers too and its outer edge follows the SAME
			// Bay.BulgeAt profile the carriageway edge was bulged by in
			// BuildCurves, so the amber fills the inclave exactly.
			{
				const double R0 = FMath::Max(0.0, S0 - Taper);
				const double R1 = FMath::Min(Len, S1 + Taper);
				TArray<FVector> Rib;
				ExtractArcWindow(P, CL, R0, R1, Rib);
				TArray<double> RibCL;
				if (Rib.Num() >= 2)
				{
					RoadNetMath::CumulativeLength(Rib, RibCL);
					TArray<double> InnerOff, OuterOff;
					InnerOff.SetNumUninitialized(Rib.Num());
					OuterOff.SetNumUninitialized(Rib.Num());
					for (int32 i = 0; i < Rib.Num(); ++i)
					{
						InnerOff[i] = Sign * Half;
						OuterOff[i] = Sign * (Half + Bay.BulgeAt(R0 + RibCL[i], Len));
					}
					TArray<FVector> In3, Out3;
					RoadNetMath::OffsetPolylineVariable(Rib, InnerOff, In3);
					RoadNetMath::OffsetPolylineVariable(Rib, OuterOff, Out3);
					if (In3.Num() >= 2 && Out3.Num() == In3.Num())
					{
						TArray<FVector2d> Loop;
						Loop.Reserve(In3.Num() * 2);
						for (int32 i = 0; i < In3.Num(); ++i)       { Loop.Emplace(In3[i].X, In3[i].Y); }
						for (int32 i = Out3.Num() - 1; i >= 0; --i) { Loop.Emplace(Out3[i].X, Out3[i].Y); }
						FPolygon2d Poly(Loop);
						if (Poly.VertexCount() >= 3 && FMath::Abs(Poly.SignedArea()) >= 1.0)
						{
							if (Poly.IsClockwise()) { Poly.Reverse(); }
							FGeneralPolygon2d BayPoly;
							BayPoly.SetOuter(Poly);
							Ctx.ZoneLaneParkPolys[z].Add(MoveTemp(BayPoly));
							++BayCount;
						}
					}
				}
			}

			// Self-check: the inclave only exists if BuildCurves actually bulged
			// the carriageway edge here. Fails loudly if the two ever disagree.
			{
				const TArray<FVector>& Edge = (Sign > 0.0) ? C->LeftEdge : C->RightEdge;
				if (Edge.Num() == P.Num())
				{
					const double Smid = 0.5 * (S0 + S1);
					int32 im = 0;
					while (im + 1 < CL.Num() && CL[im + 1] < Smid) { ++im; }
					const double Reach = FVector::Dist2D(P[im], Edge[im]);
					if (Reach < Half + 0.5 * Depth)
					{
						UE_LOG(LogRoadNet, Warning,
							TEXT("[RoadNet] Parking bay on road %d cut no pocket: carriageway edge reaches %.0f cm at the bay centre, expected ~%.0f cm."),
							RoadIdx, Reach, Half + Depth);
					}
				}
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

			auto SampleWin = [&](double S, FVector2D& OutP, FVector2D& OutTan, double& OutZ)
			{
				S = FMath::Clamp(S, 0.0, WinLen);
				int32 seg = 0;
				while (seg + 1 < WinCL.Num() - 1 && WinCL[seg + 1] < S) { ++seg; }
				const double segLen = FMath::Max(1e-3, WinCL[seg + 1] - WinCL[seg]);
				const double t = FMath::Clamp((S - WinCL[seg]) / segLen, 0.0, 1.0);
				const FVector Pos = FMath::Lerp(Win[seg], Win[seg + 1], t);
				OutP = FVector2D(Pos.X, Pos.Y);
				OutZ = Pos.Z;
				FVector2D Tan(Win[seg + 1].X - Win[seg].X, Win[seg + 1].Y - Win[seg].Y);
				if (!Tan.Normalize()) { Tan = FVector2D(1.0, 0.0); }
				OutTan = Tan;
			};

			// One divider at each stall boundary, across the FLAT window only —
			// the tapers are the entry/exit throat, not a stall.
			for (double S = 0.0; S <= WinLen + 1e-3; S += AlongSpacing)
			{
				FVector2D Base, Tan; double BaseZ = 0.0;
				SampleWin(S, Base, Tan, BaseZ);
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

			// (c) Parked cars, one per whole stall — i.e. at the MIDPOINT between
			// two dividers, not at a divider. The last stall is skipped unless it
			// fits completely, and the tapers get none: a car in the throat blocks
			// the bay's own entrance.
			const int32 NumCarMeshes = ParkingCarMeshes.Num();
			const double Fill = FMath::Clamp((double)ParkingCarFill, 0.0, 1.0);
			if (NumCarMeshes > 0 && Fill > 0.0)
			{
				for (double S = 0.0; S + AlongSpacing <= WinLen + 1e-3; S += AlongSpacing)
				{
					FVector2D Base, Tan; double BaseZ = 0.0;
					SampleWin(S + 0.5 * AlongSpacing, Base, Tan, BaseZ);
					const FVector2D Nout(Tan.Y * Sign, -Tan.X * Sign);
					const FVector2D Inner = Base + Nout * Half;
					const FVector2D Dir = (Nout * SinT + Tan * CosT).GetSafeNormal();

					// Hash the stall's world position, not a running counter: the
					// same stall then keeps the same car (and the same occupancy)
					// no matter which roads a windowed rebuild happens to touch.
					const uint32 Seed = HashCombine(
						GetTypeHash(FMath::RoundToInt(Inner.X)),
						GetTypeHash(FMath::RoundToInt(Inner.Y)));
					if ((double)(Seed % 1000u) / 1000.0 >= Fill) { continue; }

					// Parallel stalls hold the car along the kerb; perpendicular and
					// angled ones hold it along the divider, nose out.
					const bool bParallel = (Bay.Layout == ERoadNetParkingLayout::Parallel);
					const FVector2D Facing = bParallel ? Tan : Dir;
					const FVector2D Centre = bParallel
						? (Inner + Nout * (0.5 * Depth))
						: (Inner + Dir * (0.5 * DividerLen));

					const int32 MeshIdx = (int32)((Seed / 1000u) % (uint32)NumCarMeshes);
					Ctx.ParkedCars.Emplace(MeshIdx, FTransform(
						FRotator(0.0, FMath::RadiansToDegrees(FMath::Atan2(Facing.Y, Facing.X)), 0.0),
						FVector(Centre.X, Centre.Y, BaseZ)));
				}
			}
		}
	}

	if (BayCount > 0)
	{
		UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] BuildStandardParkingBays: %d bays, %d stall lines, %d parked cars."),
			BayCount, StallLines, Ctx.ParkedCars.Num());
	}
}

void URoadNetwork::BuildBikeCrossings(FRoadNetRebuildContext& Ctx) const
{
	Ctx.BikeStencils.Reset();
	const int32 NumZones = Ctx.Zones.Num();
	if (NumZones == 0 || BikeCrossings.Num() == 0) { return; }
	if (Ctx.ZoneMarkingWhitePolys.Num() != NumZones) { Ctx.ZoneMarkingWhitePolys.SetNum(NumZones); }

	// "Elephant's footprints": two rows of square blocks flanking the cycleway,
	// one row per edge. Squares, not stripes, is what distinguishes a cycle
	// crossing from a pedestrian zebra at a glance.
	constexpr double kBlockCm = 50.0;    // square side
	constexpr double kPitchCm = 90.0;    // start-to-start along the path

	int32 Blocks = 0;
	for (const FRoadNetBikeCrossing& X : BikeCrossings)
	{
		if (X.Path.Num() < 2) { continue; }
		TArray<double> CL;
		RoadNetMath::CumulativeLength(X.Path, CL);
		const double Len = CL.Last();
		if (Len < kPitchCm) { continue; }

		auto SampleAt = [&](double S, FVector2D& OutP, FVector2D& OutTan, double& OutZ)
		{
			S = FMath::Clamp(S, 0.0, Len);
			int32 seg = 0;
			while (seg + 1 < CL.Num() - 1 && CL[seg + 1] < S) { ++seg; }
			const double segLen = FMath::Max(1e-3, CL[seg + 1] - CL[seg]);
			const double t = FMath::Clamp((S - CL[seg]) / segLen, 0.0, 1.0);
			const FVector P = FMath::Lerp(X.Path[seg], X.Path[seg + 1], t);
			OutP = FVector2D(P.X, P.Y);
			OutZ = P.Z;
			FVector2D Tan(X.Path[seg + 1].X - X.Path[seg].X, X.Path[seg + 1].Y - X.Path[seg].Y);
			if (!Tan.Normalize()) { Tan = FVector2D(1.0, 0.0); }
			OutTan = Tan;
		};

		// One zone for the whole crossing, resolved at its midpoint: a crossing
		// spans a single carriageway, and splitting it across zones would give the
		// two halves different grades.
		FVector2D Mid, MidTan; double MidZ = 0.0;
		SampleAt(0.5 * Len, Mid, MidTan, MidZ);
		int32 z = INDEX_NONE;
		double BestD2 = TNumericLimits<double>::Max();
		for (int32 zi = 0; zi < NumZones; ++zi)
		{
			for (int32 RoadIdx : Ctx.Zones[zi])
			{
				const FRoadCurves* C = Ctx.Curves.Find(RoadIdx);
				if (!C || C->Sampled.Num() < 2) { continue; }
				const double D2 = FMath::Square(RoadNetMath::ProjectToPolyline(C->Sampled, Mid).Distance);
				if (D2 < BestD2) { BestD2 = D2; z = zi; }
			}
		}
		if (z == INDEX_NONE) { continue; }

		const double HalfW = 0.5 * FMath::Max(100.f, X.WidthCm);
		const double HalfB = 0.5 * kBlockCm;
		// Centre the row so a block sits at each end of the drawn path rather than
		// leaving a ragged tail wherever the length is not a whole number of pitches.
		const int32 Count = FMath::Max(1, FMath::FloorToInt(Len / kPitchCm));
		const double Span = Count * kPitchCm;
		const double S0 = 0.5 * (Len - Span) + 0.5 * kPitchCm;

		for (int32 i = 0; i < Count; ++i)
		{
			FVector2D P, Tan; double Z = 0.0;
			SampleAt(S0 + i * kPitchCm, P, Tan, Z);
			const FVector2D Perp(Tan.Y, -Tan.X);
			for (double Side : { -1.0, 1.0 })
			{
				const FVector2D C = P + Perp * (Side * HalfW);
				TArray<FVector2d> Loop;
				Loop.Emplace(C.X + (Tan.X * HalfB + Perp.X * HalfB), C.Y + (Tan.Y * HalfB + Perp.Y * HalfB));
				Loop.Emplace(C.X + (-Tan.X * HalfB + Perp.X * HalfB), C.Y + (-Tan.Y * HalfB + Perp.Y * HalfB));
				Loop.Emplace(C.X + (-Tan.X * HalfB - Perp.X * HalfB), C.Y + (-Tan.Y * HalfB - Perp.Y * HalfB));
				Loop.Emplace(C.X + (Tan.X * HalfB - Perp.X * HalfB), C.Y + (Tan.Y * HalfB - Perp.Y * HalfB));
				FPolygon2d Poly(Loop);
				if (Poly.VertexCount() < 3) { continue; }
				if (Poly.IsClockwise()) { Poly.Reverse(); }
				FGeneralPolygon2d GP;
				GP.SetOuter(Poly);
				Ctx.ZoneMarkingWhitePolys[z].Add(MoveTemp(GP));
				++Blocks;
			}
		}

		Ctx.BikeStencils.Emplace(FVector(Mid.X, Mid.Y, MidZ),
			(float)FMath::RadiansToDegrees(FMath::Atan2(MidTan.Y, MidTan.X)));
	}

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] BuildBikeCrossings: %d crossings, %d blocks."),
		BikeCrossings.Num(), Blocks);
}
