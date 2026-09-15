// RoadNetJunctionMarks.cpp — junction marking geometry (see header).
#include "RoadNetJunctionMarks.h"
#include "Polygon2.h"
#include "HAL/IConsoleManager.h"

using namespace UE::Geometry;

static TAutoConsoleVariable<float> CVarRoadNetCrosswalkLengthCm(
	TEXT("roadnet.CrosswalkLengthCm"),
	400.0f,
	TEXT("Length (cm) of a junction zebra crossing measured ALONG the road — how far the stripes reach, i.e. the depth of the painted corridor a driver crosses. The stop bar moves out with it and always sits behind the crossing. Default 400."),
	ECVF_Default);

namespace RoadNetJunctionMarks
{
	namespace
	{
		// Layout of one approach's paint, measured OUTWARD from FApproach::StopPos
		// (the junction-area boundary) back up the approach:
		//
		//   junction | 50 | ZEBRA (length) | 60 | STOP BAR | approach traffic
		//
		// The stop bar is the LAST thing a driver meets before the crossing, so it
		// belongs OUTBOARD of the zebra. It used to be the other way round — bar on
		// the boundary, zebra 1.2 m outboard of it — which told drivers to stop with
		// the bonnet already over the crossing.
		//
		// kBarOffsetCm is centre-to-edge and matches the mid-block crossing rule in
		// RoadNetMarkings.cpp, so a crossing at a junction mouth is striped exactly
		// like one painted away from a junction.
		constexpr double kZebraSetbackCm  = 50.0;   // junction boundary -> zebra near edge
		constexpr double kBarOffsetCm     = 60.0;   // zebra far edge    -> stop bar centre
		constexpr double kBarHalfThickCm  = 25.0;   // half the bar's thickness along travel

		double ZebraLengthCm()
		{
			return FMath::Clamp(
				(double)CVarRoadNetCrosswalkLengthCm.GetValueOnAnyThread(), 100.0, 2000.0);
		}

		// A CCW rectangle centred at C, extending ±HalfU along unit axis U and
		// ±HalfV along its perpendicular.
		FGeneralPolygon2d MakeRect(const FVector2D& C, const FVector2D& U, double HalfU, double HalfV)
		{
			const FVector2D V(-U.Y, U.X);
			TArray<FVector2d> Loop;
			Loop.Reserve(4);
			Loop.Emplace(C.X - U.X * HalfU - V.X * HalfV, C.Y - U.Y * HalfU - V.Y * HalfV);
			Loop.Emplace(C.X + U.X * HalfU - V.X * HalfV, C.Y + U.Y * HalfU - V.Y * HalfV);
			Loop.Emplace(C.X + U.X * HalfU + V.X * HalfV, C.Y + U.Y * HalfU + V.Y * HalfV);
			Loop.Emplace(C.X - U.X * HalfU + V.X * HalfV, C.Y - U.Y * HalfU + V.Y * HalfV);
			FPolygon2d P(Loop);
			if (P.IsClockwise()) { P.Reverse(); }
			FGeneralPolygon2d G;
			G.SetOuter(P);
			return G;
		}
	}

	void EmitZebra(const FVector2D& BandCenter, const FVector2D& Travel,
		double HalfWidthCm, double DepthCm, TArray<FGeneralPolygon2d>& OutWhite)
	{
		FVector2D U = Travel;
		if (!U.Normalize()) { U = FVector2D(1, 0); }
		const FVector2D Lateral(U.Y, -U.X);

		constexpr double kStripeHalf = 25.0, kPitch = 90.0;
		const double Half = FMath::Max(50.0, HalfWidthCm);
		const double Depth = FMath::Max(50.0, DepthCm);
		for (double Off = -Half; Off <= Half + 1.0; Off += kPitch)
		{
			OutWhite.Add(MakeRect(BandCenter + Lateral * Off, U, Depth * 0.5, kStripeHalf));
		}
	}

	void EmitStopBar(const FVector2D& StopPos, const FVector2D& Travel,
		const FVector2D& Lateral, double LoCm, double HiCm, TArray<FGeneralPolygon2d>& OutWhite)
	{
		FVector2D U = Travel;
		if (!U.Normalize()) { U = FVector2D(1, 0); }
		FVector2D L = Lateral;
		if (!L.Normalize()) { L = FVector2D(U.Y, -U.X); }

		const double Mid  = 0.5 * (HiCm + LoCm);
		const double Span = 0.5 * (HiCm - LoCm);
		if (Span < 25.0) { return; }   // narrower than the bar is thick: not a lane

		// MakeRect's V axis is U's perpendicular, so the rect is placed on the
		// Lateral axis by offsetting its centre rather than by rotating it.
		OutWhite.Add(MakeRect(StopPos + L * Mid, U,
			/*HalfU along travel*/ kBarHalfThickCm, /*HalfV lateral*/ Span));
	}

	double CrosswalkLengthCm() { return ZebraLengthCm(); }

	double ApproachPaintReachCm()
	{
		return kZebraSetbackCm + ZebraLengthCm() + kBarOffsetCm + kBarHalfThickCm;
	}

	void BuildJoint(
		const FVector2D& Center, double CenterZ,
		const TArray<FApproach>& Approaches,
		ERoadNetJunctionPreset Preset,
		bool bDriveOnLeft,
		TArray<FGeneralPolygon2d>& OutWhite,
		TArray<FSignal>& OutSignals)
	{
		if (Preset == ERoadNetJunctionPreset::None) { return; }
		const bool bStop  = (Preset == ERoadNetJunctionPreset::StopLine
			|| Preset == ERoadNetJunctionPreset::StopAndCrosswalk
			|| Preset == ERoadNetJunctionPreset::Signalized);
		const bool bCross = (Preset == ERoadNetJunctionPreset::StopAndCrosswalk
			|| Preset == ERoadNetJunctionPreset::Signalized);
		const bool bSig   = (Preset == ERoadNetJunctionPreset::Signalized);
		const bool bYield = (Preset == ERoadNetJunctionPreset::GiveWay);

		for (const FApproach& Ap : Approaches)
		{
			const double Half = FMath::Max(50.0, Ap.HalfWidthCm);
			FVector2D Out = Ap.Outward;
			if (!Out.Normalize()) { Out = FVector2D(1, 0); }
			const FVector2D StopP = Ap.StopPos;
			const FVector2D Din(-Out.X, -Out.Y);          // inbound (toward junction)
			const FVector2D Rin(Din.Y, -Din.X);           // right of inbound

			// Where the entering carriageway actually is, as an interval along Rin.
			// Fallback rule: traffic enters on the right of the inbound direction
			// where they drive on the right, and on the left where they drive on the
			// left — i.e. one half of the arm.
			// The arm's lanes answer this when they were resolved; the handedness
			// rule is only the fallback for a caller that did not fill them in.
			double EnterLo = 0.0, EnterHi = 0.0;
			if (Ap.EnterHiCm > Ap.EnterLoCm)
			{
				EnterLo = Ap.EnterLoCm;
				EnterHi = Ap.EnterHiCm;
			}
			else if (bDriveOnLeft) { EnterLo = -Half; EnterHi = 0.0; }
			else                   { EnterLo = 0.0;   EnterHi = Half; }

			// Where the bar goes. With a crossing it sits behind the zebra, so a
			// stopped car never stands on the stripes; with no crossing there is
			// nothing to clear and it stays on the junction boundary.
			const double Zebra = ZebraLengthCm();
			const FVector2D BarP = bCross
				? StopP + Out * (kZebraSetbackCm + Zebra + kBarOffsetCm)
				: StopP;

			// Nothing enters this arm — a one-way exit. No bar, no give-way dashes,
			// no signal head: there is no traffic here to stop. The zebra below still
			// goes in, because pedestrians cross an exit arm like any other.
			if (bStop && Ap.bHasEnteringTraffic)
			{
				EmitStopBar(BarP, Out, Rin, EnterLo, EnterHi, OutWhite);
			}
			else if (bYield && Ap.bHasEnteringTraffic)
			{
				// Row of small dashes across the entering lanes (give-way line).
				constexpr double kDashHalf = 20.0, kPitch = 90.0;
				for (double off = EnterLo; off <= EnterHi + 1.0; off += kPitch)
				{
					const FVector2D C = BarP + Rin * off;
					OutWhite.Add(MakeRect(C, Out, 18.0, kDashHalf));
				}
			}

			// Zebra crosswalk against the junction boundary, INSIDE the stop bar.
			if (bCross)
			{
				EmitZebra(StopP + Out * (kZebraSetbackCm + Zebra * 0.5), Out, Half, Zebra, OutWhite);
			}

			// Placeholder signal at the near corner on the entering side, facing
			// the approach. An exit-only arm gets none — nothing on it to signal.
			if (bSig && Ap.bHasEnteringTraffic)
			{
				// Just outside the far kerb of the entering lanes.
				const double OuterEdge = (FMath::Abs(EnterHi) >= FMath::Abs(EnterLo)) ? EnterHi : EnterLo;
				const FVector2D SP = StopP + Rin * (OuterEdge + FMath::Sign(OuterEdge) * 120.0);
				FSignal S;
				S.Location = FVector(SP.X, SP.Y, CenterZ);
				S.YawDeg = (float)FMath::RadiansToDegrees(FMath::Atan2(Din.Y, Din.X));
				OutSignals.Add(S);
			}
		}
	}
}
