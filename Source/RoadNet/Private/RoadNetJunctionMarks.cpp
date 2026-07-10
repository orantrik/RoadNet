// RoadNetJunctionMarks.cpp — junction marking geometry (see header).
#include "RoadNetJunctionMarks.h"
#include "Polygon2.h"

using namespace UE::Geometry;

namespace RoadNetJunctionMarks
{
	namespace
	{
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

	void BuildJoint(
		const FVector2D& Center, double CenterZ,
		const TArray<FApproach>& Approaches,
		ERoadNetJunctionPreset Preset,
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
			const FVector2D Rin(Din.Y, -Din.X);           // right of inbound = entering side

			// Stop / give-way bar across the entering half.
			if (bStop)
			{
				const FVector2D C = StopP + Rin * (Half * 0.5);
				OutWhite.Add(MakeRect(C, Out, /*HalfU along travel*/25.0, /*HalfV lateral*/Half * 0.5));
			}
			else if (bYield)
			{
				// Row of small dashes across the entering half (give-way line).
				constexpr double kDashHalf = 20.0, kPitch = 90.0;
				for (double off = 0.0; off <= Half; off += kPitch)
				{
					const FVector2D C = StopP + Rin * off;
					OutWhite.Add(MakeRect(C, Out, 18.0, kDashHalf));
				}
			}

			// Zebra crosswalk just OUTSIDE the stop bar: stripes run ALONG travel
			// and repeat laterally across the full width.
			if (bCross)
			{
				constexpr double kDepth = 400.0, kGap = 70.0, kStripeHalf = 25.0, kPitch = 90.0;
				const FVector2D CBand = StopP + Out * (50.0 + kGap + kDepth * 0.5);
				for (double off = -Half; off <= Half + 1.0; off += kPitch)
				{
					const FVector2D C = CBand + Rin * off;
					OutWhite.Add(MakeRect(C, Out, kDepth * 0.5, kStripeHalf));
				}
			}

			// Placeholder signal at the near-right corner, facing the approach.
			if (bSig)
			{
				const FVector2D SP = StopP + Rin * (Half + 120.0);
				FSignal S;
				S.Location = FVector(SP.X, SP.Y, CenterZ);
				S.YawDeg = (float)FMath::RadiansToDegrees(FMath::Atan2(Din.Y, Din.X));
				OutSignals.Add(S);
			}
		}
	}
}
