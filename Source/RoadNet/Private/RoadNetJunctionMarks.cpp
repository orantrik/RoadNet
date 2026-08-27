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
		const FVector2D& EnterSide, double HalfWidthCm, TArray<FGeneralPolygon2d>& OutWhite)
	{
		FVector2D U = Travel;
		if (!U.Normalize()) { U = FVector2D(1, 0); }
		const double Half = FMath::Max(50.0, HalfWidthCm);
		OutWhite.Add(MakeRect(StopPos + EnterSide * (Half * 0.5), U,
			/*HalfU along travel*/ 25.0, /*HalfV lateral*/ Half * 0.5));
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

			// Traffic enters on the right of the inbound direction where they
			// drive on the right, and on the left where they drive on the left.
			// Everything that belongs to one direction of travel — the stop bar,
			// the give-way dashes, the signal head — hangs off this.
			const FVector2D Enter = bDriveOnLeft ? FVector2D(-Rin.X, -Rin.Y) : Rin;

			// Stop / give-way bar across the entering half.
			if (bStop)
			{
				EmitStopBar(StopP, Out, Enter, Half, OutWhite);
			}
			else if (bYield)
			{
				// Row of small dashes across the entering half (give-way line).
				constexpr double kDashHalf = 20.0, kPitch = 90.0;
				for (double off = 0.0; off <= Half; off += kPitch)
				{
					const FVector2D C = StopP + Enter * off;
					OutWhite.Add(MakeRect(C, Out, 18.0, kDashHalf));
				}
			}

			// Zebra crosswalk just OUTSIDE the stop bar.
			if (bCross)
			{
				constexpr double kDepth = 400.0, kGap = 70.0;
				EmitZebra(StopP + Out * (50.0 + kGap + kDepth * 0.5), Out, Half, kDepth, OutWhite);
			}

			// Placeholder signal at the near corner on the entering side, facing
			// the approach.
			if (bSig)
			{
				const FVector2D SP = StopP + Enter * (Half + 120.0);
				FSignal S;
				S.Location = FVector(SP.X, SP.Y, CenterZ);
				S.YawDeg = (float)FMath::RadiansToDegrees(FMath::Atan2(Din.Y, Din.X));
				OutSignals.Add(S);
			}
		}
	}
}
