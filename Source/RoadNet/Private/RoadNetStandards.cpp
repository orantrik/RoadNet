// ===========================================================================
// RoadNetStandards.cpp — the design tables, transcribed.
//
// See docs/ISRAELI_JUNCTION_STANDARDS.md for the full extracted tables and the
// section citations behind every number here.
// ===========================================================================
#include "RoadNetStandards.h"

namespace RoadNetStandards
{
	int32 DesignSpeedKph(ERoadNetClass Class, int32 AuthoredKph)
	{
		// An authored speed is the designer's (or OSM's) statement of intent and
		// outranks the class default, which is only a guess from the road's tier.
		if (AuthoredKph > 0) { return AuthoredKph; }

		// כרך 1 Table 2.4 — design speed by road class.
		switch (Class)
		{
		case ERoadNetClass::Motorway:    return 110;
		case ERoadNetClass::Trunk:       return 100;
		case ERoadNetClass::Primary:     return  90;
		case ERoadNetClass::Secondary:   return  80;
		case ERoadNetClass::Tertiary:    return  70;
		case ERoadNetClass::Residential: return  50;
		case ERoadNetClass::Service:     return  30;
		case ERoadNetClass::Pedestrian:
		case ERoadNetClass::Path:        return  30;
		default:                         return  50;
		}
	}

	double LaneDropTaperRatio(int32 Vd)
	{
		// כרך 2 Table 5.1 — post-junction lane add/drop taper.
		if (Vd >= 90) { return 50.0; }
		if (Vd >= 70) { return 45.0; }
		return 40.0;
	}

	double LaneDropTaperLengthCm(int32 Vd, double LaneWidthCm)
	{
		// A 1:N taper closes a lane of width W over N*W of road.
		return LaneDropTaperRatio(Vd) * FMath::Max(0.0, LaneWidthCm);
	}

	double LeftTurnTaperRatio(int32 Vd)
	{
		// כרך 2 §7 — 1:10 below 80 km/h, 1:15 at or above.
		return (Vd >= 80) ? 15.0 : 10.0;
	}

	double CentrelineShiftRatio(int32 Vd)
	{
		// כרך 2 §7 — symmetric shift at 1:(Vd/2), used where the carriageway
		// cannot be widened on one side only.
		return FMath::Max(5.0, 0.5 * (double)FMath::Max(1, Vd));
	}

	int32 JunctionAngleMinDeg()       { return 70; }   // כרך 2 §3.1
	int32 JunctionAngleMaxDeg()       { return 110; }  // כרך 2 §3.1
	int32 JunctionAnglePreferredDeg() { return 90; }   // כרך 2 §3.1

	// כרך 2 §8.2.3 «שיפועים מזעריים» — drainage floors. See the header for why
	// these are minimums despite the quantity being named "maximum resultant".
	double MinJunctionResultantGrade()    { return 0.010; }   // «לא יפחת מ-1%»
	double MinJunctionLongitudinalGrade() { return 0.005; }   // 0.5% along the arms

	double MaxArmGradeAtJunction(int32 Vd)
	{
		// כרך 2 Table 8.1 — max longitudinal grade on a junction arm by the
		// arm's design speed. Faster arms are held flatter.
		return (Vd >= 80) ? 0.04 : 0.06;
	}

	double MinIslandAreaCm2()    { return 10.0 * 100.0 * 100.0; }  // כרך 2 §6.5.2 — 10 m²
	double IslandNoseSetbackCm() { return 60.0; }                  // כרך 2 §6.5.2 — 0.6 m

	namespace
	{
		// Linear interpolation over a speed-keyed table, clamped at both ends.
		// The tables are tabulated at 10 km/h steps from 50 to 100; an OSM
		// maxspeed of 55 is perfectly ordinary and must not snap to a bucket.
		double LerpBySpeed(int32 Vd, const double (&Values)[6])
		{
			constexpr int32 kFirstKph = 50;
			constexpr int32 kStepKph  = 10;
			const double T = ((double)Vd - (double)kFirstKph) / (double)kStepKph;
			if (T <= 0.0) { return Values[0]; }
			if (T >= 5.0) { return Values[5]; }
			const int32 i = (int32)T;
			return FMath::Lerp(Values[i], Values[i + 1], T - (double)i);
		}
	}

	double DecelLengthM(int32 Vd, bool bRightTurn, bool bToStop)
	{
		//                                        50    60    70    80     90     100 km/h
		static constexpr double kLeft[6]     = { 35.0, 50.0, 70.0, 95.0, 115.0, 140.0 }; // Table 7.1
		static constexpr double kRightStop[6]= { 36.0, 50.0, 69.0, 95.0, 114.0, 139.0 }; // Table 6.2, to stop
		static constexpr double kRight25[6]  = { 24.0, 38.0, 57.0, 82.0, 102.0, 127.0 }; // Table 6.2, to 25 km/h

		if (!bRightTurn) { return LerpBySpeed(Vd, kLeft); }
		return bToStop ? LerpBySpeed(Vd, kRightStop) : LerpBySpeed(Vd, kRight25);
	}

	double GradeFactor(double GradePct, bool bUphill)
	{
		// Table 6.3. Below the first band there is no correction; above 6% the
		// grade is out of spec for a deceleration lane, so the steepest band is
		// the honest answer rather than an extrapolation.
		const double G = FMath::Abs(GradePct);
		if (G < 2.5) { return 1.0; }
		if (G < 4.5) { return bUphill ? 0.90 : 1.20; }
		return bUphill ? 0.80 : 1.35;
	}

	double MinTurnLaneLengthCm()      { return 35.0 * 100.0; }  // כרך 2 §7.3.1 — 35 m floor
	double RightTurnLaneWidthCm()     { return 4.5 * 100.0; }   // כרך 2 §6 — 4.5 m at the tangent point
	double TurnTaperMeasureWidthCm()  { return 3.0 * 100.0; }   // כרך 2 §6 — decel measured from 3.0 m

	double TurnLaneExtentCm(int32 Vd, bool bRightTurn, double GradePct,
		double LaneWidthCm, double* OutParallelCm)
	{
		// Parallel segment: the tabulated deceleration length, corrected for
		// grade. Downhill needs more room to shed the same speed.
		const double DecelCm = DecelLengthM(Vd, bRightTurn, /*bToStop*/true) * 100.0
			* GradeFactor(GradePct, /*bUphill*/ GradePct > 0.0);

		// Taper: a 1:N offset rate moving the driver across one lane width.
		// LeftTurnTaperRatio already encodes the 1:10 / 1:15 split at 80 km/h,
		// which §7.3.2 applies to both hands.
		const double TaperCm = LeftTurnTaperRatio(Vd) * FMath::Max(0.0, LaneWidthCm);

		if (OutParallelCm) { *OutParallelCm = DecelCm; }
		return FMath::Max(MinTurnLaneLengthCm(), DecelCm + TaperCm);
	}

	FPaintSpec Paint(EMark Mark)
	{
		// Working values pending IS 934 — see the header. 801 carries the
		// 3 m / 5 m the generator already drew, so nothing changes shape today.
		switch (Mark)
		{
		case EMark::LaneDivider:  return { 300.0, 500.0, 12.0 };
		// 805 warns that a solid is coming, so it is the SAME line running at a
		// higher frequency — that density is the whole signal.
		case EMark::WarningDash:  return { 100.0, 100.0, 12.0 };
		// 809 guides a driver through a junction, where speeds are low and the
		// path is short: a shorter, tighter dash so a 20 m turn still reads as a
		// dashed line rather than one or two marks.
		case EMark::JunctionTurn: return { 100.0, 150.0, 12.0 };
		// Arrows are glyphs, not ribbons; the dash fields are the repeat pitch
		// along the lane, the width is the glyph's painted stroke.
		case EMark::TurnArrow:    return { 500.0, 2500.0, 15.0 };
		case EMark::LaneEndArrow: return { 400.0, 1200.0, 15.0 };
		default:                  return { 300.0, 500.0, 12.0 };
		}
	}

	bool SelfCheck(FString& OutReport)
	{
		// Deceleration length rises with speed across the whole range, for every
		// column. A non-monotonic table means a transcription slip.
		bool bMono = true;
		for (int32 V = 40; V <= 110; ++V)
		{
			const double PrevL = DecelLengthM(V - 1, false);
			const double PrevR = DecelLengthM(V - 1, true, true);
			const double Prev25 = DecelLengthM(V - 1, true, false);
			bMono &= DecelLengthM(V, false)      >= PrevL  - KINDA_SMALL_NUMBER;
			bMono &= DecelLengthM(V, true, true) >= PrevR  - KINDA_SMALL_NUMBER;
			bMono &= DecelLengthM(V, true, false)>= Prev25 - KINDA_SMALL_NUMBER;
		}

		// Clamped, not extrapolated, outside the tabulated 50-100 range.
		const bool bClamp =
			FMath::IsNearlyEqual(DecelLengthM(20,  false), 35.0) &&
			FMath::IsNearlyEqual(DecelLengthM(200, false), 140.0);

		// Tabulated values land exactly on their entries.
		const bool bExact =
			FMath::IsNearlyEqual(DecelLengthM(70, false), 70.0) &&
			FMath::IsNearlyEqual(DecelLengthM(70, true, true), 69.0) &&
			FMath::IsNearlyEqual(DecelLengthM(70, true, false), 57.0);

		// Halfway between two entries gives the midpoint.
		const bool bLerp = FMath::IsNearlyEqual(DecelLengthM(75, false), 82.5);

		// Decelerating to 25 km/h is never longer than decelerating to a stop.
		bool bStopLonger = true;
		for (int32 V = 50; V <= 100; V += 5)
		{
			bStopLonger &= DecelLengthM(V, true, true) >= DecelLengthM(V, true, false);
		}

		// Grade correction: none below 2.5%, downhill lengthens, uphill shortens,
		// and the magnitude grows with the grade.
		const bool bGrade =
			FMath::IsNearlyEqual(GradeFactor(0.0, true),  1.0) &&
			FMath::IsNearlyEqual(GradeFactor(2.4, false), 1.0) &&
			GradeFactor(3.0, false) > 1.0 && GradeFactor(3.0, true) < 1.0 &&
			GradeFactor(5.0, false) > GradeFactor(3.0, false) &&
			GradeFactor(5.0, true)  < GradeFactor(3.0, true);

		// The plan's worked example: a 70 km/h left turn on a 5% downgrade is
		// 70 m x 1.35 = 94.5 m of parallel segment.
		double ParallelCm = 0.0;
		TurnLaneExtentCm(70, /*bRightTurn*/false, /*GradePct*/-5.0, 350.0, &ParallelCm);
		const bool bWorked = FMath::IsNearlyEqual(ParallelCm, 9450.0, 1.0);

		// The 35 m floor holds even where the computed extent is shorter.
		const bool bFloor = TurnLaneExtentCm(20, false, 0.0, 1.0) >= MinTurnLaneLengthCm();

		// The extent always contains its own parallel segment plus a taper.
		double P2 = 0.0;
		const double Ext = TurnLaneExtentCm(90, true, 0.0, 350.0, &P2);
		const bool bContains = Ext >= P2 && Ext >= P2 + LeftTurnTaperRatio(90) * 350.0 - 1.0;

		// Every paint spec is usable: positive width, and a dashed entry has a
		// positive stride so the dash loop that walks it terminates.
		bool bPaint = true;
		for (const EMark M : { EMark::LaneDivider, EMark::WarningDash,
			EMark::JunctionTurn, EMark::TurnArrow, EMark::LaneEndArrow })
		{
			const FPaintSpec S = Paint(M);
			bPaint &= (S.WidthCm > 0.0) && (S.DashLenCm <= 0.0 || S.DashLenCm + S.GapLenCm > 0.0);
		}
		// 805 is the WARNING line: denser than the ordinary divider by design.
		bPaint &= (Paint(EMark::WarningDash).DashLenCm + Paint(EMark::WarningDash).GapLenCm)
			< (Paint(EMark::LaneDivider).DashLenCm + Paint(EMark::LaneDivider).GapLenCm);

		const bool bOk = bMono && bClamp && bExact && bLerp && bStopLonger && bGrade
			&& bWorked && bFloor && bContains && bPaint;

		OutReport = FString::Printf(
			TEXT("StandardsSelfCheck: %s | monotonic %s | clamped %s | exact %s | lerp %s | stop>=25kph %s | grade %s | 70kph@-5%%=94.5m %s | 35m floor %s | extent>=parallel+taper %s | paint %s"),
			bOk ? TEXT("PASS") : TEXT("FAIL"),
			bMono       ? TEXT("ok") : TEXT("FAILED"),
			bClamp      ? TEXT("ok") : TEXT("FAILED"),
			bExact      ? TEXT("ok") : TEXT("FAILED"),
			bLerp       ? TEXT("ok") : TEXT("FAILED"),
			bStopLonger ? TEXT("ok") : TEXT("FAILED"),
			bGrade      ? TEXT("ok") : TEXT("FAILED"),
			bWorked     ? TEXT("ok") : TEXT("FAILED"),
			bFloor      ? TEXT("ok") : TEXT("FAILED"),
			bContains   ? TEXT("ok") : TEXT("FAILED"),
			bPaint      ? TEXT("ok") : TEXT("FAILED"));
		return bOk;
	}
}
