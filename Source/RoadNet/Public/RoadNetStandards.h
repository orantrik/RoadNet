#pragma once
#include "CoreMinimal.h"
#include "RoadNetTypes.h"

// ===========================================================================
// RoadNetStandards — the Israeli interurban geometric-design tables, as pure
// lookups. Source: "הנחיות לתכנון גיאומטרי של דרכים בין-עירוניות", כרך 1
// (cross sections and alignment) and כרך 2 — צמתים (junctions). Section and
// table citations are in docs/ISRAELI_JUNCTION_STANDARDS.md, which holds the
// full extracted tables; this header only carries what the generator asks for.
//
// Everything here is a free function over plain numbers — no UObject, no
// engine state, no allocation — so RoadNet.LaneSelfCheck can assert against it
// without standing up a world.
//
// Lengths are CENTIMETRES (the RoadNet unit) unless a name says otherwise;
// speeds are km/h; grades are fractions (0.01 = 1%).
// ===========================================================================
namespace RoadNetStandards
{
	// ---- design speed ------------------------------------------------------

	// Resolve the design speed to look every other table up by. AuthoredKph
	// wins when it is set (OSM maxspeed=, or a hand edit); otherwise the class
	// default from כרך 1 Table 2.4 applies. Never returns 0.
	ROADNET_API int32 DesignSpeedKph(ERoadNetClass Class, int32 AuthoredKph = 0);

	// ---- lane add / drop outside the junction (§5.2.4, Table 5.1) ----------

	// Taper RATIO for a lane added or dropped past a junction, as the "N" in
	// 1:N. Faster roads get a longer, flatter taper (Table 5.1).
	ROADNET_API double LaneDropTaperRatio(int32 Vd);

	// Length (cm) of that taper for a lane of the given width. This is the
	// distance the drop must sit OUTSIDE the junction — inside it the arm keeps
	// full width (§5.2.1).
	ROADNET_API double LaneDropTaperLengthCm(int32 Vd, double LaneWidthCm);

	// ---- left-turn lanes (Chapter 7) --------------------------------------

	// Taper ratio (the N in 1:N) for widening into a left-turn lane: 1:10 below
	// 80 km/h, 1:15 at or above.
	ROADNET_API double LeftTurnTaperRatio(int32 Vd);

	// Ratio for a symmetric centreline shift where there is no room to widen on
	// one side: 1:(Vd/2).
	ROADNET_API double CentrelineShiftRatio(int32 Vd);

	// ---- junction geometry limits -----------------------------------------

	// Legal range for the angle between two junction arms (§3.1), degrees.
	// 90° is preferred; outside [Min, Max] the junction should be realigned.
	ROADNET_API int32 JunctionAngleMinDeg();
	ROADNET_API int32 JunctionAngleMaxDeg();
	ROADNET_API int32 JunctionAnglePreferredDeg();

	// §8.2.3 is a section of MINIMUM grades, and both of these are drainage
	// floors, not ceilings. The Hebrew reads «לא יפחת מ־1%» — "shall not fall
	// below 1%" — and the rule sits beside the 0.5% longitudinal minimum under
	// the heading «שיפועים מזעריים». The trap is that the quantity is *named*
	// "the maximum resultant grade" (meaning the grade in the steepest
	// direction, per formula [8.7]); the requirement is that this steepest
	// direction be steep ENOUGH for water to run off rather than pond.
	//
	// Both returned as fractions: 0.01 == 1%.
	ROADNET_API double MinJunctionResultantGrade();
	ROADNET_API double MinJunctionLongitudinalGrade();

	// Maximum longitudinal grade permitted on a junction arm (`Table 8.1`),
	// keyed on the arm's design speed. THIS is the ceiling — it is 4–6%, which
	// is also why a 1% cap on the resultant would have been unsatisfiable.
	ROADNET_API double MaxArmGradeAtJunction(int32 Vd);

	// Minimum area (cm²) for a channelizing island to be built at all, and how
	// far its nose is set back from the through-lane edge (§6.5.2).
	ROADNET_API double MinIslandAreaCm2();
	ROADNET_API double IslandNoseSetbackCm();

	// ---- turn-lane extent (כרך 2 Chapter 6 / Chapter 7) --------------------
	//
	// A dedicated turn lane is two pieces (§7.3.1): a TAPER (לוכסן) the driver
	// moves across on, then a PARALLEL SEGMENT (קטע מקביל) used for deceleration
	// and queue storage. Total length = deceleration length + storage length,
	// and storage may NOT be counted along the taper.
	//
	// This is the domain markings have to be placed over, which is why it lives
	// here rather than in the marking code: the arrows belong along the parallel
	// segment, between the end of the taper and the stop line.

	// Deceleration length (metres) for the parallel segment, on a grade under
	// the 2.5-3% threshold the tables assume.
	//   bRightTurn=false → Table 7.1 (left turn), by approach design speed.
	//   bRightTurn=true  → Table 6.2. bToStop picks the decelerate-to-stop
	//     column (the normal case for a signal or a crosswalk) over the
	//     decelerate-to-25 km/h column.
	// Interpolates linearly between tabulated speeds and clamps outside them.
	ROADNET_API double DecelLengthM(int32 Vd, bool bRightTurn, bool bToStop = true);

	// Grade correction factor for the above (Table 6.3). GradePct is the
	// magnitude of the longitudinal grade in PERCENT; bUphill picks the column.
	// Downhill lengthens the lane, uphill shortens it. 1.0 below 2.5%.
	// The tables stop at 6%, which is also the maximum grade permitted along a
	// deceleration lane — steeper input is clamped to the 4.5-6.0% band.
	ROADNET_API double GradeFactor(double GradePct, bool bUphill);

	// Absolute minimum length (cm) of a turn lane, taper included (§7.3.1).
	ROADNET_API double MinTurnLaneLengthCm();

	// Width (cm) a right-turn taper widens to at the tangent point with the turn
	// radius, and the width at which deceleration starts being measured (§6).
	ROADNET_API double RightTurnLaneWidthCm();
	ROADNET_API double TurnTaperMeasureWidthCm();

	// Full turn-lane extent (cm) upstream of the stop line: the parallel segment
	// (decel, grade-corrected) plus the taper, floored at MinTurnLaneLengthCm.
	// GradePct is signed along the direction of travel — negative is downhill.
	// OutParallelCm receives the parallel segment alone, which is the stretch
	// arrows and turn-lane block markings may be painted along.
	ROADNET_API double TurnLaneExtentCm(int32 Vd, bool bRightTurn, double GradePct,
		double LaneWidthCm, double* OutParallelCm = nullptr);

	// ---- road-surface markings (LOT"M לוח התמרורים, part 8: 801-821) -------
	//
	// The LOT"M fixes shape, colour and MEANING; it delegates dimensions to
	// Israeli Standard 934 («אופן הסימון ... יהיה בהתאם לת"י 934»), which we do
	// not have. So the dash/gap/width numbers below are working values chosen to
	// match what the generator already drew, kept per marking number rather than
	// shared through two globals — the point of the table is that when IS 934
	// arrives, one entry changes and nothing else does.
	enum class EMark : int32
	{
		LaneDivider   = 801,   // קו קטעים — dashed white lane / centre line
		WarningDash   = 805,   // dense dashes warning a double solid is starting
		JunctionTurn  = 809,   // קו קטעים, פניה בצומת — dashed turn guide
		TurnArrow     = 813,   // חצים — permitted directions of travel in the lane
		LaneEndArrow  = 814,   // diagonal arrows where a lane is about to end
	};

	// Paint geometry for one marking. DashLenCm <= 0 means a solid line.
	struct FPaintSpec
	{
		double DashLenCm = 0.0;
		double GapLenCm  = 0.0;
		double WidthCm   = 12.0;
	};
	ROADNET_API FPaintSpec Paint(EMark Mark);

	// ---- helpers -----------------------------------------------------------

	// Resultant grade of a surface carrying LongGrade along the road and
	// CrossGrade across it — formula [8.7], the grade in the steepest
	// direction. Both inputs are fractions.
	FORCEINLINE double ResultantGrade(double LongGrade, double CrossGrade)
	{
		return FMath::Sqrt(LongGrade * LongGrade + CrossGrade * CrossGrade);
	}

	// Drive every table above over its whole range and assert the properties the
	// generator relies on — monotonic in speed, correct direction of grade
	// correction, floors honoured. Run via RoadNet.StandardsSelfCheck.
	ROADNET_API bool SelfCheck(FString& OutReport);
}
