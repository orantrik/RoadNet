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

	// ---- helpers -----------------------------------------------------------

	// Resultant grade of a surface carrying LongGrade along the road and
	// CrossGrade across it — formula [8.7], the grade in the steepest
	// direction. Both inputs are fractions.
	FORCEINLINE double ResultantGrade(double LongGrade, double CrossGrade)
	{
		return FMath::Sqrt(LongGrade * LongGrade + CrossGrade * CrossGrade);
	}
}
