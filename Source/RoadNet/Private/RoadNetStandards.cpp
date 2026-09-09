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
}
