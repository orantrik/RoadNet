#pragma once
#include "CoreMinimal.h"
#include "Curve/GeneralPolygon2.h"

// ===========================================================================
// RoadNetMarkings — baked lane markings (§8.10 structured markings).
//
// Emits thin ribbon polygons along a road's centerline, split by paint colour
// so they can be meshed as two separately-coloured raised layers:
//   * WHITE  — every תקו"ה 2020 ch. 8 line: 801/805/803/808, crossings, 814.
//   * YELLOW — 807, the edge line where there is no kerb (stroked by the caller).
// ===========================================================================
struct FRoadCurves;
struct FRoadDef;

namespace RoadNetMarkings
{
	// One place this road's lines must end: a junction approach or exit, measured
	// where the road's centreline meets the junction area.
	struct FMarkingStop
	{
		FVector2D Pos = FVector2D::ZeroVector;  // on the centreline, at the junction-area boundary side
		double ExtraCm = 0.0;                   // paint stops this much further OUT from the junction
		bool bAhead = true;                     // the junction lies toward the road's END (+arc)
	};

	// A junction widening on one end of this road (RoadNetwork's FRoadNetArmWidening,
	// restated so this header does not pull in the network).
	struct FMarkingWidening
	{
		bool   bAtStart = true;
		int32  Side = 1;          // +1 = +offset (right) side, -1 = left
		double WidthCm = 0.0;     // extra width at the junction end
		double TaperCm = 0.0;     // back to zero over this length, after HoldCm
		double HoldCm = 0.0;      // full width this far from the junction end first (Table 5.1 `l`)
		int32  Lanes = 1;
		bool   bDrop = false;     // a carried-out through lane (exit side) rather than a turn bay
		bool   bFarSide = false;  // a far-side turn bay: it widens on the opposing side and pushes the centre line over
		bool   bGain = false;     // a through lane added toward a seam: its boundary is the wider road's 801
		bool   bArrows = true;    // 814 on the taper; false where the ending lane's full width lies on the other road
		bool   bSpeedChange = false; // acceleration or deceleration lane: 808 long, never 814
	};

	// A lane of this road that ends just past one of its ends (a seam drop). 814
	// goes on its full-width run before the taper (813-814 rule 3ג, fig. 5).
	struct FMarkingLaneEnd
	{
		bool   bAtStart = true;   // the seam is at the road's start
		double OffsetCm = 0.0;    // lane centre, +right of forward
		double WidthCm = 350.0;
		int32  Toward = -1;       // lateral side (+1 right, -1 left) of the lane it merges into
	};

	struct FRoadMarkingPlan
	{
		TArray<FMarkingStop> Stops;
		TArray<FMarkingWidening> Widenings;
		TArray<FMarkingLaneEnd> LaneEnds;
		int32 DesignKph = 50;
		double RoadLengthCm = 0.0;   // the arc length widenings are measured on
	};

	// Append this road's marking ribbons (closed polygons), routed by colour.
	//
	// bDriveOnLeft decides both which side of the road each direction runs on
	// and which colour the centre line takes.
	// bJointAtStart / bJointAtEnd say whether that end of the road meets a junction.
	// A crossing at a junction mouth must not stripe the direction that is LEAVING
	// that junction — the junction's own approach paint already covers that traffic,
	// and a second bar a few metres past the boundary reads as a double stop line.
	ROADNET_API void BuildRoadMarkings(
		const FRoadDef& Road,
		const FRoadCurves& Curves,
		bool bDriveOnLeft,
		bool bJointAtStart,
		bool bJointAtEnd,
		TArray<UE::Geometry::FGeneralPolygon2d>& OutWhite,
		TArray<UE::Geometry::FGeneralPolygon2d>& OutYellow,
		const FRoadMarkingPlan* Plan = nullptr);

	// 813 / 814 arrow glyph (תקו"ה 2020 813-814 fig. 6) as paint polygons. Mask bits:
	// 1 through, 2 left, 4 right, 8 U-turn. Tail is the arrow's rear end, Dir the
	// direction it points; Scale 0.75 is the narrow-lane size. bUTurnLeft says which
	// hand a U-turn sweeps to (left when driving on the right).
	ROADNET_API void EmitArrowGlyph(uint8 Mask, const FVector2D& Tail, const FVector2D& Dir,
		double Scale, bool bUTurnLeft, TArray<UE::Geometry::FGeneralPolygon2d>& Out);

	// The lane line at OffsetCm (+right of the road's forward direction) over the
	// LengthCm upstream of a junction stop, on BuildRoadMarkings' own path. OutCut
	// is a 40 cm corridor over that stretch (subtract it from the paint already
	// there); OutPaint the Dash:Gap marks of half-width HalfW laid from the stop.
	ROADNET_API void ReplaceApproachLine(const FRoadCurves& C, double OffsetCm,
		const FMarkingStop& Stop, double LengthCm, double Dash, double Gap, double HalfW,
		TArray<UE::Geometry::FGeneralPolygon2d>& OutCut, TArray<UE::Geometry::FGeneralPolygon2d>& OutPaint);

	// Dash an arbitrary polyline into ribbon polygons of total width 2*HalfWCm,
	// cutting DashLenCm of paint every DashLenCm+GapLenCm. Appends to Out.
	//
	// The road markings above dash lines they derive themselves from a lane
	// offset; this takes a path somebody else built. The junction turn guide
	// (LOT"M 809) is the reason it exists: its path is a lane-graph turn
	// movement, which has no lane offset to be derived from.
	ROADNET_API void DashPolyline(
		const TArray<FVector>& Path,
		double DashLenCm, double GapLenCm, double HalfWCm,
		TArray<UE::Geometry::FGeneralPolygon2d>& Out);

	// Drive DashPolyline over the cases that would break a junction turn guide:
	// degenerate inputs that must refuse rather than hang, the dash count over a
	// known length, and paint area roughly matching dash/(dash+gap) of the strip.
	// Run via RoadNet.DashSelfCheck.
	ROADNET_API bool DashSelfCheck(FString& OutReport);
}
