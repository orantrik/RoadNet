#pragma once
#include "CoreMinimal.h"
#include "Curve/GeneralPolygon2.h"
#include "RoadNetTypes.h"   // ERoadNetJunctionPreset

// ===========================================================================
// RoadNetJunctionMarks — junction marking geometry (§2 junctions).
//
// For one junction (a joint with >=3 arms) and a chosen preset, emits the
// approach paint — stop / give-way bar, zebra crosswalk — clipped to sit just
// outside the junction area, plus placeholder traffic-signal placements. Purely
// geometric; engine types only, no RoadBLD dependency.
// ===========================================================================
namespace RoadNetJunctionMarks
{
	// One approach of a junction: where the road's centreline crosses the
	// junction-area boundary (the stop-line position) and the OUTWARD direction
	// (away from the junction, back up the approach), plus the carriageway
	// half-width. Derived directly from the junction clip region so every
	// junction shape (T / X / Y / roundabout / multi-arm) is handled uniformly.
	struct FApproach
	{
		FVector2D StopPos = FVector2D::ZeroVector;
		FVector2D Outward = FVector2D(1, 0);   // unit, away from the junction
		double    HalfWidthCm = 300.0;

		// Does any lane on this arm ENTER the junction?
		//
		// Without this an arm is just a place where a centreline crosses the
		// junction boundary, so every arm got striped — including a one-way arm
		// whose lanes only LEAVE. A stop bar there faces no traffic at all. The
		// handedness flag cannot answer this: it says which half of a two-way arm
		// is the entering half, not whether the arm has entering traffic to begin
		// with. Defaults true so a caller that leaves it alone keeps the old
		// behaviour.
		bool bHasEnteringTraffic = true;

		// Lateral span of the ENTERING carriageway, measured +right of the inbound
		// direction (cm). On a divided road this starts at the median edge, so the
		// bar stops there instead of running from the reference line — which on a
		// dual carriageway means starting inside the median.
		//
		// EnterHiCm <= EnterLoCm means "not known": fall back to the handedness
		// rule and stripe the entering half of HalfWidthCm.
		double EnterLoCm = 0.0;
		double EnterHiCm = 0.0;
	};

	// A placeholder traffic-signal placement (world cm + facing yaw).
	struct FSignal
	{
		FVector Location = FVector::ZeroVector;
		float   YawDeg   = 0.f;
	};

	// One zebra crossing band: stripes run ALONG travel and repeat laterally to
	// cover the whole carriageway, so the band itself is handedness-agnostic.
	//
	// Shared with mid-block crossings (FRoadDef::Crossings) so a crossing painted
	// away from a junction is identical to one at a junction mouth.
	ROADNET_API void EmitZebra(
		const FVector2D& BandCenter,
		const FVector2D& Travel,      // unit, direction traffic moves through the band
		double HalfWidthCm,           // half the carriageway width to span
		double DepthCm,               // band depth along travel
		TArray<UE::Geometry::FGeneralPolygon2d>& OutWhite);

	// A solid stop bar across the lateral interval [LoCm, HiCm] of the carriageway,
	// measured along the unit direction Lateral from StopPos. An interval rather
	// than a half-width because on a divided road the bar must start at the median
	// edge, and on a one-way arm it must cover every lane rather than half of them.
	// Travel only sets the bar's thin axis, so it is direction-agnostic.
	ROADNET_API void EmitStopBar(
		const FVector2D& StopPos,
		const FVector2D& Travel,
		const FVector2D& Lateral,
		double LoCm, double HiCm,
		TArray<UE::Geometry::FGeneralPolygon2d>& OutWhite);

	// Length of a junction zebra along the road, from roadnet.CrosswalkLengthCm.
	ROADNET_API double CrosswalkLengthCm();

	// How far OUTWARD from FApproach::StopPos an approach's paint reaches: past the
	// zebra and past the stop bar behind it. Anything that has to clear the approach
	// paint — trimming the centre line back, for one — must ask for this rather than
	// keep its own copy of the layout, because the crossing length is a setting and a
	// second copy would silently stop matching the first time it changed.
	ROADNET_API double ApproachPaintReachCm();

	// Build paint + signals for one junction from its precomputed approaches.
	// White paint (bars/stripes) is APPENDED to OutWhite; signal placements are
	// APPENDED to OutSignals.
	//
	// bDriveOnLeft decides which half of each approach the stop bar, give-way
	// dashes and signal head belong to — the half that traffic enters on.
	ROADNET_API void BuildJoint(
		const FVector2D& Center,
		double CenterZ,
		const TArray<FApproach>& Approaches,
		ERoadNetJunctionPreset Preset,
		bool bDriveOnLeft,
		TArray<UE::Geometry::FGeneralPolygon2d>& OutWhite,
		TArray<FSignal>& OutSignals);
}
