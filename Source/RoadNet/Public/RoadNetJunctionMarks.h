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

	// A solid stop bar across the half of the carriageway that traffic ENTERS on.
	// EnterSide is the unit lateral direction of that half.
	ROADNET_API void EmitStopBar(
		const FVector2D& StopPos,
		const FVector2D& Travel,
		const FVector2D& EnterSide,
		double HalfWidthCm,
		TArray<UE::Geometry::FGeneralPolygon2d>& OutWhite);

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
