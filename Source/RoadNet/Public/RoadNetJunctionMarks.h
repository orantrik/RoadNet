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

	// Build paint + signals for one junction from its precomputed approaches.
	// White paint (bars/stripes) is APPENDED to OutWhite; signal placements are
	// APPENDED to OutSignals.
	ROADNET_API void BuildJoint(
		const FVector2D& Center,
		double CenterZ,
		const TArray<FApproach>& Approaches,
		ERoadNetJunctionPreset Preset,
		TArray<UE::Geometry::FGeneralPolygon2d>& OutWhite,
		TArray<FSignal>& OutSignals);
}
