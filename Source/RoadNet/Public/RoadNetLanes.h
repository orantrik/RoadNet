#pragma once
#include "CoreMinimal.h"
#include "RoadNetTypes.h"

// ===========================================================================
// RoadNetLanes — lane geometry helpers (§12.1).
//
// Turn a resolved FRoadNetLane + a road's resampled reference line into a
// world-space lane centreline / lateral offset. Used by the lane-connectivity
// graph (§12.2) and (later) per-lane ribbon rendering. Engine-only, no RoadBLD.
// ===========================================================================
namespace RoadNetLanes
{
	// Signed lateral offset (cm, +right) of a lane centre at arc distance DistCm.
	// Uniform lanes return CenterOffset; lanes with authored Left/RightEdge knots
	// return the mean of the two interpolated edge offsets.
	ROADNET_API double LaneOffsetAt(const FRoadNetLane& Lane, double DistCm);

	// Build a lane's centreline in world cm by offsetting the resampled reference
	// line laterally (per-vertex, using the local right axis). Z is carried from
	// the reference vertex.
	ROADNET_API void BuildLaneCenterline(
		const TArray<FVector>& SampledRef, const FRoadNetLane& Lane, TArray<FVector>& Out);
}
