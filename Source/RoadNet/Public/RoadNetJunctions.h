#pragma once
#include "CoreMinimal.h"
#include "RoadNetTypes.h"

// ===========================================================================
// RoadNetJunctions — the geometry vocabulary junction design is written in.
//
// Two ideas do all the work, and both stages (lane matching in BuildLaneGraph,
// channelization in BuildJunctionChannelization) share them so they cannot
// disagree about which lane is "the rightmost" or what counts as a right turn.
//
//  1. MOVEMENT — through / left / right, from the bearing change at the node.
//  2. KERB RANK — a lane's position counted from the kerb, in the driver's
//     own frame. This is the ordering כרך 2 §5.2.4 means by "the rightmost
//     lane", and it is the one ordering that survives both travel direction
//     and drive-on-left without being negated twice.
// ===========================================================================
namespace RoadNetJunctions
{
	enum class EMovement : uint8
	{
		Through,
		Left,
		Right,
		UTurn
	};

	// Beyond this much bearing change a movement stops being "through" and
	// becomes a turn. כרך 2 §3.1 keeps junction angles inside 70°–110°, so a
	// real through movement is always well inside 45°.
	inline constexpr double kTurnThresholdDeg = 45.0;

	// Past this the movement has doubled back — a U-turn, not a turn.
	inline constexpr double kUTurnThresholdDeg = 135.0;

	// Classify travelling INTO the node along FromArm and back OUT along ToArm.
	// Both bearings are OUTWARD from the node (FRoadNetJointArm::BearingRad).
	ROADNET_API EMovement Classify(double FromBearingRad, double ToBearingRad);

	// Signed bearing change of that movement, radians. Positive = clockwise
	// seen from above, which in Unreal's left-handed frame is a right turn.
	ROADNET_API double TurnAngle(double FromBearingRad, double ToBearingRad);

	// A lane's lateral position measured from the KERB, in the frame of a
	// driver travelling along it: larger = closer to the kerb, so rank 0 after
	// a descending sort is the rightmost lane in a drive-on-right network and
	// the leftmost one in a drive-on-left network.
	//
	// bForwardAlongArc says whether the traffic being ordered runs with the
	// road's +arc direction (first reference point to last), not what the lane
	// is capable of — a two-way lane is ordered from whichever side is asking.
	ROADNET_API double KerbOffset(const FRoadNetLane& Lane, bool bForwardAlongArc, bool bDriveOnLeft);

	// Indices into Lanes of the drivable lanes carrying traffic in the given
	// direction along the road, ordered kerb-first (index 0 = kerb lane).
	//
	// bEntering selects traffic heading INTO the node at the end named by
	// bAtStart; false selects traffic leaving it.
	ROADNET_API void OrderLanesFromKerb(
		const TArray<FRoadNetLane>& Lanes,
		bool bAtStart,
		bool bEntering,
		bool bDriveOnLeft,
		TArray<int32>& OutOrdered);
}
