#pragma once
#include "CoreMinimal.h"
#include "RoadNetTypes.h"

struct FRoadNetJoint;

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
		BearLeft,
		BearRight,
		Left,
		Right,
		UTurn
	};

	// One other arm, as seen from an approach. Exactly one entry is Through:
	// the smallest |turn|, with the elected main axis breaking ties. Any other
	// arm the raw classifier called Through becomes a bear.
	struct FApproachMove
	{
		int32 Arm = INDEX_NONE;
		EMovement Move = EMovement::Through;
		double AbsTurn = 0.0;
		double Signed = 0.0;
	};

	struct FThroughPair
	{
		int32 Enter = INDEX_NONE;
		int32 Leave = INDEX_NONE;
	};

	// Near-straight branches of one approach, kerb side first.
	struct FSplitBranch
	{
		int32 Arm = INDEX_NONE;
		EMovement Move = EMovement::Through;
		int32 Recv = 0;
	};

	struct FLaneBlock
	{
		int32 Arm = INDEX_NONE;
		EMovement Move = EMovement::Through;
		TArray<int32> Lanes;
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

	// Exactly one Through. Arms nothing can drive away on are not destinations.
	ROADNET_API void ClassifyApproach(const FRoadNetJoint& Joint, int32 Arm, TArray<FApproachMove>& Out);

	// Pairs plus the kerb-side entering lanes that do not fit. Lanes line up from
	// the centre side, so with more leaving lanes the unpaired ones are the
	// kerb-most (§5.2.4: a lane is added or dropped on the right). Center-turn lanes
	// are left out of both sides when bExcludeCenterTurn is set, and only here:
	// the bay test still counts them.
	// bInPlace pairs from the kerb instead (see SeamKeepsWidth): the unpaired
	// lanes are then the inner ones, beside the opposing lanes that end.
	ROADNET_API void PairThrough(
		const TArray<FRoadNetLane>& EnterLanes, const TArray<int32>& Entering,
		const TArray<FRoadNetLane>& LeaveLanes, const TArray<int32>& Leaving,
		bool bExcludeCenterTurn,
		TArray<FThroughPair>& OutPairs, TArray<int32>& OutKerbSurplus, bool bInPlace = false);

	// A seam whose two carriageways are equally wide (two-way 1+1 into one-way 2,
	// 2+1 into 1+2): a lane added in one direction takes the width of a lane that
	// ends in the other, so nothing widens or shifts and lanes continue in place.
	ROADNET_API bool SeamKeepsWidth(const TArray<FRoadNetLane>& A, const TArray<FRoadNetLane>& B);

	// An OSM turn:lanes token as a lane role. "none", an empty slot and a combo
	// ("left;through") are Auto: the lane allows every movement.
	ROADNET_API ERoadNetTurnRole TokenRole(const FString& Token);

	// The role of the entering lane at kerb rank Rank: the lane's authored role,
	// else the road's turn tokens for that direction when they match the lane
	// count, else Auto. bAtStart names the road end the lanes enter at.
	ROADNET_API ERoadNetTurnRole EnteringLaneRole(const FRoadDef& Road, const FRoadNetLane& Lane,
		int32 Rank, int32 NumEntering, bool bAtStart, bool bDriveOnLeft);

	// Contiguous kerb-order blocks, one per branch. Too few entering lanes and
	// the boundary lane is shared (it appears in two blocks).
	ROADNET_API void SplitBlocks(
		const TArray<int32>& Entering, const TArray<FSplitBranch>& BranchesKerbFirst,
		TArray<FLaneBlock>& Out);

	// Kerb-side branch first: rightmost when driving on the right, leftmost on the left.
	ROADNET_API void SortBranchesFromKerb(
		TArray<FSplitBranch>& Branches, const TArray<FApproachMove>& Moves, bool bDriveOnLeft);

	// A tangential merge (on-ramp; one-way roads joining into one): one arm only
	// leaves, every other arm only enters and reaches it as its Through or a bear.
	// The approaches share the leaving lanes side by side, median side first. The
	// straightest approach keeps all its lanes, so a ramp's lanes are the ones
	// left over (כרך 2 §5.4, Table 6.5: they become an acceleration lane).
	// A tangential diverge (exit) is the mirror: one arm only enters, the others
	// only leave, and the branches share the entering lanes the same way, so the
	// main road keeps its lanes and an exit lane nobody feeds is a deceleration
	// lane (Table 6.2, 808 rule ג fig. 21).
	struct FMergeShare
	{
		int32 Arm = INDEX_NONE;
		TArray<FThroughPair> Pairs;
		// Merge: this approach's entering lanes with no leaving lane of their own.
		// Diverge: this branch's leaving lanes no entering lane feeds.
		TArray<int32> Surplus;
		bool bKerbSide = false;    // the kerb-most share
	};
	struct FMergePlan
	{
		bool bDiverge = false;
		int32 Arm = INDEX_NONE;        // the single arm: leaving for a merge, entering for a diverge
		TArray<int32> Lanes;           // its lanes, kerb first
		TArray<FMergeShare> Shares;    // the other arms, median side first
		TArray<int32> Unused;          // its kerb-most lanes no share uses
	};
	ROADNET_API bool PlanMerge(const FRoadNetJoint& Joint,
		TFunctionRef<const TArray<FRoadNetLane>*(int32 Road)> LanesOf, bool bDriveOnLeft, FMergePlan& Out);

	// Hermite end tangents of one lane-graph curve. Every reader of an edge's
	// shape (spline export, turn guide, checker) uses this, so they agree.
	//  - quadratic: the exact cubic, 2(C - P0) and 2(P1 - C)
	//  - bHermite: chord / 3 at both ends
	//  - U-turn (not bHermite): a half circle, Control = chord midpoint +
	//    forward * radius; one cubic spans 180° with handles 4/3 r, so the
	//    Hermite tangents are ±4 (Control - midpoint)
	ROADNET_API void LaneCurveTangents(const FVector& P0, const FVector& Control, const FVector& P1,
		bool bHermite, bool bUTurn, FVector& OutLeave, FVector& OutArrive);

	ROADNET_API FVector LaneCurvePoint(const FVector& P0, const FVector& Leave,
		const FVector& P1, const FVector& Arrive, double T);
}
