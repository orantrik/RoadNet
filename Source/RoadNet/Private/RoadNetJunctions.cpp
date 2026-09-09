// ===========================================================================
// RoadNetJunctions.cpp — junction movement vocabulary + the channelization
// stage that turns arm mismatches into WIDENINGS instead of pinches.
//
// The rule that shapes this whole file (כרך 2 §5.2.1): the through-lane count
// inside a junction must never fall below the count arriving, and no lane is
// narrowed at a junction. A wide arm meeting a narrow one is therefore correct
// to arrive at full width — the mismatch is resolved by carrying the surplus
// lane OUT of the junction and tapering it away beyond, per §5.2.4 (always the
// rightmost lane) over the Table 5.1 length. Turn demand is met the same way,
// by adding a bay, never by stealing width from a through lane.
// ===========================================================================
#include "RoadNetJunctions.h"
#include "RoadNetwork.h"
#include "RoadNetStandards.h"
#include "RoadNetLog.h"

namespace RoadNetJunctions
{
	double TurnAngle(double FromBearingRad, double ToBearingRad)
	{
		// FromBearingRad points outward along the arm the traffic came in on,
		// so the heading of that traffic at the node is the reverse of it.
		return FMath::UnwindRadians(ToBearingRad - (FromBearingRad + PI));
	}

	EMovement Classify(double FromBearingRad, double ToBearingRad)
	{
		const double Turn = TurnAngle(FromBearingRad, ToBearingRad);
		const double Abs  = FMath::Abs(Turn);

		if (Abs >= FMath::DegreesToRadians(kUTurnThresholdDeg)) { return EMovement::UTurn; }
		if (Abs <  FMath::DegreesToRadians(kTurnThresholdDeg))  { return EMovement::Through; }

		// Unreal is left-handed with +X forward and +Y right, so atan2(Y, X)
		// grows clockwise seen from above — a positive turn is a right turn.
		return (Turn > 0.0) ? EMovement::Right : EMovement::Left;
	}

	double KerbOffset(const FRoadNetLane& Lane, bool bForwardAlongArc, bool bDriveOnLeft)
	{
		// CenterOffset is + to the right of the road's own right axis. Flip it
		// for traffic running against +arc and the value becomes "metres to the
		// driver's right"; flip again under drive-on-left and it becomes
		// "metres toward the kerb", which is the ordering §5.2.4 speaks in.
		double S = bForwardAlongArc ? Lane.CenterOffset : -Lane.CenterOffset;
		if (bDriveOnLeft) { S = -S; }
		return S;
	}

	void OrderLanesFromKerb(
		const TArray<FRoadNetLane>& Lanes,
		bool bAtStart,
		bool bEntering,
		bool bDriveOnLeft,
		TArray<int32>& OutOrdered)
	{
		OutOrdered.Reset();

		// At the road's START, traffic running +arc LEAVES the node and traffic
		// running −arc ARRIVES; at the END it is the other way round.
		const bool bWantForward = bAtStart ? !bEntering : bEntering;

		for (int32 i = 0; i < Lanes.Num(); ++i)
		{
			const FRoadNetLane& L = Lanes[i];
			if (!L.bDrivable()) { continue; }
			const bool bRuns = bWantForward ? L.bTravelsForward(bDriveOnLeft)
			                                : L.bTravelsBackward(bDriveOnLeft);
			if (bRuns) { OutOrdered.Add(i); }
		}

		OutOrdered.Sort([&Lanes, bWantForward, bDriveOnLeft](int32 A, int32 B)
		{
			return KerbOffset(Lanes[A], bWantForward, bDriveOnLeft)
			     > KerbOffset(Lanes[B], bWantForward, bDriveOnLeft);
		});
	}
}

// ---------------------------------------------------------------------------

void URoadNetwork::BuildJunctionChannelization(FRoadNetRebuildContext& Ctx) const
{
	using namespace RoadNetJunctions;

	Ctx.ArmWidenings.Reset();
	if (!bChannelizeJunctions) { return; }

	// Resolved lanes per road, kept across joints — a road touches two of them
	// and ResolveLanes allocates.
	//
	// Reserved up front because the loop below holds a pointer into this map
	// for one arm while looking up another: without the reservation the second
	// insert could rehash and leave the first pointer dangling. Only road
	// indices are ever added, so Roads.Num() is the true upper bound.
	TMap<int32, TArray<FRoadNetLane>> LaneCache;
	LaneCache.Reserve(Roads.Num());
	auto LanesOf = [this, &LaneCache](int32 RoadIdx) -> const TArray<FRoadNetLane>*
	{
		if (const TArray<FRoadNetLane>* Hit = LaneCache.Find(RoadIdx)) { return Hit; }
		if (!Roads.IsValidIndex(RoadIdx)) { return nullptr; }
		return &LaneCache.Add(RoadIdx, Roads[RoadIdx].Lanes.ResolveLanes(bDriveOnLeft));
	};

	// Which offset side of a road its kerb sits on, at one end, for traffic
	// heading the given way. Widening for a right-turn bay or a dropped lane
	// always grows the carriageway on the kerb side (§5.2.4, §6.3).
	auto KerbSide = [this](const TArray<FRoadNetLane>& Lanes, int32 KerbLane, bool bAtStart, bool bEntering)
	{
		const bool bWantForward = bAtStart ? !bEntering : bEntering;
		const double Off = KerbOffset(Lanes[KerbLane], bWantForward, bDriveOnLeft);
		// KerbOffset is the lane's offset already flipped into the kerb frame;
		// undo just the direction flip to recover which side of the reference
		// line the kerb is on.
		const double Signed = (bWantForward ? 1.0 : -1.0) * (bDriveOnLeft ? -Off : Off);
		return (Signed >= 0.0) ? ERoadNetSide::Right : ERoadNetSide::Left;
	};

	int32 Drops = 0, TurnBays = 0;

	for (FRoadNetJoint& J : Ctx.Joints)
	{
		if (J.Arms.Num() < 2) { continue; }

		// ---- 1. through-lane surplus: carry it out, taper it away -----------
		// Every straight continuation is balanced, not just the elected main
		// axis: at a crossroads both axes run through, and a lane vanishing on
		// the minor one is the same §5.2.1 violation as on the major one.
		for (int32 ai = 0; ai < J.Arms.Num(); ++ai)
		{
			for (int32 bi = 0; bi < J.Arms.Num(); ++bi)
			{
				if (ai == bi) { continue; }
				const FRoadNetJointArm& In  = J.Arms[ai];
				const FRoadNetJointArm& Out = J.Arms[bi];
				if (In.Road == Out.Road) { continue; }
				if (Classify(In.BearingRad, Out.BearingRad) != EMovement::Through) { continue; }

				const TArray<FRoadNetLane>* InLanes  = LanesOf(In.Road);
				const TArray<FRoadNetLane>* OutLanes = LanesOf(Out.Road);
				if (!InLanes || !OutLanes) { continue; }

				TArray<int32> Entering, Leaving;
				OrderLanesFromKerb(*InLanes,  In.bAtStart,  /*bEntering*/true,  bDriveOnLeft, Entering);
				OrderLanesFromKerb(*OutLanes, Out.bAtStart, /*bEntering*/false, bDriveOnLeft, Leaving);

				const int32 Surplus = Entering.Num() - Leaving.Num();
				if (Surplus <= 0 || Leaving.Num() == 0) { continue; }

				// §5.2.1: the junction must not swallow the surplus. Carry it
				// out of the far arm at full width, then §5.2.4 drops it on the
				// rightmost lane over the Table 5.1 taper.
				const int32 KerbLane = Leaving[0];
				double Extra = 0.0;
				for (int32 k = 0; k < Surplus; ++k)
				{
					const int32 Src = Entering[k];   // the surplus lanes are the kerb-side ones
					Extra += (*InLanes)[Src].Width;
				}

				const int32 Vd = FMath::Min(In.DesignSpeedKph, Out.DesignSpeedKph);
				const double LaneW = FMath::Max(1.0, (double)(*OutLanes)[KerbLane].Width);

				FRoadNetArmWidening W;
				W.Road     = Out.Road;
				W.bAtStart = Out.bAtStart;
				W.Side     = KerbSide(*OutLanes, KerbLane, Out.bAtStart, /*bEntering*/false);
				W.WidthCm  = Extra;
				W.TaperCm  = RoadNetStandards::LaneDropTaperLengthCm(Vd, LaneW) * (double)Surplus;
				Ctx.ArmWidenings.Add(W);
				++Drops;

				// A junction where the arms disagree on lane count IS a split;
				// the kind is defined in RoadNetTypes.h and was never assigned.
				if (J.Kind == ERoadNetJointKind::Intersection || J.Kind == ERoadNetJointKind::Seam)
				{
					J.Kind = ERoadNetJointKind::Split;
				}
			}
		}

		// ---- 2. turn bays on every approach to a real junction ---------------
		// A turn bay is width ADDED to the approach, never width taken from a
		// through lane (§6.3 right turns, §7 left turns).
		if (J.Arms.Num() < 3) { continue; }

		for (int32 ai = 0; ai < J.Arms.Num(); ++ai)
		{
			const FRoadNetJointArm& In = J.Arms[ai];

			// §6/§7 warrant turn lanes by turning VOLUME, which no import
			// carries, so road class stands in for it. Without this a residential
			// grid would grow a turn bay on every side street at every corner,
			// which is both wrong and enormous. Tertiary and above only.
			if ((int32)In.Class > (int32)ERoadNetClass::Tertiary) { continue; }

			const TArray<FRoadNetLane>* InLanes = LanesOf(In.Road);
			if (!InLanes) { continue; }

			TArray<int32> Entering;
			OrderLanesFromKerb(*InLanes, In.bAtStart, /*bEntering*/true, bDriveOnLeft, Entering);
			if (Entering.Num() == 0) { continue; }

			bool bWantsRight = false, bWantsLeft = false;
			for (int32 bi = 0; bi < J.Arms.Num(); ++bi)
			{
				if (bi == ai || J.Arms[bi].Road == In.Road) { continue; }
				switch (Classify(In.BearingRad, J.Arms[bi].BearingRad))
				{
				case EMovement::Right: bWantsRight = true; break;
				case EMovement::Left:  bWantsLeft  = true; break;
				default: break;
				}
			}

			// A single-lane approach has no lane to spare for a turn, which is
			// exactly the case the standard says to widen for. A multi-lane
			// approach already has lanes that can be signed as turn lanes.
			const bool bSingleLane = (Entering.Num() <= 1);
			if (!bSingleLane) { continue; }

			const int32 KerbLane = Entering[0];
			const double LaneW = FMath::Max(1.0, (double)(*InLanes)[KerbLane].Width);
			const ERoadNetSide Kerb = KerbSide(*InLanes, KerbLane, In.bAtStart, /*bEntering*/true);
			const ERoadNetSide Centre = (Kerb == ERoadNetSide::Right) ? ERoadNetSide::Left : ERoadNetSide::Right;

			if (bWantsRight)
			{
				// §6.3 — the right-turn bay sits outside the through lane, on
				// the kerb side, entered over the §6 approach taper.
				FRoadNetArmWidening W;
				W.Road     = In.Road;
				W.bAtStart = In.bAtStart;
				W.Side     = Kerb;
				W.WidthCm  = LaneW;
				W.TaperCm  = RoadNetStandards::LaneDropTaperLengthCm(In.DesignSpeedKph, LaneW);
				Ctx.ArmWidenings.Add(W);
				++TurnBays;
			}
			if (bWantsLeft)
			{
				// §7 — the left-turn bay is taken from the centre side, entered
				// over the 1:10 / 1:15 taper rather than the drop taper.
				FRoadNetArmWidening W;
				W.Road     = In.Road;
				W.bAtStart = In.bAtStart;
				W.Side     = Centre;
				W.WidthCm  = LaneW;
				W.TaperCm  = RoadNetStandards::LeftTurnTaperRatio(In.DesignSpeedKph) * LaneW;
				Ctx.ArmWidenings.Add(W);
				++TurnBays;
			}
		}
	}

	if (Drops > 0 || TurnBays > 0)
	{
		UE_LOG(LogRoadNet, Log,
			TEXT("[RoadNet][JUNCTION] channelized %d joint(s): %d lane drop(s) carried outside the junction, %d turn bay(s) added"),
			Ctx.Joints.Num(), Drops, TurnBays);
	}
}
