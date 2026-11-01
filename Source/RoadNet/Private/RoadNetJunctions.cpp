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
#include "RoadNetMath.h"
#include "RoadNetLanes.h"
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

	void ClassifyApproach(const FRoadNetJoint& Joint, int32 Arm, TArray<FApproachMove>& Out)
	{
		Out.Reset();
		if (!Joint.Arms.IsValidIndex(Arm)) { return; }
		const FRoadNetJointArm& From = Joint.Arms[Arm];

		int32 Best = INDEX_NONE;
		double BestAbs = TNumericLimits<double>::Max();
		for (int32 bi = 0; bi < Joint.Arms.Num(); ++bi)
		{
			if (bi == Arm) { continue; }
			const FRoadNetJointArm& To = Joint.Arms[bi];
			if (To.Road == From.Road && To.bAtStart == From.bAtStart) { continue; }
			if (!To.bLeavable) { continue; }
			const double Signed = TurnAngle(From.BearingRad, To.BearingRad);
			const double Abs = FMath::Abs(Signed);
			const bool bBetter = (Best == INDEX_NONE) || (Abs < BestAbs - 1e-6)
				|| (FMath::IsNearlyEqual(Abs, BestAbs, 1e-6) && From.bMain && To.bMain
					&& !(Joint.Arms[Best].bMain));
			if (bBetter) { Best = bi; BestAbs = Abs; }
		}

		for (int32 bi = 0; bi < Joint.Arms.Num(); ++bi)
		{
			if (bi == Arm) { continue; }
			const FRoadNetJointArm& To = Joint.Arms[bi];
			if (To.Road == From.Road && To.bAtStart == From.bAtStart) { continue; }
			if (!To.bLeavable) { continue; }
			FApproachMove M;
			M.Arm = bi;
			M.Signed = TurnAngle(From.BearingRad, To.BearingRad);
			M.AbsTurn = FMath::Abs(M.Signed);
			M.Move = Classify(From.BearingRad, To.BearingRad);
			if (bi == Best) { M.Move = EMovement::Through; }
			else if (M.Move == EMovement::Through)
			{
				M.Move = (M.Signed > 0.0) ? EMovement::BearRight : EMovement::BearLeft;
			}
			Out.Add(M);
		}
	}

	static void DropCenter(const TArray<FRoadNetLane>& Lanes, const TArray<int32>& In, bool bDrop, TArray<int32>& Out)
	{
		Out.Reset();
		for (int32 i : In)
		{
			if (bDrop && Lanes.IsValidIndex(i) && Lanes[i].Type == ERoadNetLaneType::CenterTurn) { continue; }
			Out.Add(i);
		}
	}

	void PairThrough(
		const TArray<FRoadNetLane>& EnterLanes, const TArray<int32>& Entering,
		const TArray<FRoadNetLane>& LeaveLanes, const TArray<int32>& Leaving,
		bool bExcludeCenterTurn,
		TArray<FThroughPair>& OutPairs, TArray<int32>& OutKerbSurplus, bool bInPlace)
	{
		TArray<int32> E, L;
		DropCenter(EnterLanes, Entering, bExcludeCenterTurn, E);
		DropCenter(LeaveLanes, Leaving, bExcludeCenterTurn, L);
		OutPairs.Reset();
		OutKerbSurplus.Reset();
		if (bInPlace)
		{
			const int32 N = FMath::Min(E.Num(), L.Num());
			for (int32 i = 0; i < N; ++i) { OutPairs.Add({ E[i], L[i] }); }
			for (int32 i = N; i < E.Num(); ++i) { OutKerbSurplus.Add(E[i]); }
			return;
		}
		const int32 Surplus = E.Num() - L.Num();
		if (Surplus > 0)
		{
			for (int32 k = 0; k < Surplus; ++k) { OutKerbSurplus.Add(E[k]); }
			for (int32 i = 0; i < L.Num(); ++i) { OutPairs.Add({ E[Surplus + i], L[i] }); }
		}
		else
		{
			for (int32 i = 0; i < E.Num(); ++i) { OutPairs.Add({ E[i], L[i - Surplus] }); }
		}
	}

	bool SeamKeepsWidth(const TArray<FRoadNetLane>& A, const TArray<FRoadNetLane>& B)
	{
		auto Width = [](const TArray<FRoadNetLane>& Ls)
		{
			double W = 0.0;
			for (const FRoadNetLane& L : Ls) { if (L.bDrivable()) { W += (double)L.Width; } }
			return W;
		};
		return FMath::Abs(Width(A) - Width(B)) <= 50.0;
	}

	ERoadNetTurnRole TokenRole(const FString& Token)
	{
		const FString T = Token.TrimStartAndEnd().ToLower();
		if (T == TEXT("left")) { return ERoadNetTurnRole::Left; }
		if (T == TEXT("right")) { return ERoadNetTurnRole::Right; }
		if (T == TEXT("through") || T == TEXT("straight")) { return ERoadNetTurnRole::Through; }
		if (T == TEXT("reverse") || T == TEXT("uturn")) { return ERoadNetTurnRole::UTurn; }
		if (T == TEXT("slight_left")) { return ERoadNetTurnRole::BearLeft; }
		if (T == TEXT("slight_right")) { return ERoadNetTurnRole::BearRight; }
		// ponytail: a combo ("left;through") falls to Auto and allows every movement;
		// a per-lane role set would narrow it.
		return ERoadNetTurnRole::Auto;
	}

	ERoadNetTurnRole EnteringLaneRole(const FRoadDef& Road, const FRoadNetLane& Lane,
		int32 Rank, int32 NumEntering, bool bAtStart, bool bDriveOnLeft)
	{
		if (Lane.TurnRole != ERoadNetTurnRole::Auto) { return Lane.TurnRole; }
		const TArray<FString>& Tokens = bAtStart ? Road.BackwardTurnTokens : Road.ForwardTurnTokens;
		if (Tokens.Num() != NumEntering || NumEntering == 0) { return ERoadNetTurnRole::Auto; }
		const int32 Tok = bDriveOnLeft ? Rank : (Tokens.Num() - 1 - Rank);
		return Tokens.IsValidIndex(Tok) ? TokenRole(Tokens[Tok]) : ERoadNetTurnRole::Auto;
	}

	void LaneCurveTangents(const FVector& P0, const FVector& Control, const FVector& P1,
		bool bHermite, bool bUTurn, FVector& OutLeave, FVector& OutArrive)
	{
		if (bHermite)
		{
			OutLeave = OutArrive = (P1 - P0) / 3.0;
			return;
		}
		if (bUTurn)
		{
			const FVector Bulge = Control - 0.5 * (P0 + P1);
			OutLeave = 4.0 * Bulge;
			OutArrive = -4.0 * Bulge;
			return;
		}
		OutLeave = 2.0 * (Control - P0);
		OutArrive = 2.0 * (P1 - Control);
	}

	FVector LaneCurvePoint(const FVector& P0, const FVector& Leave, const FVector& P1, const FVector& Arrive, double T)
	{
		const double T2 = T * T, T3 = T2 * T;
		return (2.0 * T3 - 3.0 * T2 + 1.0) * P0 + (T3 - 2.0 * T2 + T) * Leave
			+ (-2.0 * T3 + 3.0 * T2) * P1 + (T3 - T2) * Arrive;
	}

	void SortBranchesFromKerb(TArray<FSplitBranch>& Branches, const TArray<FApproachMove>& Moves, bool bDriveOnLeft)
	{
		auto SignedOf = [&Moves](int32 Arm)
		{
			for (const FApproachMove& M : Moves) { if (M.Arm == Arm) { return M.Signed; } }
			return 0.0;
		};
		Branches.Sort([&SignedOf, bDriveOnLeft](const FSplitBranch& A, const FSplitBranch& B)
		{
			return bDriveOnLeft ? SignedOf(A.Arm) < SignedOf(B.Arm) : SignedOf(A.Arm) > SignedOf(B.Arm);
		});
	}

	bool PlanMerge(const FRoadNetJoint& J, TFunctionRef<const TArray<FRoadNetLane>*(int32 Road)> LanesOf,
		bool bDriveOnLeft, FMergePlan& Out)
	{
		Out = FMergePlan();
		if (J.Arms.Num() < 3) { return false; }
		struct FSide { int32 Arm; TArray<int32> Lanes; double Turn; };
		TArray<FSide> Ins, Outs;
		for (int32 ai = 0; ai < J.Arms.Num(); ++ai)
		{
			const TArray<FRoadNetLane>* L = LanesOf(J.Arms[ai].Road);
			if (!L) { return false; }
			TArray<int32> E, Lv;
			OrderLanesFromKerb(*L, J.Arms[ai].bAtStart, true, bDriveOnLeft, E);
			OrderLanesFromKerb(*L, J.Arms[ai].bAtStart, false, bDriveOnLeft, Lv);
			if (E.Num() > 0 && Lv.Num() > 0) { return false; }
			for (int32 Li : E)  { if ((*L)[Li].Type == ERoadNetLaneType::CenterTurn) { return false; } }
			for (int32 Li : Lv) { if ((*L)[Li].Type == ERoadNetLaneType::CenterTurn) { return false; } }
			if (Lv.Num() > 0) { Outs.Add({ ai, MoveTemp(Lv), 0.0 }); }
			else if (E.Num() > 0) { Ins.Add({ ai, MoveTemp(E), 0.0 }); }
		}
		const bool bMergeShape = Outs.Num() == 1 && Ins.Num() >= 2;
		Out.bDiverge = Ins.Num() == 1 && Outs.Num() >= 2;
		if (!bMergeShape && !Out.bDiverge) { return false; }
		const FSide& One = Out.bDiverge ? Ins[0] : Outs[0];
		TArray<FSide>& Many = Out.bDiverge ? Outs : Ins;
		Out.Arm = One.Arm;
		Out.Lanes = One.Lanes;
		for (FSide& S : Many)
		{
			const int32 From = Out.bDiverge ? One.Arm : S.Arm;
			const int32 To = Out.bDiverge ? S.Arm : One.Arm;
			TArray<FApproachMove> Moves;
			ClassifyApproach(J, From, Moves);
			const FApproachMove* M = Moves.FindByPredicate([To](const FApproachMove& X) { return X.Arm == To; });
			if (!M || (M->Move != EMovement::Through && M->Move != EMovement::BearLeft && M->Move != EMovement::BearRight)) { return false; }
			S.Turn = TurnAngle(J.Arms[From].BearingRad, J.Arms[To].BearingRad);
		}
		// A kerb-side ramp heads in across the road and straightens out with a
		// kerb-ward turn (right when driving on the right); a kerb-side exit bears
		// kerb-ward too. Either way the median side turns most away from the kerb.
		Many.Sort([bDriveOnLeft](const FSide& A, const FSide& B) { return bDriveOnLeft ? A.Turn > B.Turn : A.Turn < B.Turn; });

		const int32 NOne = One.Lanes.Num();
		int32 Total = 0, P = 0;
		for (int32 i = 0; i < Many.Num(); ++i)
		{
			Total += Many[i].Lanes.Num();
			if (FMath::Abs(Many[i].Turn) < FMath::Abs(Many[P].Turn)) { P = i; }
		}
		// Slot 0 is the single arm's median-side lane. The other arms sit side by
		// side; when they have more lanes than it, the straightest keeps all its own.
		TArray<int32> Start;
		Start.SetNum(Many.Num());
		int32 Cursor = 0;
		for (int32 i = 0; i < Many.Num(); ++i) { Start[i] = Cursor; Cursor += Many[i].Lanes.Num(); }
		if (Total > NOne)
		{
			const int32 SP = FMath::Clamp(Start[P], 0, FMath::Max(0, NOne - Many[P].Lanes.Num()));
			const int32 Delta = SP - Start[P];
			for (int32 i = 0; i < Many.Num(); ++i) { Start[i] += Delta; }
		}
		for (int32 i = 0; i < Many.Num(); ++i)
		{
			FMergeShare Sh;
			Sh.Arm = Many[i].Arm;
			Sh.bKerbSide = (i == Many.Num() - 1);
			const TArray<int32>& E = Many[i].Lanes;
			for (int32 k = 0; k < E.Num(); ++k)
			{
				const int32 Lane = E[E.Num() - 1 - k];   // median side first
				const int32 Slot = Start[i] + k;
				if (Slot >= 0 && Slot < NOne)
				{
					const int32 Shared = One.Lanes[NOne - 1 - Slot];
					Sh.Pairs.Add(Out.bDiverge ? FThroughPair{ Shared, Lane } : FThroughPair{ Lane, Shared });
				}
				else { Sh.Surplus.Add(Lane); }
			}
			Out.Shares.Add(MoveTemp(Sh));
		}
		for (int32 Slot = NOne - 1; Slot >= Total; --Slot) { Out.Unused.Add(One.Lanes[NOne - 1 - Slot]); }
		return true;
	}

	void SplitBlocks(const TArray<int32>& Entering, const TArray<FSplitBranch>& Branches, TArray<FLaneBlock>& Out)
	{
		Out.Reset();
		const int32 N = Entering.Num();
		const int32 B = Branches.Num();
		if (N == 0 || B == 0) { return; }

		if (N < B)
		{
			for (int32 i = 0; i < B; ++i)
			{
				const int32 Lane = (B == 1) ? 0 : FMath::RoundToInt((double)i * (double)(N - 1) / (double)(B - 1));
				FLaneBlock Block;
				Block.Arm = Branches[i].Arm;
				Block.Move = Branches[i].Move;
				Block.Lanes.Add(Entering[FMath::Clamp(Lane, 0, N - 1)]);
				Out.Add(MoveTemp(Block));
			}
			return;
		}

		TArray<double> W;
		W.SetNum(B);
		double Sum = 0.0;
		for (int32 i = 0; i < B; ++i) { W[i] = (double)FMath::Max(1, Branches[i].Recv); Sum += W[i]; }
		TArray<int32> Sz;
		Sz.SetNum(B);
		int32 Got = 0;
		for (int32 i = 0; i < B; ++i)
		{
			Sz[i] = FMath::Clamp(FMath::RoundToInt((double)N * W[i] / Sum), 1, N);
			Got += Sz[i];
		}
		while (Got > N)
		{
			int32 Best = INDEX_NONE;
			for (int32 i = 0; i < B; ++i)
			{
				if (Sz[i] > 1 && (Best == INDEX_NONE || Sz[i] > Sz[Best])) { Best = i; }
			}
			if (Best == INDEX_NONE) { break; }
			--Sz[Best];
			--Got;
		}
		while (Got < N)
		{
			int32 Best = 0;
			for (int32 i = 1; i < B; ++i) { if (W[i] > W[Best]) { Best = i; } }
			++Sz[Best];
			++Got;
		}

		int32 Cursor = 0;
		for (int32 i = 0; i < B; ++i)
		{
			FLaneBlock Block;
			Block.Arm = Branches[i].Arm;
			Block.Move = Branches[i].Move;
			const int32 Take = FMath::Min(Sz[i], N - Cursor);
			for (int32 k = 0; k < Take; ++k) { Block.Lanes.Add(Entering[Cursor + k]); }
			Cursor += Take;
			Out.Add(MoveTemp(Block));
		}
	}
}

// ---------------------------------------------------------------------------

void URoadNetwork::BuildJunctionChannelization(FRoadNetRebuildContext& Ctx) const
{
	using namespace RoadNetJunctions;

	Ctx.ArmWidenings.Reset();
	Ctx.SeamShifts.Reset();
	Ctx.RequestedShiftHoldCm.Reset();
	Ctx.LateralMoves.Reset();
	Ctx.TangentialJoints.Reset();
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

	auto RoadLen = [this](int32 RoadIdx) { return Roads.IsValidIndex(RoadIdx) ? RoadNetMath::TotalLength(Roads[RoadIdx].Ref) : 0.0; };

	// The road beyond the far end of `Road` (the end opposite bNearAtStart) when
	// that end is a plain seam: two roads with the same lanes, so a shift or a
	// widening that outruns `Road` carries on into the next one unchanged.
	auto PlainSeamBeyond = [&](int32 Road, bool bNearAtStart, FRoadNetJointArm& OutNext) -> bool
	{
		for (const FRoadNetJoint& FJ : Ctx.Joints)
		{
			if (FJ.Arms.Num() != 2) { continue; }
			for (int32 fa = 0; fa < 2; ++fa)
			{
				const FRoadNetJointArm& Me = FJ.Arms[fa];
				const FRoadNetJointArm& Other = FJ.Arms[1 - fa];
				if (Me.Road != Road || Me.bAtStart == bNearAtStart || Other.Road == Road) { continue; }
				const TArray<FRoadNetLane>* A = LanesOf(Road);
				const TArray<FRoadNetLane>* B = LanesOf(Other.Road);
				if (!A || !B || A->Num() != B->Num()) { return false; }
				const double Mirror = (Me.bAtStart != Other.bAtStart) ? 1.0 : -1.0;
				for (const FRoadNetLane& La : *A)
				{
					const bool bMatch = B->ContainsByPredicate([&](const FRoadNetLane& Lb)
					{
						return FMath::Abs(Mirror * Lb.CenterOffset - La.CenterOffset) < 5.0 && FMath::Abs(Lb.Width - La.Width) < 5.0;
					});
					if (!bMatch) { return false; }
				}
				OutNext = Other;
				return true;
			}
		}
		return false;
	};

	// One shift per road end. A road shorter than its shift keeps moving at the
	// Table 4 rate through plain seams, so neither end of it steps sideways.
	// Elsewhere the far end stays where it is: the shift is gone 1 m before it.
	auto AddShift = [&](FRoadNetSeamShift Sh)
	{
		for (int32 Hop = 0; Hop < 8; ++Hop)
		{
			if (Ctx.SeamShifts.ContainsByPredicate([&Sh](const FRoadNetSeamShift& O) { return O.Road == Sh.Road && O.bAtStart == Sh.bAtStart; }))
			{
				return;
			}
			const double Len = RoadLen(Sh.Road);
			FRoadNetJointArm Next;
			if (Sh.HoldCm + Sh.TaperCm <= Len - 100.0 || Len <= 100.0 || !PlainSeamBeyond(Sh.Road, Sh.bAtStart, Next))
			{
				const double Room = FMath::Max(0.0, Len - 100.0);
				Sh.HoldCm = FMath::Clamp(FMath::Min(Sh.HoldCm, Room - Sh.TaperCm), 0.0, Room);
				Sh.TaperCm = FMath::Min(Sh.TaperCm, Room - Sh.HoldCm);
				Ctx.SeamShifts.Add(Sh);
				return;
			}
			// Over this road: full shift for the hold, then a ramp to what is still
			// to go at the far end, made of the near end's ramp out and the far
			// end's ramp in.
			const double Hold = FMath::Min(Sh.HoldCm, Len);
			const double Ramp = Len - Hold;
			const double Rest = (Sh.TaperCm > 0.0) ? Sh.ShiftCm * FMath::Max(0.0, 1.0 - Ramp / Sh.TaperCm) : (Ramp > 0.0 ? 0.0 : Sh.ShiftCm);
			FRoadNetSeamShift Near = Sh;
			Near.HoldCm = (Ramp > 0.0) ? Hold : Len + 100.0;
			Near.TaperCm = Ramp;
			Ctx.SeamShifts.Add(Near);
			if (Ramp > 0.0)
			{
				FRoadNetSeamShift Far = Sh;
				Far.bAtStart = !Sh.bAtStart;
				Far.ShiftCm = Rest;
				Far.HoldCm = 0.0;
				Far.TaperCm = Ramp;
				Ctx.SeamShifts.Add(Far);
			}
			if (FMath::Abs(Rest) < 1.0) { return; }
			const double Mirror = ((!Sh.bAtStart) != Next.bAtStart) ? 1.0 : -1.0;
			Sh.Road = Next.Road;
			Sh.bAtStart = Next.bAtStart;
			Sh.ShiftCm = Mirror * Rest;
			Sh.HoldCm -= Hold;
			Sh.TaperCm -= Ramp;
		}
	};

	// A widening that outruns its road carries on through plain seams at the same
	// taper, so it keeps its Table 5.1 / taper length and the carriageway has no
	// step where the roads meet.
	auto CarryAcrossSeams = [&](const FRoadNetArmWidening& First) -> bool
	{
		FRoadNetArmWidening C = First;
		bool bCarried = false;
		for (int32 Hop = 0; Hop < 8; ++Hop)
		{
			const double Len = RoadLen(C.Road);
			if (C.HoldCm + C.TaperCm <= Len) { break; }
			FRoadNetJointArm Next;
			if (!PlainSeamBeyond(C.Road, C.bAtStart, Next)) { break; }
			FRoadNetArmWidening N = C;
			N.Road = Next.Road;
			N.bAtStart = Next.bAtStart;
			N.FromRoad = INDEX_NONE;
			if ((!C.bAtStart) == Next.bAtStart)
			{
				N.Side = (C.Side == ERoadNetSide::Right) ? ERoadNetSide::Left : ERoadNetSide::Right;
			}
			const double Past = Len - C.HoldCm;
			if (Past <= 0.0) { N.HoldCm = C.HoldCm - Len; }
			else
			{
				N.HoldCm = 0.0;
				N.WidthCm = C.WidthCm * (1.0 - Past / C.TaperCm);
				N.TaperCm = C.TaperCm - Past;
			}
			Ctx.ArmWidenings.Add(N);
			bCarried = true;
			C = N;
		}
		return bCarried;
	};

	int32 Drops = 0, TurnBays = 0, CarriedOn = 0, ShortHolds = 0, InPlaceSeams = 0, Merges = 0, Diverges = 0;

	for (FRoadNetJoint& J : Ctx.Joints)
	{
		if (J.Arms.Num() < 2) { continue; }

		// ---- 1. through-lane surplus: carry it out, taper it away -----------
		// ClassifyApproach elects one Through per approach, so a crossroads still
		// balances both axes (each approach's opposite arm) while a Y widens only
		// the elected branch. A bent seam is that approach's only continuation,
		// so it is Through here even past the old 45° cut.
		// Two arms whose lanes continue into each other, at a seam or across a
		// junction. Shift is per arm, +right of that road's forward.
		struct FThroughAxis { int32 Arm[2] = { INDEX_NONE, INDEX_NONE }; double Shift[2] = { 0.0, 0.0 }; int32 Pairs = 0; };
		TArray<FThroughAxis, TInlineAllocator<4>> Axes;

		// A median wider on one side of a seam closes to the other's width over the
		// Table 4 length (803 rule ד), so the lanes beside it meet end to end. The
		// seam pairing below reads the closed offsets.
		double Squeeze[2] = { 0.0, 0.0 }, MedianCentre[2] = { 0.0, 0.0 };
		if (J.Arms.Num() == 2 && J.Arms[0].Road != J.Arms[1].Road)
		{
			const TArray<FRoadNetLane>* SeamLanes0 = LanesOf(J.Arms[0].Road);
			const TArray<FRoadNetLane>* SeamLanes1 = LanesOf(J.Arms[1].Road);
			if (SeamLanes0 && SeamLanes1)
			{
				const double Med0 = RoadNetLanes::MedianOf(*SeamLanes0, MedianCentre[0]);
				const double Med1 = RoadNetLanes::MedianOf(*SeamLanes1, MedianCentre[1]);
				if (FMath::Abs(Med0 - Med1) > 5.0)
				{
					const int32 WideArm = Med0 > Med1 ? 0 : 1;
					FRoadNetLateralMove MedianClose;
					MedianClose.Road     = J.Arms[WideArm].Road;
					MedianClose.bAtStart = J.Arms[WideArm].bAtStart;
					MedianClose.bSqueeze = true;
					MedianClose.PivotCm  = MedianCentre[WideArm];
					MedianClose.AmountCm = 0.5 * FMath::Abs(Med0 - Med1);
					MedianClose.TaperCm  = RoadNetStandards::LaneCountShiftLengthCm(
						FMath::Min(J.Arms[0].DesignSpeedKph, J.Arms[1].DesignSpeedKph), MedianClose.AmountCm);
					Ctx.LateralMoves.Add(MedianClose);
					Squeeze[WideArm] = MedianClose.AmountCm;
				}
			}
		}
		auto EndOffset = [&](int32 Arm, const FRoadNetLane& EndLane) -> double
		{
			const double EndOff = EndLane.CenterOffset;
			if (Arm < 0 || Arm > 1 || Squeeze[Arm] <= 0.0) { return EndOff; }
			const double Pivot = MedianCentre[Arm], Q = Squeeze[Arm];
			return EndOff > Pivot ? FMath::Max(Pivot, EndOff - Q) : FMath::Min(Pivot, EndOff + Q);
		};
		FMergePlan Merge;
		const bool bMerge = PlanMerge(J, [&LanesOf](int32 R) { return LanesOf(R); }, bDriveOnLeft, Merge);
		if (bMerge)
		{
			++(Merge.bDiverge ? Diverges : Merges);
			Ctx.TangentialJoints.Add((int32)(&J - Ctx.Joints.GetData()));
		}
		// A merge approach or an exit branch moves over so its lanes meet the lanes
		// they continue from or into (803 rule ד). Kerb-side lanes with no partner
		// line up with the acceleration or deceleration lane beside the kerb lane.
		// ponytail: one shift per road end, the mean over its lanes.
		auto ShiftShare = [&](const FMergeShare& Sh)
		{
			const FRoadNetJointArm& S = J.Arms[Merge.Arm];
			const FRoadNetJointArm& M = J.Arms[Sh.Arm];
			const TArray<FRoadNetLane>* SL = LanesOf(S.Road);
			const TArray<FRoadNetLane>* ML = LanesOf(M.Road);
			if (!SL || !ML || Merge.Lanes.Num() == 0) { return; }
			const double Mirror = (S.bAtStart != M.bAtStart) ? 1.0 : -1.0;
			double Gap = 0.0;
			int32 N = 0;
			for (const FThroughPair& Pr : Sh.Pairs)
			{
				const int32 Shared = Merge.bDiverge ? Pr.Enter : Pr.Leave;
				const int32 Mine = Merge.bDiverge ? Pr.Leave : Pr.Enter;
				Gap += Mirror * (*SL)[Shared].CenterOffset - (*ML)[Mine].CenterOffset;
				++N;
			}
			if (Sh.bKerbSide)
			{
				const int32 K = Merge.Lanes[0];
				const double Sign = KerbSide(*SL, K, S.bAtStart, /*bEntering*/Merge.bDiverge) == ERoadNetSide::Right ? 1.0 : -1.0;
				double Edge = (*SL)[K].CenterOffset + Sign * 0.5 * (*SL)[K].Width;
				for (int32 Li : Sh.Surplus)
				{
					const double W = (*ML)[Li].Width;
					Gap += Mirror * (Edge + Sign * 0.5 * W) - (*ML)[Li].CenterOffset;
					Edge += Sign * W;
					++N;
				}
			}
			if (N == 0) { return; }
			Gap /= (double)N;
			if (FMath::Abs(Gap) <= 5.0) { return; }
			FRoadNetSeamShift Shift;
			Shift.Road     = M.Road;
			Shift.bAtStart = M.bAtStart;
			Shift.ShiftCm  = Gap;
			Shift.TaperCm  = RoadNetStandards::LaneCountShiftLengthCm(FMath::Min(S.DesignSpeedKph, M.DesignSpeedKph), Gap);
			AddShift(Shift);
		};
		for (int32 ai = 0; ai < J.Arms.Num(); ++ai)
		{
			const FRoadNetJointArm& In = J.Arms[ai];
			const TArray<FRoadNetLane>* InLanes = LanesOf(In.Road);
			if (!InLanes) { continue; }

			TArray<FApproachMove> Moves;
			ClassifyApproach(J, ai, Moves);

			TArray<int32> Entering;
			OrderLanesFromKerb(*InLanes, In.bAtStart, /*bEntering*/true, bDriveOnLeft, Entering);

			bool bBear = false;
			for (const FApproachMove& M : Moves)
			{
				if (M.Move == EMovement::BearLeft || M.Move == EMovement::BearRight) { bBear = true; }
			}

			// bAccel: a merge's ramp lanes run on as an acceleration lane (Table 6.5,
			// keyed on the main road's speed) instead of a Table 5.1 drop.
			auto AddDrop = [&](int32 OutArm, const TArray<int32>& SurplusLanes, const TArray<FRoadNetLane>& OutLanes, const TArray<int32>& Leaving, bool bAccel = false)
			{
				if (SurplusLanes.Num() == 0 || Leaving.Num() == 0) { return; }
				const FRoadNetJointArm& Out = J.Arms[OutArm];
				double Extra = 0.0;
				for (int32 Src : SurplusLanes) { if (InLanes->IsValidIndex(Src)) { Extra += (*InLanes)[Src].Width; } }
				if (Extra <= 0.0) { return; }
				const int32 KerbLane = Leaving[0];
				const int32 Vd = bAccel ? Out.DesignSpeedKph : FMath::Min(In.DesignSpeedKph, Out.DesignSpeedKph);
				const double LaneW = FMath::Max(1.0, (double)OutLanes[KerbLane].Width);

				FRoadNetArmWidening W;
				W.Road     = Out.Road;
				W.bAtStart = Out.bAtStart;
				W.Side     = KerbSide(OutLanes, KerbLane, Out.bAtStart, /*bEntering*/false);
				W.WidthCm  = Extra;
				W.TaperCm  = (J.Arms.Num() == 2 ? RoadNetStandards::OpenRoadTaperLengthCm(Vd, LaneW, /*bMerge*/true)
					: RoadNetStandards::LaneDropTaperLengthCm(Vd, LaneW)) * (double)SurplusLanes.Num();
				W.Kind     = ERoadNetWideningKind::Drop;
				W.Lanes    = SurplusLanes.Num();
				if (J.Arms.Num() == 2)
				{
					W.FromRoad = In.Road;
					W.bFromAtStart = In.bAtStart;
					CarryAcrossSeams(W);
				}
				else
				{
					// Table 5.1: full width for `l` past the junction, then the taper.
					// An arm too short for both keeps the lane to its far node, where
					// it becomes that junction's kerb turn lane (808 rule א).
					// A dead end carries it off the network.
					// ponytail: measured from the node, not the junction-area edge; a
					// far node that is a seam, or a junction with no kerb turn from
					// this arm, falls back to dropping within the arm on a shortened hold.
					W.HoldCm = bAccel ? RoadNetStandards::AccelLaneLengthCm(Vd) : RoadNetStandards::LaneDropHoldLengthCm(Vd);
					if (bAccel) { W.TaperCm = RoadNetStandards::AccelLaneTaperRatio() * Extra; }
					W.bSpeedChange = bAccel;
					const double ArmLen = RoadLen(Out.Road);
					if (W.HoldCm + W.TaperCm > ArmLen && !CarryAcrossSeams(W))
					{
						const EMovement KerbTurn = bDriveOnLeft ? EMovement::Left : EMovement::Right;
						bool bFarKerbTurn = false, bFarNode = false;
						for (const FRoadNetJoint& FJ : Ctx.Joints)
						{
							for (int32 fa = 0; fa < FJ.Arms.Num(); ++fa)
							{
								if (&FJ == &J || FJ.Arms[fa].Road != Out.Road || FJ.Arms[fa].bAtStart == Out.bAtStart) { continue; }
								if (FJ.Arms.Num() < 2) { continue; }   // a dead end: the lane runs off the network
								bFarNode = true;
								if (FJ.Arms.Num() < 3) { continue; }
								TArray<FApproachMove> FarMoves;
								ClassifyApproach(FJ, fa, FarMoves);
								for (const FApproachMove& FM : FarMoves) { bFarKerbTurn |= FM.Move == KerbTurn; }
							}
						}
						if (bFarKerbTurn || !bFarNode)
						{
							W.bToNextNode = true;
							W.HoldCm = ArmLen + 100.0;
							W.TaperCm = 0.0;
						}
						else
						{
							W.HoldCm = FMath::Max(0.0, ArmLen - W.TaperCm - 1000.0);
							++ShortHolds;
						}
					}
				}
				Ctx.ArmWidenings.Add(W);
				++Drops;
				if (J.Kind == ERoadNetJointKind::Intersection || J.Kind == ERoadNetJointKind::Seam)
				{
					J.Kind = ERoadNetJointKind::Split;
				}
			};

			// A seam into MORE lanes: the added lanes are the kerb-most ones (§5.2.4),
			// opened on the narrower road by a diverge taper ending at the seam.
			auto AddGain = [&](int32 OutArm, const TArray<FThroughPair>& Pairs, const TArray<FRoadNetLane>& OutLanes, const TArray<int32>& Leaving)
			{
				if (Entering.Num() == 0 || Pairs.Num() == 0) { return; }
				TArray<int32> Recv;
				for (int32 Li : Leaving) { if (OutLanes[Li].Type != ERoadNetLaneType::CenterTurn) { Recv.Add(Li); } }
				const int32 Gain = Recv.Num() - Pairs.Num();
				if (Gain <= 0) { return; }
				double Extra = 0.0;
				for (int32 k = 0; k < Gain; ++k) { Extra += OutLanes[Recv[k]].Width; }
				if (Extra <= 0.0) { return; }
				const FRoadNetJointArm& Out = J.Arms[OutArm];
				const int32 Vd = FMath::Min(In.DesignSpeedKph, Out.DesignSpeedKph);

				FRoadNetArmWidening W;
				W.Road     = In.Road;
				W.bAtStart = In.bAtStart;
				W.Side     = KerbSide(*InLanes, Entering[0], In.bAtStart, /*bEntering*/true);
				W.WidthCm  = Extra;
				W.TaperCm  = RoadNetStandards::OpenRoadTaperLengthCm(Vd, Extra / (double)Gain, /*bMerge*/false) * (double)Gain;
				W.Kind     = ERoadNetWideningKind::Gain;
				W.Lanes    = Gain;
				Ctx.ArmWidenings.Add(W);
			};

			if (bMerge && Merge.bDiverge)
			{
				if (Merge.Arm != ai) { continue; }
				for (const FMergeShare& Sh : Merge.Shares)
				{
					const FRoadNetJointArm& Out = J.Arms[Sh.Arm];
					const TArray<FRoadNetLane>* OutLanes = LanesOf(Out.Road);
					if (!OutLanes) { continue; }
					ShiftShare(Sh);
					// Exit lanes no entering lane feeds open beside the kerb lane as a
					// parallel deceleration lane: Table 6.2 to 25 km/h at full width,
					// entered over the §6.3.4 opening taper (1:10 below 80 km/h, 1:15 at
					// or above, the same ratios as LeftTurnTaperRatio).
					if (!Sh.bKerbSide || Sh.Surplus.Num() == 0 || Entering.Num() == 0) { continue; }
					double Extra = 0.0;
					for (int32 Li : Sh.Surplus) { Extra += (*OutLanes)[Li].Width; }
					if (Extra <= 0.0) { continue; }
					const int32 Vd = In.DesignSpeedKph;
					FRoadNetArmWidening W;
					W.Road         = In.Road;
					W.bAtStart     = In.bAtStart;
					W.Side         = KerbSide(*InLanes, Entering[0], In.bAtStart, /*bEntering*/true);
					W.WidthCm      = Extra;
					W.HoldCm       = RoadNetStandards::DecelLengthM(Vd, /*bRightTurn*/true, /*bToStop*/false) * 100.0;
					W.TaperCm      = RoadNetStandards::LeftTurnTaperRatio(Vd) * Extra;
					W.Kind         = bDriveOnLeft ? ERoadNetWideningKind::LeftBay : ERoadNetWideningKind::RightBay;
					W.Lanes        = Sh.Surplus.Num();
					W.bSpeedChange = true;
					Ctx.ArmWidenings.Add(W);
				}
			}
			else if (bMerge)
			{
				const FMergeShare* Sh = Merge.Shares.FindByPredicate([ai](const FMergeShare& S) { return S.Arm == ai; });
				const FRoadNetJointArm& Out = J.Arms[Merge.Arm];
				const TArray<FRoadNetLane>* OutLanes = LanesOf(Out.Road);
				if (Sh && OutLanes)
				{
					// Kerb-side lanes with no lane of their own run on as an acceleration lane.
					if (Sh->bKerbSide) { AddDrop(Merge.Arm, Sh->Surplus, *OutLanes, Merge.Lanes, /*bAccel*/true); }
					ShiftShare(*Sh);
				}
			}
			else if (!bBear)
			{
				for (const FApproachMove& M : Moves)
				{
					if (M.Move != EMovement::Through) { continue; }
					const FRoadNetJointArm& Out = J.Arms[M.Arm];
					if (Out.Road == In.Road) { continue; }
					const TArray<FRoadNetLane>* OutLanes = LanesOf(Out.Road);
					if (!OutLanes) { continue; }
					TArray<int32> Leaving;
					OrderLanesFromKerb(*OutLanes, Out.bAtStart, /*bEntering*/false, bDriveOnLeft, Leaving);
					// A turn-only lane is not through traffic, so it never needs a lane beyond the junction.
					TArray<int32> ThroughEntering;
					for (int32 Rank = 0; Rank < Entering.Num(); ++Rank)
					{
						const ERoadNetTurnRole Role = Roads.IsValidIndex(In.Road)
							? EnteringLaneRole(Roads[In.Road], (*InLanes)[Entering[Rank]], Rank, Entering.Num(), In.bAtStart, bDriveOnLeft)
							: ERoadNetTurnRole::Auto;
						if (J.Arms.Num() == 2 || Role == ERoadNetTurnRole::Auto || Role == ERoadNetTurnRole::Through)
						{
							ThroughEntering.Add(Entering[Rank]);
						}
					}
					TArray<FThroughPair> Pairs;
					TArray<int32> Surplus;
					const bool bInPlace = J.Arms.Num() == 2 && SeamKeepsWidth(*InLanes, *OutLanes);
					PairThrough(*InLanes, ThroughEntering, *OutLanes, Leaving, /*bExcludeCenterTurn*/true, Pairs, Surplus, bInPlace);
					if (bInPlace) { InPlaceSeams += (Surplus.Num() > 0 || Leaving.Num() > Pairs.Num()) ? 1 : 0; }
					else { AddDrop(M.Arm, Surplus, *OutLanes, Leaving); }
					if (J.Arms.Num() == 2 && !bInPlace) { AddGain(M.Arm, Pairs, *OutLanes, Leaving); }
					// A T stem elects one of its two turns as Through; only arms that
					// run straight on line their lanes up.
					if (J.Arms.Num() > 2 && Classify(In.BearingRad, Out.BearingRad) != EMovement::Through) { continue; }
					// How far this approach's lanes sit from the lanes they continue
					// into. Roads whose forward directions oppose at the joint see each
					// other's offsets mirrored.
					const int32 Lo = FMath::Min(ai, M.Arm), Hi = FMath::Max(ai, M.Arm);
					FThroughAxis* Ax = Axes.FindByPredicate([Lo, Hi](const FThroughAxis& X) { return X.Arm[0] == Lo && X.Arm[1] == Hi; });
					if (!Ax)
					{
						Ax = &Axes.AddDefaulted_GetRef();
						Ax->Arm[0] = Lo;
						Ax->Arm[1] = Hi;
					}
					const int32 Me = (ai == Lo) ? 0 : 1;
					const double Mirror = (In.bAtStart != Out.bAtStart) ? 1.0 : -1.0;
					for (const FThroughPair& Pr : Pairs)
					{
						const double Gap = Mirror * EndOffset(M.Arm, (*OutLanes)[Pr.Leave]) - EndOffset(ai, (*InLanes)[Pr.Enter]);
						Ax->Shift[Me] += Gap;
						Ax->Shift[1 - Me] -= Mirror * Gap;
						++Ax->Pairs;
					}
				}
			}
			else
			{
				TArray<FSplitBranch> Branches;
				for (const FApproachMove& M : Moves)
				{
					if (M.Move != EMovement::Through && M.Move != EMovement::BearLeft && M.Move != EMovement::BearRight) { continue; }
					const FRoadNetJointArm& Out = J.Arms[M.Arm];
					if (Out.Road == In.Road) { continue; }
					const TArray<FRoadNetLane>* OutLanes = LanesOf(Out.Road);
					if (!OutLanes) { continue; }
					TArray<int32> Leaving;
					OrderLanesFromKerb(*OutLanes, Out.bAtStart, /*bEntering*/false, bDriveOnLeft, Leaving);
					int32 Recv = 0;
					for (int32 Li : Leaving)
					{
						if ((*OutLanes)[Li].Type != ERoadNetLaneType::CenterTurn) { ++Recv; }
					}
					FSplitBranch B;
					B.Arm = M.Arm;
					B.Move = M.Move;
					B.Recv = Recv;
					Branches.Add(B);
				}
				SortBranchesFromKerb(Branches, Moves, bDriveOnLeft);

				TArray<int32> EnterNoCenter;
				for (int32 E : Entering)
				{
					if ((*InLanes)[E].Type != ERoadNetLaneType::CenterTurn) { EnterNoCenter.Add(E); }
				}
				TArray<FLaneBlock> Blocks;
				SplitBlocks(EnterNoCenter, Branches, Blocks);
				for (const FLaneBlock& Block : Blocks)
				{
					const FRoadNetJointArm& Out = J.Arms[Block.Arm];
					const TArray<FRoadNetLane>* OutLanes = LanesOf(Out.Road);
					if (!OutLanes) { continue; }
					TArray<int32> Leaving;
					OrderLanesFromKerb(*OutLanes, Out.bAtStart, /*bEntering*/false, bDriveOnLeft, Leaving);
					TArray<int32> LeaveNoCenter;
					for (int32 Li : Leaving)
					{
						if ((*OutLanes)[Li].Type != ERoadNetLaneType::CenterTurn) { LeaveNoCenter.Add(Li); }
					}
					const int32 SurplusN = Block.Lanes.Num() - LeaveNoCenter.Num();
					if (SurplusN <= 0) { continue; }
					TArray<int32> Surplus;
					for (int32 k = 0; k < SurplusN && k < Block.Lanes.Num(); ++k) { Surplus.Add(Block.Lanes[k]); }
					AddDrop(Block.Arm, Surplus, *OutLanes, LeaveNoCenter);
				}
			}
		}

		// The narrower road of a through axis carries the transition (803 rule ד,
		// figs. 9 and 15), at a seam or across a junction: its reference line moves
		// over so the continuing lanes meet lane to lane, not centre line to centre
		// line. ponytail: lanes of unequal width give each pair its own offset; the
		// mean is used, so a mixed-width pair can keep a few cm of step.
		for (const FThroughAxis& Ax : Axes)
		{
			const FRoadNetJointArm& A0 = J.Arms[Ax.Arm[0]];
			const FRoadNetJointArm& A1 = J.Arms[Ax.Arm[1]];
			if (Ax.Pairs == 0 || A0.Road == A1.Road) { continue; }
			auto Drivable = [&](int32 RoadIdx)
			{
				int32 N = 0;
				if (const TArray<FRoadNetLane>* Ls = LanesOf(RoadIdx))
				{
					for (const FRoadNetLane& Ln : *Ls) { N += (Ln.bDrivable() && Ln.Type != ERoadNetLaneType::CenterTurn) ? 1 : 0; }
				}
				return N;
			};
			const int32 N0 = Drivable(A0.Road), N1 = Drivable(A1.Road);
			const int32 Narrow = (N0 != N1) ? (N0 < N1 ? 0 : 1) : (A0.Road < A1.Road ? 0 : 1);
			const FRoadNetJointArm& NA = Narrow == 0 ? A0 : A1;
			const double Shift = Ax.Shift[Narrow] / (double)Ax.Pairs;
			if (FMath::Abs(Shift) > 5.0)
			{
				FRoadNetSeamShift Sh;
				Sh.Road     = NA.Road;
				Sh.bAtStart = NA.bAtStart;
				Sh.ShiftCm  = Shift;
				Sh.TaperCm  = RoadNetStandards::LaneCountShiftLengthCm(FMath::Min(A0.DesignSpeedKph, A1.DesignSpeedKph), Shift);
				// Full shift up to this arm's stop line. The junction clip that places
				// it is built from these curves, so Rebuild measures it and re-runs
				// the plan with the measured hold; the first pass seeds it with the
				// widest crossing road, its footway, the kerb return and the crosswalk.
				if (J.Arms.Num() > 2)
				{
					const int32 Key = NA.Road * 2 + (NA.bAtStart ? 1 : 0);
					if (const double* Measured = Ctx.MeasuredShiftHoldCm.Find(Key)) { Sh.HoldCm = *Measured; }
					else
					{
						for (const FRoadNetJointArm& Cross : J.Arms)
						{
							if (&Cross == &A0 || &Cross == &A1 || !Roads.IsValidIndex(Cross.Road)) { continue; }
							const FRoadNetLaneSpec& CrossLanes = Roads[Cross.Road].Lanes;
							Sh.HoldCm = FMath::Max(Sh.HoldCm, (double)CrossLanes.HalfWidthCm() + CrossLanes.MaxSidewalkWidth() + 2500.0);
						}
					}
					Ctx.RequestedShiftHoldCm.Add(Key, Sh.HoldCm);
				}
				AddShift(Sh);
			}
		}

		// ---- 2. turn bays on every approach to a real junction ---------------
		// A turn bay is width ADDED to the approach, never width taken from a
		// through lane (§6.3 right turns, §7 left turns).
		if (J.Arms.Num() < 3) { continue; }
		// A roundabout node is an entry or exit: traffic turns by circulating, so
		// neither the ring nor its arms take a turn bay there.
		if (J.Arms.ContainsByPredicate([this](const FRoadNetJointArm& A)
			{ return Roads.IsValidIndex(A.Road) && Roads[A.Road].bRoundabout; })) { continue; }

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

			// Classify is geometric. The kerb-side turn is Right when driving on the
			// right and Left when driving on the left; the far-side turn is the other.
			const EMovement KerbTurn = bDriveOnLeft ? EMovement::Left : EMovement::Right;
			bool bWantsKerb = false, bWantsFar = false;
			for (int32 bi = 0; bi < J.Arms.Num(); ++bi)
			{
				if (bi == ai || J.Arms[bi].Road == In.Road || !J.Arms[bi].bLeavable) { continue; }
				const EMovement M = Classify(In.BearingRad, J.Arms[bi].BearingRad);
				if (M == KerbTurn) { bWantsKerb = true; }
				else if (M == EMovement::Left || M == EMovement::Right) { bWantsFar = true; }
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

			if (bWantsKerb)
			{
				// §6.3 — the kerb-side turn bay sits outside the through lane, on
				// the kerb side, entered over the §6 approach taper.
				FRoadNetArmWidening W;
				W.Road     = In.Road;
				W.bAtStart = In.bAtStart;
				W.Side     = Kerb;
				W.WidthCm  = LaneW;
				W.TaperCm  = RoadNetStandards::LaneDropTaperLengthCm(In.DesignSpeedKph, LaneW);
				W.Kind     = bDriveOnLeft ? ERoadNetWideningKind::LeftBay : ERoadNetWideningKind::RightBay;
				W.Lanes    = 1;
				Ctx.ArmWidenings.Add(W);
				++TurnBays;
			}
			if (bWantsFar)
			{
				// §7 — the far-side turn bay is taken from the centre side, entered
				// over the 1:10 / 1:15 taper rather than the drop taper.
				FRoadNetArmWidening W;
				W.Road     = In.Road;
				W.bAtStart = In.bAtStart;
				W.Side     = Centre;
				W.WidthCm  = LaneW;
				W.TaperCm  = RoadNetStandards::LeftTurnTaperRatio(In.DesignSpeedKph) * LaneW;
				W.Kind     = bDriveOnLeft ? ERoadNetWideningKind::RightBay : ERoadNetWideningKind::LeftBay;
				W.Lanes    = 1;
				W.bFarSide = true;
				// On a divided road the bay is cut into the median, which keeps 2.50 m
				// (§7.4.3); only what the median cannot give moves the far half over.
				const double CentreSign = (Centre == ERoadNetSide::Right) ? 1.0 : -1.0;
				const double BayEdge = (*InLanes)[KerbLane].CenterOffset + CentreSign * 0.5 * LaneW;
				double MedC = 0.0;
				const double MedW = RoadNetLanes::MedianOf(*InLanes, MedC);
				if (MedW > 0.0 && FMath::Abs(MedC - CentreSign * 0.5 * MedW - BayEdge) < 5.0)
				{
					W.MedianCm = FMath::Clamp(MedW - RoadNetStandards::MedianKeptAtLeftTurnCm(), 0.0, LaneW);
				}
				if (W.WidthCm - W.MedianCm > 0.5)
				{
					FRoadNetLateralMove FarShift;
					FarShift.Road     = In.Road;
					FarShift.bAtStart = In.bAtStart;
					FarShift.bSqueeze = false;
					FarShift.PivotCm  = BayEdge;
					FarShift.Dir      = (int32)CentreSign;
					FarShift.AmountCm = W.WidthCm - W.MedianCm;
					FarShift.HoldCm   = W.HoldCm;
					FarShift.TaperCm  = W.TaperCm;
					Ctx.LateralMoves.Add(FarShift);
				}
				Ctx.ArmWidenings.Add(W);
				++TurnBays;
			}
		}
	}

	// A lane carried to a far junction already is that approach's kerb turn lane.
	{
		const ERoadNetWideningKind KerbBay = bDriveOnLeft ? ERoadNetWideningKind::LeftBay : ERoadNetWideningKind::RightBay;
		TArray<FRoadNetArmWidening> Carried;
		for (const FRoadNetArmWidening& W : Ctx.ArmWidenings) { if (W.bToNextNode) { Carried.Add(W); } }
		const int32 Before = Ctx.ArmWidenings.Num();
		Ctx.ArmWidenings.RemoveAll([&](const FRoadNetArmWidening& B)
		{
			return B.Kind == KerbBay && Carried.ContainsByPredicate([&](const FRoadNetArmWidening& Cw)
				{ return Cw.Road == B.Road && Cw.bAtStart != B.bAtStart; });
		});
		TurnBays -= Before - Ctx.ArmWidenings.Num();
		CarriedOn = Carried.Num();
	}

	if (Drops > 0 || TurnBays > 0 || InPlaceSeams > 0 || Merges > 0 || Diverges > 0)
	{
		UE_LOG(LogRoadNet, Log,
			TEXT("[RoadNet][JUNCTION] channelized %d joint(s): %d lane drop(s) carried outside the junction (%d kept to the next node, %d on a shortened Table 5.1 hold), %d turn bay(s) added, %d seam lane change(s) in place, %d tangential merge(s), %d tangential exit(s)"),
			Ctx.Joints.Num(), Drops, CarriedOn, ShortHolds, TurnBays, InPlaceSeams, Merges, Diverges);
	}
	for (const FRoadNetSeamShift& Sh : Ctx.SeamShifts)
	{
		UE_LOG(LogRoadNet, Log, TEXT("[RoadNet][JUNCTION]   shift: road %d %s end, %.0f cm over %.0f cm"),
			Sh.Road, Sh.bAtStart ? TEXT("start") : TEXT("far"), Sh.ShiftCm, Sh.TaperCm);
	}
	for (const FRoadNetLateralMove& Mv : Ctx.LateralMoves)
	{
		UE_LOG(LogRoadNet, Log, TEXT("[RoadNet][JUNCTION]   %s: road %d %s end, %.0f cm about %.0f over %.0f + %.0f cm"),
			Mv.bSqueeze ? TEXT("median closes") : TEXT("far bay moves the far half"), Mv.Road,
			Mv.bAtStart ? TEXT("start") : TEXT("far"), Mv.AmountCm, Mv.PivotCm, Mv.HoldCm, Mv.TaperCm);
	}
	for (const FRoadNetArmWidening& W : Ctx.ArmWidenings)
	{
		if (!Roads.IsValidIndex(W.Road) || Roads[W.Road].Ref.Num() < 2) { continue; }
		const FVector End = W.bAtStart ? Roads[W.Road].Ref[0] : Roads[W.Road].Ref.Last();
		static const TCHAR* KindName[] = { TEXT("drop"), TEXT("right bay"), TEXT("left bay"), TEXT("gain") };
		const int32 K = (int32)W.Kind;
		UE_LOG(LogRoadNet, Log,
			TEXT("[RoadNet][JUNCTION]   %s: road %d %s end (%.0f,%.0f), %d lane(s) %.0f cm on the %s, hold %.0f, taper %.0f%s"),
			K >= 0 && K < 4 ? KindName[K] : TEXT("?"), W.Road, W.bAtStart ? TEXT("start") : TEXT("far"), End.X, End.Y,
			W.Lanes, W.WidthCm, W.Side == ERoadNetSide::Right ? TEXT("right") : TEXT("left"), W.HoldCm, W.TaperCm,
			W.bToNextNode ? TEXT(", kept to the next node") : (!W.bSpeedChange ? TEXT("") : W.Kind == ERoadNetWideningKind::Drop ? TEXT(", acceleration lane") : TEXT(", deceleration lane")));
	}
}
