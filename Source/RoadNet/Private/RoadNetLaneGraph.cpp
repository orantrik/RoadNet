// Lane graph (LANE_GRAPH_PLAN.md). One mouth, one pairing, one curve per edge.
// ResolveLanes is not touched. The checker reports; it does not repair.
#include "RoadNetwork.h"
#include "RoadNetJunctions.h"
#include "RoadNetLanes.h"
#include "RoadNetMath.h"
#include "RoadNetTiles.h"
#include "RoadNetLog.h"
#include "EngineUtils.h"
#include "RoadNetActor.h"
#include "RoadNetMouths.h"

namespace
{
	constexpr double kZGapCm = 350.0; // same threshold as the grade-zone partition

	uint32 HashQuant(double V)
	{
		return (uint32)FMath::RoundToInt(V / 10.0);
	}

	uint32 HashJoint(const FRoadNetJoint& J, const TArray<FRoadDef>& Roads, int32 Zone)
	{
		TArray<uint32> Parts;
		Parts.Reserve(J.Arms.Num() * 2 + 1);
		for (const FRoadNetJointArm& Arm : J.Arms)
		{
			if (!Roads.IsValidIndex(Arm.Road) || Roads[Arm.Road].Ref.Num() < 1) { continue; }
			const FVector& P = Arm.bAtStart ? Roads[Arm.Road].Ref[0] : Roads[Arm.Road].Ref.Last();
			Parts.Add(HashQuant(P.X));
			Parts.Add(HashQuant(P.Y));
		}
		Parts.Sort();
		uint32 H = GetTypeHash(Zone);
		for (uint32 P : Parts) { H = HashCombine(H, P); }
		return H;
	}

	uint32 HashLane(uint32 JointKey, int32 Arm, int32 Kerb, bool bEntering, bool bVirtual, int32 Virt)
	{
		uint32 H = HashCombine(JointKey, GetTypeHash(Arm));
		H = HashCombine(H, GetTypeHash(Kerb));
		H = HashCombine(H, GetTypeHash(bEntering ? 1 : 0));
		H = HashCombine(H, GetTypeHash(bVirtual ? 1 : 0));
		return HashCombine(H, GetTypeHash(Virt));
	}

	int32 ClosestXY(const TArray<FVector>& Pts, const FVector& P)
	{
		int32 Best = 0;
		double D = TNumericLimits<double>::Max();
		for (int32 i = 0; i < Pts.Num(); ++i)
		{
			const double D2 = FVector::DistSquaredXY(Pts[i], P);
			if (D2 < D) { D = D2; Best = i; }
		}
		return Best;
	}

	FVector UnitXY(const FVector& V)
	{
		FVector O(V.X, V.Y, 0.0);
		if (!O.Normalize()) { O = FVector::ForwardVector; }
		return O;
	}

	void MakeShape(const FVector& P0, const FVector& T0, const FVector& P1, const FVector& T1,
		bool bUTurn, double LaneW, FVector& C, bool& bHermite)
	{
		const FVector Chord = P1 - P0;
		const double Gap = FVector(Chord.X, Chord.Y, 0.0).Size();
		if (bUTurn)
		{
			bHermite = Gap < FMath::Max(50.0, LaneW);
			if (bHermite) { C = (P0 + P1) * 0.5; return; }
			FVector Perp(-Chord.Y, Chord.X, 0.0);
			if (!Perp.Normalize()) { Perp = FVector::RightVector; }
			if (FVector::DotProduct(Perp, T0) < 0.0) { Perp *= -1.0; }
			C = (P0 + P1) * 0.5 + Perp * (Gap * 0.5);
			C.Z = 0.5 * (P0.Z + P1.Z);
			return;
		}

		const FVector2D D0(T0.X, T0.Y);
		const FVector2D D1(T1.X, T1.Y);
		const FVector2D R(P1.X - P0.X, P1.Y - P0.Y);
		const double Det = D0.X * D1.Y - D0.Y * D1.X;
		const double A = (FMath::Abs(Det) < 1e-4) ? 0.0 : (R.X * D1.Y - R.Y * D1.X) / Det;
		const double B = (FMath::Abs(Det) < 1e-4) ? 0.0 : (D0.X * R.Y - D0.Y * R.X) / Det;
		if (FMath::Abs(Det) < 0.08 || A <= 0.0 || B <= 0.0)
		{
			bHermite = true;
			C = (P0 + P1) * 0.5;
			return;
		}
		bHermite = false;
		C = P0 + FVector(T0.X, T0.Y, 0.0) * A;
		C.Z = FMath::Lerp(P0.Z, P1.Z, 0.5);
	}

	bool RoleAllows(ERoadNetTurnRole Role, ERoadNetTurnRole Kind, ERoadNetTurnRole Bear)
	{
		if (Role == ERoadNetTurnRole::None) { return false; }
		if (Role == ERoadNetTurnRole::Auto) { return true; }
		if (Role == ERoadNetTurnRole::BearLeft || Role == ERoadNetTurnRole::BearRight) { return Role == Bear; }
		return Role == Kind;
	}

	ERoadNetTurnRole KindOf(RoadNetJunctions::EMovement M)
	{
		using namespace RoadNetJunctions;
		switch (M)
		{
		case EMovement::Left:  return ERoadNetTurnRole::Left;
		case EMovement::Right: return ERoadNetTurnRole::Right;
		case EMovement::UTurn: return ERoadNetTurnRole::UTurn;
		default:               return ERoadNetTurnRole::Through;
		}
	}

	ERoadNetTurnRole BearOf(RoadNetJunctions::EMovement M)
	{
		using namespace RoadNetJunctions;
		if (M == EMovement::BearLeft) { return ERoadNetTurnRole::BearLeft; }
		if (M == EMovement::BearRight) { return ERoadNetTurnRole::BearRight; }
		return ERoadNetTurnRole::Auto;
	}
}

void URoadNetwork::BuildLaneGraph(FRoadNetRebuildContext& Ctx) const
{
	Ctx.LaneConnections.Reset();
	Ctx.ArmMouths.Reset();
	if (!bBuildLaneGraph) { return; }

	// This pass's reports. A full rebuild replaces the previous set; a windowed
	// one drops only the codes this function owns, so a stale I7 cannot linger.
	TArray<FString> PassReports;
	auto Report = [&PassReports](const FString& S) { PassReports.AddUnique(S); };

	using namespace RoadNetJunctions;

	struct FCache
	{
		TArray<FRoadNetLane> Lanes;
		TArray<TArray<FVector>> CL;
	};
	TMap<int32, FCache> Cache;
	Cache.Reserve(Roads.Num());
	auto Get = [&](int32 RoadIdx) -> const FCache*
	{
		if (const FCache* Hit = Cache.Find(RoadIdx)) { return Hit; }
		const FRoadCurves* C = Ctx.Curves.Find(RoadIdx);
		if (!C || C->Sampled.Num() < 2 || !Roads.IsValidIndex(RoadIdx)) { return nullptr; }
		FCache New;
		New.Lanes = Roads[RoadIdx].Lanes.ResolveLanes(bDriveOnLeft);
		New.CL.SetNum(New.Lanes.Num());
		for (int32 i = 0; i < New.Lanes.Num(); ++i)
		{
			RoadNetLanes::BuildLaneCenterline(C->Sampled, New.Lanes[i], New.CL[i], &C->Moves);
		}
		return &Cache.Add(RoadIdx, MoveTemp(New));
	};

	TMap<int32, int32> RoadZone;
	for (int32 z = 0; z < Ctx.Zones.Num(); ++z)
	{
		for (int32 r : Ctx.Zones[z]) { RoadZone.Add(r, z); }
	}

	auto ZoneOf = [&](const FRoadNetJoint& J) -> int32
	{
		for (const FRoadNetJointArm& Arm : J.Arms)
		{
			if (const int32* Z = RoadZone.Find(Arm.Road)) { return *Z; }
		}
		return 0;
	};

	// Mouths. The stop-line crossing when the arm leaves a clip polygon; the
	// road end when it never does (a seam, or a road that stays inside).
	auto MouthFor = [&](int32 ji, int32 ai) -> FRoadNetRebuildContext::FArmMouth
	{
		FRoadNetRebuildContext::FArmMouth M;
		M.Joint = ji;
		M.Arm = ai;
		const FRoadNetJoint& J = Ctx.Joints[ji];
		const FRoadNetJointArm& Arm = J.Arms[ai];
		const FRoadCurves* C = Ctx.Curves.Find(Arm.Road);
		const FVector End = (C && C->Sampled.Num() > 0)
			? (Arm.bAtStart ? C->Sampled[0] : C->Sampled.Last())
			: (Roads.IsValidIndex(Arm.Road)
				? (Arm.bAtStart ? Roads[Arm.Road].Ref[0] : Roads[Arm.Road].Ref.Last())
				: FVector(J.Location.X, J.Location.Y, J.Z));
		M.Pos = End;
		M.bNode = true;
		M.TravelIn = UnitXY(FVector(J.Location.X, J.Location.Y, J.Z) - End);

		const int32* Zone = RoadZone.Find(Arm.Road);
		if (C && Zone && Ctx.ZoneJunctionClip.IsValidIndex(*Zone))
		{
			const double Half = Roads.IsValidIndex(Arm.Road)
				? FMath::Max(50.0, (double)Roads[Arm.Road].Lanes.HalfWidthCm()) : 300.0;
			double Best = TNumericLimits<double>::Max();
			for (const UE::Geometry::FGeneralPolygon2d& GP : Ctx.ZoneJunctionClip[*Zone])
			{
				TArray<RoadNetMouths::FCrossing> Hits;
				RoadNetMouths::ClipCrossings(C->Sampled, GP, Half, Hits);
				for (const RoadNetMouths::FCrossing& Hit : Hits)
				{
					const double D = FVector2D::DistSquared(Hit.Boundary, FVector2D(End.X, End.Y));
					if (D < Best)
					{
						Best = D;
						M.Pos = FVector(Hit.Stop.X, Hit.Stop.Y, Hit.Z);
						M.bNode = false;
						M.TravelIn = UnitXY(FVector(-Hit.OutN.X, -Hit.OutN.Y, 0.0));
					}
				}
			}
		}
		if (C && C->Sampled.Num() >= 2)
		{
			const int32 N = C->Sampled.Num();
			double Along = 0.0;
			if (Arm.bAtStart)
			{
				for (int32 i = 1; i < N; ++i)
				{
					Along += FVector::Dist2D(C->Sampled[i - 1], C->Sampled[i]);
					if (FVector::DistSquaredXY(C->Sampled[i], M.Pos) <= FVector::DistSquaredXY(C->Sampled[i - 1], M.Pos))
					{
						break;
					}
				}
			}
			else
			{
				for (int32 i = N - 1; i > 0; --i)
				{
					Along += FVector::Dist2D(C->Sampled[i], C->Sampled[i - 1]);
					if (FVector::DistSquaredXY(C->Sampled[i - 1], M.Pos) <= FVector::DistSquaredXY(C->Sampled[i], M.Pos))
					{
						break;
					}
				}
			}
			M.TrimFromEndCm = Along;
		}
		return M;
	};

	struct FNode
	{
		FRoadNetLaneNode Pub;
		int32 Road = INDEX_NONE;
		int32 Lane = INDEX_NONE;
		int32 Virt = INDEX_NONE;
		TArray<FVector> Centerline;
	};
	TArray<FNode> Nodes;
	TArray<FRoadNetLaneEdge> Edges;
	TArray<FRoadNetLaneConnection> Cons;
	int32 Reductions = 0;
	int32 NextVirt = 1;

	auto AddNode = [&](uint32 JointKey, int32 Arm, int32 Kerb, bool bEntering, bool bVirtual,
		int32 Lane, int32 Road, const FVector& Pos, const FVector& Tangent, bool bCenter, bool bExempt,
		const TArray<FVector>& CL) -> int32
	{
		FNode N;
		N.Pub.Key = HashLane(JointKey, Arm, Kerb, bEntering, bVirtual, bVirtual ? NextVirt : 0);
		N.Pub.JointKey = JointKey;
		N.Pub.Arm = Arm;
		N.Pub.KerbRank = Kerb;
		N.Pub.Road = Road;
		N.Pub.bEntering = bEntering;
		N.Pub.bVirtual = bVirtual;
		N.Pub.bCenterTurn = bCenter;
		N.Pub.bDegreeExempt = bExempt;
		N.Pub.Position = Pos;
		N.Pub.Tangent = Tangent;
		const FIntPoint Tile = RoadNetTiles::WorldToTile(Pos.X, Pos.Y, TileSizeCm);
		N.Pub.TileX = Tile.X;
		N.Pub.TileY = Tile.Y;
		N.Road = Road;
		N.Lane = Lane;
		N.Virt = bVirtual ? NextVirt++ : INDEX_NONE;
		N.Centerline = CL;
		if (bVirtual) { N.Pub.Key = HashLane(JointKey, Arm, Kerb, bEntering, true, N.Virt); }
		Nodes.Add(MoveTemp(N));
		return Nodes.Num() - 1;
	};

	auto Emit = [&](int32 From, int32 To, ERoadNetTurnRole Kind, ERoadNetTurnRole Bear, bool bMerge, bool bFork,
		bool bOnRoad, bool bToAtStart, bool bUTurn)
	{
		if (!Nodes.IsValidIndex(From) || !Nodes.IsValidIndex(To)) { return; }
		const FNode& A = Nodes[From];
		const FNode& B = Nodes[To];
		if (!A.Pub.bVirtual && Roads.IsValidIndex(A.Road))
		{
			const TArray<FRoadNetLane> Ls = Roads[A.Road].Lanes.ResolveLanes(bDriveOnLeft);
			if (Ls.IsValidIndex(A.Lane))
			{
				if (!RoleAllows(Ls[A.Lane].TurnRole, Kind, Bear)) { return; }
			}
		}
		FVector C;
		bool bHermite = false;
		const double LaneW = 350.0;
		MakeShape(A.Pub.Position, A.Pub.Tangent, B.Pub.Position, B.Pub.Tangent, bUTurn, LaneW, C, bHermite);

		FRoadNetLaneEdge E;
		E.FromKey = A.Pub.Key;
		E.ToKey = B.Pub.Key;
		E.Kind = Kind;
		E.Bear = Bear;
		E.bMerge = bMerge;
		E.bFork = bFork;
		E.bOnRoad = bOnRoad;
		E.bHermite = bHermite;
		E.P0 = A.Pub.Position;
		E.P1 = B.Pub.Position;
		E.Control = C;
		E.Key = HashCombine(HashCombine(A.Pub.JointKey, A.Pub.Key), HashCombine(B.Pub.Key, GetTypeHash((int32)Kind)));
		Edges.Add(E);

		FRoadNetLaneConnection Cn;
		Cn.From.Road = A.Road;
		Cn.From.Lane = A.Pub.bVirtual ? INDEX_NONE : A.Lane;
		Cn.To.Road = B.Road;
		Cn.To.Lane = B.Pub.bVirtual ? INDEX_NONE : B.Lane;
		Cn.Joint = 0;
		Cn.Entry = A.Pub.Position;
		Cn.Exit = B.Pub.Position;
		Cn.bThrough = (Kind == ERoadNetTurnRole::Through);
		Cn.Kind = Kind;
		Cn.Bear = Bear;
		Cn.Control = C;
		Cn.bHermite = bHermite;
		Cn.bMerge = bMerge;
		Cn.bFork = bFork;
		Cn.bVirtualFrom = A.Pub.bVirtual;
		Cn.FromVirtual = A.Virt;
		Cn.VirtualCenterline = A.Centerline;
		Cn.bOnRoad = bOnRoad;
		Cn.bToAtStart = bToAtStart;
		Cn.EdgeKey = E.Key;
		Cons.Add(Cn);
	};

	// Joint index is filled after we know it. Emit captured ji via a pointer the
	// loop sets. Stored on the connection below by rewriting Joint after Emit.
	int32 CurJoint = INDEX_NONE;
	TArray<uint32> CompleteJointKeys;

	for (int32 ji = 0; ji < Ctx.Joints.Num(); ++ji)
	{
		const FRoadNetJoint& J = Ctx.Joints[ji];
		if (J.Arms.Num() < 2) { continue; }
		bool bComplete = true;
		for (const FRoadNetJointArm& Arm : J.Arms)
		{
			if (!Get(Arm.Road)) { bComplete = false; break; }
		}
		if (!bComplete) { continue; }

		const uint32 JKey = HashJoint(J, Roads, ZoneOf(J));
		CompleteJointKeys.Add(JKey);
		CurJoint = ji;

		TArray<FRoadNetRebuildContext::FArmMouth> Mouths;
		TMap<uint64, int32> SharedLeave;
		Mouths.SetNum(J.Arms.Num());
		for (int32 ai = 0; ai < J.Arms.Num(); ++ai)
		{
			Mouths[ai] = MouthFor(ji, ai);
			Ctx.ArmMouths.Add(Mouths[ai]);
		}
		FMergePlan Merge;
		const bool bMergeJoint = PlanMerge(J, [&Get](int32 R) -> const TArray<FRoadNetLane>*
		{
			const FCache* C = Get(R);
			return C ? &C->Lanes : nullptr;
		}, bDriveOnLeft, Merge);

		auto LanePos = [&](int32 Road, int32 Lane, const FVector& Mouth, FVector& Pos, FVector& Tan)
		{
			const FCache* C = Get(Road);
			Pos = Mouth;
			Tan = UnitXY(FVector(J.Location.X, J.Location.Y, J.Z) - Mouth);
			if (!C || !C->CL.IsValidIndex(Lane) || C->CL[Lane].Num() < 2) { return; }
			const int32 i = ClosestXY(C->CL[Lane], Mouth);
			Pos = C->CL[Lane][i];
			const int32 j = FMath::Clamp(i + (i == 0 ? 1 : -1), 0, C->CL[Lane].Num() - 1);
			Tan = UnitXY(C->CL[Lane][i] - C->CL[Lane][j]);
			if (FVector::DotProduct(Tan, FVector(J.Location.X - Pos.X, J.Location.Y - Pos.Y, 0.0)) < 0.0)
			{
				Tan *= -1.0;
			}
		};

		for (int32 ai = 0; ai < J.Arms.Num(); ++ai)
		{
			const FRoadNetJointArm& In = J.Arms[ai];
			const FCache* InC = Get(In.Road);
			if (!InC) { continue; }

			TArray<int32> Entering, LeavingSame;
			OrderLanesFromKerb(InC->Lanes, In.bAtStart, true, bDriveOnLeft, Entering);
			OrderLanesFromKerb(InC->Lanes, In.bAtStart, false, bDriveOnLeft, LeavingSame);

			TArray<FApproachMove> Moves;
			ClassifyApproach(J, ai, Moves);
			const bool bSeam = (J.Arms.Num() == 2);
			bool bBear = false;
			for (const FApproachMove& M : Moves)
			{
				if (M.Move == EMovement::BearLeft || M.Move == EMovement::BearRight) { bBear = true; }
			}

			const TArray<FString>& Tokens = In.bAtStart ? Roads[In.Road].BackwardTurnTokens : Roads[In.Road].ForwardTurnTokens;
			if (Tokens.Num() > 0 && Tokens.Num() != Entering.Num())
			{
				Report(FString::Printf(TEXT("I-turn: road %d end has %d turn tokens and %d entering lanes; tag ignored"),
					In.Road, Tokens.Num(), Entering.Num()));
			}

			auto RoleFor = [&](int32 KerbRank, int32 LaneIdx) -> ERoadNetTurnRole
			{
				return RoadNetJunctions::EnteringLaneRole(Roads[In.Road], InC->Lanes[LaneIdx], KerbRank,
					Entering.Num(), In.bAtStart, bDriveOnLeft);
			};
			// A turn-only lane is not through traffic, so it never needs a lane beyond the junction.
			TArray<int32> ThroughEntering;
			for (int32 Rank = 0; Rank < Entering.Num(); ++Rank)
			{
				if (bSeam || RoleAllows(RoleFor(Rank, Entering[Rank]), ERoadNetTurnRole::Through, ERoadNetTurnRole::Auto))
				{
					ThroughEntering.Add(Entering[Rank]);
				}
			}

			TMap<int32, int32> EnterNode; // lane index -> node
			TArray<int32> EnterOrder;     // node indices, kerb first
			bool bHasCenter = false;
			for (int32 Rank = 0; Rank < Entering.Num(); ++Rank)
			{
				const int32 Lane = Entering[Rank];
				const bool bCenter = InC->Lanes[Lane].Type == ERoadNetLaneType::CenterTurn;
				bHasCenter |= bCenter;
				FVector Pos, Tan;
				LanePos(In.Road, Lane, Mouths[ai].Pos, Pos, Tan);
				const int32 Ni = AddNode(JKey, ai, Rank, true, false, Lane, In.Road, Pos, Tan, bCenter, false, InC->CL[Lane]);
				EnterNode.Add(Lane, Ni);
				EnterOrder.Add(Ni);
			}

			// Turn bays become the only source of that turn.
			auto WideningOn = [&](ERoadNetWideningKind Kind) -> const FRoadNetArmWidening*
			{
				const ERoadNetWideningKind KerbKind = bDriveOnLeft ? ERoadNetWideningKind::LeftBay : ERoadNetWideningKind::RightBay;
				for (const FRoadNetArmWidening& W : Ctx.ArmWidenings)
				{
					if (W.Kind == Kind && W.Road == In.Road && W.bAtStart == In.bAtStart) { return &W; }
					// A lane carried here from the junction at the other end of this arm.
					if (Kind == KerbKind && W.bToNextNode && W.Road == In.Road && W.bAtStart != In.bAtStart) { return &W; }
				}
				return nullptr;
			};
			// Classify is geometric; which turn is on the kerb side depends on the driving side.
			const EMovement KerbTurn = bDriveOnLeft ? EMovement::Left : EMovement::Right;
			const EMovement FarTurn = bDriveOnLeft ? EMovement::Right : EMovement::Left;
			int32 KerbBayNode = INDEX_NONE, FarBayNode = INDEX_NONE;
			auto MakeBay = [&](const FRoadNetArmWidening& W, bool bKerb) -> int32
			{
				if (Entering.Num() == 0) { return INDEX_NONE; }
				const int32 KerbLane = Entering[0];
				FRoadNetLane Fake = InC->Lanes[KerbLane];
				const double Sign = (W.Side == ERoadNetSide::Right) ? 1.0 : -1.0;
				Fake.CenterOffset = InC->Lanes[KerbLane].CenterOffset
					+ Sign * (0.5 * InC->Lanes[KerbLane].Width + 0.5 * W.WidthCm / FMath::Max(1, W.Lanes));
				Fake.LeftEdge.Reset();
				Fake.RightEdge.Reset();
				TArray<FVector> CL;
				// A far bay is the gap its own move opens, so the move must not carry it.
				const FRoadCurves& RC = Ctx.Curves[In.Road];
				RoadNetLanes::BuildLaneCenterline(RC.Sampled, Fake, CL, W.bFarSide ? nullptr : &RC.Moves);
				FVector Pos, Tan;
				const int32 i = ClosestXY(CL, Mouths[ai].Pos);
				Pos = CL.IsValidIndex(i) ? CL[i] : Mouths[ai].Pos;
				Tan = Mouths[ai].TravelIn;
				return AddNode(JKey, ai, bKerb ? -1 : 1000, true, true, INDEX_NONE, In.Road, Pos, Tan, false, true, CL);
			};
			const ERoadNetWideningKind KerbBayKind = bDriveOnLeft ? ERoadNetWideningKind::LeftBay : ERoadNetWideningKind::RightBay;
			const ERoadNetWideningKind FarBayKind = bDriveOnLeft ? ERoadNetWideningKind::RightBay : ERoadNetWideningKind::LeftBay;
			if (const FRoadNetArmWidening* W = WideningOn(KerbBayKind))
			{
				KerbBayNode = MakeBay(*W, true);
			}
			if (const FRoadNetArmWidening* W = WideningOn(FarBayKind))
			{
				FarBayNode = MakeBay(*W, false);
			}

			// Raw is the unelected classification. Channelization builds bays from it,
			// so a bay is wired to the arm it was built for even where that arm won
			// the single Through (a T stem has two 90° turns and one is elected).
			struct FDest { int32 Arm; EMovement Move; EMovement Raw; TArray<int32> Leaving; TArray<int32> LeaveNodes; };
			TArray<FDest> Dests;
			for (const FApproachMove& M : Moves)
			{
				const FRoadNetJointArm& Out = J.Arms[M.Arm];
				const FCache* OutC = Get(Out.Road);
				if (!OutC) { continue; }
				FDest D;
				D.Arm = M.Arm;
				D.Move = bSeam ? EMovement::Through : M.Move;
				D.Raw = Classify(In.BearingRad, Out.BearingRad);
				OrderLanesFromKerb(OutC->Lanes, Out.bAtStart, false, bDriveOnLeft, D.Leaving);
				for (int32 Rank = 0; Rank < D.Leaving.Num(); ++Rank)
				{
					const int32 Lane = D.Leaving[Rank];
					if (OutC->Lanes[Lane].Type == ERoadNetLaneType::CenterTurn) { D.LeaveNodes.Add(INDEX_NONE); continue; }
					const uint64 Key = ((uint64)M.Arm << 32) | (uint32)Rank;
					if (const int32* Have = SharedLeave.Find(Key)) { D.LeaveNodes.Add(*Have); continue; }
					FVector Pos, Tan;
					LanePos(Out.Road, Lane, Mouths[M.Arm].Pos, Pos, Tan);
					Tan = -Mouths[M.Arm].TravelIn;
					if (Tan.IsNearlyZero()) { Tan = UnitXY(Pos - FVector(J.Location.X, J.Location.Y, J.Z)); }
					const int32 Ni = AddNode(JKey, M.Arm, Rank, false, false, Lane, Out.Road, Pos, Tan, false, false, OutC->CL[Lane]);
					SharedLeave.Add(Key, Ni);
					D.LeaveNodes.Add(Ni);
				}
				Dests.Add(MoveTemp(D));
			}

			auto DestByArm = [&](int32 Arm) -> FDest*
			{
				for (FDest& D : Dests) { if (D.Arm == Arm) { return &D; } }
				return nullptr;
			};

			auto Connect = [&](int32 FromNode, int32 ToNode, EMovement Move, bool bMerge, bool bFork, bool bOnRoad, bool bToStart)
			{
				if (FromNode == INDEX_NONE || ToNode == INDEX_NONE) { return; }
				const ERoadNetTurnRole Kind = KindOf(Move);
				const ERoadNetTurnRole Bear = BearOf(Move);
				const int32 Lane = Nodes[FromNode].Lane;
				if (!Nodes[FromNode].Pub.bVirtual && InC->Lanes.IsValidIndex(Lane))
				{
					int32 Rank = 0;
					Entering.Find(Lane, Rank);
					if (!RoleAllows(RoleFor(Rank, Lane), Kind, Bear)) { return; }
				}
				const int32 Before = Cons.Num();
				Emit(FromNode, ToNode, Kind, Bear, bMerge, bFork, bOnRoad, bToStart, Move == EMovement::UTurn);
				if (Cons.Num() > Before) { Cons.Last().Joint = ji; }
			};

			auto SurplusMerge = [&](int32 FromLaneNode, FDest& D)
			{
				const FRoadNetJointArm& Out = J.Arms[D.Arm];
				const FRoadNetArmWidening* Drop = nullptr;
				for (const FRoadNetArmWidening& W : Ctx.ArmWidenings)
				{
					if (W.Kind == ERoadNetWideningKind::Drop && W.Road == Out.Road && W.bAtStart == Out.bAtStart)
					{
						Drop = &W;
						break;
					}
				}
				const FCache* OutC = Get(Out.Road);
				if (!OutC || D.Leaving.Num() == 0 || D.LeaveNodes.Num() == 0 || D.LeaveNodes[0] == INDEX_NONE)
				{
					return;
				}
				if (bChannelizeJunctions && Drop && Ctx.Curves.Contains(Out.Road))
				{
					FRoadNetLane Fake = OutC->Lanes[D.Leaving[0]];
					const double Sign = (Drop->Side == ERoadNetSide::Right) ? 1.0 : -1.0;
					Fake.CenterOffset += Sign * (0.5 * Fake.Width + 0.5 * Drop->WidthCm / FMath::Max(1, Drop->Lanes));
					Fake.LeftEdge.Reset();
					Fake.RightEdge.Reset();
					TArray<FVector> CL;
					RoadNetLanes::BuildLaneCenterline(Ctx.Curves[Out.Road].Sampled, Fake, CL, &Ctx.Curves[Out.Road].Moves);
					const double ArmLen = Ctx.Curves[Out.Road].Length;
					// A drop that outruns its arm carries on into the road beyond a plain seam.
					const TArray<FVector>& DropArmS = Ctx.Curves[Out.Road].Sampled;
					const FVector DropArmFar = Out.bAtStart ? DropArmS.Last() : DropArmS[0];
					const bool bCarriedOn = Ctx.ArmWidenings.ContainsByPredicate([&](const FRoadNetArmWidening& Cw)
					{
						const FRoadCurves* NextC = (Cw.Kind == ERoadNetWideningKind::Drop && Cw.Road != Out.Road) ? Ctx.Curves.Find(Cw.Road) : nullptr;
						return NextC && NextC->Sampled.Num() > 0
							&& FVector::Dist2D(Cw.bAtStart ? NextC->Sampled[0] : NextC->Sampled.Last(), DropArmFar) < 300.0;
					});
					if (!Drop->bToNextNode && !bCarriedOn && Drop->HoldCm + Drop->TaperCm > ArmLen + 1.0)
					{
						Report(FString::Printf(TEXT("I8: taper %.0f cm longer than arm %d (%.0f cm)"),
							Drop->HoldCm + Drop->TaperCm, Out.Road, ArmLen));
					}
					FVector Pos = Mouths[D.Arm].Pos;
					if (CL.Num() > 0) { Pos = CL[ClosestXY(CL, Mouths[D.Arm].Pos)]; }
					const int32 Virt = AddNode(JKey, D.Arm, -2, false, true, INDEX_NONE, Out.Road, Pos,
						-Mouths[D.Arm].TravelIn, false, false, CL);
					Connect(FromLaneNode, Virt, EMovement::Through, false, false, false, Out.bAtStart);
					// Carried to the far node: it is that junction's kerb turn lane.
					if (Drop->bToNextNode) { return; }
					// Rejoin on the kerb lane at the taper end, clamped to the arm.
					const FRoadCurves& RC = Ctx.Curves[Out.Road];
					const double Keep = FMath::Min(Drop->HoldCm + Drop->TaperCm, FMath::Max(0.0, ArmLen - 10.0));
					FVector Join = OutC->CL[D.Leaving[0]].Last();
					TArray<double> Acc;
					RoadNetMath::CumulativeLength(RC.Sampled, Acc);
					const double Target = Out.bAtStart ? Keep : (Acc.Last() - Keep);
					int32 Seg = 0;
					while (Seg + 1 < Acc.Num() && Acc[Seg + 1] < Target) { ++Seg; }
					if (Acc.IsValidIndex(Seg))
					{
						const FVector RefPt = RC.Sampled[FMath::Clamp(Seg, 0, RC.Sampled.Num() - 1)];
						Join = OutC->CL[D.Leaving[0]][ClosestXY(OutC->CL[D.Leaving[0]], RefPt)];
					}
					FNode& V = Nodes[Virt];
					V.Pub.Tangent = -Mouths[D.Arm].TravelIn;
					// Every dropped lane into this arm rejoins at the same place.
					const uint64 ShareKey = ((uint64)D.Arm << 32) | 0xFFFFFFFFu;
					int32 JoinNode = INDEX_NONE;
					if (const int32* Have = SharedLeave.Find(ShareKey)) { JoinNode = *Have; }
					else
					{
						JoinNode = AddNode(JKey, D.Arm, 0, false, false, D.Leaving[0], Out.Road, Join,
							-Mouths[D.Arm].TravelIn, false, true, OutC->CL[D.Leaving[0]]);
						// Same lane as the kerb leaving node at the mouth, but a different
						// place: without a salt the two would share one key.
						Nodes[JoinNode].Pub.Key = HashCombine(Nodes[JoinNode].Pub.Key, 0x4A4F494Eu);
						SharedLeave.Add(ShareKey, JoinNode);
					}
					Connect(Virt, JoinNode, EMovement::Through, true, false, true, Out.bAtStart);
					++Reductions;
				}
				else
				{
					Connect(FromLaneNode, D.LeaveNodes[0], EMovement::Through, true, false, false, Out.bAtStart);
					++Reductions;
				}
			};

			TSet<int32> UsedEntering;
			auto Mark = [&](int32 NodeIdx) { if (Nodes.IsValidIndex(NodeIdx)) { UsedEntering.Add(Nodes[NodeIdx].Lane); } };

			const FMergeShare* MergeShare = (bMergeJoint && !Merge.bDiverge)
				? Merge.Shares.FindByPredicate([ai](const FMergeShare& S) { return S.Arm == ai; }) : nullptr;
			if (bMergeJoint && Merge.bDiverge && Merge.Arm == ai)
			{
				for (const FMergeShare& Sh : Merge.Shares)
				{
					FDest* D = DestByArm(Sh.Arm);
					if (!D) { continue; }
					const bool bToStart = J.Arms[D->Arm].bAtStart;
					auto LeaveNodeOf = [D](int32 Lane)
					{
						const int32 At = D->Leaving.IndexOfByKey(Lane);
						return D->LeaveNodes.IsValidIndex(At) ? D->LeaveNodes[At] : INDEX_NONE;
					};
					for (const FThroughPair& Pr : Sh.Pairs)
					{
						const int32* FN = EnterNode.Find(Pr.Enter);
						Connect(FN ? *FN : INDEX_NONE, LeaveNodeOf(Pr.Leave), D->Move, false, false, false, bToStart);
						if (FN) { Mark(*FN); }
					}
					// Branch lanes no entering lane feeds: the kerb branch's come from the
					// deceleration lane, others fork off the nearest entering lane.
					if (Entering.Num() == 0) { continue; }
					const int32* Edge = EnterNode.Find(Sh.bKerbSide ? Entering[0] : Entering.Last());
					const bool bBay = Sh.bKerbSide && KerbBayNode != INDEX_NONE;
					const int32 Src = bBay ? KerbBayNode : (Edge ? *Edge : INDEX_NONE);
					for (int32 S : Sh.Surplus) { Connect(Src, LeaveNodeOf(S), D->Move, false, !bBay, false, bToStart); }
					if (bBay) { Mark(KerbBayNode); }
					// Entering lanes no branch takes end in the kerb branch's kerb lane.
					if (Sh.bKerbSide && D->Leaving.Num() > 0)
					{
						for (int32 U : Merge.Unused)
						{
							const int32* FN = EnterNode.Find(U);
							if (!FN) { continue; }
							Connect(*FN, LeaveNodeOf(D->Leaving[0]), D->Move, true, false, false, bToStart);
							++Reductions;
							Mark(*FN);
						}
					}
				}
			}
			else if (MergeShare)
			{
				if (FDest* D = DestByArm(Merge.Arm))
				{
					const bool bToStart = J.Arms[D->Arm].bAtStart;
					auto LeaveNodeOf = [D](int32 Lane)
					{
						const int32 At = D->Leaving.IndexOfByKey(Lane);
						return D->LeaveNodes.IsValidIndex(At) ? D->LeaveNodes[At] : INDEX_NONE;
					};
					for (const FThroughPair& Pr : MergeShare->Pairs)
					{
						const int32* FN = EnterNode.Find(Pr.Enter);
						Connect(FN ? *FN : INDEX_NONE, LeaveNodeOf(Pr.Leave), D->Move, false, false, false, bToStart);
						if (FN) { Mark(*FN); }
					}
					for (int32 S : MergeShare->Surplus)
					{
						const int32* FN = EnterNode.Find(S);
						if (!FN) { continue; }
						if (MergeShare->bKerbSide) { SurplusMerge(*FN, *D); }
						else
						{
							Connect(*FN, LeaveNodeOf(Merge.Lanes.Last()), D->Move, true, false, false, bToStart);
							++Reductions;
						}
						Mark(*FN);
					}
					// Leaving lanes nobody brings traffic to open off the kerb-most lane.
					if (MergeShare->bKerbSide && Entering.Num() > 0)
					{
						if (const int32* FN = EnterNode.Find(Entering[0]))
						{
							for (int32 U : Merge.Unused) { Connect(*FN, LeaveNodeOf(U), D->Move, false, true, false, bToStart); }
						}
					}
				}
			}
			else if (!bBear)
			{
				for (FDest& D : Dests)
				{
					if (D.Move != EMovement::Through && !bSeam) { continue; }
					if (!bSeam && D.Move != EMovement::Through) { continue; }
					TArray<FThroughPair> Pairs;
					TArray<int32> Surplus;
					const FCache* OutC = Get(J.Arms[D.Arm].Road);
					if (!OutC) { continue; }
					const bool bInPlace = bSeam && SeamKeepsWidth(InC->Lanes, OutC->Lanes);
					PairThrough(InC->Lanes, ThroughEntering, OutC->Lanes, D.Leaving, true, Pairs, Surplus, bInPlace);
					// In place the extra lanes are the inner ones: they merge into, or fork
					// off, the innermost paired lane right at the seam.
					const int32 InnerAt = Pairs.Num() > 0 ? D.Leaving.IndexOfByKey(Pairs.Last().Leave) : INDEX_NONE;
					const int32 InnerLeave = D.LeaveNodes.IsValidIndex(InnerAt) ? D.LeaveNodes[InnerAt] : INDEX_NONE;
					for (const FThroughPair& P : Pairs)
					{
						const int32* FN = EnterNode.Find(P.Enter);
						int32 TN = INDEX_NONE;
						const int32 At = D.Leaving.IndexOfByKey(P.Leave);
						if (D.LeaveNodes.IsValidIndex(At)) { TN = D.LeaveNodes[At]; }
						Connect(FN ? *FN : INDEX_NONE, TN, EMovement::Through, false, false, false, J.Arms[D.Arm].bAtStart);
						if (FN) { Mark(*FN); }
					}
					for (int32 S : Surplus)
					{
						if (const int32* FN = EnterNode.Find(S))
						{
							if (bInPlace)
							{
								Connect(*FN, InnerLeave, EMovement::Through, true, false, false, J.Arms[D.Arm].bAtStart);
								++Reductions;
							}
							else { SurplusMerge(*FN, D); }
							Mark(*FN);
						}
					}
					// Lane gain: the unpaired receiving lanes are the kerb-most (§5.2.4).
					// They branch off the kerb lane: along the seam's diverge taper when
					// channelization built one, else at the mouth.
					TArray<int32> Gained;   // leave nodes, kerb first
					for (int32 li = 0; li < D.Leaving.Num(); ++li)
					{
						if (!D.LeaveNodes.IsValidIndex(li) || D.LeaveNodes[li] == INDEX_NONE) { continue; }
						if (!Pairs.ContainsByPredicate([&](const FThroughPair& P) { return P.Leave == D.Leaving[li]; })) { Gained.Add(D.LeaveNodes[li]); }
					}
					const int32* KerbFN = Pairs.Num() > 0 ? EnterNode.Find(bInPlace ? Pairs.Last().Enter : Pairs[0].Enter) : nullptr;
					const FRoadNetArmWidening* GainW = nullptr;
					if (bSeam && bChannelizeJunctions && Ctx.Curves.Contains(In.Road))
					{
						for (const FRoadNetArmWidening& W : Ctx.ArmWidenings)
						{
							if (W.Kind == ERoadNetWideningKind::Gain && W.Road == In.Road && W.bAtStart == In.bAtStart) { GainW = &W; break; }
						}
					}
					if (KerbFN && Gained.Num() > 0 && GainW)
					{
						const int32 KerbLane = Pairs[0].Enter;
						const FRoadCurves& RC = Ctx.Curves[In.Road];
						const double ArmLen = RC.Length;
						if (GainW->TaperCm > ArmLen + 1.0)
						{
							Report(FString::Printf(TEXT("I8: taper %.0f cm longer than arm %d (%.0f cm)"), GainW->TaperCm, In.Road, ArmLen));
						}
						const double Keep = FMath::Min(GainW->TaperCm, FMath::Max(0.0, ArmLen - 10.0));
						TArray<double> Acc;
						RoadNetMath::CumulativeLength(RC.Sampled, Acc);
						const double Target = In.bAtStart ? Keep : (Acc.Last() - Keep);
						int32 Seg = 0;
						while (Seg + 1 < Acc.Num() && Acc[Seg + 1] < Target) { ++Seg; }
						const TArray<FVector>& KerbCL = InC->CL[KerbLane];
						const FVector ForkPos = KerbCL[ClosestXY(KerbCL, RC.Sampled[FMath::Clamp(Seg, 0, RC.Sampled.Num() - 1)])];
						// Same lane as the kerb entering node at the mouth, but a different place.
						const int32 Fork = AddNode(JKey, ai, 0, true, false, KerbLane, In.Road, ForkPos,
							Mouths[ai].TravelIn, false, true, KerbCL);
						Nodes[Fork].Pub.Key = HashCombine(Nodes[Fork].Pub.Key, 0x464F524Bu);
						const double Sign = (GainW->Side == ERoadNetSide::Right) ? 1.0 : -1.0;
						const double KerbW = InC->Lanes[KerbLane].Width;
						const double EachW = GainW->WidthCm / (double)FMath::Max(1, GainW->Lanes);
						for (int32 g = 0; g < Gained.Num(); ++g)
						{
							FRoadNetLane Fake = InC->Lanes[KerbLane];
							Fake.CenterOffset += Sign * (0.5 * KerbW + ((double)(Gained.Num() - g) - 0.5) * EachW);
							Fake.LeftEdge.Reset();
							Fake.RightEdge.Reset();
							TArray<FVector> CL;
							RoadNetLanes::BuildLaneCenterline(RC.Sampled, Fake, CL, &RC.Moves);
							const FVector Pos = CL.Num() > 0 ? CL[ClosestXY(CL, Mouths[ai].Pos)] : Mouths[ai].Pos;
							const int32 Virt = AddNode(JKey, ai, -3 - g, true, true, INDEX_NONE, In.Road, Pos,
								Mouths[ai].TravelIn, false, true, CL);
							Connect(Fork, Virt, EMovement::Through, false, true, true, In.bAtStart);
							Connect(Virt, Gained[g], EMovement::Through, false, false, false, J.Arms[D.Arm].bAtStart);
						}
					}
					else if (KerbFN)
					{
						for (int32 Node : Gained)
						{
							Connect(*KerbFN, Node, EMovement::Through, false, true, false, J.Arms[D.Arm].bAtStart);
						}
					}
					if (bSeam) { break; }
				}
			}
			else
			{
				TArray<FSplitBranch> Branches;
				for (const FApproachMove& M : Moves)
				{
					if (M.Move != EMovement::Through && M.Move != EMovement::BearLeft && M.Move != EMovement::BearRight) { continue; }
					FDest* D = DestByArm(M.Arm);
					if (!D) { continue; }
					int32 Recv = 0;
					for (int32 Node : D->LeaveNodes) { if (Node != INDEX_NONE) { ++Recv; } }
					Branches.Add({ M.Arm, M.Move, Recv });
				}
				SortBranchesFromKerb(Branches, Moves, bDriveOnLeft);
				TArray<int32> EnterNoCenter;
				for (int32 E : Entering)
				{
					if (InC->Lanes[E].Type != ERoadNetLaneType::CenterTurn) { EnterNoCenter.Add(E); }
				}
				TArray<FLaneBlock> Blocks;
				SplitBlocks(EnterNoCenter, Branches, Blocks);
				TSet<int32> Seen;
				for (const FLaneBlock& Block : Blocks)
				{
					FDest* D = DestByArm(Block.Arm);
					if (!D) { continue; }
					TArray<int32> LeaveReal;
					for (int32 i = 0; i < D->Leaving.Num(); ++i)
					{
						if (D->LeaveNodes.IsValidIndex(i) && D->LeaveNodes[i] != INDEX_NONE) { LeaveReal.Add(i); }
					}
					const int32 N = FMath::Min(Block.Lanes.Num(), LeaveReal.Num());
					for (int32 i = 0; i < N; ++i)
					{
						const int32* FN = EnterNode.Find(Block.Lanes[i]);
						const bool bShare = Seen.Contains(Block.Lanes[i]);
						Seen.Add(Block.Lanes[i]);
						Connect(FN ? *FN : INDEX_NONE, D->LeaveNodes[LeaveReal[i]], Block.Move, false, bShare, false, J.Arms[Block.Arm].bAtStart);
						if (FN) { Mark(*FN); }
					}
					for (int32 i = N; i < Block.Lanes.Num(); ++i)
					{
						if (const int32* FN = EnterNode.Find(Block.Lanes[i]))
						{
							SurplusMerge(*FN, *D);
							Mark(*FN);
						}
					}
					if (N > 0 && LeaveReal.Num() > N)
					{
						const int32* FN = EnterNode.Find(Block.Lanes.Last());
						for (int32 i = N; i < LeaveReal.Num(); ++i)
						{
							if (FN) { Connect(*FN, D->LeaveNodes[LeaveReal[i]], Block.Move, false, true, false, J.Arms[Block.Arm].bAtStart); }
						}
					}
				}
			}

			if (!bSeam)
			{
				auto FirstLeave = [&](EMovement Want, bool bRaw) -> TPair<int32, bool>
				{
					for (FDest& D : Dests)
					{
						if ((bRaw ? D.Raw : D.Move) != Want || D.LeaveNodes.Num() == 0) { continue; }
						int32 Node = INDEX_NONE;
						if (Want == FarTurn || Want == EMovement::UTurn)
						{
							for (int32 i = D.LeaveNodes.Num() - 1; i >= 0; --i)
							{
								if (D.LeaveNodes[i] != INDEX_NONE) { Node = D.LeaveNodes[i]; break; }
							}
						}
						else if (D.LeaveNodes[0] != INDEX_NONE) { Node = D.LeaveNodes[0]; }
						if (Node != INDEX_NONE) { return TPair<int32, bool>(Node, J.Arms[D.Arm].bAtStart); }
					}
					return TPair<int32, bool>(INDEX_NONE, false);
				};
				const TPair<int32, bool> Kerb = FirstLeave(KerbTurn, KerbBayNode != INDEX_NONE);
				const TPair<int32, bool> Far = FirstLeave(FarTurn, false);
				const TPair<int32, bool> BayFar = (FarBayNode != INDEX_NONE) ? FirstLeave(FarTurn, true) : Far;
				// turn:lanes naming the same turn on several lanes (left|left|through):
				// past the outermost, each tagged lane takes the next receiving lane
				// inward, the surplus merging into the last.
				auto ExtendTaggedTurn = [&](EMovement Turn, bool bFromFar)
				{
					for (FDest& D : Dests)
					{
						if (D.Move != Turn) { continue; }
						TArray<int32> Recv;
						for (int32 i = 0; i < D.LeaveNodes.Num(); ++i)
						{
							const int32 Node = D.LeaveNodes[bFromFar ? D.LeaveNodes.Num() - 1 - i : i];
							if (Node != INDEX_NONE) { Recv.Add(Node); }
						}
						if (Recv.Num() == 0) { continue; }
						for (int32 k = 1; k < Entering.Num(); ++k)
						{
							const int32 Rank = bFromFar ? Entering.Num() - 1 - k : k;
							const int32 Lane = Entering[Rank];
							if (InC->Lanes[Lane].Type == ERoadNetLaneType::CenterTurn || RoleFor(Rank, Lane) != KindOf(Turn)) { break; }
							Connect(EnterNode[Lane], Recv[FMath::Min(k, Recv.Num() - 1)], Turn, k >= Recv.Num(), false, false,
								J.Arms[D.Arm].bAtStart);
							Mark(EnterNode[Lane]);
						}
						return;
					}
				};
				if (Kerb.Key != INDEX_NONE)
				{
					const int32 Src = (KerbBayNode != INDEX_NONE) ? KerbBayNode
						: (EnterOrder.Num() ? EnterOrder[0] : INDEX_NONE);
					Connect(Src, Kerb.Key, KerbTurn, false, false, false, Kerb.Value);
					Mark(Src);
					if (KerbBayNode == INDEX_NONE) { ExtendTaggedTurn(KerbTurn, false); }
				}
				if (!bHasCenter && FarBayNode == INDEX_NONE && Far.Key != INDEX_NONE && EnterOrder.Num() > 0)
				{
					Connect(EnterOrder.Last(), Far.Key, FarTurn, false, false, false, Far.Value);
					Mark(EnterOrder.Last());
					ExtendTaggedTurn(FarTurn, true);
				}
				for (int32 Rank = 0; Rank < Entering.Num(); ++Rank)
				{
					const int32 Lane = Entering[Rank];
					if (InC->Lanes[Lane].Type != ERoadNetLaneType::CenterTurn && FarBayNode == INDEX_NONE) { continue; }
					const int32 Src = (InC->Lanes[Lane].Type == ERoadNetLaneType::CenterTurn)
						? EnterNode[Lane] : FarBayNode;
					const TPair<int32, bool>& SrcFar = (Src == FarBayNode) ? BayFar : Far;
					if (SrcFar.Key != INDEX_NONE) { Connect(Src, SrcFar.Key, FarTurn, false, false, false, SrcFar.Value); Mark(Src); }
					int32 URank = INDEX_NONE;
					for (int32 r = LeavingSame.Num() - 1; r >= 0; --r)
					{
						if (InC->Lanes[LeavingSame[r]].Type != ERoadNetLaneType::CenterTurn) { URank = r; break; }
					}
					if (URank != INDEX_NONE)
					{
						const uint64 Key = ((uint64)ai << 32) | (uint32)URank;
						int32 UNode = INDEX_NONE;
						if (const int32* Have = SharedLeave.Find(Key)) { UNode = *Have; }
						else
						{
							FVector Pos, Tan;
							const int32 Inner = LeavingSame[URank];
							LanePos(In.Road, Inner, Mouths[ai].Pos, Pos, Tan);
							UNode = AddNode(JKey, ai, URank, false, false, Inner, In.Road, Pos,
								-Mouths[ai].TravelIn, false, false, InC->CL[Inner]);
							SharedLeave.Add(Key, UNode);
						}
						Connect(Src, UNode, EMovement::UTurn, false, false, false, In.bAtStart);
						Mark(Src);
					}
				}

				// Orphans join the neighbour movement, else merge into its target.
				for (int32 Rank = 0; Rank < Entering.Num(); ++Rank)
				{
					const int32 Lane = Entering[Rank];
					if (InC->Lanes[Lane].Type == ERoadNetLaneType::CenterTurn) { continue; }
					if (UsedEntering.Contains(Lane)) { continue; }
					const int32 Neigh = (Rank > 0) ? Rank - 1 : (Rank + 1 < Entering.Num() ? Rank + 1 : INDEX_NONE);
					if (Neigh == INDEX_NONE || !EnterNode.Contains(Entering[Neigh])) { continue; }
					const int32 Me = EnterNode[Lane];
					int32 Target = INDEX_NONE;
					bool bToStart = false;
					for (const FRoadNetLaneConnection& Cn : Cons)
					{
						if (Cn.Joint == ji && Cn.From.Road == In.Road && Cn.From.Lane == Entering[Neigh])
						{
							for (int32 ni = 0; ni < Nodes.Num(); ++ni)
							{
								if (Nodes[ni].Road == Cn.To.Road && Nodes[ni].Lane == Cn.To.Lane && !Nodes[ni].Pub.bEntering)
								{
									Target = ni;
									bToStart = Cn.bToAtStart;
									break;
								}
							}
							if (Target != INDEX_NONE) { break; }
						}
					}
					if (Target != INDEX_NONE)
					{
						Connect(Me, Target, EMovement::Through, true, false, false, bToStart);
						Mark(Me);
					}
				}
			}
		}
	}

	Cons.Sort([](const FRoadNetLaneConnection& A, const FRoadNetLaneConnection& B) { return A.EdgeKey < B.EdgeKey; });
	Edges.Sort([](const FRoadNetLaneEdge& A, const FRoadNetLaneEdge& B) { return A.Key < B.Key; });
	Nodes.Sort([](const FNode& A, const FNode& B) { return A.Pub.Key < B.Pub.Key; });

	FRoadNetLaneGraph Built;
	Built.Reductions = Reductions;
	Built.Nodes.Reserve(Nodes.Num());
	for (const FNode& N : Nodes) { Built.Nodes.Add(N.Pub); }
	Built.Edges = Edges;

	{
		TMap<uint32, int32> InDeg, OutDeg;
		TSet<uint32> EdgeKeys;
		TMap<uint32, const FRoadNetLaneNode*> ByKey;
		for (const FRoadNetLaneNode& N : Built.Nodes)
		{
			if (ByKey.Contains(N.Key)) { Report(FString::Printf(TEXT("I2: duplicate node %u"), N.Key)); }
			ByKey.Add(N.Key, &N);
		}
		for (const FRoadNetLaneEdge& E : Built.Edges)
		{
			if (EdgeKeys.Contains(E.Key))
			{
				Report(FString::Printf(TEXT("I2: duplicate edge %u"), E.Key));
			}
			EdgeKeys.Add(E.Key);
			InDeg.FindOrAdd(E.ToKey) += 1;
			OutDeg.FindOrAdd(E.FromKey) += 1;
			const FRoadNetLaneNode* const* To = ByKey.Find(E.ToKey);
			const FRoadNetLaneNode* const* From = ByKey.Find(E.FromKey);
			if (From && *From && FVector::DistSquared((*From)->Position, E.P0) > 25.0)
			{
				Report(FString::Printf(TEXT("I5: edge %u does not start on its mouth"), E.Key));
			}
			if (To && *To && FVector::DistSquared((*To)->Position, E.P1) > 25.0)
			{
				Report(FString::Printf(TEXT("I5: edge %u does not end on its mouth"), E.Key));
			}
			if (!E.bHermite && To && *To)
			{
				FVector Leave, Arrive;
				LaneCurveTangents(E.P0, E.Control, E.P1, E.bHermite, E.Kind == ERoadNetTurnRole::UTurn, Leave, Arrive);
				const FVector Dir = Arrive.GetSafeNormal();
				const FVector Tan = (*To)->Tangent.GetSafeNormal();
				if (!Dir.IsNearlyZero() && !Tan.IsNearlyZero()
					&& FVector::DotProduct(Dir, Tan) < FMath::Cos(FMath::DegreesToRadians(5.0)))
				{
					Report(FString::Printf(TEXT("I5: edge %u meets its leaving lane at %.1f deg"),
						E.Key, FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(FVector::DotProduct(Dir, Tan), -1.0, 1.0)))));
				}
			}
		}
		for (const FRoadNetLaneNode& N : Built.Nodes)
		{
			if (N.bEntering && N.bVirtual && N.bDegreeExempt && OutDeg.FindRef(N.Key) < 1)
			{
				Report(FString::Printf(TEXT("I1: bay %u does not leave the junction"), N.Key));
			}
			if (N.bDegreeExempt || (N.bCenterTurn && !N.bEntering)) { continue; }
			if (N.bEntering && OutDeg.FindRef(N.Key) < 1)
			{
				Report(FString::Printf(TEXT("I1: entering lane %u has no successor (road %d arm %d kerb %d%s at %.0f,%.0f)"),
					N.Key, N.Road, N.Arm, N.KerbRank, N.bVirtual ? TEXT(" virtual") : TEXT(""), N.Position.X, N.Position.Y));
			}
			if (!N.bEntering && InDeg.FindRef(N.Key) < 1)
			{
				Report(FString::Printf(TEXT("I1: leaving lane %u was not fed (road %d arm %d kerb %d%s at %.0f,%.0f)"),
					N.Key, N.Road, N.Arm, N.KerbRank, N.bVirtual ? TEXT(" virtual") : TEXT(""), N.Position.X, N.Position.Y));
			}
		}

		TMap<uint32, TArray<const FRoadNetLaneNode*>> ByJoint;
		for (const FRoadNetLaneNode& N : Built.Nodes) { ByJoint.FindOrAdd(N.JointKey).Add(&N); }
		for (const TPair<uint32, TArray<const FRoadNetLaneNode*>>& JV : ByJoint)
		{
			TSet<int32> Arms;
			for (const FRoadNetLaneNode* N : JV.Value) { Arms.Add(N->Arm); }
			if (Arms.Num() != 2) { continue; }
			int32 EnterN = 0, LeaveN = 0, ThroughN = 0;
			for (const FRoadNetLaneNode* N : JV.Value)
			{
				if (N->bVirtual) { continue; }
				if (N->bEntering) { ++EnterN; } else { ++LeaveN; }
			}
			if (EnterN != LeaveN || EnterN == 0) { continue; }
			for (const FRoadNetLaneEdge& E : Built.Edges)
			{
				const FRoadNetLaneNode* const* From = ByKey.Find(E.FromKey);
				if (!From || !*From || (*From)->JointKey != JV.Key) { continue; }
				if (E.Kind == ERoadNetTurnRole::Through && E.Bear == ERoadNetTurnRole::Auto
					&& !E.bMerge && !E.bFork && !E.bOnRoad)
				{
					++ThroughN;
				}
			}
			if (ThroughN != EnterN)
			{
				Report(FString::Printf(TEXT("I3: seam joint %u has %d lanes and %d through edges"),
					JV.Key, EnterN, ThroughN));
			}
		}

		auto MoveOrder = [this](const FRoadNetLaneEdge& E) -> int32
		{
			int32 O = 2;
			if (E.Kind == ERoadNetTurnRole::Right) { O = 0; }
			else if (E.Bear == ERoadNetTurnRole::BearRight) { O = 1; }
			else if (E.Bear == ERoadNetTurnRole::BearLeft) { O = 3; }
			else if (E.Kind == ERoadNetTurnRole::Left) { O = 4; }
			else if (E.Kind == ERoadNetTurnRole::UTurn) { O = 5; }
			if (bDriveOnLeft)
			{
				if (O == 0) { O = 4; }
				else if (O == 4) { O = 0; }
				else if (O == 1) { O = 3; }
				else if (O == 3) { O = 1; }
			}
			return O;
		};
		TMap<uint64, TArray<const FRoadNetLaneNode*>> Approaches;
		for (const FRoadNetLaneNode& N : Built.Nodes)
		{
			if (!N.bEntering || N.bVirtual) { continue; }
			Approaches.FindOrAdd(((uint64)N.JointKey << 32) | (uint32)N.Arm).Add(&N);
		}
		for (TPair<uint64, TArray<const FRoadNetLaneNode*>>& Ap : Approaches)
		{
			Ap.Value.Sort([](const FRoadNetLaneNode& A, const FRoadNetLaneNode& B) { return A.KerbRank < B.KerbRank; });
			int32 PrevMax = -1;
			int32 PrevKerb = MIN_int32;
			TMap<int32, int32> OuterTarget;
			for (const FRoadNetLaneNode* N : Ap.Value)
			{
				if (N->KerbRank == PrevKerb) { continue; }
				int32 Lo = MAX_int32, Hi = -1;
				TMap<int32, int32> LaneTarget;
				for (const FRoadNetLaneEdge& E : Built.Edges)
				{
					if (E.FromKey != N->Key) { continue; }
					const int32 M = MoveOrder(E);
					Lo = FMath::Min(Lo, M);
					Hi = FMath::Max(Hi, M);
					const FRoadNetLaneNode* const* To = ByKey.Find(E.ToKey);
					const int32 Target = (To && *To) ? (*To)->KerbRank : 0;
					int32& Slot = LaneTarget.FindOrAdd(M, Target);
					Slot = FMath::Min(Slot, Target);
				}
				for (const TPair<int32, int32>& Tv : LaneTarget)
				{
					if (const int32* Was = OuterTarget.Find(Tv.Key))
					{
						if (Tv.Value < *Was)
						{
							Report(FString::Printf(TEXT("I4: approach lane %u crosses an earlier movement"), N->Key));
						}
					}
					int32& Keep = OuterTarget.FindOrAdd(Tv.Key, Tv.Value);
					Keep = FMath::Max(Keep, Tv.Value);
				}
				if (Lo != MAX_int32 && Lo < PrevMax)
				{
					Report(FString::Printf(TEXT("I4: approach lane %u turns before an outer lane"), N->Key));
				}
				if (Hi >= 0) { PrevMax = Hi; }
				PrevKerb = N->KerbRank;
			}
		}
	}

	URoadNetwork* Self = const_cast<URoadNetwork*>(this);
	if (Ctx.bFullCommit)
	{
		Self->LaneGraph = Built;
	}
	else
	{
		TSet<uint32> Drop;
		for (uint32 K : CompleteJointKeys) { Drop.Add(K); }
		for (const FRoadNetLaneNode& N : Self->LaneGraph.Nodes)
		{
			if (Ctx.DirtyTiles.Contains(FIntPoint(N.TileX, N.TileY))) { Drop.Add(N.JointKey); }
		}
		Self->LaneGraph.Nodes.RemoveAll([&Drop](const FRoadNetLaneNode& N) { return Drop.Contains(N.JointKey); });
		Self->LaneGraph.Edges.RemoveAll([&Drop, &Self](const FRoadNetLaneEdge& E)
		{
			for (const FRoadNetLaneNode& N : Self->LaneGraph.Nodes)
			{
				if (N.Key == E.FromKey) { return false; }
			}
			return true;
		});
		Self->LaneGraph.Nodes.Append(Built.Nodes);
		Self->LaneGraph.Edges.Append(Built.Edges);
		Self->LaneGraph.Reductions = Reductions;
	}

	// I7 arm end touching another road mid-span. I9 Z gap. I8 was reported above.
	// An end already welded into a joint with the other road is connected.
	TMap<uint64, int32> EndJoint;
	for (int32 ji = 0; ji < Ctx.Joints.Num(); ++ji)
	{
		for (const FRoadNetJointArm& Arm : Ctx.Joints[ji].Arms)
		{
			EndJoint.Add(((uint64)Arm.Road << 1) | (Arm.bAtStart ? 1u : 0u), ji);
		}
	}
	for (int32 r = 0; r < Roads.Num(); ++r)
	{
		if (!Roads[r].IsValid() || Roads[r].Ref.Num() < 2) { continue; }
		const FVector Ends[2] = { Roads[r].Ref[0], Roads[r].Ref.Last() };
		for (int32 e = 0; e < 2; ++e)
		{
			const FVector& End = Ends[e];
			const int32* Ji = EndJoint.Find(((uint64)r << 1) | (e == 0 ? 1u : 0u));
			for (int32 o = 0; o < Roads.Num(); ++o)
			{
				if (o == r || Roads[o].Ref.Num() < 2) { continue; }
				if (Ji && Ctx.Joints[*Ji].Arms.ContainsByPredicate([o](const FRoadNetJointArm& A) { return A.Road == o; }))
				{
					continue;
				}
				const RoadNetMath::FProjectResult PR = RoadNetMath::ProjectToPolyline(Roads[o].Ref, FVector2D(End.X, End.Y));
				if (PR.Distance > 250.0) { continue; }
				const double Len = RoadNetMath::TotalLength(Roads[o].Ref);
				if (PR.AlongDist > 250.0 && PR.AlongDist < Len - 250.0)
				{
					Report(FString::Printf(TEXT("I7: road %d ends on road %d mid-span (%.0f cm off, %.0f of %.0f cm along, end at %.0f,%.0f)"),
						r, o, PR.Distance, PR.AlongDist, Len, End.X, End.Y));
				}
			}
		}
	}
	for (const FRoadNetJoint& J : Ctx.Joints)
	{
		double Z0 = TNumericLimits<double>::Max(), Z1 = -Z0;
		for (const FRoadNetJointArm& Arm : J.Arms)
		{
			if (!Roads.IsValidIndex(Arm.Road) || Roads[Arm.Road].Ref.Num() < 1) { continue; }
			const double Z = Arm.bAtStart ? Roads[Arm.Road].Ref[0].Z : Roads[Arm.Road].Ref.Last().Z;
			Z0 = FMath::Min(Z0, Z);
			Z1 = FMath::Max(Z1, Z);
		}
		if (Z1 - Z0 > kZGapCm)
		{
			Report(FString::Printf(TEXT("I9: joint arms differ by %.0f cm in Z"), Z1 - Z0));
		}
	}

	Self->LaneGraph.Reports.RemoveAll([](const FString& S)
	{
		return S.StartsWith(TEXT("I1:")) || S.StartsWith(TEXT("I2:")) || S.StartsWith(TEXT("I3:"))
			|| S.StartsWith(TEXT("I4:")) || S.StartsWith(TEXT("I5:")) || S.StartsWith(TEXT("I7:"))
			|| S.StartsWith(TEXT("I8:")) || S.StartsWith(TEXT("I9:")) || S.StartsWith(TEXT("I-turn:"));
	});
	for (const FString& R : PassReports) { Self->LaneGraph.Reports.Add(R); }
	if (Self->LaneGraph.Reports.Num() > 0)
	{
		UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet][LANES] lane-graph check: %d finding(s). First ones:"),
			Self->LaneGraph.Reports.Num());
		for (int32 i = 0; i < FMath::Min(40, Self->LaneGraph.Reports.Num()); ++i)
		{
			UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet][LANES]   %s"), *Self->LaneGraph.Reports[i]);
		}
	}

	Ctx.LaneConnections = MoveTemp(Cons);
	if (Reductions > 0)
	{
		UE_LOG(LogRoadNet, Log, TEXT("[RoadNet][LANES] %d surplus lane(s) merge inside or just past a junction."), Reductions);
	}
}

FString URoadNetwork::DumpLaneGraph() const
{
	FString Out;
	TArray<FRoadNetLaneNode> Nodes = LaneGraph.Nodes;
	Nodes.Sort([](const FRoadNetLaneNode& A, const FRoadNetLaneNode& B) { return A.Key < B.Key; });
	for (const FRoadNetLaneNode& N : Nodes)
	{
		Out += FString::Printf(TEXT("NODE %u\n"), N.Key);
		bool bAny = false;
		for (const FRoadNetLaneEdge& E : LaneGraph.Edges)
		{
			if (E.FromKey != N.Key) { continue; }
			bAny = true;
			Out += FString::Printf(TEXT("  %u %u %d %d %d %d\n"), E.Key, E.ToKey,
				(int32)E.Kind, (int32)E.Bear, E.bMerge ? 1 : 0, E.bFork ? 1 : 0);
		}
		if (!bAny) { Out += TEXT("  -\n"); }
	}
	return Out;
}

static FAutoConsoleCommand GDumpLaneGraph(
	TEXT("RoadNet.DumpLaneGraph"),
	TEXT("Print the lane graph: nodes, then each node's successors (an explicit empty list)."),
	FConsoleCommandDelegate::CreateLambda([]()
	{
		UWorld* World = nullptr;
#if WITH_EDITOR
		if (GEditor) { World = GEditor->GetEditorWorldContext().World(); }
#endif
		if (!World) { return; }
		for (TActorIterator<ARoadNetActor> It(World); It; ++It)
		{
			if (URoadNetwork* Net = It->GetNetwork())
			{
				UE_LOG(LogRoadNet, Log, TEXT("%s"), *Net->DumpLaneGraph());
			}
		}
	}));
