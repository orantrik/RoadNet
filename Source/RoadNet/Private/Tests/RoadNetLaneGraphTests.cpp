#include "RoadNetwork.h"
#include "RoadNetJunctions.h"
#include "RoadNetMath.h"
#include "RoadNetStandards.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	FRoadDef LaneRoad(TArray<FVector> Ref, TArray<int64> Ids, int32 Total, bool bOneway)
	{
		FRoadDef R;
		R.Ref = MoveTemp(Ref);
		R.NodeIds = MoveTemp(Ids);
		R.Lanes.Total = Total;
		R.Lanes.bOneway = bOneway;
		return R;
	}

	int32 OutDegree(const FRoadNetLaneGraph& G, uint32 Key)
	{
		int32 N = 0;
		for (const FRoadNetLaneEdge& E : G.Edges) { if (E.FromKey == Key) { ++N; } }
		return N;
	}

	int32 InDegree(const FRoadNetLaneGraph& G, uint32 Key)
	{
		int32 N = 0;
		for (const FRoadNetLaneEdge& E : G.Edges) { if (E.ToKey == Key) { ++N; } }
		return N;
	}

	const FRoadNetLaneNode* NodeByKey(const FRoadNetLaneGraph& G, uint32 Key)
	{
		for (const FRoadNetLaneNode& N : G.Nodes) { if (N.Key == Key) { return &N; } }
		return nullptr;
	}

	TSet<uint32> JointKeys(const FRoadNetLaneGraph& G)
	{
		TSet<uint32> Keys;
		for (const FRoadNetLaneNode& N : G.Nodes) { Keys.Add(N.JointKey); }
		return Keys;
	}

	bool HasReport(const FRoadNetLaneGraph& G, const TCHAR* Prefix)
	{
		for (const FString& R : G.Reports) { if (R.StartsWith(Prefix)) { return true; } }
		return false;
	}

	// Dump text back into "NODE key" plus its successor lines. The formatted
	// result has to equal the dump, and every listed edge has to be a real one.
	FString ParseAndFormat(const FString& Dump, TArray<uint32>& OutEdgeKeys, TArray<uint32>& OutToKeys)
	{
		FString Out;
		TArray<FString> Lines;
		Dump.ParseIntoArrayLines(Lines);
		uint32 Node = 0;
		bool bInNode = false;
		for (const FString& Line : Lines)
		{
			if (Line.StartsWith(TEXT("NODE ")))
			{
				Node = (uint32)FCString::Strtoui64(*Line + 5, nullptr, 10);
				bInNode = true;
				Out += FString::Printf(TEXT("NODE %u\n"), Node);
				continue;
			}
			if (!bInNode) { continue; }
			FString Trim = Line.TrimStartAndEnd();
			if (Trim == TEXT("-"))
			{
				Out += TEXT("  -\n");
				continue;
			}
			TArray<FString> Parts;
			Trim.ParseIntoArrayWS(Parts);
			if (Parts.Num() != 6) { continue; }
			const uint32 Edge = (uint32)FCString::Strtoui64(*Parts[0], nullptr, 10);
			const uint32 To = (uint32)FCString::Strtoui64(*Parts[1], nullptr, 10);
			OutEdgeKeys.Add(Edge);
			OutToKeys.Add(To);
			Out += FString::Printf(TEXT("  %s %s %s %s %s %s\n"),
				*Parts[0], *Parts[1], *Parts[2], *Parts[3], *Parts[4], *Parts[5]);
		}
		return Out;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetLanePairingTest, "RoadNet.Lanes.PairingAndSplits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRoadNetLanePairingTest::RunTest(const FString&)
{
	using namespace RoadNetJunctions;

	TArray<FRoadNetLane> Enter, Leave;
	Enter.SetNum(3);
	Leave.SetNum(2);
	TArray<FThroughPair> Pairs;
	TArray<int32> Surplus;
	PairThrough(Enter, { 0, 1, 2 }, Leave, { 0, 1 }, true, Pairs, Surplus);
	TestTrue(TEXT("the kerb lane is the one carried out"), Surplus.Num() == 1 && Surplus[0] == 0);
	TestTrue(TEXT("the two inner lanes pair in kerb order"),
		Pairs.Num() == 2 && Pairs[0].Enter == 1 && Pairs[0].Leave == 0 && Pairs[1].Enter == 2 && Pairs[1].Leave == 1);

	// תקו"ה 801-809 Table 4: L/W per speed; below 60 km/h L/W = 2V/3.
	TestEqual(TEXT("lane-count shift at 80 km/h is 1:50"), RoadNetStandards::LaneCountShiftLengthCm(80, 100.0), 5000.0, 1.0);
	TestEqual(TEXT("lane-count shift at 110 km/h is 1:75"), RoadNetStandards::LaneCountShiftLengthCm(110, 100.0), 7500.0, 1.0);
	TestEqual(TEXT("lane-count shift at 45 km/h follows 2V/3"), RoadNetStandards::LaneCountShiftLengthCm(45, -100.0), 3000.0, 1.0);

	{
		TArray<FRoadNetLane> Two, Three;
		Two.SetNum(2);
		Three.SetNum(3);
		PairThrough(Two, { 0, 1 }, Three, { 0, 1, 2 }, true, Pairs, Surplus);
		TestTrue(TEXT("2 into 3: lanes line up from the median, the kerb lane is the added one"),
			Surplus.Num() == 0 && Pairs.Num() == 2 && Pairs[0].Enter == 0 && Pairs[0].Leave == 1 && Pairs[1].Enter == 1 && Pairs[1].Leave == 2);
	}

	Enter[1].Type = ERoadNetLaneType::CenterTurn;
	PairThrough(Enter, { 0, 1, 2 }, Leave, { 0, 1 }, true, Pairs, Surplus);
	bool bCenterUsed = Surplus.Contains(1);
	for (const FThroughPair& P : Pairs) { bCenterUsed |= P.Enter == 1; }
	TestFalse(TEXT("a centre-turn lane is neither surplus nor paired"), bCenterUsed);
	TestEqual(TEXT("the two remaining lanes pair"), Pairs.Num(), 2);

	auto BlocksOf = [](const TArray<int32>& Lanes, const TArray<FSplitBranch>& Branches)
	{
		TArray<FLaneBlock> Blocks;
		SplitBlocks(Lanes, Branches, Blocks);
		return Blocks;
	};
	FSplitBranch A{ 0, EMovement::BearRight, 1 };
	FSplitBranch B{ 1, EMovement::Through, 1 };
	TArray<FLaneBlock> Two = BlocksOf({ 0, 1 }, { A, B });
	TestEqual(TEXT("two lanes, two branches: a block each"), Two.Num(), 2);
	TestEqual(TEXT("first block is one lane"), Two.IsValidIndex(0) ? Two[0].Lanes.Num() : 0, 1);
	TestEqual(TEXT("second block is one lane"), Two.IsValidIndex(1) ? Two[1].Lanes.Num() : 0, 1);

	TArray<FLaneBlock> Shared = BlocksOf({ 0 }, { A, B });
	TestEqual(TEXT("one lane is shared by both branches"), Shared.Num(), 2);
	TestTrue(TEXT("both blocks name that lane"),
		Shared.Num() == 2 && Shared[0].Lanes.Num() == 1 && Shared[1].Lanes.Num() == 1
		&& Shared[0].Lanes[0] == 0 && Shared[1].Lanes[0] == 0);

	FSplitBranch Wide{ 0, EMovement::BearRight, 2 };
	FSplitBranch Narrow{ 1, EMovement::Through, 1 };
	TArray<FLaneBlock> Three = BlocksOf({ 0, 1, 2 }, { Wide, Narrow });
	TestEqual(TEXT("three lanes into 2+1: first block takes two"), Three.IsValidIndex(0) ? Three[0].Lanes.Num() : 0, 2);
	TestEqual(TEXT("three lanes into 2+1: second block takes one"), Three.IsValidIndex(1) ? Three[1].Lanes.Num() : 0, 1);

	FRoadNetJoint Seam;
	FRoadNetJointArm S0, S1;
	S0.Road = 0; S0.BearingRad = 0.0;
	S1.Road = 1; S1.BearingRad = FMath::DegreesToRadians(60.0);
	Seam.Arms = { S0, S1 };
	TArray<FApproachMove> Moves;
	ClassifyApproach(Seam, 0, Moves);
	TestEqual(TEXT("a bent seam's only continuation is through"), Moves.Num(), 1);
	TestEqual(TEXT("60 degrees is still through on a seam"), (int32)(Moves.Num() ? Moves[0].Move : EMovement::Left), (int32)EMovement::Through);

	FRoadNetJoint Cross;
	for (int32 i = 0; i < 4; ++i)
	{
		FRoadNetJointArm Arm;
		Arm.Road = i;
		Arm.BearingRad = i * PI * 0.5;
		Cross.Arms.Add(Arm);
	}
	ClassifyApproach(Cross, 0, Moves);
	int32 Throughs = 0;
	for (const FApproachMove& M : Moves) { if (M.Move == EMovement::Through) { ++Throughs; } }
	TestEqual(TEXT("a crossroads has one through"), Throughs, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetLaneGraphTest, "RoadNet.Lanes.Graph",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRoadNetLaneGraphTest::RunTest(const FString&)
{
	auto Degrees = [this](const FRoadNetLaneGraph& G, const TCHAR* What)
	{
		for (const FRoadNetLaneEdge& E : G.Edges)
		{
			if (!E.bHermite)
			{
				const FRoadNetLaneNode* To = NodeByKey(G, E.ToKey);
				FVector Leave, Arrive;
				RoadNetJunctions::LaneCurveTangents(E.P0, E.Control, E.P1, E.bHermite,
					E.Kind == ERoadNetTurnRole::UTurn, Leave, Arrive);
				const FVector Dir = Arrive.GetSafeNormal();
				if (To && !Dir.IsNearlyZero() && !To->Tangent.IsNearlyZero())
				{
					const double Dot = FVector::DotProduct(Dir, To->Tangent.GetSafeNormal());
					TestTrue(FString::Printf(TEXT("%s: curve meets the leaving lane"), What),
						Dot > FMath::Cos(FMath::DegreesToRadians(5.0)));
				}
			}
		}
		TSet<uint32> Keys;
		for (const FRoadNetLaneEdge& E : G.Edges)
		{
			TestFalse(FString::Printf(TEXT("%s: duplicate edge"), What), Keys.Contains(E.Key));
			Keys.Add(E.Key);
		}
		for (const FRoadNetLaneNode& N : G.Nodes)
		{
			if (N.bEntering && N.bVirtual && N.bDegreeExempt)
			{
				TestTrue(FString::Printf(TEXT("%s: a bay leaves the junction"), What), OutDegree(G, N.Key) >= 1);
			}
			if (N.bDegreeExempt) { continue; }
			TSet<int32> JointArms;
			for (const FRoadNetLaneNode& O : G.Nodes) { if (O.JointKey == N.JointKey) { JointArms.Add(O.Arm); } }
			const FString Where = FString::Printf(TEXT("road %d arm %d kerb %d, joint has %d arms"),
				N.Road, N.Arm, N.KerbRank, JointArms.Num());
			if (N.bEntering)
			{
				TestTrue(FString::Printf(TEXT("%s: entering lane has a successor (%s)"), What, *Where), OutDegree(G, N.Key) >= 1);
			}
			else
			{
				TestTrue(FString::Printf(TEXT("%s: leaving lane was fed (%s)"), What, *Where), InDegree(G, N.Key) >= 1);
			}
		}
	};

	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 2, false));
		Net->AddRoad(LaneRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 2, false));
		Net->Rebuild();
		TestEqual(TEXT("seam: one through each way"), Net->LaneGraph.Edges.Num(), 2);
		TestEqual(TEXT("seam: nothing to merge"), Net->LaneGraph.Reductions, 0);
		for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges)
		{
			TestEqual(TEXT("seam: through"), (int32)E.Kind, (int32)ERoadNetTurnRole::Through);
		}
		Degrees(Net->LaneGraph, TEXT("seam"));
		const FString First = Net->DumpLaneGraph();
		const TArray<FVector2D> Stops = Net->LastApproachStops;
		const TArray<FRoadNetArmWidening> Widen = Net->LastArmWidenings;
		const double Surface = Net->LastSurfaceAreaCm2;
		const double Sidewalk = Net->LastSidewalkAreaCm2;
		TArray<uint32> ParsedEdges, ParsedTo;
		TestEqual(TEXT("the dump parses back to itself"), ParseAndFormat(First, ParsedEdges, ParsedTo), First);
		TestEqual(TEXT("the dump lists every edge"), ParsedEdges.Num(), Net->LaneGraph.Edges.Num());
		for (int32 i = 0; i < ParsedEdges.Num(); ++i)
		{
			bool bFound = false;
			for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges)
			{
				if (E.Key == ParsedEdges[i] && E.ToKey == ParsedTo[i]) { bFound = true; break; }
			}
			TestTrue(TEXT("a parsed edge is in the graph"), bFound);
		}
		Net->Rebuild();
		TestEqual(TEXT("a second rebuild dumps the same graph"), Net->DumpLaneGraph(), First);
		TestEqual(TEXT("a second rebuild keeps the same stops"), Net->LastApproachStops.Num(), Stops.Num());
		for (int32 i = 0; i < Stops.Num() && i < Net->LastApproachStops.Num(); ++i)
		{
			TestTrue(TEXT("stop position is stable"), Net->LastApproachStops[i].Equals(Stops[i], 0.1));
		}
		TestEqual(TEXT("a second rebuild keeps the same widenings"), Net->LastArmWidenings.Num(), Widen.Num());
		TestTrue(TEXT("surface area is stable"), FMath::IsNearlyEqual(Net->LastSurfaceAreaCm2, Surface, 1.0));
		TestTrue(TEXT("sidewalk area is stable"), FMath::IsNearlyEqual(Net->LastSidewalkAreaCm2, Sidewalk, 1.0));

		URoadNetwork* Rev = NewObject<URoadNetwork>(GetTransientPackage());
		Rev->AddRoad(LaneRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 2, false));
		Rev->AddRoad(LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 2, false));
		Rev->Rebuild();
		const TSet<uint32> AKeys = JointKeys(Net->LaneGraph);
		const TSet<uint32> BKeys = JointKeys(Rev->LaneGraph);
		bool bSameJoints = AKeys.Num() == BKeys.Num();
		for (uint32 K : AKeys) { bSameJoints &= BKeys.Contains(K); }
		if (!bSameJoints)
		{
			AddInfo(TEXT("I10: joint clustering differs after the roads are added in the other order"));
		}
		else
		{
			TestEqual(TEXT("the same joint clustering dumps the same graph"), Rev->DumpLaneGraph(), First);
		}
	}

	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 3, true));
		Net->AddRoad(LaneRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 2, true));
		Net->Rebuild();
		int32 Entering = 0;
		for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
		{
			if (N.Road == 0 && N.bEntering && !N.bVirtual)
			{
				++Entering;
				TestTrue(TEXT("3 into 2: every entering lane leaves"), OutDegree(Net->LaneGraph, N.Key) >= 1);
			}
		}
		TestEqual(TEXT("3 into 2: three entering lanes"), Entering, 3);
		TestTrue(TEXT("3 into 2: the kerb lane is carried out"), Net->LaneGraph.Reductions >= 1);
		bool bMerge = false;
		for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges) { bMerge |= E.bMerge; }
		TestTrue(TEXT("3 into 2: the carried lane merges"), bMerge);
		Degrees(Net->LaneGraph, TEXT("3 into 2"));
	}

	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 2, false));
		Net->AddRoad(LaneRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 2, false));
		Net->AddRoad(LaneRoad({ {10000, 8000, 0}, {10000, 0, 0} }, { 3, 10 }, 3, true));
		Net->Rebuild();
		int32 Stem = 0, Middle = 0;
		for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
		{
			if (N.Road != 2 || !N.bEntering || N.bVirtual) { continue; }
			++Stem;
			TestTrue(TEXT("T: every stem lane leaves"), OutDegree(Net->LaneGraph, N.Key) >= 1);
			if (N.KerbRank == 1) { ++Middle; }
		}
		TestEqual(TEXT("T: three stem lanes"), Stem, 3);
		TestEqual(TEXT("T: the middle lane is in the graph"), Middle, 1);
		Degrees(Net->LaneGraph, TEXT("T"));
	}

	{
		const double Bear = FMath::DegreesToRadians(20.0);
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 2, true));
		Net->AddRoad(LaneRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 1, true));
		Net->AddRoad(LaneRoad({
			{10000, 0, 0},
			{10000 + 10000 * FMath::Cos(Bear), 10000 * FMath::Sin(Bear), 0} }, { 10, 3 }, 1, true));
		Net->Rebuild();
		int32 Entering = 0, Bears = 0;
		for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
		{
			if (N.Road == 0 && N.bEntering && !N.bVirtual)
			{
				++Entering;
				TestTrue(TEXT("Y: every approach lane leaves"), OutDegree(Net->LaneGraph, N.Key) >= 1);
			}
		}
		for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges)
		{
			if (E.Bear == ERoadNetTurnRole::BearLeft || E.Bear == ERoadNetTurnRole::BearRight)
			{
				++Bears;
				TestEqual(TEXT("Y: a bear is a through movement"), (int32)E.Kind, (int32)ERoadNetTurnRole::Through);
			}
		}
		TestEqual(TEXT("Y: two entering lanes"), Entering, 2);
		TestEqual(TEXT("Y: one bear branch"), Bears, 1);
		Degrees(Net->LaneGraph, TEXT("Y"));
	}

	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(LaneRoad({ {-10000, 0, 0}, {0, 0, 0} }, { 1, 10 }, 3, false));
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, { 10, 2 }, 2, false));
		Net->AddRoad(LaneRoad({ {0, 10000, 0}, {0, 0, 0} }, { 5, 10 }, 2, false));
		Net->AddRoad(LaneRoad({ {0, -10000, 0}, {0, 0, 0} }, { 4, 10 }, 2, false));
		TestTrue(TEXT("centre lane set"), Net->SetLaneType(0, 1, ERoadNetLaneType::CenterTurn));
		Net->Rebuild();
		int32 FromCenter = 0;
		for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
		{
			if (!N.bCenterTurn || !N.bEntering) { continue; }
			for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges)
			{
				if (E.FromKey != N.Key) { continue; }
				++FromCenter;
				TestTrue(TEXT("centre lane feeds only a left or a U-turn"),
					E.Kind == ERoadNetTurnRole::Left || E.Kind == ERoadNetTurnRole::UTurn);
			}
		}
		TestTrue(TEXT("centre lane has a movement"), FromCenter >= 1);
		Degrees(Net->LaneGraph, TEXT("centre"));
	}

	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef Loop = LaneRoad({
			{0, 0, 0}, {5000, 0, 0}, {5000, 5000, 0}, {-5000, 5000, 0}, {-5000, 0, 0}, {0, 0, 0}
		}, { 7, -1, -1, -1, -1, 7 }, 2, false);
		Net->AddRoad(Loop);
		Net->Rebuild();
		TestTrue(TEXT("loop: the two ends connect"), Net->LaneGraph.Edges.Num() >= 1);
		for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges)
		{
			TestEqual(TEXT("loop: through only"), (int32)E.Kind, (int32)ERoadNetTurnRole::Through);
		}
		Degrees(Net->LaneGraph, TEXT("loop"));
	}

	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		TArray<FVector> Ring;
		RoadNetMath::SampleCircle(FVector2D::ZeroVector, 2000.0, 0.0, 32, Ring);
		Net->AddRoad(LaneRoad(Ring, {}, 2, true));
		// x = 1000 lands the trimmed end mid-chord (60 deg), not on a ring vertex.
		Net->AddRoad(LaneRoad({ {1000, 6000, 0}, {1000, 2200, 0} }, {}, 2, true));
		const TArray<int32> Which = { 0 };
		TestTrue(TEXT("clean roundabout runs"), Net->CleanRoundabout(Which));
		Net->Rebuild();
		bool bI7 = false;
		for (const FString& R : Net->LaneGraph.Reports) { bI7 |= R.StartsWith(TEXT("I7:")); }
		TestTrue(TEXT("clean roundabout reports the unconnected entry"), bI7);
		int32 Entry = INDEX_NONE;
		for (int32 i = 0; i < Net->GetRoads().Num(); ++i)
		{
			const TArray<FVector>& Ref = Net->GetRoads()[i].Ref;
			if (Ref.Num() >= 2 && !Ref[0].Equals(Ref.Last(), 50.0)) { Entry = i; }
		}
		int32 EntryNodes = 0;
		if (Entry != INDEX_NONE)
		{
			for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes) { if (N.Road == Entry) { ++EntryNodes; } }
		}
		TestEqual(TEXT("clean roundabout entry is not in the lane graph"), EntryNodes, 0);

		TestTrue(TEXT("make junctions splits the ring at the entry"), Net->SplitRoadsAtCrossings() >= 1);
		Net->Rebuild();
		TestFalse(TEXT("make junctions clears I7"), HasReport(Net->LaneGraph, TEXT("I7:")));
		EntryNodes = 0;
		for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes) { if (N.Road == Entry) { ++EntryNodes; } }
		TestTrue(TEXT("the roundabout entry is in the lane graph"), Entry != INDEX_NONE && EntryNodes >= 1);
		for (int32 i = 0; i < Net->GetRoads().Num(); ++i)
		{
			const TArray<FVector>& Ref = Net->GetRoads()[i].Ref;
			AddInfo(FString::Printf(TEXT("roundabout road %d: %d pts, (%.0f,%.0f) -> (%.0f,%.0f), oneway %d"), i, Ref.Num(),
				Ref[0].X, Ref[0].Y, Ref.Last().X, Ref.Last().Y, Net->GetRoads()[i].Lanes.bOneway ? 1 : 0));
		}
		Degrees(Net->LaneGraph, TEXT("roundabout"));

		TestEqual(TEXT("the split ring keeps its roundabout"), Net->GetRoundaboutConfigs().Num(), 1);
		TArray<int32> RingIdx;
		for (int32 i = 0; i < Net->GetRoads().Num(); ++i) { if (Net->GetRoads()[i].bRoundabout) { RingIdx.Add(i); } }
		for (int32 i : RingIdx)
		{
			const TArray<FVector> Ref = Net->GetRoads()[i].Ref;
			for (int32 p = 0; p < Ref.Num(); ++p) { Net->MoveRoadPoint(i, p, Ref[p] + FVector(300.0, 0.0, 0.0)); }
		}
		Net->Rebuild();
		TestTrue(TEXT("a moved ring takes its island along"), Net->GetRoundaboutConfigs().Num() == 1
			&& Net->GetRoundaboutConfigs()[0].Location.Equals(FVector2D(300.0, 0.0), 30.0));
		for (int32 k = RingIdx.Num() - 1; k >= 0; --k) { Net->RemoveRoad(RingIdx[k]); }
		Net->Rebuild();
		TestEqual(TEXT("an erased ring leaves no roundabout behind"), Net->GetRoundaboutConfigs().Num(), 0);
	}

	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef A = LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 2 }, 2, true);
		A.BackwardTurnTokens = { TEXT("left") };
		A.ForwardTurnTokens = { TEXT("right") };
		Net->AddRoad(A);
		const int32 Tail = Net->SplitRoadAt(0, 5000.0);
		TestTrue(TEXT("split returns the tail"), Tail != INDEX_NONE);
		const TArray<FRoadDef>& Roads = Net->GetRoads();
		TestEqual(TEXT("head keeps the start tokens"), Roads[0].BackwardTurnTokens.Num(), 1);
		TestEqual(TEXT("head loses the end tokens"), Roads[0].ForwardTurnTokens.Num(), 0);
		if (Roads.IsValidIndex(Tail))
		{
			TestEqual(TEXT("tail keeps the end tokens"), Roads[Tail].ForwardTurnTokens.Num(), 1);
			TestEqual(TEXT("tail's new start has no tokens"), Roads[Tail].BackwardTurnTokens.Num(), 0);
		}

		URoadNetwork* Merged = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef M0 = LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, {}, 2, true);
		FRoadDef M1 = LaneRoad({ {0, 800, 0}, {10000, 800, 0} }, {}, 2, true);
		M0.ForwardTurnTokens = { TEXT("through") };
		M1.BackwardTurnTokens = { TEXT("left") };
		Merged->AddRoad(M0);
		Merged->AddRoad(M1);
		const TArray<int32> Pair = { 0, 1 };
		TestTrue(TEXT("merge roads runs"), Merged->MergeRoads(Pair));
		TestEqual(TEXT("a merged road drops turn tokens"), Merged->GetRoads().Last().ForwardTurnTokens.Num(), 0);
		TestEqual(TEXT("a merged road drops both ends"), Merged->GetRoads().Last().BackwardTurnTokens.Num(), 0);
	}

	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef A = LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 1, true);
		A.ForwardTurnTokens = { TEXT("through") };
		FRoadDef B = LaneRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 1, true);
		Net->AddRoad(A);
		Net->AddRoad(B);
		Net->Rebuild();
		int32 FromA = 0;
		for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
		{
			if (N.Road == 0 && N.bEntering && !N.bVirtual) { FromA += OutDegree(Net->LaneGraph, N.Key); }
		}
		TestTrue(TEXT("turn:lanes through keeps the seam"), FromA >= 1);

		URoadNetwork* None = NewObject<URoadNetwork>(GetTransientPackage());
		A.ForwardTurnTokens = { TEXT("none") };
		None->AddRoad(A);
		None->AddRoad(B);
		None->Rebuild();
		int32 Kept = 0;
		for (const FRoadNetLaneNode& N : None->LaneGraph.Nodes)
		{
			if (N.Road == 0 && N.bEntering && !N.bVirtual) { Kept += OutDegree(None->LaneGraph, N.Key); }
		}
		TestTrue(TEXT("turn:lanes none is an unmarked lane, not a closed one"), Kept >= 1);
	}

	{
		// left|left|through at a crossroads: both left lanes turn, into their own receiving lanes.
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef W = LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 3, true);
		W.ForwardTurnTokens = { TEXT("left"), TEXT("left"), TEXT("through") };
		Net->AddRoad(W);
		Net->AddRoad(LaneRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 2, true));
		Net->AddRoad(LaneRoad({ {10000, 0, 0}, {10000, -10000, 0} }, { 10, 3 }, 2, true));
		Net->AddRoad(LaneRoad({ {10000, 10000, 0}, {10000, 0, 0} }, { 4, 10 }, 2, true));
		Net->Rebuild();
		int32 Entering = 0, Fed = 0;
		TSet<uint32> LeftTargets;
		for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
		{
			if (N.Road != 0 || !N.bEntering || N.bVirtual) { continue; }
			++Entering;
			Fed += OutDegree(Net->LaneGraph, N.Key) >= 1 ? 1 : 0;
			for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges)
			{
				if (E.FromKey == N.Key && E.Kind == ERoadNetTurnRole::Left) { LeftTargets.Add(E.ToKey); }
			}
		}
		TestEqual(TEXT("double left: three entering lanes"), Entering, 3);
		TestEqual(TEXT("double left: every entering lane leaves"), Fed, 3);
		TestEqual(TEXT("double left: two left lanes into two receiving lanes"), LeftTargets.Num(), 2);
	}

	{
		const double Bend = FMath::DegreesToRadians(60.0);
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 2, false));
		Net->AddRoad(LaneRoad({
			{10000, 0, 0},
			{10000 + 10000 * FMath::Cos(Bend), 10000 * FMath::Sin(Bend), 0} }, { 10, 2 }, 2, false));
		Net->Rebuild();
		TestEqual(TEXT("60 degree seam: one through each way"), Net->LaneGraph.Edges.Num(), 2);
		for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges)
		{
			TestEqual(TEXT("60 degree seam stays through"), (int32)E.Kind, (int32)ERoadNetTurnRole::Through);
			TestEqual(TEXT("60 degree seam is not a bear"), (int32)E.Bear, (int32)ERoadNetTurnRole::Auto);
		}
		Degrees(Net->LaneGraph, TEXT("60 seam"));
	}

	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 2, false));
		Net->AddRoad(LaneRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 2, false));
		Net->AddRoad(LaneRoad({ {10000, 8000, 0}, {10000, 0, 0} }, { 3, 10 }, 4, true));
		Net->Rebuild();
		int32 Stem = 0;
		for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
		{
			if (N.Road != 2 || !N.bEntering || N.bVirtual) { continue; }
			++Stem;
			TestTrue(TEXT("4-lane stem: every lane leaves"), OutDegree(Net->LaneGraph, N.Key) >= 1);
		}
		TestEqual(TEXT("4-lane stem: four entering lanes"), Stem, 4);
		Degrees(Net->LaneGraph, TEXT("4-lane stem"));
	}

	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		for (int32 i = 0; i < 5; ++i)
		{
			const double A = i * 2.0 * PI / 5.0;
			const FVector End(10000.0 * FMath::Cos(A), 10000.0 * FMath::Sin(A), 0.0);
			Net->AddRoad(LaneRoad({ End, FVector::ZeroVector }, { 100 + i, 10 }, 2, false));
		}
		Net->Rebuild();
		Degrees(Net->LaneGraph, TEXT("5-way"));
		TestTrue(TEXT("5-way has movements"), Net->LaneGraph.Edges.Num() >= 5);
	}

	{
		const double Bear = FMath::DegreesToRadians(20.0);
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 2, true));
		Net->AddRoad(LaneRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 2, true));
		Net->AddRoad(LaneRoad({
			{10000, 0, 0},
			{10000 + 10000 * FMath::Cos(Bear), 10000 * FMath::Sin(Bear), 0} }, { 10, 3 }, 2, true));
		Net->Rebuild();
		// A tangential exit: the straight branch keeps both lanes, the kerb-side
		// branch's lanes open as a deceleration lane on the approach.
		bool bDecel = false;
		for (const FRoadNetArmWidening& W : Net->LastArmWidenings)
		{
			bDecel |= W.Road == 0 && !W.bAtStart && W.bSpeedChange && W.Lanes == 2 && W.HoldCm > 0.0;
		}
		TestTrue(TEXT("Y 2 into 2+2 opens the kerb branch's lanes as a deceleration lane"), bDecel);
		for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
		{
			if (!N.bVirtual && !N.bDegreeExempt && !N.bEntering && N.Road != 0)
			{
				TestTrue(*FString::Printf(TEXT("Y 2 into 2+2: road %d kerb rank %d is fed"), N.Road, N.KerbRank), InDegree(Net->LaneGraph, N.Key) >= 1);
			}
		}
		Degrees(Net->LaneGraph, TEXT("Y fan"));
	}

	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef Stem = LaneRoad({ {0, 8000, 0}, {0, 0, 0} }, { 3, 10 }, 1, true);
		Stem.Class = ERoadNetClass::Tertiary;
		Net->AddRoad(LaneRoad({ {-10000, 0, 0}, {0, 0, 0} }, { 1, 10 }, 2, false));
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, { 10, 2 }, 2, false));
		Net->AddRoad(Stem);
		Net->Rebuild();
		bool bBay = false;
		for (const FRoadNetArmWidening& W : Net->LastArmWidenings)
		{
			bBay |= W.Kind == ERoadNetWideningKind::RightBay || W.Kind == ERoadNetWideningKind::LeftBay;
		}
		bool bVirtual = false;
		for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes) { bVirtual |= N.bVirtual && N.bEntering; }
		TestTrue(TEXT("a tertiary single lane gets a turn bay"), bBay);
		TestTrue(TEXT("the bay is a lane in the graph"), bVirtual);
		Degrees(Net->LaneGraph, TEXT("bay"));
	}

	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {8000, 0, 0} }, { 1, 10 }, 3, true));
		Net->AddRoad(LaneRoad({ {8000, 0, 0}, {11000, 0, 0} }, { 10, 2 }, 2, true));
		Net->Rebuild();
		TestTrue(TEXT("an arm shorter than its taper is reported"), HasReport(Net->LaneGraph, TEXT("I8:")));
	}

	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->SetDriveOnLeft(true);
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 2, false));
		Net->AddRoad(LaneRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 2, false));
		Net->AddRoad(LaneRoad({ {10000, 8000, 0}, {10000, 0, 0} }, { 3, 10 }, 3, true));
		Net->Rebuild();
		int32 Stem = 0;
		for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
		{
			if (N.Road != 2 || !N.bEntering || N.bVirtual) { continue; }
			++Stem;
			TestTrue(TEXT("drive on the left: every stem lane leaves"), OutDegree(Net->LaneGraph, N.Key) >= 1);
		}
		TestEqual(TEXT("drive on the left: three stem lanes"), Stem, 3);
		Degrees(Net->LaneGraph, TEXT("left"));
	}

	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->bChannelizeJunctions = false;
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 3, true));
		Net->AddRoad(LaneRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 2, true));
		Net->Rebuild();
		bool bInside = false;
		for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges) { bInside |= E.bMerge && !E.bOnRoad; }
		TestTrue(TEXT("channelization off merges inside the junction"), bInside);
		TestTrue(TEXT("channelization off still counts the reduction"), Net->LaneGraph.Reductions >= 1);
		Degrees(Net->LaneGraph, TEXT("no channel"));
	}

	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(LaneRoad({ {0, 500, 0}, {20000, 500, 0} }, { 1, 2 }, 2, true));
		Net->AddRoad(LaneRoad({ {20000, -500, 0}, {0, -500, 0} }, { 3, 4 }, 2, true));
		Net->AddRoad(LaneRoad({ {10000, 10000, 0}, {10000, -10000, 0} }, { 5, 6 }, 2, false));
		Net->SplitRoadsAtCrossings();
		Net->Rebuild();
		int32 Big = 0;
		TMap<uint32, TSet<int32>> Arms;
		for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes) { Arms.FindOrAdd(N.JointKey).Add(N.Arm); }
		for (const TPair<uint32, TSet<int32>>& J : Arms) { if (J.Value.Num() >= 3) { ++Big; } }
		TestTrue(TEXT("dual carriageway: the crossing becomes a junction"), Big >= 1);
		Degrees(Net->LaneGraph, TEXT("dual carriageway"));
	}

	// Table 5.1: a lane dropped past a junction runs full width for `l`, then
	// tapers. Where the arm is too short for both, it is carried to the next
	// junction and becomes that junction's kerb turn lane.
	for (const bool bShort : { false, true })
	{
		const TCHAR* Name = bShort ? TEXT("junction drop, short arm") : TEXT("junction drop, long arm");
		const double ArmEnd = bShort ? 15000.0 : 40000.0;
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(LaneRoad({ {-30000, 0, 0}, {0, 0, 0} }, { 1, 10 }, 3, true));
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {ArmEnd, 0, 0} }, { 10, 4 }, 2, true));
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {0, -20000, 0} }, { 10, 3 }, 2, false));
		if (bShort)
		{
			Net->AddRoad(LaneRoad({ {ArmEnd, 0, 0}, {ArmEnd + 25000, 0, 0} }, { 4, 5 }, 2, true));
			Net->AddRoad(LaneRoad({ {ArmEnd, 0, 0}, {ArmEnd, 10000, 0} }, { 4, 6 }, 2, false));
			Net->AddRoad(LaneRoad({ {ArmEnd, 0, 0}, {ArmEnd, -10000, 0} }, { 4, 7 }, 2, false));
		}
		Net->Rebuild();

		const FRoadNetArmWidening* W = nullptr;
		bool bFarBay = false;
		for (const FRoadNetArmWidening& Cand : Net->LastArmWidenings)
		{
			if (Cand.Kind == ERoadNetWideningKind::Drop && Cand.Road == 1 && Cand.bAtStart) { W = &Cand; }
			bFarBay |= Cand.Road == 1 && !Cand.bAtStart && Cand.Kind == ERoadNetWideningKind::RightBay;
		}
		TestNotNull(*FString::Printf(TEXT("%s: the surplus lane is carried out"), Name), W);
		if (!W) { continue; }
		TestEqual(*FString::Printf(TEXT("%s: carried to the next node"), Name), W->bToNextNode, bShort);
		TestFalse(*FString::Printf(TEXT("%s: no length report"), Name), HasReport(Net->LaneGraph, TEXT("I8:")));
		if (!bShort)
		{
			TestTrue(*FString::Printf(TEXT("%s: full width for Table 5.1 l"), Name), W->HoldCm >= 9000.0 - 1.0);
			TestEqual(*FString::Printf(TEXT("%s: full width just before the taper"), Name), W->BulgeAt(W->HoldCm - 1.0, ArmEnd), W->WidthCm, 1.0);
			TestTrue(*FString::Printf(TEXT("%s: gone past the taper"), Name), W->BulgeAt(W->HoldCm + W->TaperCm + 1.0, ArmEnd) <= 0.0);
			bool bMerge = false;
			for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges) { bMerge |= E.bMerge; }
			TestTrue(*FString::Printf(TEXT("%s: the lane rejoins after the taper"), Name), bMerge);
		}
		else
		{
			TestEqual(*FString::Printf(TEXT("%s: full width the whole arm"), Name), W->BulgeAt(ArmEnd, ArmEnd), W->WidthCm, 1.0);
			TestFalse(*FString::Printf(TEXT("%s: the carried lane replaces the far kerb bay"), Name), bFarBay);
			bool bTurns = false;
			for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
			{
				if (N.Road != 1 || !N.bEntering || !N.bVirtual) { continue; }
				for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges)
				{
					bTurns |= E.FromKey == N.Key && E.Kind == ERoadNetTurnRole::Right;
				}
			}
			TestTrue(*FString::Printf(TEXT("%s: the carried lane turns right at the far junction"), Name), bTurns);
		}
		Degrees(Net->LaneGraph, Name);
	}

	// S16: left||right on three lanes into a two-lane exit. Only the middle lane
	// goes straight, so nothing is dropped beyond the junction.
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef Tagged = LaneRoad({ {-30000, 0, 0}, {0, 0, 0} }, { 1, 10 }, 3, true);
		Tagged.ForwardTurnTokens = { TEXT("left"), TEXT(""), TEXT("right") };
		Net->AddRoad(Tagged);
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {30000, 0, 0} }, { 10, 2 }, 2, true));
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {0, -20000, 0} }, { 10, 3 }, 2, false));
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {0, 20000, 0} }, { 10, 4 }, 2, false));
		Net->Rebuild();
		bool bDrop = false;
		for (const FRoadNetArmWidening& W : Net->LastArmWidenings) { bDrop |= W.Kind == ERoadNetWideningKind::Drop; }
		TestFalse(TEXT("turn-only lanes: nothing is dropped past the junction"), bDrop);
		TestFalse(TEXT("turn-only lanes: every leaving lane is fed"), HasReport(Net->LaneGraph, TEXT("I1:")));
		Degrees(Net->LaneGraph, TEXT("turn-only lanes"));
	}

	// A straight (or slightly bent) road changing lane count: the kerb lanes
	// are added or dropped beyond the seam, never at a painted junction.
	struct FSeamCase { int32 In, Out; double BendY; };
	for (const FSeamCase& C : { FSeamCase{ 1, 2, 0.0 }, FSeamCase{ 2, 3, 0.0 }, FSeamCase{ 1, 3, 0.0 },
		FSeamCase{ 3, 4, 0.0 }, FSeamCase{ 3, 1, 0.0 }, FSeamCase{ 2, 3, 1500.0 } })
	{
		const FString Name = FString::Printf(TEXT("seam %d->%d%s"), C.In, C.Out, C.BendY != 0.0 ? TEXT(" bent") : TEXT(""));
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {30000, 0, 0} }, { 1, 10 }, C.In, true));
		Net->AddRoad(LaneRoad({ {30000, 0, 0}, {60000, C.BendY, 0} }, { 10, 2 }, C.Out, true));
		Net->Rebuild();

		int32 Entering = 0, Leaving = 0;
		for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
		{
			if (N.bVirtual || N.bDegreeExempt) { continue; }
			if (N.Road == 0 && N.bEntering)
			{
				++Entering;
				TestTrue(*FString::Printf(TEXT("%s: every entering lane leaves"), *Name), OutDegree(Net->LaneGraph, N.Key) >= 1);
			}
			if (N.Road == 1 && !N.bEntering)
			{
				++Leaving;
				TestTrue(*FString::Printf(TEXT("%s: every leaving lane is fed"), *Name), InDegree(Net->LaneGraph, N.Key) >= 1);
			}
		}
		TestEqual(*FString::Printf(TEXT("%s: entering lanes"), *Name), Entering, C.In);
		TestEqual(*FString::Printf(TEXT("%s: leaving lanes"), *Name), Leaving, C.Out);
		TestEqual(*FString::Printf(TEXT("%s: no junction stop line"), *Name), Net->LastApproachStops.Num(), 0);

		const ERoadNetWideningKind Want = C.Out > C.In ? ERoadNetWideningKind::Gain : ERoadNetWideningKind::Drop;
		const int32 WantRoad = C.Out > C.In ? 0 : 1;
		const FRoadNetArmWidening* W = nullptr;
		for (const FRoadNetArmWidening& Cand : Net->LastArmWidenings)
		{
			if (Cand.Kind == Want && Cand.Road == WantRoad) { W = &Cand; break; }
		}
		TestNotNull(*FString::Printf(TEXT("%s: the narrow road widens on its kerb side"), *Name), W);
		if (W)
		{
			const int32 Diff = FMath::Abs(C.Out - C.In);
			TestEqual(*FString::Printf(TEXT("%s: one widening lane per added or dropped lane"), *Name), W->Lanes, Diff);
			TestTrue(*FString::Printf(TEXT("%s: widening is the extra lanes' width"), *Name),
				W->WidthCm > 250.0 * Diff && W->WidthCm < 400.0 * Diff);
			TestEqual(*FString::Printf(TEXT("%s: widening sits at the seam end"), *Name), W->bAtStart, WantRoad == 1);
			TestEqual(*FString::Printf(TEXT("%s: the kerb is on the right"), *Name), (int32)W->Side, (int32)ERoadNetSide::Right);
		}

		// The inner lanes carry straight on: the median lane of the narrow road
		// feeds (or is fed by) the median lane of the wide one.
		const FRoadNetLaneNode* InMedian = nullptr;
		const FRoadNetLaneNode* OutMedian = nullptr;
		for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
		{
			if (N.bVirtual || N.bDegreeExempt) { continue; }   // the gain fork shares the kerb lane's rank
			if (N.Road == 0 && N.bEntering && (!InMedian || N.KerbRank > InMedian->KerbRank)) { InMedian = &N; }
			if (N.Road == 1 && !N.bEntering && (!OutMedian || N.KerbRank > OutMedian->KerbRank)) { OutMedian = &N; }
		}
		bool bStraight = false;
		for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges)
		{
			bStraight |= InMedian && OutMedian && E.FromKey == InMedian->Key && E.ToKey == OutMedian->Key;
		}
		TestTrue(*FString::Printf(TEXT("%s: the median lanes pair through"), *Name), bStraight);
		// 803 rule ד: the narrower road moves over, so continuing lanes meet end to end.
		if (InMedian && OutMedian && C.BendY == 0.0)
		{
			const double Gap = FVector::Dist2D(InMedian->Position, OutMedian->Position);
			TestTrue(*FString::Printf(TEXT("%s: continuing lanes meet at the seam (gap %.0f cm)"), *Name, Gap), Gap < 15.0);
		}
		Degrees(Net->LaneGraph, *Name);
	}

	// A 3+1 road running straight on into a 1+1 road at a T: the through lanes
	// meet lane to lane across the junction, not centre line to centre line. With
	// a 12.7 m connector into an older 1+1 road, the shift and the dropped lanes
	// carry on through that plain seam instead of stepping at it.
	for (const bool bShortArm : { false, true })
	{
		const TCHAR* Name = bShortArm ? TEXT("T 3+1 into a short 1+1") : TEXT("T 3+1 into 1+1");
		const double ArmEnd = bShortArm ? -1268.0 : -20000.0;
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef Wide = LaneRoad({ {20000, 0, 0}, {0, 0, 0} }, { 2, 10 }, 4, false);
		Wide.Lanes.Forward = 3;
		Wide.Lanes.Backward = 1;
		Net->AddRoad(Wide);
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {ArmEnd, 0, 0} }, { 10, 3 }, 2, false));
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {0, -20000, 0} }, { 10, 4 }, 2, false));
		if (bShortArm) { Net->AddRoad(LaneRoad({ {ArmEnd, 0, 0}, {-20000, 0, 0} }, { 3, 5 }, 2, false)); }
		Net->Rebuild();

		const FRoadNetLaneGraph& G = Net->LaneGraph;
		auto NearJunction = [](const FRoadNetLaneNode& N) { return FVector::Dist2D(N.Position, FVector::ZeroVector) < 3000.0; };
		int32 Checked = 0;
		for (const FRoadNetLaneEdge& E : G.Edges)
		{
			const FRoadNetLaneNode* From = NodeByKey(G, E.FromKey);
			const FRoadNetLaneNode* To = NodeByKey(G, E.ToKey);
			if (!From || !To || From->bVirtual || To->bVirtual || E.Kind != ERoadNetTurnRole::Through) { continue; }
			if (!NearJunction(*From) || !NearJunction(*To)) { continue; }
			const bool bAxis = (From->Road == 0 && To->Road == 1) || (From->Road == 1 && To->Road == 0);
			if (!bAxis) { continue; }
			++Checked;
			TestTrue(*FString::Printf(TEXT("%s: road %d lane meets road %d lane end to end (%.0f cm sideways)"),
				Name, From->Road, To->Road, FMath::Abs(From->Position.Y - To->Position.Y)),
				FMath::Abs(From->Position.Y - To->Position.Y) < 15.0);
		}
		TestEqual(*FString::Printf(TEXT("%s: one through pair each way"), Name), Checked, 2);
		if (bShortArm)
		{
			bool bCarried = false;
			for (const FRoadNetArmWidening& W : Net->LastArmWidenings) { bCarried |= W.Road == 3 && W.Kind == ERoadNetWideningKind::Drop; }
			TestTrue(*FString::Printf(TEXT("%s: the dropped lanes carry on past the seam"), Name), bCarried);
			TestFalse(*FString::Printf(TEXT("%s: no length report"), Name), HasReport(G, TEXT("I8:")));
			for (const FRoadNetLaneEdge& E : G.Edges)
			{
				const FRoadNetLaneNode* From = NodeByKey(G, E.FromKey);
				const FRoadNetLaneNode* To = NodeByKey(G, E.ToKey);
				if (!From || !To || NearJunction(*From)) { continue; }
				if ((From->Road == 1 && To->Road == 3) || (From->Road == 3 && To->Road == 1))
				{
					TestTrue(*FString::Printf(TEXT("%s: lanes meet at the plain seam (%.0f cm)"), Name, FVector::Dist2D(E.P0, E.P1)),
						FVector::Dist2D(E.P0, E.P1) < 15.0);
				}
			}
		}
		Degrees(G, Name);
	}

	// Equal carriageways (S05: two-way 1+1 against one-way 2, both 7 m): the lane
	// that ends in one direction frees the width the other direction gains, so
	// nothing widens or shifts. The kerb lane continues in place; the extra lane
	// is the inner one.
	for (const bool bGain : { true, false })
	{
		const TCHAR* Name = bGain ? TEXT("1+1 into one-way 2") : TEXT("one-way 2 into 1+1");
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {30000, 0, 0} }, { 1, 10 }, 2, !bGain));
		Net->AddRoad(LaneRoad({ {30000, 0, 0}, {60000, 0, 0} }, { 10, 2 }, 2, bGain));
		Net->Rebuild();
		for (const FRoadNetArmWidening& W : Net->LastArmWidenings)
		{
			TestFalse(*FString::Printf(TEXT("%s: no widening at the seam (road %d kind %d)"), Name, W.Road, (int32)W.Kind),
				W.Kind == ERoadNetWideningKind::Gain || W.Kind == ERoadNetWideningKind::Drop);
		}
		TestEqual(*FString::Printf(TEXT("%s: no junction stop line"), Name), Net->LastApproachStops.Num(), 0);
		auto Node = [&](int32 Road, bool bEntering, int32 Rank) -> const FRoadNetLaneNode*
		{
			for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
			{
				if (!N.bVirtual && !N.bDegreeExempt && N.Road == Road && N.bEntering == bEntering && N.KerbRank == Rank) { return &N; }
			}
			return nullptr;
		};
		auto Edge = [&](const FRoadNetLaneNode* A, const FRoadNetLaneNode* B) -> const FRoadNetLaneEdge*
		{
			if (!A || !B) { return nullptr; }
			for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges) { if (E.FromKey == A->Key && E.ToKey == B->Key) { return &E; } }
			return nullptr;
		};
		const FRoadNetLaneNode* InKerb = Node(0, true, 0);
		const FRoadNetLaneNode* OutKerb = Node(1, false, 0);
		TestNotNull(*FString::Printf(TEXT("%s: the kerb lanes connect"), Name), Edge(InKerb, OutKerb));
		if (InKerb && OutKerb)
		{
			const double Gap = FVector::Dist2D(InKerb->Position, OutKerb->Position);
			TestTrue(*FString::Printf(TEXT("%s: the kerb lane continues in place (gap %.0f cm)"), Name, Gap), Gap < 15.0);
		}
		if (bGain)
		{
			TestNotNull(TEXT("1+1 into one-way 2: the inner lane forks off the through lane"), Edge(InKerb, Node(1, false, 1)));
		}
		else
		{
			const FRoadNetLaneEdge* M = Edge(Node(0, true, 1), OutKerb);
			TestTrue(TEXT("one-way 2 into 1+1: the inner lane merges into the kerb lane"), M && M->bMerge);
		}
	}

	// Tangential merge (S06): a 2-lane road and a 1-lane ramp from the kerb side
	// join. Into 3 lanes the main lanes take the two median-side lanes and the
	// ramp the kerb lane, nothing crossing; into 2 the ramp lane runs on as an
	// acceleration lane.
	for (const int32 OutN : { 3, 2 })
	{
		const FString Name = FString::Printf(TEXT("merge 2+1 into %d"), OutN);
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {30000, 0, 0} }, { 1, 10 }, 2, true));
		Net->AddRoad(LaneRoad({ {10000, 7000, 0}, {30000, 0, 0} }, { 3, 10 }, 1, true));
		Net->AddRoad(LaneRoad({ {30000, 0, 0}, {60000, 0, 0} }, { 10, 2 }, OutN, true));
		Net->Rebuild();
		auto Node = [&](int32 Road, bool bEntering, int32 Rank) -> const FRoadNetLaneNode*
		{
			for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
			{
				if (!N.bVirtual && !N.bDegreeExempt && N.Road == Road && N.bEntering == bEntering && N.KerbRank == Rank) { return &N; }
			}
			return nullptr;
		};
		auto Linked = [&](const FRoadNetLaneNode* A, const FRoadNetLaneNode* B)
		{
			if (!A || !B) { return false; }
			for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges) { if (E.FromKey == A->Key && E.ToKey == B->Key) { return true; } }
			return false;
		};
		const FRoadNetLaneNode* MainKerb = Node(0, true, 0);
		const FRoadNetLaneNode* MainInner = Node(0, true, 1);
		const FRoadNetLaneNode* Ramp = Node(1, true, 0);
		if (OutN == 3)
		{
			TestTrue(*FString::Printf(TEXT("%s: main kerb lane continues in the middle lane"), *Name), Linked(MainKerb, Node(2, false, 1)));
			TestTrue(*FString::Printf(TEXT("%s: main inner lane continues in the median lane"), *Name), Linked(MainInner, Node(2, false, 2)));
			TestTrue(*FString::Printf(TEXT("%s: the ramp becomes the kerb lane"), *Name), Linked(Ramp, Node(2, false, 0)));
			TestFalse(*FString::Printf(TEXT("%s: the ramp does not cross the main lanes"), *Name),
				Linked(Ramp, Node(2, false, 1)) || Linked(Ramp, Node(2, false, 2)));
			TestFalse(*FString::Printf(TEXT("%s: the main road does not fork into the ramp's lane"), *Name), Linked(MainKerb, Node(2, false, 0)));
			if (MainKerb && Node(2, false, 1))
			{
				const double Off = FMath::Abs(MainKerb->Position.Y - Node(2, false, 1)->Position.Y);
				TestTrue(*FString::Printf(TEXT("%s: the main road moves over to its lanes (%.0f cm off)"), *Name, Off), Off < 60.0);
			}
		}
		else
		{
			TestTrue(*FString::Printf(TEXT("%s: main lanes keep both leaving lanes"), *Name),
				Linked(MainKerb, Node(2, false, 0)) && Linked(MainInner, Node(2, false, 1)));
			TestTrue(*FString::Printf(TEXT("%s: the ramp lane goes somewhere"), *Name), Ramp && OutDegree(Net->LaneGraph, Ramp->Key) >= 1);
			bool bAccel = false;
			for (const FRoadNetArmWidening& W : Net->LastArmWidenings)
			{
				bAccel |= W.Road == 2 && W.Kind == ERoadNetWideningKind::Drop && W.bSpeedChange;
			}
			TestTrue(*FString::Printf(TEXT("%s: the ramp lane runs on as an acceleration lane"), *Name), bAccel);
			// The terrain conform flattens the acceleration lane, not just the nominal road.
			const FRoadNetDeformCorridor* Cor = Net->GetDeformCorridors().FindByPredicate([](const FRoadNetDeformCorridor& C)
			{
				return C.Points.Num() > 1 && FVector::Dist2D(C.Points[0], FVector(30000, 0, 0)) < 2000.0 && C.Points.Last().X > 50000.0;
			});
			TestTrue(*FString::Printf(TEXT("%s: the conform corridor widens over the acceleration lane"), *Name),
				Cor && Cor->ExtraHalfCm.Num() == Cor->Points.Num() && Cor->ExtraHalfCm[0] >= 300.f && Cor->ExtraHalfCm.Last() < 10.f);
		}
		TestEqual(*FString::Printf(TEXT("%s: no stop line at a merge (301)"), *Name), Net->LastApproachStops.Num(), 0);
		TestTrue(*FString::Printf(TEXT("%s: an 815 gore between ramp and main road (fig. 7)"), *Name), Net->LastTangentialGores >= 1);
		Degrees(Net->LaneGraph, *Name);
	}

	// Tangential exit: a one-way road splits into a 2-lane main road and a 1-lane
	// kerb-side exit. From 3 lanes the kerb lane takes the exit and the other two
	// carry on; from 2 the main road keeps both and the exit lane opens as a
	// deceleration lane (Table 6.2, 808 fig. 21).
	for (const int32 InN : { 3, 2 })
	{
		const FString Name = FString::Printf(TEXT("exit %d into 2+1"), InN);
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {30000, 0, 0} }, { 1, 10 }, InN, true));
		Net->AddRoad(LaneRoad({ {30000, 0, 0}, {60000, 0, 0} }, { 10, 2 }, 2, true));
		Net->AddRoad(LaneRoad({ {30000, 0, 0}, {50000, 7000, 0} }, { 10, 3 }, 1, true));
		Net->Rebuild();
		auto Node = [&](int32 Road, bool bEntering, int32 Rank) -> const FRoadNetLaneNode*
		{
			for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
			{
				if (!N.bVirtual && !N.bDegreeExempt && N.Road == Road && N.bEntering == bEntering && N.KerbRank == Rank) { return &N; }
			}
			return nullptr;
		};
		auto Linked = [&](const FRoadNetLaneNode* A, const FRoadNetLaneNode* B)
		{
			if (!A || !B) { return false; }
			for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges) { if (E.FromKey == A->Key && E.ToKey == B->Key) { return true; } }
			return false;
		};
		const FRoadNetLaneNode* Exit = Node(2, false, 0);
		bool bDecel = false;
		for (const FRoadNetArmWidening& W : Net->LastArmWidenings)
		{
			bDecel |= W.Road == 0 && !W.bAtStart && W.Kind == ERoadNetWideningKind::RightBay && W.bSpeedChange && W.HoldCm > 0.0;
		}
		if (InN == 3)
		{
			TestTrue(*FString::Printf(TEXT("%s: the kerb lane takes the exit"), *Name), Linked(Node(0, true, 0), Exit));
			TestTrue(*FString::Printf(TEXT("%s: the other lanes carry on"), *Name),
				Linked(Node(0, true, 1), Node(1, false, 0)) && Linked(Node(0, true, 2), Node(1, false, 1)));
			TestFalse(*FString::Printf(TEXT("%s: no deceleration lane needed"), *Name), bDecel);
			if (Node(0, true, 1) && Node(1, false, 0))
			{
				const double Off = FMath::Abs(Node(0, true, 1)->Position.Y - Node(1, false, 0)->Position.Y);
				TestTrue(*FString::Printf(TEXT("%s: the main road moves over to its lanes (%.0f cm off)"), *Name, Off), Off < 60.0);
			}
		}
		else
		{
			TestTrue(*FString::Printf(TEXT("%s: the main road keeps both lanes"), *Name),
				Linked(Node(0, true, 0), Node(1, false, 0)) && Linked(Node(0, true, 1), Node(1, false, 1)));
			TestFalse(*FString::Printf(TEXT("%s: no through lane is taken for the exit"), *Name),
				Linked(Node(0, true, 0), Exit) || Linked(Node(0, true, 1), Exit));
			TestTrue(*FString::Printf(TEXT("%s: the exit lane is fed"), *Name), Exit && InDegree(Net->LaneGraph, Exit->Key) >= 1);
			TestTrue(*FString::Printf(TEXT("%s: a deceleration lane opens before the exit"), *Name), bDecel);
		}
		TestEqual(*FString::Printf(TEXT("%s: no stop line at an exit"), *Name), Net->LastApproachStops.Num(), 0);
		TestTrue(*FString::Printf(TEXT("%s: an 815 gore between exit and main road (fig. 6)"), *Name), Net->LastTangentialGores >= 1);
		Degrees(Net->LaneGraph, *Name);
	}

	// A median on one side of a seam only (the user adds one to a single road):
	// the wider median closes over the Table 4 length so every lane meets the
	// lane it continues into end to end, in both directions.
	struct FMedianSeam { int32 In; double MIn; int32 Out; double MOut; };
	for (const FMedianSeam& C : { FMedianSeam{ 2, 400.0, 2, 0.0 }, FMedianSeam{ 2, 0.0, 2, 400.0 },
		FMedianSeam{ 4, 300.0, 2, 0.0 }, FMedianSeam{ 2, 0.0, 4, 300.0 }, FMedianSeam{ 2, 600.0, 2, 200.0 } })
	{
		const FString Name = FString::Printf(TEXT("median seam %d|%.0f -> %d|%.0f"), C.In, C.MIn, C.Out, C.MOut);
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef A = LaneRoad({ {0, 0, 0}, {30000, 0, 0} }, { 1, 10 }, C.In, false);
		FRoadDef B = LaneRoad({ {30000, 0, 0}, {60000, 0, 0} }, { 10, 2 }, C.Out, false);
		A.Lanes.bMedian = C.MIn > 0.0; A.Lanes.MedianWidth = (float)C.MIn;
		B.Lanes.bMedian = C.MOut > 0.0; B.Lanes.MedianWidth = (float)C.MOut;
		Net->AddRoad(A);
		Net->AddRoad(B);
		Net->Rebuild();
		const int32 Wide = C.MIn > C.MOut ? 0 : 1;
		const FRoadNetLateralMove* Mv = Net->LastLateralMoves.FindByPredicate([&](const FRoadNetLateralMove& M)
			{ return M.bSqueeze && M.Road == Wide; });
		TestNotNull(*FString::Printf(TEXT("%s: the wider median closes"), *Name), Mv);
		if (Mv)
		{
			TestEqual(*FString::Printf(TEXT("%s: it closes at the seam"), *Name), Mv->bAtStart, Wide == 1);
			TestEqual(*FString::Printf(TEXT("%s: by half the difference each side"), *Name), Mv->AmountCm, 0.5 * FMath::Abs(C.MIn - C.MOut), 1.0);
			TestTrue(*FString::Printf(TEXT("%s: over a taper"), *Name), Mv->TaperCm > 1000.0);
		}
		int32 Throughs = 0;
		for (const FRoadNetLaneEdge& E : Net->LaneGraph.Edges)
		{
			const FRoadNetLaneNode* From = NodeByKey(Net->LaneGraph, E.FromKey);
			const FRoadNetLaneNode* To = NodeByKey(Net->LaneGraph, E.ToKey);
			if (!From || !To || From->bVirtual || To->bVirtual || From->Road == To->Road) { continue; }
			++Throughs;
			const double Gap = FVector::Dist2D(From->Position, To->Position);
			TestTrue(*FString::Printf(TEXT("%s: road %d rank %d meets road %d rank %d (gap %.0f cm)"), *Name,
				From->Road, From->KerbRank, To->Road, To->KerbRank, Gap), Gap < 15.0);
		}
		TestTrue(*FString::Printf(TEXT("%s: lanes pair through both ways"), *Name), Throughs >= 2);
		TestEqual(*FString::Printf(TEXT("%s: no junction stop line"), *Name), Net->LastApproachStops.Num(), 0);
		Degrees(Net->LaneGraph, *Name);
	}

	// A left-turn bay on a divided road is cut into the median, which keeps
	// 2.50 m (כרך 2 §7.4.3). A 6 m median gives the whole lane; a 4 m one gives
	// 1.5 m and the opposing half moves over by the other 2 m.
	for (const double Median : { 600.0, 400.0 })
	{
		const FString Name = FString::Printf(TEXT("far bay in a %.0f cm median"), Median);
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef App = LaneRoad({ {-30000, 0, 0}, {0, 0, 0} }, { 1, 10 }, 2, false);
		App.Class = ERoadNetClass::Tertiary;
		App.Lanes.bMedian = true;
		App.Lanes.MedianWidth = (float)Median;
		Net->AddRoad(App);
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {30000, 0, 0} }, { 10, 2 }, 2, false));
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {0, -20000, 0} }, { 10, 3 }, 2, false));
		Net->AddRoad(LaneRoad({ {0, 0, 0}, {0, 20000, 0} }, { 10, 4 }, 2, false));
		Net->Rebuild();
		const FRoadNetArmWidening* W = Net->LastArmWidenings.FindByPredicate([](const FRoadNetArmWidening& Cand)
			{ return Cand.Road == 0 && !Cand.bAtStart && Cand.bFarSide; });
		TestNotNull(*FString::Printf(TEXT("%s: the approach gets a far-side bay"), *Name), W);
		if (!W) { continue; }
		const double Given = FMath::Clamp(Median - 250.0, 0.0, W->WidthCm);
		TestEqual(*FString::Printf(TEXT("%s: the median gives all it can spare"), *Name), W->MedianCm, Given, 1.0);
		const FRoadNetLateralMove* Mv = Net->LastLateralMoves.FindByPredicate([](const FRoadNetLateralMove& M)
			{ return !M.bSqueeze && M.Road == 0 && !M.bAtStart; });
		if (W->WidthCm - Given > 1.0)
		{
			TestNotNull(*FString::Printf(TEXT("%s: the opposing half moves over"), *Name), Mv);
			if (Mv) { TestEqual(*FString::Printf(TEXT("%s: by the rest of the lane"), *Name), Mv->AmountCm, W->WidthCm - Given, 1.0); }
		}
		else
		{
			TestNull(*FString::Printf(TEXT("%s: nothing else moves"), *Name), Mv);
		}

		// At the junction end: the median keeps 2.50 m and the bay is exactly the
		// lane between it and the through lane.
		FRoadCurves Ref;
		Ref.Sampled = App.Ref;
		for (const FRoadNetLateralMove& M : Net->LastLateralMoves) { if (M.Road == 0) { Ref.Moves.Add(M); } }
		FRoadNetMedianProfile Prof;
		TestTrue(*FString::Printf(TEXT("%s: the road has a median"), *Name), Prof.Init(App, 0, Ref, Net->LastArmWidenings, false));
		double Lo = 0.0, Hi = 0.0;
		Prof.EdgesAt(Prof.LengthCm, Lo, Hi);
		TestEqual(*FString::Printf(TEXT("%s: 2.50 m of median is kept"), *Name), Hi - Lo, 250.0, 1.0);
		// Entering traffic runs +X on the +Y side, so the through lane's inner edge is +Median/2.
		TestEqual(*FString::Printf(TEXT("%s: the bay is one lane wide"), *Name), 0.5 * Median - Hi, W->WidthCm, 1.0);

		// The bay never overlaps the opposing lane in the lane graph.
		const FRoadNetLaneNode* Bay = nullptr;
		const FRoadNetLaneNode* Opp = nullptr;
		for (const FRoadNetLaneNode& N : Net->LaneGraph.Nodes)
		{
			if (N.Road != 0) { continue; }
			if (N.bEntering && N.bVirtual && N.KerbRank == 1000) { Bay = &N; }
			if (!N.bEntering && !N.bVirtual) { Opp = &N; }
		}
		TestTrue(*FString::Printf(TEXT("%s: bay and opposing lane exist"), *Name), Bay && Opp);
		if (Bay && Opp)
		{
			const double Apart = FMath::Abs(Bay->Position.Y - Opp->Position.Y);
			TestTrue(*FString::Printf(TEXT("%s: bay clear of the opposing lane (%.0f cm apart)"), *Name, Apart), Apart >= W->WidthCm - 20.0);
		}
		Degrees(Net->LaneGraph, *Name);
	}

	// Adding a median puts it between the two directions, never beside a
	// sidewalk or between two lanes of the same direction.
	for (const bool bUneven : { false, true })
	{
		const TCHAR* Name = bUneven ? TEXT("median on 2+1") : TEXT("median beside sidewalks");
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef R = LaneRoad({ {0, 0, 0}, {20000, 0, 0} }, { 1, 2 }, 0, false);
		R.Lanes.Forward = 1;
		R.Lanes.Backward = bUneven ? 2 : 1;
		R.Lanes.bSidewalkLeft = R.Lanes.bSidewalkRight = !bUneven;
		R.Lanes.SidewalkWidth = 200.f;
		Net->AddRoad(R);
		TestTrue(*FString::Printf(TEXT("%s: toggled on"), Name), Net->ToggleMedian(0));
		const TArray<FRoadNetLane>& L = Net->GetRoads()[0].Lanes.DetailedLanes;
		const int32 Mi = L.IndexOfByPredicate([](const FRoadNetLane& Ln) { return Ln.Type == ERoadNetLaneType::Median; });
		TestTrue(*FString::Printf(TEXT("%s: median has a lane on each side"), Name), Mi > 0 && Mi + 1 < L.Num());
		if (Mi > 0 && Mi + 1 < L.Num())
		{
			TestTrue(*FString::Printf(TEXT("%s: backward traffic on its left"), Name), L[Mi - 1].bDrivable() && L[Mi - 1].bTravelsBackward(false));
			TestTrue(*FString::Printf(TEXT("%s: forward traffic on its right"), Name), L[Mi + 1].bDrivable() && L[Mi + 1].bTravelsForward(false));
		}
		Net->Rebuild();
		Degrees(Net->LaneGraph, Name);
	}

	return true;
}

#endif
