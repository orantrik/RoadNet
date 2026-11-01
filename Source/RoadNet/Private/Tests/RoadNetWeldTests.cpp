#include "RoadNetwork.h"
#include "Misc/AutomationTest.h"
#include "UObject/Package.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	FRoadDef Road(std::initializer_list<FVector> Pts, ERoadNetClass Class = ERoadNetClass::Residential)
	{
		FRoadDef R;
		R.Ref = Pts;
		R.Class = Class;
		return R;
	}

	// Every road end within Radius of At, and whether they all share one point and one id.
	struct FJunctionCheck { int32 Ends = 0; bool bOnePoint = true; bool bOneId = true; };
	FJunctionCheck CheckJunction(const URoadNetwork* Net, const FVector2D& At, double Radius)
	{
		FJunctionCheck C;
		FVector First = FVector::ZeroVector;
		int64 Id = -2;
		for (const FRoadDef& R : Net->GetRoads())
		{
			for (int32 i : { 0, R.Ref.Num() - 1 })
			{
				if (FVector2D::Distance(FVector2D(R.Ref[i]), At) > Radius) { continue; }
				const int64 EndId = R.NodeIds.IsValidIndex(i) ? R.NodeIds[i] : -1;
				if (C.Ends == 0) { First = R.Ref[i]; Id = EndId; }
				C.bOnePoint = C.bOnePoint && R.Ref[i].Equals(First, 0.5);
				C.bOneId = C.bOneId && EndId == Id && EndId >= 0;
				++C.Ends;
			}
		}
		return C;
	}
}

// A T where the side road stops 1.5 m short of the through road (no shared node, hand-drawn:
// no ids at all). The through road splits there and all three ends become one point, one id.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetWeldTJunctionTest, "RoadNet.Weld.TJunction",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRoadNetWeldTJunctionTest::RunTest(const FString&)
{
	URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
	Net->AddRoad(Road({ FVector(-5000, 0, 0), FVector(0, 0, 0), FVector(5000, 0, 0) }, ERoadNetClass::Secondary));
	Net->AddRoad(Road({ FVector(0, 150, 0), FVector(0, 5000, 0) }));

	const int32 Moved = Net->WeldJunctionPoints(300.0);
	TestTrue(TEXT("something moved"), Moved > 0);
	TestEqual(TEXT("the through road was split at the junction"), Net->NumRoads(), 3);
	const FJunctionCheck C = CheckJunction(Net, FVector2D(0, 0), 400.0);
	TestEqual(TEXT("three road ends at the junction"), C.Ends, 3);
	TestTrue(TEXT("one shared point"), C.bOnePoint);
	TestTrue(TEXT("one shared node id"), C.bOneId);
	for (const FRoadDef& R : Net->GetRoads())
	{
		TestEqual(TEXT("node ids are filled for every point"), R.NodeIds.Num(), R.Ref.Num());
	}
	return true;
}

// The OSM 2.5 m overshoot: the side road runs past the through road. The overshoot is
// trimmed and the side road ends on the through road's (split) point.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetWeldOvershootTest, "RoadNet.Weld.Overshoot",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRoadNetWeldOvershootTest::RunTest(const FString&)
{
	URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
	Net->AddRoad(Road({ FVector(-5000, 0, 0), FVector(5000, 0, 0) }, ERoadNetClass::Primary));
	Net->AddRoad(Road({ FVector(0, -250, 0), FVector(0, 0, 0), FVector(0, 6000, 0) }));

	Net->WeldJunctionPoints(300.0);
	const FJunctionCheck C = CheckJunction(Net, FVector2D(0, 0), 400.0);
	TestEqual(TEXT("three road ends at the junction (overshoot trimmed, through road split)"), C.Ends, 3);
	TestTrue(TEXT("one shared point"), C.bOnePoint);
	for (const FRoadDef& R : Net->GetRoads())
	{
		for (const FVector& P : R.Ref) { TestTrue(TEXT("nothing left south of the through road"), P.Y >= -1.0); }
	}
	return true;
}

// Four arms whose ends sit 1-2.5 m apart around the same crossing: one point.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetWeldSloppyCrossTest, "RoadNet.Weld.SloppyFourWay",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRoadNetWeldSloppyCrossTest::RunTest(const FString&)
{
	URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
	Net->AddRoad(Road({ FVector(-5000, 0, 0), FVector(-100, 20, 0) }));
	Net->AddRoad(Road({ FVector(5000, 0, 0), FVector(120, -30, 0) }));
	Net->AddRoad(Road({ FVector(0, 5000, 0), FVector(40, 110, 0) }));
	Net->AddRoad(Road({ FVector(0, -5000, 0), FVector(-60, -90, 0) }));

	Net->WeldJunctionPoints(300.0);
	const FJunctionCheck C = CheckJunction(Net, FVector2D(0, 0), 400.0);
	TestEqual(TEXT("four ends"), C.Ends, 4);
	TestTrue(TEXT("one shared point"), C.bOnePoint);
	TestTrue(TEXT("one shared node id"), C.bOneId);
	TestEqual(TEXT("no road split or removed"), Net->NumRoads(), 4);
	return true;
}

// A bridge ending above a street is grade-separated: never welded or split.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetWeldBridgeTest, "RoadNet.Weld.BridgeLeftAlone",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRoadNetWeldBridgeTest::RunTest(const FString&)
{
	URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
	Net->AddRoad(Road({ FVector(-5000, 0, 0), FVector(5000, 0, 0) }));
	FRoadDef Bridge = Road({ FVector(0, 100, 600), FVector(0, 5000, 600) });
	Bridge.bBridge = true;
	Bridge.Layer = 1;
	Net->AddRoad(Bridge);

	TestEqual(TEXT("nothing moved"), Net->WeldJunctionPoints(300.0), 0);
	TestEqual(TEXT("street not split"), Net->NumRoads(), 2);
	return true;
}

// A welded network is a fixed point; OnlyRoads leaves junctions it does not touch alone.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetWeldIdempotentTest, "RoadNet.Weld.IdempotentAndScoped",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRoadNetWeldIdempotentTest::RunTest(const FString&)
{
	URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
	// Junction A (seeded) and junction B (not seeded), far apart.
	Net->AddRoad(Road({ FVector(-5000, 0, 0), FVector(-80, 0, 0) }));
	const int32 Seed = Net->AddRoad(Road({ FVector(5000, 0, 0), FVector(90, 40, 0) }));
	Net->AddRoad(Road({ FVector(-5000, 50000, 0), FVector(-80, 50000, 0) }));
	Net->AddRoad(Road({ FVector(5000, 50000, 0), FVector(90, 50040, 0) }));

	const TArray<FGuid> Only = { Net->GetRoads()[Seed].Id };
	Net->WeldJunctionPoints(300.0, &Only);
	TestTrue(TEXT("seeded junction welded"), CheckJunction(Net, FVector2D(0, 0), 400.0).bOnePoint);
	TestFalse(TEXT("unseeded junction untouched"), CheckJunction(Net, FVector2D(0, 50000), 400.0).bOnePoint);

	Net->WeldJunctionPoints(300.0);
	TestTrue(TEXT("full weld joins the other junction"), CheckJunction(Net, FVector2D(0, 50000), 400.0).bOnePoint);
	TestEqual(TEXT("second full weld moves nothing"), Net->WeldJunctionPoints(300.0), 0);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
