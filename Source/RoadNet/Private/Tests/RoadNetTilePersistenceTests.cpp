#include "RoadNetwork.h"
#include "RoadNetTileActor.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

// Live road tiles are the level's road geometry until someone converts them to
// meshes. A transient tile is dropped on save and by the cook, which leaves a
// reopened or packaged level with no roads.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetTilesArePersistentTest, "RoadNet.Tiles.Persistent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRoadNetTilesArePersistentTest::RunTest(const FString&)
{
	UWorld* World = UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld*/ false);
	FWorldContext& Ctx = GEngine->CreateNewWorldContext(EWorldType::Editor);
	Ctx.SetCurrentWorld(World);

	URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
	Net->SetWorld(World);
	FRoadDef R;
	R.Ref = { FVector(0, 0, 0), FVector(10000, 0, 0) };
	R.NodeIds = { 1, 2 };
	R.Lanes.Total = 2;
	Net->AddRoad(R);
	Net->Rebuild();

	int32 Tiles = 0;
	for (TActorIterator<ARoadNetTileActor> It(World); It; ++It)
	{
		ARoadNetTileActor* Tile = *It;
		++Tiles;
		TestFalse(TEXT("a tile is saved with the level"), Tile->HasAnyFlags(RF_Transient));
		TestTrue(TEXT("a tile carries the generated tag"), Tile->ActorHasTag(ARoadNetTileActor::GeneratedTag));
#if WITH_EDITOR
		TestFalse(TEXT("a tile is not spatially loaded"), Tile->GetIsSpatiallyLoaded());
#endif
	}
	TestTrue(TEXT("the rebuild committed at least one tile"), Tiles > 0);

	GEngine->DestroyWorldContext(World);
	World->DestroyWorld(false);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
