// Editor lock for the lane graph (plan Phase 0a). Each fixture is committed to a
// transient world in all four driving-side x channelization variants. Everything
// the rebuild commits is written out as text: stops, widenings, areas, ZoneGraph
// runs, the lane-graph dump, and every tile component (mesh layer bounds, HISM
// instances, splines per tag, decals). The text must survive a second rebuild
// unchanged, and must equal the checked-in baseline beside this file. A missing
// baseline is written and reported; delete it to re-record after a declared diff.
#include "Misc/AutomationTest.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "RoadNetwork.h"
#include "RoadNetMath.h"
#include "RoadNetTileActor.h"
#include "Engine/World.h"
#include "Components/SplineComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/DecalComponent.h"
#include "EngineUtils.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	FRoadDef LockRoad(TArray<FVector> Ref, TArray<int64> Ids, int32 Total, bool bOneway)
	{
		FRoadDef R;
		R.Ref = MoveTemp(Ref);
		R.NodeIds = MoveTemp(Ids);
		R.Lanes.Total = Total;
		R.Lanes.bOneway = bOneway;
		return R;
	}

	FString Cm(double V) { return FString::Printf(TEXT("%.0f"), V); }
	FString Cm(const FVector& V) { return Cm(V.X) + TEXT(",") + Cm(V.Y) + TEXT(",") + Cm(V.Z); }

	FString Snapshot(URoadNetwork* Net, UWorld* World)
	{
		TArray<FString> Lines;
		for (const FVector2D& S : Net->LastApproachStops)
		{
			Lines.Add(TEXT("stop ") + Cm(S.X) + TEXT(",") + Cm(S.Y));
		}
		for (const FRoadNetArmWidening& W : Net->LastArmWidenings)
		{
			Lines.Add(FString::Printf(TEXT("widen road=%d start=%d side=%d kind=%d lanes=%d w=%s taper=%s"),
				W.Road, W.bAtStart ? 1 : 0, (int32)W.Side, (int32)W.Kind, W.Lanes, *Cm(W.WidthCm), *Cm(W.TaperCm)));
		}
		Lines.Add(FString::Printf(TEXT("area surface=%.0f sidewalk=%.0f"),
			Net->LastSurfaceAreaCm2 / 100.0, Net->LastSidewalkAreaCm2 / 100.0));
		Lines.Add(FString::Printf(TEXT("reductions %d"), Net->LaneGraph.Reductions));
		for (const FRoadNetZoneRun& Z : Net->ZoneGraphRuns)
		{
			int32 Road = INDEX_NONE;
			for (int32 i = 0; i < Net->GetRoads().Num(); ++i) { if (Net->GetRoads()[i].Id == Z.RoadId) { Road = i; } }
			Lines.Add(FString::Printf(TEXT("zone road=%d f=%d b=%d w=%s pts=%d %s %s"), Road, Z.LanesForward,
				Z.LanesBackward, *Cm(Z.LaneWidthCm), Z.Points.Num(),
				Z.Points.Num() ? *Cm(Z.Points[0]) : TEXT("-"), Z.Points.Num() ? *Cm(Z.Points.Last()) : TEXT("-")));
		}
		for (TActorIterator<ARoadNetTileActor> It(World); It; ++It)
		{
			const FString Tile = FString::Printf(TEXT("tile %d,%d"), It->TileCoord.X, It->TileCoord.Y);
			TArray<UActorComponent*> Comps;
			It->GetComponents(Comps);
			for (const UActorComponent* C : Comps)
			{
				if (const USplineComponent* Sp = Cast<USplineComponent>(C))
				{
					TArray<FString> Tags;
					for (const FName& T : Sp->ComponentTags) { Tags.Add(T.ToString()); }
					Tags.Sort();
					const int32 N = Sp->GetNumberOfSplinePoints();
					Lines.Add(FString::Printf(TEXT("%s spline [%s] pts=%d %s %s"), *Tile, *FString::Join(Tags, TEXT(",")), N,
						N ? *Cm(Sp->GetLocationAtSplinePoint(0, ESplineCoordinateSpace::World)) : TEXT("-"),
						N ? *Cm(Sp->GetLocationAtSplinePoint(N - 1, ESplineCoordinateSpace::World)) : TEXT("-")));
				}
				else if (const UInstancedStaticMeshComponent* H = Cast<UInstancedStaticMeshComponent>(C))
				{
					for (int32 i = 0; i < H->GetInstanceCount(); ++i)
					{
						FTransform T;
						H->GetInstanceTransform(i, T, /*bWorldSpace=*/true);
						Lines.Add(FString::Printf(TEXT("%s hism %s %s yaw=%.0f"), *Tile, *H->GetName(),
							*Cm(T.GetLocation()), T.Rotator().Yaw));
					}
				}
				else if (const UDecalComponent* D = Cast<UDecalComponent>(C))
				{
					Lines.Add(FString::Printf(TEXT("%s decal %s yaw=%.0f size=%s"), *Tile,
						*Cm(D->GetComponentLocation()), D->GetComponentRotation().Yaw, *Cm(D->DecalSize)));
				}
				else if (const UPrimitiveComponent* P = Cast<UPrimitiveComponent>(C))
				{
					const FBox B = P->Bounds.GetBox();
					if (B.IsValid)
					{
						Lines.Add(FString::Printf(TEXT("%s mesh %s %s %s"), *Tile, *P->GetName(), *Cm(B.Min), *Cm(B.Max)));
					}
				}
			}
		}
		Lines.Sort();
		return FString::Join(Lines, TEXT("\n")) + TEXT("\n") + Net->DumpLaneGraph();
	}

	struct FLockFixture
	{
		const TCHAR* Name;
		TFunction<void(URoadNetwork*)> Build;
	};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetLaneLockTest, "RoadNet.Lanes.SplineLock",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRoadNetLaneLockTest::RunTest(const FString&)
{
	const TArray<FLockFixture> Fixtures = {
		{ TEXT("seam"), [](URoadNetwork* N)
		{
			N->AddRoad(LockRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 2, false));
			N->AddRoad(LockRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 2, false));
		} },
		{ TEXT("seam 60"), [](URoadNetwork* N)
		{
			const double B = FMath::DegreesToRadians(60.0);
			N->AddRoad(LockRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 2, false));
			N->AddRoad(LockRoad({ {10000, 0, 0}, {10000 + 10000 * FMath::Cos(B), 10000 * FMath::Sin(B), 0} }, { 10, 2 }, 2, false));
		} },
		{ TEXT("seam 3 into 2"), [](URoadNetwork* N)
		{
			N->AddRoad(LockRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 3, true));
			N->AddRoad(LockRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 2, true));
		} },
		{ TEXT("cross 2+2"), [](URoadNetwork* N)
		{
			N->AddRoad(LockRoad({ {-10000, 0, 0}, {0, 0, 0} }, { 1, 10 }, 2, false));
			N->AddRoad(LockRoad({ {0, 0, 0}, {10000, 0, 0} }, { 10, 2 }, 2, false));
			N->AddRoad(LockRoad({ {0, 10000, 0}, {0, 0, 0} }, { 3, 10 }, 2, false));
			N->AddRoad(LockRoad({ {0, 0, 0}, {0, -10000, 0} }, { 10, 4 }, 2, false));
		} },
		{ TEXT("T 3-lane stem"), [](URoadNetwork* N)
		{
			N->AddRoad(LockRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 2, false));
			N->AddRoad(LockRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 2, false));
			N->AddRoad(LockRoad({ {10000, 8000, 0}, {10000, 0, 0} }, { 3, 10 }, 3, true));
		} },
		{ TEXT("T 3+3 main"), [](URoadNetwork* N)
		{
			N->AddRoad(LockRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 6, false));
			N->AddRoad(LockRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 6, false));
			N->AddRoad(LockRoad({ {10000, 8000, 0}, {10000, 0, 0} }, { 3, 10 }, 2, false));
		} },
		{ TEXT("Y 2 into 1+1"), [](URoadNetwork* N)
		{
			const double B = FMath::DegreesToRadians(20.0);
			N->AddRoad(LockRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 2, true));
			N->AddRoad(LockRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 1, true));
			N->AddRoad(LockRoad({ {10000, 0, 0}, {10000 + 10000 * FMath::Cos(B), 10000 * FMath::Sin(B), 0} }, { 10, 3 }, 1, true));
		} },
		{ TEXT("Y 1 into 1+1"), [](URoadNetwork* N)
		{
			const double B = FMath::DegreesToRadians(20.0);
			N->AddRoad(LockRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 1, true));
			N->AddRoad(LockRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 1, true));
			N->AddRoad(LockRoad({ {10000, 0, 0}, {10000 + 10000 * FMath::Cos(B), 10000 * FMath::Sin(B), 0} }, { 10, 3 }, 1, true));
		} },
		{ TEXT("Y 3 into 2+1"), [](URoadNetwork* N)
		{
			const double B = FMath::DegreesToRadians(20.0);
			N->AddRoad(LockRoad({ {0, 0, 0}, {10000, 0, 0} }, { 1, 10 }, 3, true));
			N->AddRoad(LockRoad({ {10000, 0, 0}, {20000, 0, 0} }, { 10, 2 }, 1, true));
			N->AddRoad(LockRoad({ {10000, 0, 0}, {10000 + 10000 * FMath::Cos(B), 10000 * FMath::Sin(B), 0} }, { 10, 3 }, 2, true));
		} },
		{ TEXT("centre turn"), [](URoadNetwork* N)
		{
			N->AddRoad(LockRoad({ {-10000, 0, 0}, {0, 0, 0} }, { 1, 10 }, 3, false));
			N->AddRoad(LockRoad({ {0, 0, 0}, {10000, 0, 0} }, { 10, 2 }, 2, false));
			N->AddRoad(LockRoad({ {0, 10000, 0}, {0, 0, 0} }, { 5, 10 }, 2, false));
			N->AddRoad(LockRoad({ {0, -10000, 0}, {0, 0, 0} }, { 4, 10 }, 2, false));
			N->SetLaneType(0, 1, ERoadNetLaneType::CenterTurn);
		} },
		{ TEXT("tertiary bay"), [](URoadNetwork* N)
		{
			FRoadDef Stem = LockRoad({ {0, 8000, 0}, {0, 0, 0} }, { 3, 10 }, 1, true);
			Stem.Class = ERoadNetClass::Tertiary;
			N->AddRoad(LockRoad({ {-10000, 0, 0}, {0, 0, 0} }, { 1, 10 }, 2, false));
			N->AddRoad(LockRoad({ {0, 0, 0}, {10000, 0, 0} }, { 10, 2 }, 2, false));
			N->AddRoad(Stem);
		} },
		{ TEXT("dual carriageway"), [](URoadNetwork* N)
		{
			N->AddRoad(LockRoad({ {0, 500, 0}, {20000, 500, 0} }, { 1, 2 }, 2, true));
			N->AddRoad(LockRoad({ {20000, -500, 0}, {0, -500, 0} }, { 3, 4 }, 2, true));
			N->AddRoad(LockRoad({ {10000, 10000, 0}, {10000, -10000, 0} }, { 5, 6 }, 2, false));
			N->SplitRoadsAtCrossings();
		} },
		{ TEXT("roundabout"), [](URoadNetwork* N)
		{
			TArray<FVector> Ring;
			RoadNetMath::SampleCircle(FVector2D::ZeroVector, 2000.0, 0.0, 32, Ring);
			N->AddRoad(LockRoad(Ring, {}, 2, true));
			N->AddRoad(LockRoad({ {1000, 6000, 0}, {1000, 2200, 0} }, {}, 2, true));
			N->CleanRoundabout(TArray<int32>{ 0 });
			N->SplitRoadsAtCrossings();
		} },
	};

	FString All;
	for (const FLockFixture& F : Fixtures)
	{
		for (int32 Variant = 0; Variant < 4; ++Variant)
		{
			const bool bLeft = (Variant & 1) != 0;
			const bool bChannel = (Variant & 2) == 0;
			const FString What = FString::Printf(TEXT("%s%s%s"), F.Name,
				bLeft ? TEXT(" left") : TEXT(""), bChannel ? TEXT("") : TEXT(" no-channel"));

			// 52 worlds are built and torn down inside one frame; render scenes for them
			// preceded both DXGI device-removed crashes, and the snapshot reads no render state.
			const FWorldInitializationValues NoScenes = FWorldInitializationValues().InitializeScenes(false)
				.CreatePhysicsScene(false).CreateFXSystem(false).RequiresHitProxies(false).AllowAudioPlayback(false)
				.CreateNavigation(false).CreateAISystem(false).SetTransactional(false);
			UWorld* World = UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld=*/false, NAME_None,
				nullptr, /*bAddToRoot=*/true, ERHIFeatureLevel::Num, &NoScenes);
			if (!TestNotNull(TEXT("a test world could be created"), World)) { return false; }
			URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
			Net->SetWorld(World);
			Net->SetDriveOnLeft(bLeft);
			Net->bChannelizeJunctions = bChannel;
			F.Build(Net);
			Net->Rebuild();
			const FString First = Snapshot(Net, World);
			Net->Rebuild();
			TestEqual(FString::Printf(TEXT("%s: a second rebuild commits the same state"), *What), Snapshot(Net, World), First);
			for (const FString& R : Net->LaneGraph.Reports)
			{
				const bool bConnectivity = R.StartsWith(TEXT("I1:")) || R.StartsWith(TEXT("I2:"))
					|| R.StartsWith(TEXT("I3:")) || R.StartsWith(TEXT("I4:")) || R.StartsWith(TEXT("I5:"));
				if (bConnectivity) { AddError(FString::Printf(TEXT("%s: %s"), *What, *R)); }
			}
			All += FString::Printf(TEXT("## %s\n"), *What) + First + TEXT("\n");
			World->DestroyWorld(/*bInformEngineOfWorld=*/false);
		}
	}

	const FString Baseline = FPaths::Combine(FPaths::GetPath(FString(ANSI_TO_TCHAR(__FILE__))), TEXT("LaneLock.baseline.txt"));
	FString Recorded;
	if (!FFileHelper::LoadFileToString(Recorded, *Baseline))
	{
		if (HasAnyErrors())
		{
			AddWarning(TEXT("Lane lock baseline not recorded: fix the connectivity errors first"));
			return false;
		}
		TestTrue(TEXT("the lock baseline was written"), FFileHelper::SaveStringToFile(All, *Baseline));
		AddWarning(FString::Printf(TEXT("Lane lock baseline recorded at %s"), *Baseline));
		return true;
	}
	Recorded.ReplaceInline(TEXT("\r\n"), TEXT("\n"));
	if (Recorded != All)
	{
		TArray<FString> A, B;
		Recorded.ParseIntoArray(A, TEXT("\n"), false);
		All.ParseIntoArray(B, TEXT("\n"), false);
		FString Section;
		int32 Shown = 0;
		for (int32 i = 0; i < FMath::Max(A.Num(), B.Num()) && Shown < 20; ++i)
		{
			const FString L = A.IsValidIndex(i) ? A[i] : FString();
			const FString R = B.IsValidIndex(i) ? B[i] : FString();
			if (R.StartsWith(TEXT("## "))) { Section = R; }
			if (L != R)
			{
				AddError(FString::Printf(TEXT("lock diff in %s, line %d: baseline '%s' now '%s'"), *Section, i + 1, *L, *R));
				++Shown;
			}
		}
	}
	return true;
}

#endif
