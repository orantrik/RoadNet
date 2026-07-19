// RoadNetActor.cpp — level-persistent home of a URoadNetwork (§9.1).
#include "RoadNetActor.h"
#include "RoadNetwork.h"
#include "RoadNetLog.h"
#include "Components/SplineComponent.h"

ARoadNetActor::ARoadNetActor()
{
	PrimaryActorTick.bCanEverTick = false;

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

	Network = CreateDefaultSubobject<URoadNetwork>(TEXT("RoadNetwork"));
	// The road source-of-truth (Roads array) lives on the Network sub-object, so
	// it must be transactional for interactive edits to be undoable (Ctrl+Z).
	if (Network) { Network->SetFlags(RF_Transactional); }
}

URoadNetwork* ARoadNetActor::GetNetwork()
{
	if (!Network)
	{
		Network = NewObject<URoadNetwork>(this, TEXT("RoadNetwork"), RF_Transactional);
	}
	// Ensure the network always participates in the transaction buffer (older
	// saved actors may have a non-transactional sub-object) so draw/move/delete/
	// lane/median/junction edits can be undone and redone.
	Network->SetFlags(RF_Transactional);
	Network->SetWorld(GetWorld());
	return Network;
}

void ARoadNetActor::AddDraftSpline()
{
	Modify();   // capture the DraftSplines change for undo

	USplineComponent* Spline = NewObject<USplineComponent>(this, NAME_None, RF_Transactional);
	if (!Spline) { return; }

	Spline->SetupAttachment(GetRootComponent());
	Spline->SetMobility(EComponentMobility::Movable);
	Spline->RegisterComponent();

	// Two starter points offset from the actor so they're grabbable in-viewport.
	Spline->ClearSplinePoints(false);
	const FVector Base = GetActorLocation();
	Spline->AddSplinePoint(Base + FVector(0, 0, 0),      ESplineCoordinateSpace::World, false);
	Spline->AddSplinePoint(Base + FVector(2000, 0, 0),   ESplineCoordinateSpace::World, false);
	Spline->UpdateSpline();

	AddInstanceComponent(Spline);
	DraftSplines.Add(Spline);

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] Added draft spline #%d — edit its points, then RebuildFromDrafts."), DraftSplines.Num());
}

void ARoadNetActor::RebuildFromDrafts()
{
	URoadNetwork* Net = GetNetwork();
	if (!Net) { return; }

	Modify();
	Net->Modify();

	// Refresh only hand-drawn roads; OSM-imported roads are left untouched.
	Net->RemoveRoadsBySource(ERoadNetSource::HandDrawn);

	const double Spacing = FMath::Max(50.0, (double)DraftSampleSpacingCm);
	int32 Built = 0;

	for (const TObjectPtr<USplineComponent>& SplinePtr : DraftSplines)
	{
		USplineComponent* Spline = SplinePtr.Get();
		if (!Spline) { continue; }
		const double Length = Spline->GetSplineLength();
		if (Length < Spacing) { continue; }

		FRoadDef R;
		R.Source = ERoadNetSource::HandDrawn;
		R.Class  = DraftClass;

		for (double S = 0.0; S <= Length; S += Spacing)
		{
			R.Ref.Add(Spline->GetLocationAtDistanceAlongSpline(S, ESplineCoordinateSpace::World));
		}
		// Guarantee the exact endpoint is present.
		R.Ref.Add(Spline->GetLocationAtDistanceAlongSpline(Length, ESplineCoordinateSpace::World));
		if (R.Ref.Num() < 2) { continue; }

		FRoadNetLaneSpec& L = R.Lanes;
		L.bOneway          = bDraftOneway;
		L.Total            = FMath::Max(1, DraftLaneCount);
		L.LaneWidthDefault = FMath::Max(150.f, DraftLaneWidthCm);
		L.bSidewalkLeft    = bDraftSidewalks;
		L.bSidewalkRight   = bDraftSidewalks;
		L.SidewalkWidth    = FMath::Max(50.f, DraftSidewalkWidthCm);

		Net->AddRoad(R);
		++Built;
	}

	Net->Rebuild();
	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] RebuildFromDrafts: built %d hand-drawn road(s) from %d draft spline(s)."),
		Built, DraftSplines.Num());
}

void ARoadNetActor::ClearHandDrawn()
{
	URoadNetwork* Net = GetNetwork();
	if (!Net) { return; }
	Modify();
	Net->Modify();
	const int32 Removed = Net->RemoveRoadsBySource(ERoadNetSource::HandDrawn);
	Net->Rebuild();
	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] ClearHandDrawn: removed %d hand-drawn road(s)."), Removed);
}
