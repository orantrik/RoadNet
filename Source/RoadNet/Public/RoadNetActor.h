#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RoadNetTypes.h"
#include "RoadNetActor.generated.h"

class URoadNetwork;
class USplineComponent;

// ===========================================================================
// ARoadNetActor — the level-persistent home of a URoadNetwork (§9.1).
//
// This is the convergence point for BOTH authoring modes:
//   * OSM import calls GetNetwork(), refreshes Source==OSM roads, and rebuilds.
//   * Hand-draw adds USplineComponent drafts; RebuildFromDrafts() turns them
//     into Source==HandDrawn roads and rebuilds.
// Because the network (and its Roads array) is a serialized UPROPERTY on this
// actor, hand-drawn roads survive OSM re-imports (§9.4).
// ===========================================================================
UCLASS(BlueprintType, hidecategories=(Input, Replication, Collision, LOD, Cooking))
class ROADNET_API ARoadNetActor : public AActor
{
	GENERATED_BODY()

public:
	ARoadNetActor();

	// The persistent road network (source-of-truth roads + rebuild pipeline).
	// EditAnywhere so its inline material overrides are editable in the details panel.
	UPROPERTY(EditAnywhere, Instanced, Category = "RoadNet")
	TObjectPtr<URoadNetwork> Network;

	// ---- hand-draw draft defaults (applied to newly built draft roads) -----
	UPROPERTY(EditAnywhere, Category = "RoadNet|HandDraw", meta = (ClampMin = "1"))
	int32 DraftLaneCount = 2;

	UPROPERTY(EditAnywhere, Category = "RoadNet|HandDraw")
	bool bDraftOneway = false;

	UPROPERTY(EditAnywhere, Category = "RoadNet|HandDraw")
	bool bDraftSidewalks = true;

	// Sidewalk width (cm) applied to newly hand-drawn roads when bDraftSidewalks.
	UPROPERTY(EditAnywhere, Category = "RoadNet|HandDraw",
		meta = (ClampMin = "50.0", UIMin = "100.0", UIMax = "600.0"))
	float DraftSidewalkWidthCm = 200.f;

	UPROPERTY(EditAnywhere, Category = "RoadNet|HandDraw", meta = (ClampMin = "150.0"))
	float DraftLaneWidthCm = 350.f;

	UPROPERTY(EditAnywhere, Category = "RoadNet|HandDraw")
	ERoadNetClass DraftClass = ERoadNetClass::Residential;

	// Spacing used to sample draft splines into a reference polyline (cm).
	UPROPERTY(EditAnywhere, Category = "RoadNet|HandDraw", meta = (ClampMin = "50.0"))
	float DraftSampleSpacingCm = 200.f;

	// Draft splines the user edits directly in the viewport.
	UPROPERTY(VisibleAnywhere, Category = "RoadNet|HandDraw")
	TArray<TObjectPtr<USplineComponent>> DraftSplines;

	// Add a new editable draft spline (2 default points) to hand-draw a road.
	UFUNCTION(CallInEditor, Category = "RoadNet|HandDraw")
	void AddDraftSpline();

	// Convert all draft splines into HandDrawn roads and rebuild the network.
	UFUNCTION(CallInEditor, Category = "RoadNet|HandDraw")
	void RebuildFromDrafts();

	// Remove all hand-drawn roads (keeps OSM) and rebuild.
	UFUNCTION(CallInEditor, Category = "RoadNet|HandDraw")
	void ClearHandDrawn();

	// Get (lazily create) the owned network, bound to this actor's world.
	URoadNetwork* GetNetwork();
};
