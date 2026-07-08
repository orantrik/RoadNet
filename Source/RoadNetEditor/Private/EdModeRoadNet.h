#pragma once
#include "CoreMinimal.h"
#include "EdMode.h"
#include "EditorModeManager.h"
#include "HitProxies.h"

// ===========================================================================
// FEdModeRoadNet — click-to-draw + edit road authoring (§9.3).
//
// DRAW (a draft is in progress): left-click drops reference points onto the
// world (line-traced to terrain, snapped to nearby geometry); Enter /
// double-click / right-click finalize them into a Source==HandDrawn road on
// the level's ARoadNetActor; Backspace removes the last point; Escape cancels.
//
// EDIT (idle — no draft): left-click on an existing hand-drawn road point
// selects it; the transform widget moves it (rebuild on release); Delete
// removes it; Insert/double-click on a segment... (split is a later add).
// The mode never selects level actors.
// ===========================================================================
class ARoadNetActor;

// Hit proxy for one editable reference point of a hand-drawn road.
struct HRoadNetPointProxy : public HHitProxy
{
	DECLARE_HIT_PROXY();
	HRoadNetPointProxy(int32 InRoad, int32 InPoint)
		: HHitProxy(HPP_UI), RoadIndex(InRoad), PointIndex(InPoint) {}
	virtual EMouseCursor::Type GetMouseCursor() override { return EMouseCursor::Crosshairs; }
	int32 RoadIndex;
	int32 PointIndex;
};

// Hit proxy for one segment (Ref[i]..Ref[i+1]) of a hand-drawn road — used by
// the Ctrl+click mid-span split gesture. Lower priority than the point proxy so
// clicking exactly on a vertex still selects the vertex.
struct HRoadNetSegmentProxy : public HHitProxy
{
	DECLARE_HIT_PROXY();
	HRoadNetSegmentProxy(int32 InRoad, int32 InSeg)
		: HHitProxy(HPP_Wireframe), RoadIndex(InRoad), SegIndex(InSeg) {}
	virtual EMouseCursor::Type GetMouseCursor() override { return EMouseCursor::CardinalCross; }
	int32 RoadIndex;
	int32 SegIndex;
};

class FEdModeRoadNet : public FEdMode
{
public:
	static FEditorModeID GetModeID() { return FEditorModeID(TEXT("EM_RoadNet")); }

	virtual void Enter() override;
	virtual void Exit() override;

	virtual bool HandleClick(FEditorViewportClient* InViewportClient, HHitProxy* HitProxy, const FViewportClick& Click) override;
	virtual bool InputKey(FEditorViewportClient* ViewportClient, FViewport* Viewport, FKey Key, EInputEvent Event) override;
	virtual bool MouseMove(FEditorViewportClient* ViewportClient, FViewport* Viewport, int32 MouseX, int32 MouseY) override;
	virtual void Render(const FSceneView* View, FViewport* Viewport, FPrimitiveDrawInterface* PDI) override;

	// Transform-widget editing of the selected point.
	virtual bool ShouldDrawWidget() const override;
	virtual FVector GetWidgetLocation() const override;
	virtual bool UsesTransformWidget() const override { return true; }
	virtual EAxisList::Type GetWidgetAxisToDraw(UE::Widget::EWidgetMode InWidgetMode) const override;
	virtual bool InputDelta(FEditorViewportClient* InViewportClient, FViewport* InViewport, FVector& InDrag, FRotator& InRot, FVector& InScale) override;
	virtual bool EndTracking(FEditorViewportClient* InViewportClient, FViewport* InViewport) override;

	virtual bool IsSelectionAllowed(AActor* InActor, bool bInSelection) const override { return false; }
	virtual bool ShowModeWidgets() const override { return false; }
	virtual bool UsesToolkits() const override { return false; }

private:
	ARoadNetActor* GetOrSpawnNetActor();
	class URoadNetwork* GetNetwork() const;
	bool LineTraceCursor(FEditorViewportClient* ViewportClient, FVector& OutHit) const;
	// Snap Query to the nearest existing road vertex / draft point within radius.
	// Returns true and writes OutSnap when a candidate is found.
	bool FindSnap(const FVector& Query, FVector& OutSnap) const;
	// Cursor point with snapping applied (updates bSnapActive / SnapPoint).
	FVector ResolveCursorPoint(const FVector& RawHit);
	void FinalizeDraft();
	bool GetSelectedPoint(FVector& OutPos) const;
	void ClearSelection();

	TArray<FVector> DraftPoints;
	FVector HoverPoint = FVector::ZeroVector;
	bool bHasHover = false;
	bool bSnapActive = false;
	FVector SnapPoint = FVector::ZeroVector;

	// Edit selection (valid only when a draft is NOT in progress).
	int32 SelRoad = INDEX_NONE;
	int32 SelPoint = INDEX_NONE;
	bool bDirtyDuringDrag = false;

	TWeakObjectPtr<ARoadNetActor> NetActorPtr;
};
