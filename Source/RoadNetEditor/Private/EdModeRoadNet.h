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
	virtual void DrawHUD(FEditorViewportClient* ViewportClient, FViewport* Viewport, const FSceneView* View, FCanvas* Canvas) override;

	// Marquee box-select of control points (left-drag on empty space).
	virtual bool StartTracking(FEditorViewportClient* InViewportClient, FViewport* InViewport) override;
	virtual bool CapturedMouseMove(FEditorViewportClient* InViewportClient, FViewport* InViewport, int32 InMouseX, int32 InMouseY) override;

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

	// ---- multi-point selection helpers ------------------------------------
	// Is (Road,Point) currently in the multi-selection?
	bool IsPointSelected(int32 Road, int32 Point) const;
	// Replace the selection with a single point (plain click).
	void SelectSinglePoint(int32 Road, int32 Point);
	// Add the point if absent, remove it if present (Shift+click).
	void ToggleSelPoint(int32 Road, int32 Point);
	// Average world position of every selected point (transform-widget anchor).
	bool GetSelectionCentroid(FVector& OutPos) const;
	// Offset every selected point by Delta (widget drag). No rebuild (on release).
	void MoveSelectedPoints(const FVector& Delta);
	// Delete every selected point (safe order: roads + points descending so
	// index shifts and whole-road removals never invalidate pending deletes).
	// Returns the number of points removed.
	int32 DeleteSelectedPoints();
	// Select every visible control point whose screen position falls inside the
	// marquee box (bAdd keeps the current selection, else replaces it).
	void SelectPointsInMarquee(FEditorViewportClient* ViewportClient, bool bAdd);
	// Mark BOTH the actor and its network dirty for the open transaction so the
	// road source-of-truth (URoadNetwork::Roads) is captured by undo/redo.
	void ModifyForEdit();
	// Proximity pick under the cursor (used when the carriageway mesh occludes
	// the thin point/segment hit proxies). Selects the nearest editable control
	// point, else the nearest road centreline. bToggle (Shift) adds/removes the
	// picked point from the multi-selection. Returns true when it selected.
	bool TrySelectUnderCursor(FEditorViewportClient* ViewportClient, bool bToggle);
	// Which lane (left→right index, see URoadNetwork::GetLanesLeftToRight) of
	// RoadIdx the world point sits over, by nearest lane-centre offset line;
	// INDEX_NONE if none. Used to pick the lane to highlight/edit.
	int32 PickLaneAt(int32 RoadIdx, const FVector& WorldHit) const;

	TArray<FVector> DraftPoints;
	FVector HoverPoint = FVector::ZeroVector;
	bool bHasHover = false;
	bool bSnapActive = false;
	FVector SnapPoint = FVector::ZeroVector;

	// Edit selection (valid only when a draft is NOT in progress).
	//   * Whole-road selection: SelRoad set, SelPoint == INDEX_NONE, SelPoints empty.
	//   * Point selection: SelPoints holds every selected (Road,Point); SelRoad /
	//     SelPoint mirror the PRIMARY (first) point so lane/median/junction ops
	//     still have a road context and the road isn't drawn as whole-selected.
	int32 SelRoad = INDEX_NONE;
	int32 SelPoint = INDEX_NONE;
	// Selected lane (left→right index) of SelRoad for interactive lane editing;
	// INDEX_NONE when no lane is picked. Set when a road/lane is clicked, drawn
	// as a highlighted ribbon, and targeted by Shift+= / Shift+- (insert a lane
	// on that side) and B (cycle its type: driving → bicycle → parking).
	int32 SelLane = INDEX_NONE;
	bool bDirtyDuringDrag = false;

	// Multi-point selection: (RoadIndex, PointIndex) pairs. Move/delete act on all.
	TArray<FIntPoint> SelPoints;

	// Marquee box-select drag state (screen pixels).
	bool bMarquee = false;
	bool bMarqueeMoved = false;
	FIntPoint MarqueeStart = FIntPoint::ZeroValue;
	FIntPoint MarqueeCur   = FIntPoint::ZeroValue;

	// "Edit all points" toggle (hotkey P). When ON, EVERY road's control points
	// (imported OR hand-drawn) render as draggable handles and become
	// select/move/delete targets — not just hand-drawn roads. Off by default
	// because a city-scale OSM import has thousands of nodes; opt in to edit
	// imported geometry (edits persist until the next re-import of that road).
	bool bShowAllPoints = false;

	TWeakObjectPtr<ARoadNetActor> NetActorPtr;
};
