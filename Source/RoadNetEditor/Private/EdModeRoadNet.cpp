// EdModeRoadNet.cpp — click-to-draw road authoring mode (§9.3).
#include "EdModeRoadNet.h"
#include "RoadNetActor.h"
#include "RoadNetwork.h"
#include "RoadNetTypes.h"
#include "RoadNetLog.h"
#include "EditorViewportClient.h"
#include "SceneView.h"
#include "SceneManagement.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"
#include "ScopedTransaction.h"
#include "Editor.h"

#define LOCTEXT_NAMESPACE "RoadNetEditor"

IMPLEMENT_HIT_PROXY(HRoadNetPointProxy, HHitProxy);
IMPLEMENT_HIT_PROXY(HRoadNetSegmentProxy, HHitProxy);

namespace
{
	constexpr float  kPointSize  = 14.f;
	constexpr float  kSnapSize   = 22.f;
	constexpr double kSnapCm     = 600.0;   // world-space snap radius (slightly > weld radius)
	const FColor     kColorPoint = FColor(80, 200, 120);
	const FColor     kColorLine  = FColor(60, 170, 255);
	const FColor     kColorPreview = FColor(255, 200, 60);
	const FColor     kColorSnap  = FColor(255, 80, 80);
	const FColor     kColorEditPt = FColor(120, 180, 255);
	const FColor     kColorEditLine = FColor(90, 120, 160);
}

void FEdModeRoadNet::Enter()
{
	FEdMode::Enter();
	GetOrSpawnNetActor();
	DraftPoints.Reset();
	bHasHover = false;
	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] Draw mode entered. Left-click to place points, Enter/double-click to finish, Backspace to undo, Esc to cancel."));
}

void FEdModeRoadNet::Exit()
{
	DraftPoints.Reset();
	bHasHover = false;
	ClearSelection();
	NetActorPtr = nullptr;
	FEdMode::Exit();
}

URoadNetwork* FEdModeRoadNet::GetNetwork() const
{
	ARoadNetActor* Actor = NetActorPtr.Get();
	return Actor ? Actor->GetNetwork() : nullptr;
}

void FEdModeRoadNet::ClearSelection()
{
	SelRoad = INDEX_NONE;
	SelPoint = INDEX_NONE;
	bDirtyDuringDrag = false;
}

bool FEdModeRoadNet::GetSelectedPoint(FVector& OutPos) const
{
	if (SelRoad == INDEX_NONE || SelPoint == INDEX_NONE) { return false; }
	const URoadNetwork* Net = GetNetwork();
	if (!Net) { return false; }
	const TArray<FRoadDef>& Roads = Net->GetRoads();
	if (!Roads.IsValidIndex(SelRoad) || !Roads[SelRoad].Ref.IsValidIndex(SelPoint)) { return false; }
	OutPos = Roads[SelRoad].Ref[SelPoint];
	return true;
}

ARoadNetActor* FEdModeRoadNet::GetOrSpawnNetActor()
{
	if (NetActorPtr.IsValid()) { return NetActorPtr.Get(); }

	UWorld* World = GetWorld();
	if (!World) { return nullptr; }

	ARoadNetActor* Actor = Cast<ARoadNetActor>(UGameplayStatics::GetActorOfClass(World, ARoadNetActor::StaticClass()));
	if (!Actor)
	{
		Actor = World->SpawnActor<ARoadNetActor>();
#if WITH_EDITOR
		if (Actor) { Actor->SetActorLabel(TEXT("RoadNet")); }
#endif
	}
	NetActorPtr = Actor;
	return Actor;
}

bool FEdModeRoadNet::LineTraceCursor(FEditorViewportClient* ViewportClient, FVector& OutHit) const
{
	if (!ViewportClient) { return false; }
	FViewport* Viewport = ViewportClient->Viewport;

	FSceneViewFamilyContext ViewFamily(FSceneViewFamily::ConstructionValues(
		Viewport, ViewportClient->GetScene(), ViewportClient->EngineShowFlags).SetRealtimeUpdate(ViewportClient->IsRealtime()));
	FSceneView* View = ViewportClient->CalcSceneView(&ViewFamily);
	if (!View) { return false; }

	const FViewportCursorLocation Cursor(View, ViewportClient, Viewport->GetMouseX(), Viewport->GetMouseY());
	const FVector Origin = Cursor.GetOrigin();
	const FVector Dir    = Cursor.GetDirection();

	UWorld* World = GetWorld();
	if (!World) { return false; }

	FHitResult Hit;
	FCollisionQueryParams Params(SCENE_QUERY_STAT(RoadNetDraw), true);
	if (World->LineTraceSingleByChannel(Hit, Origin, Origin + Dir * 1.0e7, ECC_Visibility, Params))
	{
		OutHit = Hit.Location;
		return true;
	}

	// Fall back to the Z=0 ground plane so drawing works without collision.
	if (!FMath::IsNearlyZero(Dir.Z))
	{
		const double T = -Origin.Z / Dir.Z;
		if (T > 0) { OutHit = Origin + Dir * T; return true; }
	}
	return false;
}

bool FEdModeRoadNet::FindSnap(const FVector& Query, FVector& OutSnap) const
{
	const ARoadNetActor* Actor = NetActorPtr.Get();
	if (!Actor) { return false; }
	const URoadNetwork* Net = const_cast<ARoadNetActor*>(Actor)->GetNetwork();
	if (!Net) { return false; }

	const double R2 = kSnapCm * kSnapCm;
	double BestD2 = R2;
	bool bFound = false;

	auto Consider = [&](const FVector& P)
	{
		// Compare in 2D (plan view) so snapping is not fooled by terrain height.
		const double D2 = FVector2D::DistSquared(FVector2D(P.X, P.Y), FVector2D(Query.X, Query.Y));
		if (D2 < BestD2) { BestD2 = D2; OutSnap = P; bFound = true; }
	};

	// Existing road vertices (endpoints first — they form junctions; interior
	// vertices let a new road tee onto an existing centreline).
	for (const FRoadDef& Road : Net->GetRoads())
	{
		for (const FVector& P : Road.Ref) { Consider(P); }
	}
	// Draft's own earlier points (lets the user close a loop cleanly), except the
	// immediately previous point (snapping to it would make a zero-length segment).
	for (int32 i = 0; i + 1 < DraftPoints.Num(); ++i) { Consider(DraftPoints[i]); }

	return bFound;
}

FVector FEdModeRoadNet::ResolveCursorPoint(const FVector& RawHit)
{
	FVector Snapped;
	if (FindSnap(RawHit, Snapped))
	{
		bSnapActive = true;
		SnapPoint = Snapped;
		return Snapped;
	}
	bSnapActive = false;
	return RawHit;
}

bool FEdModeRoadNet::HandleClick(FEditorViewportClient* InViewportClient, HHitProxy* HitProxy, const FViewportClick& Click)
{
	if (Click.GetKey() == EKeys::LeftMouseButton)
	{
		if (Click.GetEvent() == IE_DoubleClick)
		{
			FinalizeDraft();
			return true;
		}
		// EDIT: while idle (no draft), clicks operate on existing hand-drawn roads.
		if (DraftPoints.Num() == 0)
		{
			// Ctrl+click a segment → insert a point there (mid-span split).
			if (Click.IsControlDown())
			{
				if (HRoadNetSegmentProxy* S = HitProxyCast<HRoadNetSegmentProxy>(HitProxy))
				{
					FVector Hit;
					if (LineTraceCursor(InViewportClient, Hit))
					{
						if (URoadNetwork* Net = GetNetwork())
						{
							const TArray<FRoadDef>& Roads = Net->GetRoads();
							FVector InsertPos = Hit;
							if (Roads.IsValidIndex(S->RoadIndex) &&
								Roads[S->RoadIndex].Ref.IsValidIndex(S->SegIndex + 1))
							{
								// Project the click onto the segment so the new point lands on the line.
								InsertPos = FMath::ClosestPointOnSegment(
									Hit, Roads[S->RoadIndex].Ref[S->SegIndex], Roads[S->RoadIndex].Ref[S->SegIndex + 1]);
							}
							const FScopedTransaction Transaction(LOCTEXT("RoadNetSplit", "Split RoadNet Road"));
							if (ARoadNetActor* Actor = NetActorPtr.Get()) { Actor->Modify(); }
							if (Net->InsertRoadPoint(S->RoadIndex, S->SegIndex, InsertPos))
							{
								SelRoad = S->RoadIndex;
								SelPoint = S->SegIndex + 1;
								Net->Rebuild();
							}
						}
					}
					if (InViewportClient) { InViewportClient->Invalidate(); }
					return true;
				}
			}
			// Plain click on a point proxy selects that point.
			if (HRoadNetPointProxy* P = HitProxyCast<HRoadNetPointProxy>(HitProxy))
			{
				SelRoad = P->RoadIndex;
				SelPoint = P->PointIndex;
				if (InViewportClient) { InViewportClient->Invalidate(); }
				return true;
			}
			// Plain click on a segment selects the whole road (SelPoint = NONE).
			if (HRoadNetSegmentProxy* S = HitProxyCast<HRoadNetSegmentProxy>(HitProxy))
			{
				SelRoad = S->RoadIndex;
				SelPoint = INDEX_NONE;
				if (InViewportClient) { InViewportClient->Invalidate(); }
				return true;
			}
		}
		// DRAW: any other click places a draft point (and clears any selection).
		ClearSelection();
		FVector Hit;
		if (LineTraceCursor(InViewportClient, Hit))
		{
			DraftPoints.Add(ResolveCursorPoint(Hit));
			if (InViewportClient) { InViewportClient->Invalidate(); }
		}
		return true;
	}
	if (Click.GetKey() == EKeys::RightMouseButton)
	{
		FinalizeDraft();
		return true;
	}
	return FEdMode::HandleClick(InViewportClient, HitProxy, Click);
}

bool FEdModeRoadNet::InputKey(FEditorViewportClient* ViewportClient, FViewport* Viewport, FKey Key, EInputEvent Event)
{
	if (Event == IE_Pressed)
	{
		if (Key == EKeys::Enter) { FinalizeDraft(); return true; }
		if (Key == EKeys::Escape)
		{
			DraftPoints.Reset();
			ClearSelection();
			if (ViewportClient) { ViewportClient->Invalidate(); }
			return true;
		}
		if (Key == EKeys::BackSpace || Key == EKeys::Delete)
		{
			// Drawing: drop the last placed draft point.
			if (DraftPoints.Num() > 0)
			{
				DraftPoints.Pop();
				if (ViewportClient) { ViewportClient->Invalidate(); }
				return true;
			}
			// Idle + point selection: delete the selected road point and rebuild.
			if (SelRoad != INDEX_NONE && SelPoint != INDEX_NONE)
			{
				if (URoadNetwork* Net = GetNetwork())
				{
					const FScopedTransaction Transaction(LOCTEXT("RoadNetDeletePoint", "Delete RoadNet Point"));
					if (ARoadNetActor* Actor = NetActorPtr.Get()) { Actor->Modify(); }
					bool bRoadRemoved = false;
					if (Net->DeleteRoadPoint(SelRoad, SelPoint, bRoadRemoved))
					{
						Net->Rebuild();
					}
				}
				ClearSelection();
				if (ViewportClient) { ViewportClient->Invalidate(); }
				return true;
			}
			// Idle + whole-road selection: delete the entire road and rebuild.
			if (SelRoad != INDEX_NONE && SelPoint == INDEX_NONE)
			{
				if (URoadNetwork* Net = GetNetwork())
				{
					const FScopedTransaction Transaction(LOCTEXT("RoadNetDeleteRoad", "Delete RoadNet Road"));
					if (ARoadNetActor* Actor = NetActorPtr.Get()) { Actor->Modify(); }
					if (Net->RemoveRoad(SelRoad)) { Net->Rebuild(); }
				}
				ClearSelection();
				if (ViewportClient) { ViewportClient->Invalidate(); }
				return true;
			}
		}
	}
	return FEdMode::InputKey(ViewportClient, Viewport, Key, Event);
}

bool FEdModeRoadNet::MouseMove(FEditorViewportClient* ViewportClient, FViewport* Viewport, int32 MouseX, int32 MouseY)
{
	FVector Hit;
	bHasHover = LineTraceCursor(ViewportClient, Hit);
	if (bHasHover)
	{
		HoverPoint = ResolveCursorPoint(Hit);
		if (ViewportClient) { ViewportClient->Invalidate(); }
	}
	else
	{
		bSnapActive = false;
	}
	return false;
}

void FEdModeRoadNet::Render(const FSceneView* View, FViewport* Viewport, FPrimitiveDrawInterface* PDI)
{
	FEdMode::Render(View, Viewport, PDI);

	// EDIT overlay: while idle, draw hand-drawn roads' points as clickable
	// handles (with hit proxies) plus faint centreline segments.
	if (DraftPoints.Num() == 0)
	{
		if (const URoadNetwork* Net = GetNetwork())
		{
			const TArray<FRoadDef>& Roads = Net->GetRoads();
			for (int32 r = 0; r < Roads.Num(); ++r)
			{
				const FRoadDef& Road = Roads[r];
				if (Road.Source != ERoadNetSource::HandDrawn || Road.Ref.Num() < 2) { continue; }
				const bool bWholeRoadSel = (r == SelRoad && SelPoint == INDEX_NONE);
				for (int32 i = 0; i + 1 < Road.Ref.Num(); ++i)
				{
					PDI->SetHitProxy(new HRoadNetSegmentProxy(r, i));
					PDI->DrawLine(Road.Ref[i], Road.Ref[i + 1],
						bWholeRoadSel ? kColorSnap : kColorEditLine, SDPG_World, bWholeRoadSel ? 3.f : 1.5f);
					PDI->SetHitProxy(nullptr);
				}
				for (int32 i = 0; i < Road.Ref.Num(); ++i)
				{
					const bool bSel = (r == SelRoad && i == SelPoint) || bWholeRoadSel;
					PDI->SetHitProxy(new HRoadNetPointProxy(r, i));
					PDI->DrawPoint(Road.Ref[i], bSel ? kColorSnap : kColorEditPt, kPointSize, SDPG_Foreground);
					PDI->SetHitProxy(nullptr);
				}
			}
		}
	}

	for (int32 i = 0; i < DraftPoints.Num(); ++i)
	{
		PDI->DrawPoint(DraftPoints[i], kColorPoint, kPointSize, SDPG_Foreground);
		if (i > 0)
		{
			PDI->DrawLine(DraftPoints[i - 1], DraftPoints[i], kColorLine, SDPG_Foreground, 3.f);
		}
	}
	if (DraftPoints.Num() > 0 && bHasHover)
	{
		PDI->DrawLine(DraftPoints.Last(), HoverPoint, kColorPreview, SDPG_Foreground, 2.f);
	}
	// Highlight the active snap target so the user sees where the point will weld.
	if (bHasHover && bSnapActive)
	{
		PDI->DrawPoint(SnapPoint, kColorSnap, kSnapSize, SDPG_Foreground);
	}
}

bool FEdModeRoadNet::ShouldDrawWidget() const
{
	FVector Ignored;
	return DraftPoints.Num() == 0 && GetSelectedPoint(Ignored);
}

FVector FEdModeRoadNet::GetWidgetLocation() const
{
	FVector Pos;
	if (GetSelectedPoint(Pos)) { return Pos; }
	return FEdMode::GetWidgetLocation();
}

EAxisList::Type FEdModeRoadNet::GetWidgetAxisToDraw(UE::Widget::EWidgetMode InWidgetMode) const
{
	if (InWidgetMode == UE::Widget::WM_Translate) { return EAxisList::XYZ; }
	return EAxisList::None;
}

bool FEdModeRoadNet::InputDelta(FEditorViewportClient* InViewportClient, FViewport* InViewport, FVector& InDrag, FRotator& InRot, FVector& InScale)
{
	if (InViewportClient && InViewportClient->GetCurrentWidgetAxis() != EAxisList::None)
	{
		FVector Pos;
		if (GetSelectedPoint(Pos))
		{
			if (URoadNetwork* Net = GetNetwork())
			{
				// Open a drag-spanning transaction on the first delta and capture
				// the pre-move state, so a single Undo reverts the whole drag.
				if (!bDirtyDuringDrag && GEditor)
				{
					GEditor->BeginTransaction(LOCTEXT("RoadNetMovePoint", "Move RoadNet Point"));
					if (ARoadNetActor* Actor = NetActorPtr.Get()) { Actor->Modify(); }
					bDirtyDuringDrag = true;
				}
				Net->MoveRoadPoint(SelRoad, SelPoint, Pos + InDrag);   // rebuild on release
			}
			return true;
		}
	}
	return false;
}

bool FEdModeRoadNet::EndTracking(FEditorViewportClient* InViewportClient, FViewport* InViewport)
{
	if (bDirtyDuringDrag)
	{
		bDirtyDuringDrag = false;
		if (URoadNetwork* Net = GetNetwork()) { Net->Rebuild(); }
		if (GEditor) { GEditor->EndTransaction(); }
		return true;
	}
	return FEdMode::EndTracking(InViewportClient, InViewport);
}

void FEdModeRoadNet::FinalizeDraft()
{
	if (DraftPoints.Num() < 2)
	{
		DraftPoints.Reset();
		return;
	}

	ARoadNetActor* Actor = GetOrSpawnNetActor();
	if (!Actor) { DraftPoints.Reset(); return; }

	URoadNetwork* Net = Actor->GetNetwork();
	if (!Net) { DraftPoints.Reset(); return; }

	const FScopedTransaction Transaction(LOCTEXT("RoadNetDrawRoad", "Draw RoadNet Road"));
	Actor->Modify();

	FRoadDef R;
	R.Source = ERoadNetSource::HandDrawn;
	R.Class  = Actor->DraftClass;
	R.Ref    = DraftPoints;

	FRoadNetLaneSpec& L = R.Lanes;
	L.bOneway          = Actor->bDraftOneway;
	L.Total            = FMath::Max(1, Actor->DraftLaneCount);
	L.LaneWidthDefault = FMath::Max(150.f, Actor->DraftLaneWidthCm);
	L.bSidewalkLeft    = Actor->bDraftSidewalks;
	L.bSidewalkRight   = Actor->bDraftSidewalks;

	Net->AddRoad(R);
	Net->Rebuild();

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] Draw: committed a hand-drawn road with %d points."), DraftPoints.Num());
	DraftPoints.Reset();
}

#undef LOCTEXT_NAMESPACE
