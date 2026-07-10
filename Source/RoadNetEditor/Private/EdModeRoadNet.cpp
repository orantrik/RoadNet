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
	const FColor     kColorOsmLine = FColor(230, 150, 60);   // imported roads (selectable for lane edits)

	const TCHAR* PresetName(ERoadNetJunctionPreset P)
	{
		switch (P)
		{
		case ERoadNetJunctionPreset::StopLine:         return TEXT("Stop line");
		case ERoadNetJunctionPreset::StopAndCrosswalk: return TEXT("Stop + crosswalk");
		case ERoadNetJunctionPreset::Signalized:       return TEXT("Signalized (stop + crosswalk + lights)");
		case ERoadNetJunctionPreset::GiveWay:          return TEXT("Give way");
		default:                                       return TEXT("None");
		}
	}

	FColor PresetColor(ERoadNetJunctionPreset P)
	{
		switch (P)
		{
		case ERoadNetJunctionPreset::StopLine:         return FColor(255, 90, 90);
		case ERoadNetJunctionPreset::StopAndCrosswalk: return FColor(255, 160, 60);
		case ERoadNetJunctionPreset::Signalized:       return FColor(80, 220, 120);
		case ERoadNetJunctionPreset::GiveWay:          return FColor(255, 230, 60);
		default:                                       return FColor(150, 150, 160); // None
		}
	}
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
			// Idle delete. Resolve a target from the explicit selection, else
			// hover-pick under the cursor (mirrors the lane hotkeys so no prior
			// click is needed — clicks were easy to lose to draft placement).
			//   * nearest HAND-DRAWN control point within a tight radius → delete
			//     just that point (point handles only exist for hand-drawn roads);
			//   * else nearest road (any source) within a wider radius → delete
			//     the whole road.
			int32 DelRoad  = SelRoad;
			int32 DelPoint = SelPoint;
			if (DelRoad == INDEX_NONE && ViewportClient)
			{
				FVector Hit;
				if (LineTraceCursor(ViewportClient, Hit))
				{
					if (URoadNetwork* Net = GetNetwork())
					{
						const TArray<FRoadDef>& Roads = Net->GetRoads();
						double BestPtD2 = FMath::Square(500.0);   // 5 m: point pick
						int32  PtRoad = INDEX_NONE, PtIdx = INDEX_NONE;
						double BestRdD2 = FMath::Square(2000.0);  // 20 m: road pick
						int32  RdRoad = INDEX_NONE;
						for (int32 r = 0; r < Roads.Num(); ++r)
						{
							const FRoadDef& Rd = Roads[r];
							if (Rd.Source == ERoadNetSource::HandDrawn)
							{
								for (int32 i = 0; i < Rd.Ref.Num(); ++i)
								{
									const double D2 = FVector::DistSquaredXY(Hit, Rd.Ref[i]);
									if (D2 < BestPtD2) { BestPtD2 = D2; PtRoad = r; PtIdx = i; }
								}
							}
							for (int32 i = 0; i + 1 < Rd.Ref.Num(); ++i)
							{
								const FVector C = FMath::ClosestPointOnSegment(Hit, Rd.Ref[i], Rd.Ref[i + 1]);
								const double D2 = FVector::DistSquaredXY(Hit, C);
								if (D2 < BestRdD2) { BestRdD2 = D2; RdRoad = r; }
							}
						}
						if (PtRoad != INDEX_NONE)      { DelRoad = PtRoad; DelPoint = PtIdx; }
						else if (RdRoad != INDEX_NONE) { DelRoad = RdRoad; DelPoint = INDEX_NONE; }
					}
				}
			}

			if (DelRoad != INDEX_NONE)
			{
				if (URoadNetwork* Net = GetNetwork())
				{
					if (DelPoint != INDEX_NONE)
					{
						const FScopedTransaction Transaction(LOCTEXT("RoadNetDeletePoint", "Delete RoadNet Point"));
						if (ARoadNetActor* Actor = NetActorPtr.Get()) { Actor->Modify(); }
						bool bRoadRemoved = false;
						if (Net->DeleteRoadPoint(DelRoad, DelPoint, bRoadRemoved))
						{
							Net->Rebuild();
							if (GEngine)
							{
								GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan,
									bRoadRemoved
										? TEXT("RoadNet: point removed (road had too few points, road deleted)")
										: TEXT("RoadNet: point deleted"));
							}
						}
					}
					else
					{
						const FScopedTransaction Transaction(LOCTEXT("RoadNetDeleteRoad", "Delete RoadNet Road"));
						if (ARoadNetActor* Actor = NetActorPtr.Get()) { Actor->Modify(); }
						if (Net->RemoveRoad(DelRoad))
						{
							Net->Rebuild();
							if (GEngine)
							{
								GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan,
									FString::Printf(TEXT("RoadNet: road %d deleted"), DelRoad));
							}
						}
					}
				}
				ClearSelection();
				if (ViewportClient) { ViewportClient->Invalidate(); }
				return true;
			}

			if (GEngine)
			{
				GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Orange,
					TEXT("RoadNet: nothing to delete — hover a road (or hand-drawn point), then press Delete"));
			}
			return true;
		}

		// Lane editing (draft not in progress):
		//   '=' / '+' / NumpadAdd      → add a lane        (Shift = left side)
		//   '-' / '_' / NumpadSubtract → remove outer lane (Shift = left side)
		// Targets the selected road, else auto-picks the road under the cursor.
		if (DraftPoints.Num() == 0)
		{
			const bool bAdd = (Key == EKeys::Equals || Key == EKeys::Add);
			const bool bRem = (Key == EKeys::Hyphen || Key == EKeys::Subtract);
			if (bAdd || bRem)
			{
				URoadNetwork* Net = GetNetwork();
				if (!Net) { return true; }

				// Resolve a target road: current selection, else nearest road to
				// the cursor within a pick radius so no explicit click is needed.
				int32 Target = SelRoad;
				if (Target == INDEX_NONE && ViewportClient)
				{
					FVector Hit;
					if (LineTraceCursor(ViewportClient, Hit))
					{
						const TArray<FRoadDef>& Roads = Net->GetRoads();
						double BestD2 = FMath::Square(2000.0); // 20 m pick radius
						for (int32 r = 0; r < Roads.Num(); ++r)
						{
							const FRoadDef& Rd = Roads[r];
							for (int32 i = 0; i + 1 < Rd.Ref.Num(); ++i)
							{
								const FVector C = FMath::ClosestPointOnSegment(Hit, Rd.Ref[i], Rd.Ref[i + 1]);
								const double D2 = FVector::DistSquaredXY(Hit, C);
								if (D2 < BestD2) { BestD2 = D2; Target = r; }
							}
						}
					}
				}

				if (Target == INDEX_NONE)
				{
					if (GEngine)
					{
						GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Orange,
							TEXT("RoadNet: no road under cursor — hover over (or click) a road, then press = / -"));
					}
					return true;
				}

				const bool bLeft = Viewport &&
					(Viewport->KeyState(EKeys::LeftShift) || Viewport->KeyState(EKeys::RightShift));
				const ERoadNetSide Side = bLeft ? ERoadNetSide::Left : ERoadNetSide::Right;

				const FScopedTransaction Transaction(LOCTEXT("RoadNetEditLane", "Edit RoadNet Lane"));
				if (ARoadNetActor* Actor = NetActorPtr.Get()) { Actor->Modify(); }

				const bool bOk = bAdd ? Net->AddLane(Target, Side) : Net->RemoveLane(Target, Side);
				if (bOk)
				{
					SelRoad = Target;   // keep it selected for repeated edits
					SelPoint = INDEX_NONE;
					Net->Rebuild();
					if (GEngine)
					{
						GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan,
							FString::Printf(TEXT("RoadNet: road %d now has %d lanes (%s %s)"),
								Target, Net->GetLaneCount(Target),
								bAdd ? TEXT("added") : TEXT("removed"),
								bLeft ? TEXT("left") : TEXT("right")));
					}
					if (ViewportClient) { ViewportClient->Invalidate(); }
				}
				else if (GEngine)
				{
					GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Orange,
						TEXT("RoadNet: lane edit not applied (already at min 1 lane, or road has authored DetailedLanes)."));
				}
				return true;
			}

			// Junction smoothing (draft not in progress):
			//   '[' → less smoothing, ']' → more smoothing (Shift = ×5 step).
			// Applies network-wide; rebuilds so all junctions refresh at once.
			const bool bSmoothDown = (Key == EKeys::LeftBracket);
			const bool bSmoothUp   = (Key == EKeys::RightBracket);
			if (bSmoothDown || bSmoothUp)
			{
				URoadNetwork* Net = GetNetwork();
				if (!Net) { return true; }

				const bool bCoarse = Viewport &&
					(Viewport->KeyState(EKeys::LeftShift) || Viewport->KeyState(EKeys::RightShift));
				const double Step = (bCoarse ? 50.0 : 10.0) * (bSmoothUp ? 1.0 : -1.0);

				const FScopedTransaction Transaction(LOCTEXT("RoadNetJunctionSmooth", "Adjust RoadNet Junction Smoothing"));
				if (ARoadNetActor* Actor = NetActorPtr.Get()) { Actor->Modify(); }

				const double NewVal = Net->AdjustJunctionSmoothing(Step);
				Net->Rebuild();
				if (GEngine)
				{
					GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan,
						FString::Printf(TEXT("RoadNet: junction smoothing = %.0f cm ( [ less / ] more, Shift = x5 )"), NewVal));
				}
				if (ViewportClient) { ViewportClient->Invalidate(); }
				return true;
			}

			// Junction markings (draft not in progress):
			//   'J' → cycle the treatment of the junction nearest the cursor
			//         (None → Stop → Stop+Crosswalk → Signalized → GiveWay).
			//         Shift+J reverses the cycle.
			if (Key == EKeys::J)
			{
				URoadNetwork* Net = GetNetwork();
				if (!Net) { return true; }

				FVector2D Loc = FVector2D::ZeroVector;
				bool bFound = false;
				double BestD2 = FMath::Square(3000.0); // 30 m junction pick radius
				FVector Hit;
				if (ViewportClient && LineTraceCursor(ViewportClient, Hit))
				{
					for (const URoadNetwork::FRoadNetJunctionView& V : Net->GetJunctionViews())
					{
						const double D2 = FVector::DistSquaredXY(Hit, V.Location);
						if (D2 < BestD2) { BestD2 = D2; Loc = FVector2D(V.Location.X, V.Location.Y); bFound = true; }
					}
				}
				if (!bFound)
				{
					if (GEngine)
					{
						GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Orange,
							TEXT("RoadNet: no junction under cursor — hover a junction (3+ roads), then press J"));
					}
					return true;
				}

				const bool bBack = Viewport &&
					(Viewport->KeyState(EKeys::LeftShift) || Viewport->KeyState(EKeys::RightShift));

				const FScopedTransaction Transaction(LOCTEXT("RoadNetJunctionMark", "Cycle RoadNet Junction Marking"));
				if (ARoadNetActor* Actor = NetActorPtr.Get()) { Actor->Modify(); }

				const ERoadNetJunctionPreset P = Net->CycleJunctionPresetNear(Loc, bBack ? -1 : 1);
				Net->Rebuild();
				if (GEngine)
				{
					GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan,
						FString::Printf(TEXT("RoadNet: junction marking = %s  ( J next / Shift+J prev )"), PresetName(P)));
				}
				if (ViewportClient) { ViewportClient->Invalidate(); }
				return true;
			}

			// Junction corner islands (draft not in progress):
			//   'K' → toggle curbed grass channelizing islands in the corner
			//         (negative-space) areas of the junction nearest the cursor.
			if (Key == EKeys::K)
			{
				URoadNetwork* Net = GetNetwork();
				if (!Net) { return true; }

				FVector2D Loc = FVector2D::ZeroVector;
				bool bFound = false;
				double BestD2 = FMath::Square(3000.0); // 30 m junction pick radius
				FVector Hit;
				if (ViewportClient && LineTraceCursor(ViewportClient, Hit))
				{
					for (const URoadNetwork::FRoadNetJunctionView& V : Net->GetJunctionViews())
					{
						const double D2 = FVector::DistSquaredXY(Hit, V.Location);
						if (D2 < BestD2) { BestD2 = D2; Loc = FVector2D(V.Location.X, V.Location.Y); bFound = true; }
					}
				}
				if (!bFound)
				{
					if (GEngine)
					{
						GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Orange,
							TEXT("RoadNet: no junction under cursor — hover a junction (3+ roads), then press K"));
					}
					return true;
				}

				const FScopedTransaction Transaction(LOCTEXT("RoadNetJunctionIsland", "Toggle RoadNet Junction Islands"));
				if (ARoadNetActor* Actor = NetActorPtr.Get()) { Actor->Modify(); }

				const bool bOn = Net->ToggleJunctionIslandsNear(Loc);
				Net->Rebuild();
				if (GEngine)
				{
					GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan,
						FString::Printf(TEXT("RoadNet: corner islands = %s  ( press K to toggle )"),
							bOn ? TEXT("ON") : TEXT("OFF")));
				}
				if (ViewportClient) { ViewportClient->Invalidate(); }
				return true;
			}

			// Median editing (draft not in progress):
			//   'M'         → toggle the central median on the target road
			//   'Shift+M'   → cycle median edge (Plantable → CurbOnly → Sidewalk+Curb → Plantable+Sidewalk+Curb)
			//   ',' / '.'   → narrow / widen the median (Shift = ×5 step)
			// Targets the selected road, else the nearest road under the cursor.
			if (Key == EKeys::M || Key == EKeys::Comma || Key == EKeys::Period)
			{
				URoadNetwork* Net = GetNetwork();
				if (!Net) { return true; }

				int32 Target = SelRoad;
				if (Target == INDEX_NONE && ViewportClient)
				{
					FVector Hit;
					if (LineTraceCursor(ViewportClient, Hit))
					{
						const TArray<FRoadDef>& Roads = Net->GetRoads();
						double BestD2 = FMath::Square(2000.0); // 20 m pick radius
						for (int32 r = 0; r < Roads.Num(); ++r)
						{
							const FRoadDef& Rd = Roads[r];
							for (int32 i = 0; i + 1 < Rd.Ref.Num(); ++i)
							{
								const FVector C = FMath::ClosestPointOnSegment(Hit, Rd.Ref[i], Rd.Ref[i + 1]);
								const double D2 = FVector::DistSquaredXY(Hit, C);
								if (D2 < BestD2) { BestD2 = D2; Target = r; }
							}
						}
					}
				}
				if (Target == INDEX_NONE)
				{
					if (GEngine)
					{
						GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Orange,
							TEXT("RoadNet: no road under cursor — hover a road, then M (median) / , . (width)"));
					}
					return true;
				}

				const bool bShift = Viewport &&
					(Viewport->KeyState(EKeys::LeftShift) || Viewport->KeyState(EKeys::RightShift));

				const FScopedTransaction Transaction(LOCTEXT("RoadNetMedian", "Edit RoadNet Median"));
				if (ARoadNetActor* Actor = NetActorPtr.Get()) { Actor->Modify(); }

				FString Msg;
				if (Key == EKeys::M)
				{
					if (bShift)
					{
						const ERoadNetMedianEdge E = Net->CycleMedianEdge(Target, +1);
						const TCHAR* EN = (E == ERoadNetMedianEdge::Plantable) ? TEXT("Plantable")
							: (E == ERoadNetMedianEdge::CurbOnly) ? TEXT("Curb only")
							: (E == ERoadNetMedianEdge::SidewalkAndCurb) ? TEXT("Sidewalk + Curb")
							: TEXT("Plantable + Sidewalk + Curb");
						Msg = FString::Printf(TEXT("RoadNet: median edge = %s"), EN);
					}
					else
					{
						const bool bOn = Net->ToggleMedian(Target);
						Msg = FString::Printf(TEXT("RoadNet: median %s on road %d  ( Shift+M edge, , . width )"),
							bOn ? TEXT("ON") : TEXT("OFF"), Target);
					}
				}
				else // ',' narrow / '.' widen
				{
					const float Step = (bShift ? 100.f : 20.f) * (Key == EKeys::Period ? 1.f : -1.f);
					const float W = Net->AdjustMedianWidth(Target, Step);
					Msg = FString::Printf(TEXT("RoadNet: median width = %.0f cm"), W);
				}

				SelRoad = Target;
				SelPoint = INDEX_NONE;
				Net->Rebuild();
				if (GEngine) { GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan, Msg); }
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

	// EDIT overlay: while idle, draw every road's centreline segments as
	// clickable handles so ANY road (imported OR hand-drawn) can be selected
	// for lane editing. Per-point handles (for geometry edits) are drawn only
	// for hand-drawn roads — OSM points are re-derived on import, so we don't
	// invite dragging/deleting them. Imported roads use a distinct colour.
	if (DraftPoints.Num() == 0)
	{
		if (const URoadNetwork* Net = GetNetwork())
		{
			const TArray<FRoadDef>& Roads = Net->GetRoads();
			for (int32 r = 0; r < Roads.Num(); ++r)
			{
				const FRoadDef& Road = Roads[r];
				if (Road.Ref.Num() < 2) { continue; }
				const bool bHand = (Road.Source == ERoadNetSource::HandDrawn);
				const bool bWholeRoadSel = (r == SelRoad && SelPoint == INDEX_NONE);

				for (int32 i = 0; i + 1 < Road.Ref.Num(); ++i)
				{
					PDI->SetHitProxy(new HRoadNetSegmentProxy(r, i));
					const FColor Col = bWholeRoadSel ? kColorSnap : (bHand ? kColorEditLine : kColorOsmLine);
					PDI->DrawLine(Road.Ref[i], Road.Ref[i + 1], Col, SDPG_World, bWholeRoadSel ? 3.f : 1.5f);
					PDI->SetHitProxy(nullptr);
				}

				if (bHand)
				{
					for (int32 i = 0; i < Road.Ref.Num(); ++i)
					{
						const bool bSel = (r == SelRoad && i == SelPoint) || bWholeRoadSel;
						PDI->SetHitProxy(new HRoadNetPointProxy(r, i));
						PDI->DrawPoint(Road.Ref[i], bSel ? kColorSnap : kColorEditPt, kPointSize, SDPG_Foreground);
						PDI->SetHitProxy(nullptr);
					}
				}
			}

			// Junction markers: a ring per real junction (3+ arms), coloured by
			// its current marking preset. Hover one and press 'J' to cycle it.
			for (const URoadNetwork::FRoadNetJunctionView& V : Net->GetJunctionViews())
			{
				const FColor Col = PresetColor(V.Preset);
				PDI->DrawPoint(V.Location + FVector(0, 0, 30.f), Col, 22.f, SDPG_Foreground);
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
