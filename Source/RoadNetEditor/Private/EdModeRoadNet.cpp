// EdModeRoadNet.cpp — click-to-draw road authoring mode (§9.3).
#include "EdModeRoadNet.h"
#include "RoadNetEditorBridge.h"
#include "RoadNetActor.h"
#include "RoadNetwork.h"
#include "RoadNetTypes.h"
#include "RoadNetMath.h"
#include "RoadNetLog.h"
#include "EditorViewportClient.h"
#include "SceneView.h"
#include "SceneManagement.h"
#include "CanvasItem.h"
#include "CanvasTypes.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"
#include "ScopedTransaction.h"
#include "Editor.h"
#include "HAL/IConsoleManager.h"

#define LOCTEXT_NAMESPACE "RoadNetEditor"

IMPLEMENT_HIT_PROXY(HRoadNetPointProxy, HHitProxy);
IMPLEMENT_HIT_PROXY(HRoadNetSegmentProxy, HHitProxy);
IMPLEMENT_HIT_PROXY(HRoadNetLaneProxy, HHitProxy);
IMPLEMENT_HIT_PROXY(HRoadNetEdgeProxy, HHitProxy);

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
	const FColor     kColorOsmPt = FColor(255, 170, 70);     // imported road points (shown with "edit all points")

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
	LastTool = ActiveTool();
	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] Draw mode: pick a TOOL (OSM Roads panel or keys 1-5): 1 Draw, 2 Points, 3 Lanes, 4 Junctions, 5 Edge. Each tool scopes its own clicks/hotkeys. Full list: 'OSM Roads' panel > Legend tab."));
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Cyan,
			TEXT("RoadNet: pick a TOOL — OSM Roads panel or keys 1-5:  1 Draw  |  2 Points  |  3 Lanes  |  4 Junctions  |  5 Edge\n"
			     "Draw: LMB adds points, Enter/RMB finish.   Points: LMB select, marquee, drag=move, Del delete, Ctrl+LMB split, U merge, P edit-all.\n"
			     "Lanes: click a lane, =/- add/remove, Shift+= / Shift+- insert beside it, B cycle type (bike/parking).\n"
			     "Junctions: J mark, K island, [ ] smoothing, M median (Shift+M edge, , . width).   Edge: drag outer-edge vertices.\n"
			     "Ctrl+Z undo  |  Full cheat-sheet: 'OSM Roads' panel > Legend tab"));
	}
}

void FEdModeRoadNet::Exit()
{
	// Don't lose a queued smoothing rebuild if the user leaves the mode.
	FlushPendingSmoothing(nullptr);
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

ERoadNetDrawTool FEdModeRoadNet::ActiveTool() const
{
	static IConsoleVariable* CV = IConsoleManager::Get().FindConsoleVariable(TEXT("roadnet.DrawTool"));
	const int32 V = CV ? CV->GetInt() : 0;
	return (ERoadNetDrawTool)(uint8)FMath::Clamp(V, 0, 4);
}

void FEdModeRoadNet::SetActiveTool(ERoadNetDrawTool Tool)
{
	if (IConsoleVariable* CV = IConsoleManager::Get().FindConsoleVariable(TEXT("roadnet.DrawTool")))
	{
		CV->Set((int32)Tool, ECVF_SetByConsole);
	}
}

// Defined further down; forward-declared so the debounce flush (above it) can
// scope the smoothing rebuild to the junction's arm roads.
static void CollectRoadsNearPoint(const URoadNetwork* Net, const FVector2D& Loc, double RadiusCm, TArray<int32>& Out);

// A square world-XY box centred on a junction. Passed to URoadNetwork::Rebuild
// so a junction edit (markings / islands / smoothing) commits only that
// junction's grid cells instead of the full length of its (possibly long) arms.
static FBox2D JunctionDirtyBox(const FVector2D& Loc, double HalfCm)
{
	return FBox2D(Loc - FVector2D(HalfCm, HalfCm), Loc + FVector2D(HalfCm, HalfCm));
}
// Half-size of a junction's dirty box (see JunctionDirtyBox). Comfortably covers
// a junction's fillet / markings / islands / signal placeholders; the rebuild
// then dilates by the geometry reach so border cells still commit.
static constexpr double kJunctionDirtyHalfCm = 6000.0; // 60 m

void FEdModeRoadNet::FlushPendingSmoothing(FEditorViewportClient* ViewportClient)
{
	if (!bSmoothingRebuildPending) { return; }
	bSmoothingRebuildPending = false;
	if (URoadNetwork* Net = GetNetwork())
	{
		// Disc-scoped: only the queued junction's tiles re-commit.
		TArray<int32> Arms;
		CollectRoadsNearPoint(Net, SmoothingPendingLoc, 3000.0, Arms);
		Net->Rebuild(Arms, JunctionDirtyBox(SmoothingPendingLoc, kJunctionDirtyHalfCm));
	}
	if (ViewportClient) { ViewportClient->Invalidate(); }
}

void FEdModeRoadNet::Tick(FEditorViewportClient* ViewportClient, float DeltaTime)
{
	FEdMode::Tick(ViewportClient, DeltaTime);

	// Debounced junction-smoothing rebuild: fire once the user has paused for a
	// short window after their last '[' / ']' tap (coalesces a burst into one
	// full rebuild). See the smoothing handler in InputKey.
	if (bSmoothingRebuildPending)
	{
		constexpr double kSmoothDebounceSec = 0.35;
		if (FPlatformTime::Seconds() - SmoothingLastEditTime >= kSmoothDebounceSec)
		{
			FlushPendingSmoothing(ViewportClient);
		}
	}

	// Watch the shared tool CVar (the panel or a number key may change it). On a
	// change, drop any transient state that belongs to the previous tool so a
	// half-finished draw or marquee never leaks into the new tool.
	const ERoadNetDrawTool T = ActiveTool();
	if (T != LastTool)
	{
		// Don't strand a queued smoothing rebuild when switching tools.
		FlushPendingSmoothing(ViewportClient);
		LastTool = T;
		if (T != ERoadNetDrawTool::Draw) { DraftPoints.Reset(); }
		else                             { ClearSelection(); }
		bMarquee = false;
		bMarqueeMoved = false;
		if (GEngine)
		{
			static const TCHAR* Names[] = { TEXT("Draw"), TEXT("Points"), TEXT("Lanes"), TEXT("Junctions"), TEXT("Edge") };
			GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Yellow,
				FString::Printf(TEXT("RoadNet tool: %s"), Names[(int32)T]));
		}
		if (ViewportClient) { ViewportClient->Invalidate(); }
	}
}

void FEdModeRoadNet::ClearSelection()
{
	SelRoad = INDEX_NONE;
	SelPoint = INDEX_NONE;
	SelLane = INDEX_NONE;
	SelLaneId.Invalidate();
	SelEdgeKnot = INDEX_NONE;
	SelPoints.Reset();
	bDirtyDuringDrag = false;
}

bool FEdModeRoadNet::RefFrameAt(int32 RoadIdx, double ArcCm, FVector& OutPoint, FVector2D& OutRight) const
{
	const URoadNetwork* Net = GetNetwork();
	if (!Net) { return false; }
	const TArray<FRoadDef>& Roads = Net->GetRoads();
	if (!Roads.IsValidIndex(RoadIdx)) { return false; }
	const TArray<FVector>& Ref = Roads[RoadIdx].Ref;
	if (Ref.Num() < 2) { return false; }

	double Acc = 0.0;
	for (int32 i = 0; i + 1 < Ref.Num(); ++i)
	{
		const double Seg = FVector::Dist2D(Ref[i], Ref[i + 1]);
		if (Seg <= KINDA_SMALL_NUMBER) { continue; }
		if (ArcCm <= Acc + Seg || i + 2 == Ref.Num())
		{
			const double T = FMath::Clamp((ArcCm - Acc) / Seg, 0.0, 1.0);
			OutPoint = FMath::Lerp(Ref[i], Ref[i + 1], T);
			const FVector2D Tan = (FVector2D(Ref[i + 1].X, Ref[i + 1].Y) - FVector2D(Ref[i].X, Ref[i].Y)).GetSafeNormal();
			OutRight = FVector2D(Tan.Y, -Tan.X);   // +right of travel (matches RoadNetMath::RightAxis)
			return true;
		}
		Acc += Seg;
	}
	OutPoint = Ref.Last();
	const FVector2D Tan = (FVector2D(Ref.Last().X, Ref.Last().Y) - FVector2D(Ref[Ref.Num() - 2].X, Ref[Ref.Num() - 2].Y)).GetSafeNormal();
	OutRight = FVector2D(Tan.Y, -Tan.X);
	return true;
}

bool FEdModeRoadNet::GetEdgeKnotWorld(int32 RoadIdx, bool bSideRight, int32 Knot, FVector& OutWorld) const
{
	const URoadNetwork* Net = GetNetwork();
	if (!Net) { return false; }
	TArray<FRoadNetEdgeKnot> Profile;
	Net->GetOuterEdgeForDisplay(RoadIdx, bSideRight ? ERoadNetSide::Right : ERoadNetSide::Left, Profile);
	if (!Profile.IsValidIndex(Knot)) { return false; }

	FVector P; FVector2D Rt;
	if (!RefFrameAt(RoadIdx, Profile[Knot].Distance, P, Rt)) { return false; }
	const FVector2D W = FVector2D(P.X, P.Y) + Rt * Profile[Knot].Offset;
	OutWorld = FVector(W.X, W.Y, P.Z + 20.0);
	return true;
}

bool FEdModeRoadNet::ProjectToEdgeOffset(int32 RoadIdx, const FVector& W, double& OutOffset, double& OutArc) const
{
	const URoadNetwork* Net = GetNetwork();
	if (!Net) { return false; }
	const TArray<FRoadDef>& Roads = Net->GetRoads();
	if (!Roads.IsValidIndex(RoadIdx) || Roads[RoadIdx].Ref.Num() < 2) { return false; }
	const TArray<FVector>& Ref = Roads[RoadIdx].Ref;

	double Acc = 0.0, BestD2 = TNumericLimits<double>::Max();
	OutOffset = 0.0; OutArc = 0.0;
	const FVector2D Q(W.X, W.Y);
	for (int32 i = 0; i + 1 < Ref.Num(); ++i)
	{
		const FVector2D A(Ref[i].X, Ref[i].Y), B(Ref[i + 1].X, Ref[i + 1].Y);
		const FVector2D AB = B - A;
		const double Len2 = AB.SizeSquared();
		const double T = (Len2 > KINDA_SMALL_NUMBER) ? FMath::Clamp(FVector2D::DotProduct(Q - A, AB) / Len2, 0.0, 1.0) : 0.0;
		const FVector2D P = A + AB * T;
		const double D2 = FVector2D::DistSquared(Q, P);
		if (D2 < BestD2)
		{
			BestD2 = D2;
			const FVector2D Tan = AB.GetSafeNormal();
			const FVector2D Rt(Tan.Y, -Tan.X);
			OutOffset = FVector2D::DotProduct(Q - P, Rt);
			OutArc = Acc + FMath::Sqrt(Len2) * T;
		}
		Acc += FMath::Sqrt(Len2);
	}
	return true;
}

void FEdModeRoadNet::SelectLaneOnRoad(int32 RoadIdx, int32 LaneLtoR)
{
	SelPoints.Reset();
	SelRoad = RoadIdx;
	SelPoint = INDEX_NONE;
	SelLane = LaneLtoR;
	SelLaneId.Invalidate();
	if (LaneLtoR == INDEX_NONE) { return; }

	URoadNetwork* Net = GetNetwork();
	if (!Net) { return; }
	// Materialise so the lane gains a stable LaneId we can track across rebuilds.
	Net->MaterializeLanes(RoadIdx);
	const TArray<FRoadNetLane> Lanes = Net->GetLanesLeftToRight(RoadIdx);
	if (Lanes.IsValidIndex(LaneLtoR)) { SelLaneId = Lanes[LaneLtoR].LaneId; }
}

void FEdModeRoadNet::ResolveSelLaneFromId()
{
	if (!SelLaneId.IsValid() || SelRoad == INDEX_NONE) { return; }
	const URoadNetwork* Net = GetNetwork();
	if (!Net) { return; }
	const TArray<FRoadNetLane> Lanes = Net->GetLanesLeftToRight(SelRoad);
	for (int32 i = 0; i < Lanes.Num(); ++i)
	{
		if (Lanes[i].LaneId == SelLaneId) { SelLane = i; return; }
	}
	// Lane vanished (e.g. removed) — keep the index clamped so the highlight
	// falls on a neighbour rather than a stale/out-of-range lane.
	if (Lanes.Num() > 0) { SelLane = FMath::Clamp(SelLane, 0, Lanes.Num() - 1); }
	else                 { SelLane = INDEX_NONE; SelLaneId.Invalidate(); }
}

int32 FEdModeRoadNet::PickLaneAt(int32 RoadIdx, const FVector& WorldHit) const
{
	const URoadNetwork* Net = GetNetwork();
	if (!Net) { return INDEX_NONE; }
	const TArray<FRoadDef>& Roads = Net->GetRoads();
	if (!Roads.IsValidIndex(RoadIdx) || Roads[RoadIdx].Ref.Num() < 2) { return INDEX_NONE; }

	const TArray<FRoadNetLane> Lanes = Net->GetLanesLeftToRight(RoadIdx);
	if (Lanes.Num() == 0) { return INDEX_NONE; }

	// Nearest lane by distance from the click to each lane-centre offset line
	// (offsetting the raw reference polyline — good enough for a picker).
	const TArray<FVector>& Ref = Roads[RoadIdx].Ref;
	int32 Best = INDEX_NONE;
	double BestD2 = TNumericLimits<double>::Max();
	for (int32 i = 0; i < Lanes.Num(); ++i)
	{
		TArray<FVector> C;
		RoadNetMath::OffsetPolyline(Ref, Lanes[i].CenterOffset, C);
		for (int32 s = 0; s + 1 < C.Num(); ++s)
		{
			const FVector P = FMath::ClosestPointOnSegment(WorldHit, C[s], C[s + 1]);
			const double D2 = FVector::DistSquaredXY(WorldHit, P);
			if (D2 < BestD2) { BestD2 = D2; Best = i; }
		}
	}
	return Best;
}

bool FEdModeRoadNet::IsPointSelected(int32 Road, int32 Point) const
{
	return SelPoints.Contains(FIntPoint(Road, Point));
}

void FEdModeRoadNet::SelectSinglePoint(int32 Road, int32 Point)
{
	SelPoints.Reset();
	SelPoints.Add(FIntPoint(Road, Point));
	SelRoad = Road;
	SelPoint = Point;   // >= 0 so this is a POINT selection, not a whole-road one
	SelLane = INDEX_NONE;
}

void FEdModeRoadNet::ToggleSelPoint(int32 Road, int32 Point)
{
	const FIntPoint Key(Road, Point);
	SelLane = INDEX_NONE;
	const int32 Existing = SelPoints.Find(Key);
	if (Existing != INDEX_NONE)
	{
		SelPoints.RemoveAt(Existing);
	}
	else
	{
		SelPoints.Add(Key);
	}
	// Keep the primary (road context) pointing at a still-selected point.
	if (SelPoints.Num() > 0)
	{
		SelRoad  = SelPoints[0].X;
		SelPoint = SelPoints[0].Y;
	}
	else
	{
		SelRoad = INDEX_NONE;
		SelPoint = INDEX_NONE;
	}
}

bool FEdModeRoadNet::GetSelectionCentroid(FVector& OutPos) const
{
	const URoadNetwork* Net = GetNetwork();
	if (!Net || SelPoints.Num() == 0) { return false; }
	const TArray<FRoadDef>& Roads = Net->GetRoads();
	FVector Sum = FVector::ZeroVector;
	int32 N = 0;
	for (const FIntPoint& S : SelPoints)
	{
		if (Roads.IsValidIndex(S.X) && Roads[S.X].Ref.IsValidIndex(S.Y))
		{
			Sum += Roads[S.X].Ref[S.Y];
			++N;
		}
	}
	if (N == 0) { return false; }
	OutPos = Sum / (double)N;
	return true;
}

void FEdModeRoadNet::MoveSelectedPoints(const FVector& Delta)
{
	URoadNetwork* Net = GetNetwork();
	if (!Net || SelPoints.Num() == 0) { return; }
	const TArray<FRoadDef>& Roads = Net->GetRoads();
	for (const FIntPoint& S : SelPoints)
	{
		if (Roads.IsValidIndex(S.X) && Roads[S.X].Ref.IsValidIndex(S.Y))
		{
			Net->MoveRoadPoint(S.X, S.Y, Roads[S.X].Ref[S.Y] + Delta);   // rebuild on release
		}
	}
}

int32 FEdModeRoadNet::DeleteSelectedPoints()
{
	URoadNetwork* Net = GetNetwork();
	if (!Net || SelPoints.Num() == 0) { return 0; }

	// Group the selection by road so we can delete highest indices first (index
	// shifts within a road never invalidate a lower pending delete), and process
	// roads highest-first (a whole-road removal only shifts higher road indices,
	// which are already done).
	TMap<int32, TArray<int32>> ByRoad;
	for (const FIntPoint& S : SelPoints) { ByRoad.FindOrAdd(S.X).AddUnique(S.Y); }

	TArray<int32> RoadKeys;
	ByRoad.GetKeys(RoadKeys);
	RoadKeys.Sort([](const int32& A, const int32& B) { return A > B; });

	int32 Removed = 0;
	for (int32 r : RoadKeys)
	{
		TArray<int32>& Pts = ByRoad[r];
		Pts.Sort([](const int32& A, const int32& B) { return A > B; });
		for (int32 p : Pts)
		{
			bool bRoadRemoved = false;
			if (Net->DeleteRoadPoint(r, p, bRoadRemoved))
			{
				++Removed;
				// Road collapsed (< 2 points): its remaining selected points no
				// longer exist — stop deleting on this road.
				if (bRoadRemoved) { break; }
			}
		}
	}
	return Removed;
}

void FEdModeRoadNet::SelectPointsInMarquee(FEditorViewportClient* ViewportClient, bool bAdd)
{
	URoadNetwork* Net = GetNetwork();
	if (!Net || !ViewportClient || !ViewportClient->Viewport) { return; }

	FViewport* Viewport = ViewportClient->Viewport;
	FSceneViewFamilyContext ViewFamily(FSceneViewFamily::ConstructionValues(
		Viewport, ViewportClient->GetScene(), ViewportClient->EngineShowFlags).SetRealtimeUpdate(ViewportClient->IsRealtime()));
	FSceneView* View = ViewportClient->CalcSceneView(&ViewFamily);
	if (!View) { return; }

	const int32 MinX = FMath::Min(MarqueeStart.X, MarqueeCur.X);
	const int32 MaxX = FMath::Max(MarqueeStart.X, MarqueeCur.X);
	const int32 MinY = FMath::Min(MarqueeStart.Y, MarqueeCur.Y);
	const int32 MaxY = FMath::Max(MarqueeStart.Y, MarqueeCur.Y);

	if (!bAdd) { SelPoints.Reset(); }

	const TArray<FRoadDef>& Roads = Net->GetRoads();
	for (int32 r = 0; r < Roads.Num(); ++r)
	{
		const FRoadDef& Rd = Roads[r];
		// Only points that are actually SHOWN are selectable (hand-drawn always,
		// imported only while "edit all points" is on).
		if (!(Rd.Source == ERoadNetSource::HandDrawn || bShowAllPoints)) { continue; }
		for (int32 i = 0; i < Rd.Ref.Num(); ++i)
		{
			FVector2D Px;
			if (!View->WorldToPixel(Rd.Ref[i], Px)) { continue; } // behind camera
			if (Px.X >= MinX && Px.X <= MaxX && Px.Y >= MinY && Px.Y <= MaxY)
			{
				SelPoints.AddUnique(FIntPoint(r, i));
			}
		}
	}

	// Refresh the primary (road context) from the resulting selection.
	if (SelPoints.Num() > 0)
	{
		SelRoad  = SelPoints[0].X;
		SelPoint = SelPoints[0].Y;
	}
	else
	{
		SelRoad = INDEX_NONE;
		SelPoint = INDEX_NONE;
	}

	if (GEngine && SelPoints.Num() > 0)
	{
		GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan,
			FString::Printf(TEXT("RoadNet: %d control point(s) selected — drag widget to move, Del to delete (Shift+drag to add)"),
				SelPoints.Num()));
	}
}

void FEdModeRoadNet::ModifyForEdit()
{
	// Modify the actor (owns the sub-object) AND the network (owns the Roads
	// array) so the whole authoring state is snapshotted into the transaction.
	if (ARoadNetActor* Actor = NetActorPtr.Get())
	{
		Actor->Modify();
		if (URoadNetwork* Net = Actor->GetNetwork()) { Net->Modify(); }
	}
}

bool FEdModeRoadNet::TrySelectUnderCursor(FEditorViewportClient* ViewportClient, bool bToggle, bool bRoadOnly)
{
	URoadNetwork* Net = GetNetwork();
	if (!Net || !ViewportClient) { return false; }

	FVector Hit;
	if (!LineTraceCursor(ViewportClient, Hit)) { return false; }

	const TArray<FRoadDef>& Roads = Net->GetRoads();
	double BestPtD2 = FMath::Square(450.0);   // 4.5 m: hand-drawn control-point pick
	int32  PtRoad = INDEX_NONE, PtIdx = INDEX_NONE;
	double BestRdD2 = FMath::Square(400.0);   // 4 m: road-centreline pick
	int32  RdRoad = INDEX_NONE;
	for (int32 r = 0; r < Roads.Num(); ++r)
	{
		const FRoadDef& Rd = Roads[r];
		// Per-point handles exist for hand-drawn roads always, and for imported
		// roads while "edit all points" (P) is on. Only those are point-selectable.
		// The Lanes/Junctions/Edge tools pass bRoadOnly so a click always resolves
		// to a whole road, never a control point.
		if (!bRoadOnly && (Rd.Source == ERoadNetSource::HandDrawn || bShowAllPoints))
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

	if (PtRoad != INDEX_NONE)
	{
		if (bToggle) { ToggleSelPoint(PtRoad, PtIdx); }
		else         { SelectSinglePoint(PtRoad, PtIdx); }
	}
	else if (RdRoad != INDEX_NONE)
	{
		// Whole-road select (not a point pick) — drop any point selection and
		// pick the lane under the cursor for lane editing.
		SelPoints.Reset();
		SelRoad = RdRoad;
		SelPoint = INDEX_NONE;
		SelLane = PickLaneAt(RdRoad, Hit);
	}
	else { return false; }

	if (ViewportClient) { ViewportClient->Invalidate(); }
	return true;
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
	const ERoadNetDrawTool Tool = ActiveTool();

	if (Click.GetKey() == EKeys::LeftMouseButton)
	{
		// DRAW tool: every left click drops a road point; double-click finalizes.
		if (Tool == ERoadNetDrawTool::Draw)
		{
			if (Click.GetEvent() == IE_DoubleClick) { FinalizeDraft(); return true; }
			ClearSelection();
			FVector Hit;
			if (LineTraceCursor(InViewportClient, Hit))
			{
				DraftPoints.Add(ResolveCursorPoint(Hit));
				if (InViewportClient) { InViewportClient->Invalidate(); }
			}
			return true;
		}

		// Non-draw tools never draw. Drop any stray draft and ignore double-clicks.
		if (DraftPoints.Num() > 0) { DraftPoints.Reset(); }
		if (Click.GetEvent() == IE_DoubleClick) { return true; }

		if (Tool == ERoadNetDrawTool::Points)
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
								InsertPos = FMath::ClosestPointOnSegment(
									Hit, Roads[S->RoadIndex].Ref[S->SegIndex], Roads[S->RoadIndex].Ref[S->SegIndex + 1]);
							}
							const FScopedTransaction Transaction(LOCTEXT("RoadNetSplit", "Split RoadNet Road"));
							ModifyForEdit();
							if (Net->InsertRoadPoint(S->RoadIndex, S->SegIndex, InsertPos))
							{
								SelRoad = S->RoadIndex;
								SelPoint = S->SegIndex + 1;
								const int32 M = S->RoadIndex;
								Net->Rebuild(MakeArrayView(&M, 1));   // windowed: one road reshaped
							}
						}
					}
					if (InViewportClient) { InViewportClient->Invalidate(); }
					return true;
				}
			}
			// Point proxy → select (Shift = toggle multi-selection).
			if (HRoadNetPointProxy* P = HitProxyCast<HRoadNetPointProxy>(HitProxy))
			{
				if (Click.IsShiftDown()) { ToggleSelPoint(P->RoadIndex, P->PointIndex); }
				else                     { SelectSinglePoint(P->RoadIndex, P->PointIndex); }
				if (InViewportClient) { InViewportClient->Invalidate(); }
				return true;
			}
			// Segment proxy → whole-road select (no lane pick in the Points tool).
			if (HRoadNetSegmentProxy* S = HitProxyCast<HRoadNetSegmentProxy>(HitProxy))
			{
				SelPoints.Reset();
				SelRoad = S->RoadIndex; SelPoint = INDEX_NONE; SelLane = INDEX_NONE;
				if (InViewportClient) { InViewportClient->Invalidate(); }
				return true;
			}
			// Proximity fallback (the carriageway mesh occludes the thin proxies).
			if (TrySelectUnderCursor(InViewportClient, Click.IsShiftDown(), /*bRoadOnly*/false)) { return true; }
			// Click on empty space → clear the selection.
			ClearSelection();
			if (InViewportClient) { InViewportClient->Invalidate(); }
			return true;
		}

		if (Tool == ERoadNetDrawTool::Lanes)
		{
			// Lane proxy → select that exact lane (stable LaneId).
			if (HRoadNetLaneProxy* Lp = HitProxyCast<HRoadNetLaneProxy>(HitProxy))
			{
				SelectLaneOnRoad(Lp->RoadIndex, Lp->LaneLtoR);
				if (InViewportClient) { InViewportClient->Invalidate(); }
				return true;
			}
			// Segment proxy → select the road AND the lane under the cursor.
			if (HRoadNetSegmentProxy* S = HitProxyCast<HRoadNetSegmentProxy>(HitProxy))
			{
				FVector Hit;
				const int32 L = LineTraceCursor(InViewportClient, Hit) ? PickLaneAt(S->RoadIndex, Hit) : INDEX_NONE;
				SelectLaneOnRoad(S->RoadIndex, L);
				if (InViewportClient) { InViewportClient->Invalidate(); }
				return true;
			}
			// Proximity fallback resolves a road (and its lane via PickLaneAt).
			if (TrySelectUnderCursor(InViewportClient, false, /*bRoadOnly*/true))
			{
				SelectLaneOnRoad(SelRoad, SelLane);   // upgrade to a stable id
				return true;
			}
			return true;
		}

		if (Tool == ERoadNetDrawTool::Edge)
		{
			// Edge handle proxy → select that outer-edge vertex (widget drag target).
			if (HRoadNetEdgeProxy* Ep = HitProxyCast<HRoadNetEdgeProxy>(HitProxy))
			{
				SelPoints.Reset();
				SelRoad = Ep->RoadIndex; SelPoint = INDEX_NONE; SelLane = INDEX_NONE;
				bSelEdgeRight = Ep->bSideRight; SelEdgeKnot = Ep->KnotIndex;
				if (InViewportClient) { InViewportClient->Invalidate(); }
				return true;
			}
			// Ctrl+click near an edge of the selected road → insert a new vertex.
			if (Click.IsControlDown() && SelRoad != INDEX_NONE)
			{
				FVector Hit;
				if (URoadNetwork* Net = GetNetwork(); Net && LineTraceCursor(InViewportClient, Hit))
				{
					double Off = 0.0, Arc = 0.0;
					if (ProjectToEdgeOffset(SelRoad, Hit, Off, Arc))
					{
						const bool bRight = (Off >= 0.0);
						const FScopedTransaction Transaction(LOCTEXT("RoadNetAddEdgeKnot", "Add RoadNet Edge Vertex"));
						ModifyForEdit();
						const int32 NewK = Net->AddOuterEdgeKnot(SelRoad,
							bRight ? ERoadNetSide::Right : ERoadNetSide::Left, Arc, Off);
						if (NewK != INDEX_NONE) { bSelEdgeRight = bRight; SelEdgeKnot = NewK; const int32 M = SelRoad; Net->Rebuild(MakeArrayView(&M, 1)); }
					}
				}
				if (InViewportClient) { InViewportClient->Invalidate(); }
				return true;
			}
			// Segment proxy → select the road (its edge handles then appear).
			if (HRoadNetSegmentProxy* S = HitProxyCast<HRoadNetSegmentProxy>(HitProxy))
			{
				SelPoints.Reset();
				SelRoad = S->RoadIndex; SelPoint = INDEX_NONE; SelLane = INDEX_NONE; SelEdgeKnot = INDEX_NONE;
				if (InViewportClient) { InViewportClient->Invalidate(); }
				return true;
			}
			if (TrySelectUnderCursor(InViewportClient, false, /*bRoadOnly*/true)) { SelEdgeKnot = INDEX_NONE; return true; }
			return true;
		}

		// Junctions tool: whole-road select for context (median target etc.).
		if (HRoadNetSegmentProxy* S = HitProxyCast<HRoadNetSegmentProxy>(HitProxy))
		{
			SelPoints.Reset();
			SelRoad = S->RoadIndex; SelPoint = INDEX_NONE; SelLane = INDEX_NONE;
			if (InViewportClient) { InViewportClient->Invalidate(); }
			return true;
		}
		if (TrySelectUnderCursor(InViewportClient, false, /*bRoadOnly*/true)) { return true; }
		return true;
	}
	if (Click.GetKey() == EKeys::RightMouseButton)
	{
		if (Tool == ERoadNetDrawTool::Draw) { FinalizeDraft(); return true; }
		return false;   // let RMB drive the camera / context menu in the edit tools
	}
	return FEdMode::HandleClick(InViewportClient, HitProxy, Click);
}

// Road indices whose centreline passes within RadiusCm of a junction location.
// Used to scope a junction edit (marking preset / corner islands) to a WINDOWED
// rebuild instead of a full-network one. Empty result => caller does a full
// rebuild (safe fallback).
static void CollectRoadsNearPoint(const URoadNetwork* Net, const FVector2D& Loc, double RadiusCm, TArray<int32>& Out)
{
	if (!Net) { return; }
	const double R2 = RadiusCm * RadiusCm;
	const TArray<FRoadDef>& Roads = Net->GetRoads();
	for (int32 r = 0; r < Roads.Num(); ++r)
	{
		const FRoadDef& Rd = Roads[r];
		for (int32 i = 0; i + 1 < Rd.Ref.Num(); ++i)
		{
			const FVector2D A(Rd.Ref[i].X, Rd.Ref[i].Y);
			const FVector2D B(Rd.Ref[i + 1].X, Rd.Ref[i + 1].Y);
			const FVector2D C = FMath::ClosestPointOnSegment2D(Loc, A, B);
			if (FVector2D::DistSquared(C, Loc) < R2) { Out.Add(r); break; }
		}
	}
}

bool FEdModeRoadNet::InputKey(FEditorViewportClient* ViewportClient, FViewport* Viewport, FKey Key, EInputEvent Event)
{
	if (Event == IE_Pressed)
	{
		// Number keys 1-5 switch the active sub-tool (mirrors the OSM Roads panel
		// toggle). Kept global so you can flip tools without reaching for the panel.
		if (Key == EKeys::One || Key == EKeys::Two || Key == EKeys::Three ||
			Key == EKeys::Four || Key == EKeys::Five)
		{
			const ERoadNetDrawTool NewTool =
				(Key == EKeys::One)   ? ERoadNetDrawTool::Draw :
				(Key == EKeys::Two)   ? ERoadNetDrawTool::Points :
				(Key == EKeys::Three) ? ERoadNetDrawTool::Lanes :
				(Key == EKeys::Four)  ? ERoadNetDrawTool::Junctions : ERoadNetDrawTool::Edge;
			SetActiveTool(NewTool);
			if (ViewportClient) { ViewportClient->Invalidate(); }
			return true;
		}

		// Active sub-tool for this key press. Every editing hotkey below is scoped
		// to its tool so a key never means two things at once.
		const ERoadNetDrawTool Tool = ActiveTool();

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
			// Edge tool: Delete removes the selected outer-edge vertex.
			if (Tool == ERoadNetDrawTool::Edge)
			{
				if (SelRoad != INDEX_NONE && SelEdgeKnot != INDEX_NONE)
				{
					if (URoadNetwork* Net = GetNetwork())
					{
						const FScopedTransaction Transaction(LOCTEXT("RoadNetRemoveEdgeKnot", "Remove RoadNet Edge Vertex"));
						ModifyForEdit();
						if (Net->RemoveOuterEdgeKnot(SelRoad,
							bSelEdgeRight ? ERoadNetSide::Right : ERoadNetSide::Left, SelEdgeKnot))
						{
							const int32 M = SelRoad;
							SelEdgeKnot = INDEX_NONE;
							Net->Rebuild(MakeArrayView(&M, 1));   // windowed: one road's edge
							if (GEngine)
							{
								GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan,
									TEXT("RoadNet: edge vertex removed"));
							}
						}
					}
					if (ViewportClient) { ViewportClient->Invalidate(); }
				}
				return true;
			}
			// Point/road deletion belongs to the Points tool only.
			if (Tool != ERoadNetDrawTool::Points) { return true; }
			// Multi-point delete: if a set of control points is selected (marquee
			// or Shift+click), delete them all in one undoable step.
			if (SelPoints.Num() > 0)
			{
				if (URoadNetwork* Net = GetNetwork())
				{
					const FScopedTransaction Transaction(LOCTEXT("RoadNetDeletePoints", "Delete RoadNet Points"));
					ModifyForEdit();
					const int32 N = DeleteSelectedPoints();
					Net->Rebuild();
					if (GEngine)
					{
						GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan,
							FString::Printf(TEXT("RoadNet: deleted %d selected control point(s)"), N));
					}
				}
				ClearSelection();
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
							if (Rd.Source == ERoadNetSource::HandDrawn || bShowAllPoints)
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
						ModifyForEdit();
						bool bRoadRemoved = false;
						if (Net->DeleteRoadPoint(DelRoad, DelPoint, bRoadRemoved))
						{
							// Removing a point only reshapes one road (windowed);
							// but if the road itself was dropped the array indices
							// shift, so fall back to a full rebuild.
							if (bRoadRemoved) { Net->Rebuild(); }
							else { const int32 M = DelRoad; Net->Rebuild(MakeArrayView(&M, 1)); }
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
						ModifyForEdit();
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
			// 'P' → toggle "edit all points": reveal + enable move/delete on EVERY
			// road's control points (imported included), not just hand-drawn ones.
			if (Tool == ERoadNetDrawTool::Points && Key == EKeys::P)
			{
				bShowAllPoints = !bShowAllPoints;
				if (GEngine)
				{
					GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Cyan,
						bShowAllPoints
							? TEXT("RoadNet: EDIT ALL POINTS on — every road's control points are draggable/deletable (P to hide)")
							: TEXT("RoadNet: edit all points off — only hand-drawn roads show point handles (P to show all)"));
				}
				if (ViewportClient) { ViewportClient->Invalidate(); }
				return true;
			}

			// 'U' → force-merge the roads owning the selected points into ONE
			// multi-lane road (ignores the import proximity test). Select points
			// on 2+ roads (marquee / Shift+click), then press U.
			if (Tool == ERoadNetDrawTool::Points && Key == EKeys::U)
			{
				TArray<int32> RoadIdx;
				for (const FIntPoint& S : SelPoints) { RoadIdx.AddUnique(S.X); }
				// A whole-road selection (segment click) contributes its road too.
				if (SelRoad != INDEX_NONE && SelPoint == INDEX_NONE) { RoadIdx.AddUnique(SelRoad); }

				if (RoadIdx.Num() < 2)
				{
					if (GEngine)
					{
						GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Orange,
							TEXT("RoadNet: select control points on 2+ roads (marquee / Shift+click), then U to force-merge"));
					}
					return true;
				}
				if (URoadNetwork* Net = GetNetwork())
				{
					const FScopedTransaction Transaction(LOCTEXT("RoadNetMergeRoads", "Merge RoadNet Roads"));
					ModifyForEdit();
					const bool bOk = Net->MergeRoads(RoadIdx);
					if (bOk) { Net->Rebuild(); }
					if (GEngine)
					{
						GEngine->AddOnScreenDebugMessage(-1, 3.0f, bOk ? FColor::Cyan : FColor::Orange,
							bOk ? FString::Printf(TEXT("RoadNet: merged %d roads into one multi-lane road"), RoadIdx.Num())
							    : TEXT("RoadNet: merge failed (need 2+ valid roads)"));
					}
				}
				ClearSelection();
				if (ViewportClient) { ViewportClient->Invalidate(); }
				return true;
			}

			const bool bAdd = (Key == EKeys::Equals || Key == EKeys::Add);
			const bool bRem = (Key == EKeys::Hyphen || Key == EKeys::Subtract);
			if ((bAdd || bRem) && Tool == ERoadNetDrawTool::Lanes)
			{
				URoadNetwork* Net = GetNetwork();
				if (!Net) { return true; }

				const bool bShift = Viewport &&
					(Viewport->KeyState(EKeys::LeftShift) || Viewport->KeyState(EKeys::RightShift));

				// Lane-relative INSERT: Shift + a SELECTED lane. Shift+= inserts a
				// new lane to the RIGHT of the highlighted lane, Shift+- to the LEFT
				// (plain = / - stay as the general add/remove below). Requires a lane
				// pick (click a road/lane first) so we know the anchor.
				if (bShift && SelRoad != INDEX_NONE && SelLane != INDEX_NONE)
				{
					const FScopedTransaction Transaction(LOCTEXT("RoadNetInsertLane", "Insert RoadNet Lane"));
					ModifyForEdit();
					const int32 NewSel = Net->InsertLaneRelative(SelRoad, SelLane, /*bRightSide*/bAdd);
					if (NewSel != INDEX_NONE)
					{
						SelLane = NewSel;
						SelPoint = INDEX_NONE;
						const int32 M = SelRoad;
						Net->Rebuild(MakeArrayView(&M, 1));   // windowed: one road's lanes
						if (GEngine)
						{
							GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan,
								FString::Printf(TEXT("RoadNet: inserted lane to the %s of the selected lane (road %d now has %d lanes)"),
									bAdd ? TEXT("RIGHT") : TEXT("LEFT"), SelRoad, Net->GetLaneCount(SelRoad)));
						}
						if (ViewportClient) { ViewportClient->Invalidate(); }
					}
					else if (GEngine)
					{
						GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Orange,
							TEXT("RoadNet: click a road/lane to select a lane first, then Shift+= / Shift+-"));
					}
					return true;
				}

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
				ModifyForEdit();

				const bool bOk = bAdd ? Net->AddLane(Target, Side) : Net->RemoveLane(Target, Side);
				if (bOk)
				{
					SelRoad = Target;   // keep it selected for repeated edits
					SelPoint = INDEX_NONE;
					const int32 M = Target;
					Net->Rebuild(MakeArrayView(&M, 1));   // windowed: one road's lanes
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

			// Lane type cycle (draft not in progress):
			//   'B' → cycle the SELECTED lane's type: driving → bicycle → parking →
			//         driving (Shift+B reverses). Best used on an OUTER lane to turn
			//         it into a bike path / parking bay. Requires a lane pick.
			if (Tool == ERoadNetDrawTool::Lanes && Key == EKeys::B)
			{
				URoadNetwork* Net = GetNetwork();
				if (!Net) { return true; }
				if (SelRoad == INDEX_NONE || SelLane == INDEX_NONE)
				{
					if (GEngine)
					{
						GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Orange,
							TEXT("RoadNet: click a road/lane to select a lane first, then B to cycle its type"));
					}
					return true;
				}
				const bool bShiftB = Viewport &&
					(Viewport->KeyState(EKeys::LeftShift) || Viewport->KeyState(EKeys::RightShift));

				const FScopedTransaction Transaction(LOCTEXT("RoadNetLaneType", "Cycle RoadNet Lane Type"));
				ModifyForEdit();
				const ERoadNetLaneType NewType = Net->CycleLaneType(SelRoad, SelLane, bShiftB ? -1 : 1);
				{ const int32 M = SelRoad; Net->Rebuild(MakeArrayView(&M, 1)); }   // windowed
				if (GEngine)
				{
					const TCHAR* TypeName =
						(NewType == ERoadNetLaneType::Bicycle) ? TEXT("Bicycle path") :
						(NewType == ERoadNetLaneType::Parking) ? TEXT("Parking bay")  : TEXT("Driving lane");
					GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan,
						FString::Printf(TEXT("RoadNet: selected lane is now a %s"), TypeName));
				}
				if (ViewportClient) { ViewportClient->Invalidate(); }
				return true;
			}

			// Standard parking bay (Lanes tool, draft not in progress):
			//   'P'       → add a standard parking bay to the selected road on the
			//               selected lane's side, using the panel-selected layout
			//               (roadnet.ParkingLayout: 0=Parallel 1=Perp 2=Angled).
			//   'Shift+P' → clear all standard parking bays from the road.
			// Falls back to the road under the cursor + right side when no lane is
			// selected, so it also works as a quick "select side + action".
			if (Tool == ERoadNetDrawTool::Lanes && Key == EKeys::P)
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
						double BestD2 = FMath::Square(2000.0);
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
							TEXT("RoadNet: select/hover a road (and a lane for its side), then P to add a parking bay"));
					}
					return true;
				}

				const bool bShiftP = Viewport &&
					(Viewport->KeyState(EKeys::LeftShift) || Viewport->KeyState(EKeys::RightShift));

				const FScopedTransaction Transaction(LOCTEXT("RoadNetParkingBay", "Edit RoadNet Parking Bay"));
				ModifyForEdit();

				FString Msg;
				if (bShiftP)
				{
					const int32 Removed = Net->ClearParkingBays(Target);
					Msg = FString::Printf(TEXT("RoadNet: cleared %d parking bay(s) from road %d"), Removed, Target);
				}
				else
				{
					// Side from the selected lane (else default right).
					ERoadNetSide Side = ERoadNetSide::Right;
					const TArray<FRoadNetLane> Lanes = Net->GetLanesLeftToRight(Target);
					if (Lanes.IsValidIndex(SelLane))
					{
						Side = (Lanes[SelLane].CenterOffset < 0.0) ? ERoadNetSide::Left : ERoadNetSide::Right;
					}

					int32 LayoutInt = 0;
					if (IConsoleVariable* CV = IConsoleManager::Get().FindConsoleVariable(TEXT("roadnet.ParkingLayout")))
					{
						LayoutInt = FMath::Clamp(CV->GetInt(), 0, 2);
					}
					const ERoadNetParkingLayout Layout = (ERoadNetParkingLayout)(uint8)LayoutInt;
					const TCHAR* LayoutName =
						(Layout == ERoadNetParkingLayout::Parallel)      ? TEXT("Parallel") :
						(Layout == ERoadNetParkingLayout::Perpendicular) ? TEXT("Perpendicular") : TEXT("Angled");

					Net->AddStandardParkingBay(Target, Side, Layout);
					Msg = FString::Printf(TEXT("RoadNet: added %s parking bay on road %d (%s side)"),
						LayoutName, Target, (Side == ERoadNetSide::Left) ? TEXT("left") : TEXT("right"));
				}

				SelRoad = Target;
				SelPoint = INDEX_NONE;
				{ const int32 M = Target; Net->Rebuild(MakeArrayView(&M, 1)); }   // windowed: one road's stalls
				if (GEngine) { GEngine->AddOnScreenDebugMessage(-1, 2.5f, FColor::Cyan, Msg); }
				if (ViewportClient) { ViewportClient->Invalidate(); }
				return true;
			}

			// Junction smoothing (draft not in progress):
			//   '[' → less smoothing, ']' → more smoothing (Shift = ×5 step).
			// PER-JUNCTION: tunes only the junction under the cursor and re-commits
			// only that junction's tiles. Rapid taps on the same junction are
			// debounced into one scoped rebuild (see Tick / FlushPendingSmoothing).
			const bool bSmoothDown = (Key == EKeys::LeftBracket);
			const bool bSmoothUp   = (Key == EKeys::RightBracket);
			if ((bSmoothDown || bSmoothUp) && Tool == ERoadNetDrawTool::Junctions)
			{
				URoadNetwork* Net = GetNetwork();
				if (!Net) { return true; }

				// Junction under the cursor (same pick as J / K).
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
							TEXT("RoadNet: no junction under cursor — hover a junction, then press [ or ]"));
					}
					return true;
				}

				const bool bCoarse = Viewport &&
					(Viewport->KeyState(EKeys::LeftShift) || Viewport->KeyState(EKeys::RightShift));
				const double Step = (bCoarse ? 50.0 : 10.0) * (bSmoothUp ? 1.0 : -1.0);

				const FScopedTransaction Transaction(LOCTEXT("RoadNetJunctionSmooth", "Adjust RoadNet Junction Smoothing"));
				ModifyForEdit();

				// If a different junction was queued, commit it first so its change
				// isn't lost when we retarget the debounce to this junction.
				if (bSmoothingRebuildPending &&
					FVector2D::DistSquared(SmoothingPendingLoc, Loc) > FMath::Square(kJunctionDirtyHalfCm))
				{
					FlushPendingSmoothing(ViewportClient);
				}

				FVector2D JLoc = Loc;
				const double NewVal = Net->AdjustJunctionSmoothingNear(Loc, Step, JLoc);
				// Apply the (cheap) value now, DEFER the scoped rebuild so a burst of
				// taps on this junction coalesces into one commit.
				bSmoothingRebuildPending = true;
				SmoothingPendingLoc = JLoc;
				SmoothingLastEditTime = FPlatformTime::Seconds();
				if (GEngine)
				{
					GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan,
						FString::Printf(TEXT("RoadNet: junction smoothing = %.0f cm ( [ less / ] more, Shift = x5 )  — applying…"), NewVal));
				}
				if (ViewportClient) { ViewportClient->Invalidate(); }
				return true;
			}

			// Junction markings (draft not in progress):
			//   'J' → cycle the treatment of the junction nearest the cursor
			//         (None → Stop → Stop+Crosswalk → Signalized → GiveWay).
			//         Shift+J reverses the cycle.
			if (Tool == ERoadNetDrawTool::Junctions && Key == EKeys::J)
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
				ModifyForEdit();

				const ERoadNetJunctionPreset P = Net->CycleJunctionPresetNear(Loc, bBack ? -1 : 1);
				// Disc-scoped: commit ONLY this junction's tiles (not the full
				// length of its arms), so cost is independent of arm road length.
				{ TArray<int32> Arms; CollectRoadsNearPoint(Net, Loc, 3000.0, Arms);
				  Net->Rebuild(Arms, JunctionDirtyBox(Loc, kJunctionDirtyHalfCm)); }
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
			if (Tool == ERoadNetDrawTool::Junctions && Key == EKeys::K)
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
				ModifyForEdit();

				const bool bOn = Net->ToggleJunctionIslandsNear(Loc);
				// Disc-scoped: commit ONLY this junction's tiles.
				{ TArray<int32> Arms; CollectRoadsNearPoint(Net, Loc, 3000.0, Arms);
				  Net->Rebuild(Arms, JunctionDirtyBox(Loc, kJunctionDirtyHalfCm)); }
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
			if (Tool == ERoadNetDrawTool::Junctions &&
				(Key == EKeys::M || Key == EKeys::Comma || Key == EKeys::Period))
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
				ModifyForEdit();

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
				{ const int32 M = Target; Net->Rebuild(MakeArrayView(&M, 1)); }   // windowed: one road's median
				if (GEngine) { GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan, Msg); }
				if (ViewportClient) { ViewportClient->Invalidate(); }
				return true;
			}

			// Sidewalk width (Edge tool): ',' narrow / '.' widen the sidewalk on
			// the target road (Shift = ×5 step). Targets SelRoad, else the road
			// under the cursor. The sidewalk, curb and edge marking all follow.
			if (Tool == ERoadNetDrawTool::Edge &&
				(Key == EKeys::Comma || Key == EKeys::Period))
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
							TEXT("RoadNet: no road under cursor — hover a road, then , . to size the sidewalk"));
					}
					return true;
				}

				const bool bShift = Viewport &&
					(Viewport->KeyState(EKeys::LeftShift) || Viewport->KeyState(EKeys::RightShift));
				const float Step = (bShift ? 100.f : 20.f) * (Key == EKeys::Period ? 1.f : -1.f);

				const FScopedTransaction Transaction(LOCTEXT("RoadNetSidewalk", "Edit RoadNet Sidewalk"));
				ModifyForEdit();
				const float W = Net->AdjustSidewalkWidth(Target, Step);
				SelRoad = Target;
				SelPoint = INDEX_NONE;
				{ const int32 M = Target; Net->Rebuild(MakeArrayView(&M, 1)); }   // windowed: one road's sidewalk
				if (GEngine)
				{
					GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan,
						FString::Printf(TEXT("RoadNet: sidewalk width = %.0f cm"), W));
				}
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

	// EDIT overlay: in every non-Draw tool, draw each road's centreline segments
	// as clickable handles so ANY road (imported OR hand-drawn) can be selected.
	// Which extra handles show (points / junction rings / lane highlight) is
	// scoped to the active tool so overlays never fight for the same click.
	const ERoadNetDrawTool Tool = ActiveTool();
	if (Tool != ERoadNetDrawTool::Draw)
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

				// Per-point handles (Points tool only): hand-drawn roads always,
				// imported roads only while "edit all points" (P) is on. Imported
				// handles use a warmer tint so it's clear which points came from OSM.
				if (Tool == ERoadNetDrawTool::Points && (bHand || bShowAllPoints))
				{
					const FColor PtCol = bHand ? kColorEditPt : kColorOsmPt;
					for (int32 i = 0; i < Road.Ref.Num(); ++i)
					{
						const bool bSel = IsPointSelected(r, i) || bWholeRoadSel;
						PDI->SetHitProxy(new HRoadNetPointProxy(r, i));
						PDI->DrawPoint(Road.Ref[i], bSel ? kColorSnap : PtCol,
							bSel ? kPointSize + 4.f : kPointSize, SDPG_Foreground);
						PDI->SetHitProxy(nullptr);
					}
				}
			}

			// Junction markers (Junctions tool only): a ring per real junction (3+
			// arms), coloured by its marking preset. Hover one and press 'J'.
			if (Tool == ERoadNetDrawTool::Junctions)
			{
				for (const URoadNetwork::FRoadNetJunctionView& V : Net->GetJunctionViews())
				{
					const FColor Col = PresetColor(V.Preset);
					PDI->DrawPoint(V.Location + FVector(0, 0, 30.f), Col, 22.f, SDPG_Foreground);
				}
			}

			// Lanes tool: on the selected road, draw a clickable proxy per lane
			// (its centre line) so each lane is an independent, precisely-pickable
			// entity, then highlight the selected lane's centre + edges (tinted by
			// type). Restricted to SelRoad so a city-scale import stays cheap; the
			// first pick comes from the segment proxy + PickLaneAt.
			if (Tool == ERoadNetDrawTool::Lanes
				&& SelRoad != INDEX_NONE && SelPoint == INDEX_NONE
				&& Roads.IsValidIndex(SelRoad) && Roads[SelRoad].Ref.Num() >= 2)
			{
				ResolveSelLaneFromId();   // keep the highlight on the same lane across rebuilds
				const TArray<FRoadNetLane> Lanes = Net->GetLanesLeftToRight(SelRoad);
				const TArray<FVector>& Ref = Roads[SelRoad].Ref;
				const FVector Lift(0, 0, 22.f);

				auto DrawOff = [&](double Off, const FColor& Col, float Thick)
				{
					TArray<FVector> C;
					RoadNetMath::OffsetPolyline(Ref, Off, C);
					for (int32 s = 0; s + 1 < C.Num(); ++s)
					{
						PDI->DrawLine(C[s] + Lift, C[s + 1] + Lift, Col, SDPG_Foreground, Thick);
					}
				};

				// Per-lane pick handles (centre line of every lane).
				for (int32 li = 0; li < Lanes.Num(); ++li)
				{
					const bool bSel = (li == SelLane);
					const FColor Lc = bSel ? FColor(70, 200, 255) : FColor(150, 150, 160);
					PDI->SetHitProxy(new HRoadNetLaneProxy(SelRoad, li));
					TArray<FVector> C;
					RoadNetMath::OffsetPolyline(Ref, Lanes[li].CenterOffset, C);
					for (int32 s = 0; s + 1 < C.Num(); ++s)
					{
						PDI->DrawLine(C[s] + Lift, C[s + 1] + Lift, Lc, SDPG_Foreground, bSel ? 4.f : 1.5f);
					}
					PDI->SetHitProxy(nullptr);
				}

				// Selected-lane edges, tinted by type (bike=green, parking=amber).
				if (Lanes.IsValidIndex(SelLane))
				{
					const FRoadNetLane& Ln = Lanes[SelLane];
					const FColor HL =
						(Ln.Type == ERoadNetLaneType::Bicycle) ? FColor(60, 220, 90)  :
						(Ln.Type == ERoadNetLaneType::Parking) ? FColor(255, 210, 40) : FColor(70, 200, 255);
					DrawOff(Ln.CenterOffset + 0.5 * Ln.Width, HL, 2.5f);
					DrawOff(Ln.CenterOffset - 0.5 * Ln.Width, HL, 2.5f);
				}
			}

			// Edge tool: on the selected road, draw both outer-edge polylines with a
			// draggable handle at every knot. Ctrl+click a line inserts a vertex,
			// Delete removes the selected one, dragging the widget reshapes the edge
			// (the sidewalk, curb and edge marking all follow).
			if (Tool == ERoadNetDrawTool::Edge && SelRoad != INDEX_NONE
				&& Roads.IsValidIndex(SelRoad) && Roads[SelRoad].Ref.Num() >= 2)
			{
				auto DrawSide = [&](bool bRight)
				{
					TArray<FRoadNetEdgeKnot> Profile;
					Net->GetOuterEdgeForDisplay(SelRoad,
						bRight ? ERoadNetSide::Right : ERoadNetSide::Left, Profile);
					const FColor EdgeCol = bRight ? FColor(255, 140, 60) : FColor(255, 195, 95);

					TArray<FVector> Line; Line.Reserve(Profile.Num());
					for (int32 k = 0; k < Profile.Num(); ++k)
					{
						FVector W; if (GetEdgeKnotWorld(SelRoad, bRight, k, W)) { Line.Add(W); }
					}
					for (int32 s = 0; s + 1 < Line.Num(); ++s)
					{
						PDI->DrawLine(Line[s], Line[s + 1], EdgeCol, SDPG_Foreground, 2.f);
					}
					for (int32 k = 0; k < Profile.Num(); ++k)
					{
						FVector W; if (!GetEdgeKnotWorld(SelRoad, bRight, k, W)) { continue; }
						const bool bSel = (SelEdgeKnot == k && bSelEdgeRight == bRight);
						PDI->SetHitProxy(new HRoadNetEdgeProxy(SelRoad, bRight, k));
						PDI->DrawPoint(W, bSel ? kColorSnap : EdgeCol,
							bSel ? kPointSize + 5.f : kPointSize + 2.f, SDPG_Foreground);
						PDI->SetHitProxy(nullptr);
					}
				};
				DrawSide(true);
				DrawSide(false);
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
	if (DraftPoints.Num() != 0) { return false; }
	const ERoadNetDrawTool T = ActiveTool();
	// Points tool: widget over the point selection.
	if (T == ERoadNetDrawTool::Points)
	{
		FVector Ignored;
		return GetSelectionCentroid(Ignored);
	}
	// Edge tool: widget over the selected outer-edge handle.
	if (T == ERoadNetDrawTool::Edge)
	{
		return SelRoad != INDEX_NONE && SelEdgeKnot != INDEX_NONE;
	}
	return false;
}

FVector FEdModeRoadNet::GetWidgetLocation() const
{
	if (ActiveTool() == ERoadNetDrawTool::Edge && SelRoad != INDEX_NONE && SelEdgeKnot != INDEX_NONE)
	{
		FVector W;
		if (GetEdgeKnotWorld(SelRoad, bSelEdgeRight, SelEdgeKnot, W)) { return W; }
	}
	FVector Pos;
	if (GetSelectionCentroid(Pos)) { return Pos; }
	return FEdMode::GetWidgetLocation();
}

EAxisList::Type FEdModeRoadNet::GetWidgetAxisToDraw(UE::Widget::EWidgetMode InWidgetMode) const
{
	if (InWidgetMode == UE::Widget::WM_Translate) { return EAxisList::XYZ; }
	return EAxisList::None;
}

bool FEdModeRoadNet::InputDelta(FEditorViewportClient* InViewportClient, FViewport* InViewport, FVector& InDrag, FRotator& InRot, FVector& InScale)
{
	const bool bWidgetGrabbed = InViewportClient && InViewportClient->GetCurrentWidgetAxis() != EAxisList::None;

	// Edge tool: drag the selected outer-edge handle. Convert the new handle world
	// position to a signed lateral offset vs the reference line and store it; the
	// mesh rebuilds on release (EndTracking) like the point-move path.
	if (bWidgetGrabbed && ActiveTool() == ERoadNetDrawTool::Edge
		&& SelRoad != INDEX_NONE && SelEdgeKnot != INDEX_NONE)
	{
		if (URoadNetwork* Net = GetNetwork())
		{
			if (!bDirtyDuringDrag && GEditor)
			{
				GEditor->BeginTransaction(LOCTEXT("RoadNetMoveEdge", "Move RoadNet Edge Vertex"));
				ModifyForEdit();
				bDirtyDuringDrag = true;
			}
			FVector W;
			if (GetEdgeKnotWorld(SelRoad, bSelEdgeRight, SelEdgeKnot, W))
			{
				W += InDrag;
				double Off = 0.0, Arc = 0.0;
				if (ProjectToEdgeOffset(SelRoad, W, Off, Arc))
				{
					Net->SetOuterEdgeKnotOffset(SelRoad,
						bSelEdgeRight ? ERoadNetSide::Right : ERoadNetSide::Left, SelEdgeKnot, Off);
					if (InViewportClient) { InViewportClient->Invalidate(); }
				}
			}
		}
		return true;
	}

	if (bWidgetGrabbed && SelPoints.Num() > 0)
	{
		if (URoadNetwork* Net = GetNetwork())
		{
			// Open a drag-spanning transaction on the first delta and capture the
			// pre-move state, so a single Undo reverts the whole (multi-point) drag.
			if (!bDirtyDuringDrag && GEditor)
			{
				GEditor->BeginTransaction(LOCTEXT("RoadNetMovePoints", "Move RoadNet Points"));
				ModifyForEdit();
				bDirtyDuringDrag = true;
			}
			MoveSelectedPoints(InDrag);   // moves the WHOLE selection; rebuild on release
		}
		return true;
	}
	return false;
}

bool FEdModeRoadNet::StartTracking(FEditorViewportClient* InViewportClient, FViewport* InViewport)
{
	// Left-drag on EMPTY space (no transform-widget axis grabbed) in the Points
	// tool begins a marquee box-select of control points. Grabbing the widget
	// (axis != None) falls through to the normal move path.
	if (ActiveTool() == ERoadNetDrawTool::Points && DraftPoints.Num() == 0 && InViewportClient &&
		InViewportClient->GetCurrentWidgetAxis() == EAxisList::None && InViewport)
	{
		bMarquee = true;
		bMarqueeMoved = false;
		MarqueeStart = FIntPoint(InViewport->GetMouseX(), InViewport->GetMouseY());
		MarqueeCur = MarqueeStart;
		return true;
	}
	return FEdMode::StartTracking(InViewportClient, InViewport);
}

bool FEdModeRoadNet::CapturedMouseMove(FEditorViewportClient* InViewportClient, FViewport* InViewport, int32 InMouseX, int32 InMouseY)
{
	if (bMarquee)
	{
		MarqueeCur = FIntPoint(InMouseX, InMouseY);
		// Only treat it as a box once it has meaningfully moved (a tiny jitter on
		// a click shouldn't wipe the selection).
		if (FMath::Abs(MarqueeCur.X - MarqueeStart.X) > 3 || FMath::Abs(MarqueeCur.Y - MarqueeStart.Y) > 3)
		{
			bMarqueeMoved = true;
		}
		if (InViewportClient) { InViewportClient->Invalidate(); }
		return true;
	}
	return FEdMode::CapturedMouseMove(InViewportClient, InViewport, InMouseX, InMouseY);
}

bool FEdModeRoadNet::EndTracking(FEditorViewportClient* InViewportClient, FViewport* InViewport)
{
	if (bMarquee)
	{
		bMarquee = false;
		if (bMarqueeMoved)
		{
			const bool bAdd = InViewport &&
				(InViewport->KeyState(EKeys::LeftShift) || InViewport->KeyState(EKeys::RightShift));
			SelectPointsInMarquee(InViewportClient, bAdd);
		}
		bMarqueeMoved = false;
		if (InViewportClient) { InViewportClient->Invalidate(); }
		return true;
	}
	if (bDirtyDuringDrag)
	{
		bDirtyDuringDrag = false;
		if (URoadNetwork* Net = GetNetwork())
		{
			// Scope the rebuild to the roads the drag actually reshaped: the point
			// selection (may span several roads) plus the selected road (covers a
			// segment-drag and the Edge-tool handle drag). Empty → full rebuild.
			TArray<int32> Moved;
			for (const FIntPoint& S : SelPoints) { Moved.AddUnique(S.X); }
			if (SelRoad != INDEX_NONE) { Moved.AddUnique(SelRoad); }
			Net->Rebuild(Moved);   // windowed when Moved is non-empty
		}
		if (GEditor) { GEditor->EndTransaction(); }
		return true;
	}
	return FEdMode::EndTracking(InViewportClient, InViewport);
}

void FEdModeRoadNet::DrawHUD(FEditorViewportClient* ViewportClient, FViewport* Viewport, const FSceneView* View, FCanvas* Canvas)
{
	FEdMode::DrawHUD(ViewportClient, Viewport, View, Canvas);

	// Persistent top-left label of the active tool so it's always obvious which
	// clicks/hotkeys are live (the panel toggle + 1-5 keys drive this).
	if (Canvas && GEngine && GEngine->GetSmallFont())
	{
		static const TCHAR* Names[] = { TEXT("DRAW"), TEXT("POINTS"), TEXT("LANES"), TEXT("JUNCTIONS"), TEXT("EDGE") };
		static const FLinearColor Cols[] = {
			FLinearColor(0.4f, 1.0f, 0.4f), FLinearColor(0.4f, 0.8f, 1.0f),
			FLinearColor(0.3f, 0.9f, 1.0f), FLinearColor(1.0f, 0.85f, 0.3f),
			FLinearColor(1.0f, 0.6f, 0.3f) };
		const int32 Ti = (int32)ActiveTool();
		Canvas->DrawShadowedString(12.f, 12.f,
			*FString::Printf(TEXT("RoadNet tool: %s   (1-5 or OSM Roads panel to switch)"), Names[Ti]),
			GEngine->GetSmallFont(), Cols[Ti]);
	}

	if (bMarquee && bMarqueeMoved && Canvas)
	{
		const float X = (float)FMath::Min(MarqueeStart.X, MarqueeCur.X);
		const float Y = (float)FMath::Min(MarqueeStart.Y, MarqueeCur.Y);
		const float W = (float)FMath::Abs(MarqueeCur.X - MarqueeStart.X);
		const float H = (float)FMath::Abs(MarqueeCur.Y - MarqueeStart.Y);
		FCanvasBoxItem Box(FVector2D(X, Y), FVector2D(W, H));
		Box.SetColor(FLinearColor(0.35f, 0.8f, 1.0f, 1.0f));
		Box.LineThickness = 1.5f;
		Canvas->DrawItem(Box);
	}
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
	Net->Modify();   // roads live on the network sub-object; capture it for undo

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
	L.SidewalkWidth    = FMath::Max(50.f, Actor->DraftSidewalkWidthCm);

	const int32 NewRoad = Net->AddRoad(R);
	// Windowed: a freshly drawn road only dirties its own footprint (its window
	// picks up any roads it connects to at a junction). Falls back to full if the
	// index is somehow invalid.
	if (NewRoad != INDEX_NONE) { Net->Rebuild(MakeArrayView(&NewRoad, 1)); }
	else                       { Net->Rebuild(); }

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] Draw: committed a hand-drawn road with %d points."), DraftPoints.Num());
	DraftPoints.Reset();
}

bool FEdModeRoadNet::AddParkingBayToActiveSelection(uint8 LayoutInt, FString& OutMsg)
{
	URoadNetwork* Net = GetNetwork();
	if (!Net)
	{
		OutMsg = TEXT("No RoadNet network in the level — import or draw a road first.");
		return false;
	}
	if (SelRoad == INDEX_NONE)
	{
		OutMsg = TEXT("Select a road (and a lane for its side) in the RoadNet mode first.");
		return false;
	}

	// Side from the selected lane (else default to the right side).
	ERoadNetSide Side = ERoadNetSide::Right;
	const TArray<FRoadNetLane> Lanes = Net->GetLanesLeftToRight(SelRoad);
	if (Lanes.IsValidIndex(SelLane))
	{
		Side = (Lanes[SelLane].CenterOffset < 0.0) ? ERoadNetSide::Left : ERoadNetSide::Right;
	}

	const ERoadNetParkingLayout Layout = (ERoadNetParkingLayout)(uint8)FMath::Clamp((int32)LayoutInt, 0, 2);
	const TCHAR* LayoutName =
		(Layout == ERoadNetParkingLayout::Parallel)      ? TEXT("Parallel") :
		(Layout == ERoadNetParkingLayout::Perpendicular) ? TEXT("Perpendicular") : TEXT("Angled");

	const FScopedTransaction Transaction(LOCTEXT("RoadNetParkingBayPanel", "Add RoadNet Parking Bay"));
	ModifyForEdit();
	Net->AddStandardParkingBay(SelRoad, Side, Layout);
	{ const int32 M = SelRoad; Net->Rebuild(MakeArrayView(&M, 1)); }   // windowed

	OutMsg = FString::Printf(TEXT("Added %s parking bay on road %d (%s side)."),
		LayoutName, SelRoad, (Side == ERoadNetSide::Left) ? TEXT("left") : TEXT("right"));
	return true;
}

bool RoadNetEditorBridge::AddParkingBayToActiveSelection(uint8 LayoutInt, FString& OutMsg)
{
	FEdMode* Mode = GLevelEditorModeTools().GetActiveMode(FEdModeRoadNet::GetModeID());
	if (!Mode)
	{
		OutMsg = TEXT("Activate the RoadNet edit mode and select a road first.");
		return false;
	}
	return static_cast<FEdModeRoadNet*>(Mode)->AddParkingBayToActiveSelection(LayoutInt, OutMsg);
}

#undef LOCTEXT_NAMESPACE
