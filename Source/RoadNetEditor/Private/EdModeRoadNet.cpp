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

// On-screen width of the draw-time road ghost, in PIXELS. The ghost is drawn
// with screen-space thickness so it holds this width at any camera distance —
// the old world-space 1.5-2.5 cm lines were sub-pixel hairlines at city zoom.
static TAutoConsoleVariable<float> CVarRoadNetGhostThickness(
	TEXT("roadnet.GhostThickness"),
	3.0f,
	TEXT("Draw-time road ghost line width in screen pixels (the halo behind each line is 2 px wider). Default 3."),
	ECVF_Default);

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
	// Draw-time road ghost palette. Every ghost line is stroked twice — a wider
	// near-black halo, then the bright line — so it reads against dark asphalt
	// and against pale untextured terrain equally.
	const FColor     kColorGhostHalo   = FColor(8, 8, 12);
	const FColor     kColorGhostCurb   = FColor(255, 255, 255);
	const FColor     kColorGhostLane   = FColor(215, 228, 245);
	const FColor     kColorGhostCentre = FColor(255, 210, 40);
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
	// Baseline the conform watch on whatever the network already is, so entering
	// the mode does not re-sculpt terrain that a prior import already conformed.
	bConformPending = false;
	LastConformSerial = GetNetwork() ? GetNetwork()->GetRebuildSerial() : 0;
	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] Draw mode: pick a TOOL (OSM Roads panel or keys 1-5): 1 Draw, 2 Points, 3 Lanes, 4 Junctions, 5 Edge. Each tool scopes its own clicks/hotkeys. Full list: 'OSM Roads' panel > Legend tab."));
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Cyan,
			TEXT("RoadNet: pick a TOOL — OSM Roads panel or keys 1-5:  1 Draw  |  2 Points  |  3 Lanes  |  4 Junctions  |  5 Edge\n"
			     "Draw: LMB adds points, Enter/RMB finish.   Points: LMB select, marquee, drag=move, Del delete, Ctrl+LMB split, U merge, P edit-all.\n"
			     "Lanes: click a lane, =/- add/remove, Shift+= / Shift+- insert beside it, B cycle type (bike/parking).\n"
			     "Junctions: J mark, K island, [ ] smoothing, M median (Shift+M edge, , . width).   Edge: drag outer-edge vertices; P = parking bay left+right of selected point.\n"
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

ERoadNetDrawShape FEdModeRoadNet::ActiveShape() const
{
	static IConsoleVariable* CV = IConsoleManager::Get().FindConsoleVariable(TEXT("roadnet.DrawShape"));
	const int32 V = CV ? CV->GetInt() : 0;
	return (ERoadNetDrawShape)(uint8)FMath::Clamp(V, 0, 3);
}

double FEdModeRoadNet::DrawAngleRad() const
{
	static IConsoleVariable* CV = IConsoleManager::Get().FindConsoleVariable(TEXT("roadnet.DrawAngleDeg"));
	const int32 Deg = CV ? CV->GetInt() : 90;
	return FMath::DegreesToRadians((double)FMath::Clamp(Deg, 2, 175));
}

void FEdModeRoadNet::DrawRoadGhost(FPrimitiveDrawInterface* PDI, const TArray<FVector>& Center) const
{
	if (!PDI || Center.Num() < 2) { return; }

	// Layout from the draft settings the finished road will use (fallback ~2
	// lanes if there is no actor yet).
	int32  LaneCount = 2;
	double LaneWidth = 350.0;
	double WalkWidth = 200.0;
	if (const ARoadNetActor* A = NetActorPtr.Get())
	{
		LaneCount = FMath::Max(1, A->DraftLaneCount);
		LaneWidth = FMath::Max(150.f, A->DraftLaneWidthCm);
		WalkWidth = A->bDraftSidewalks ? FMath::Max(50.f, A->DraftSidewalkWidthCm) : 0.0;
	}
	const double CarriageHalf = 0.5 * LaneCount * LaneWidth;
	const double Half         = CarriageHalf + WalkWidth;

	// Carriageway boundaries: the two curbs plus every lane divider.
	TArray<double> Bounds;
	Bounds.Reserve(LaneCount + 1);
	for (int32 k = 0; k <= LaneCount; ++k) { Bounds.Add(-CarriageHalf + k * LaneWidth); }
	// Self-check: the ladder must be symmetric about the centreline, strictly
	// increasing, and have exactly one more boundary than there are lanes.
	ensureMsgf(Bounds.Num() == LaneCount + 1, TEXT("[RoadNet] ghost lane ladder: %d boundaries for %d lanes"),
		Bounds.Num(), LaneCount);
	for (int32 k = 0; k <= LaneCount; ++k)
	{
		ensureMsgf(k == 0 || Bounds[k] > Bounds[k - 1], TEXT("[RoadNet] ghost lane ladder not increasing at %d"), k);
		ensureMsgf(FMath::IsNearlyEqual(Bounds[k], -Bounds[LaneCount - k], 0.01),
			TEXT("[RoadNet] ghost lane ladder not symmetric at %d"), k);
	}

	// Lift clear of the ground: the ghost is traced ON the landscape, so at the
	// sampled elevation it z-fights and reads as a broken hairline.
	constexpr double kLiftCm = 25.0;
	const float Thick = FMath::Max(0.5f, CVarRoadNetGhostThickness.GetValueOnAnyThread());

	auto Stroke = [&](const TArray<FVector>& Poly, const FColor& Colour, float Scale)
	{
		const float Wide = Thick * Scale;
		for (int32 i = 1; i < Poly.Num(); ++i)
		{
			const FVector A(Poly[i - 1].X, Poly[i - 1].Y, Poly[i - 1].Z + kLiftCm);
			const FVector B(Poly[i].X,     Poly[i].Y,     Poly[i].Z     + kLiftCm);
			PDI->DrawLine(A, B, kColorGhostHalo, SDPG_Foreground, Wide + 2.f, 0.f, /*bScreenSpace*/ true);
			PDI->DrawLine(A, B, Colour,          SDPG_Foreground, Wide,       0.f, /*bScreenSpace*/ true);
		}
	};

	TArray<FVector> Right, Left, Poly;
	RoadNetMath::OffsetPolyline(Center, +Half, Right);
	RoadNetMath::OffsetPolyline(Center, -Half, Left);

	// Outer footprint (the sidewalk edge), only when it is not the curb itself.
	if (WalkWidth > 1.0)
	{
		Stroke(Right, kColorLine, 1.0f);
		Stroke(Left,  kColorLine, 1.0f);
	}

	// Curbs, then the lane dividers — the centre divider styled yellow so the
	// ghost matches the built road's palette.
	for (int32 k = 0; k <= LaneCount; ++k)
	{
		const bool bCurb   = (k == 0 || k == LaneCount);
		const bool bCentre = !bCurb && FMath::IsNearlyZero(Bounds[k], 1.0);
		RoadNetMath::OffsetPolyline(Center, Bounds[k], Poly);
		Stroke(Poly,
			bCurb ? kColorGhostCurb : (bCentre ? kColorGhostCentre : kColorGhostLane),
			bCurb ? 1.0f            : (bCentre ? 0.85f             : 0.6f));
	}

	// The draft path itself, on top.
	Stroke(Center, kColorPreview, 0.6f);

	// Cross rungs, dimmer now that the lane lines convey the surface (capped so
	// long freehand roads stay cheap). Right/Left carry one vertex per Center.
	const int32 N = FMath::Min3(Center.Num(), Right.Num(), Left.Num());
	const int32 Step = FMath::Max(1, N / 32);
	for (int32 i = 0; i < N; i += Step)
	{
		const FVector A(Left[i].X,  Left[i].Y,  Left[i].Z  + kLiftCm);
		const FVector B(Right[i].X, Right[i].Y, Right[i].Z + kLiftCm);
		PDI->DrawLine(A, B, kColorGhostLane, SDPG_Foreground, Thick * 0.5f, 0.f, /*bScreenSpace*/ true);
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

// How long an adjustment keeps its target after the last keystroke. Long enough
// to tap ',' four times in a row without losing the road, short enough that you
// never wonder why the next key went somewhere unexpected.
static TAutoConsoleVariable<float> CVarRoadNetAutoReleaseSec(
	TEXT("roadnet.AutoReleaseSec"),
	1.5f,
	TEXT("Idle seconds before a repeat adjustment (median/sidewalk width, junction smoothing, lane type) releases the road it was acting on. 0 disables auto-release for those; one-shot placements always release immediately."),
	ECVF_Default);

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

	// Release the road an adjustment was acting on once the user has stopped
	// nudging it, so the next keystroke does not land on a stale target. Never
	// mid-gesture: a live drag or an unfinished draft still owns the selection.
	if (bAutoReleasePending)
	{
		const double IdleSec = (double)CVarRoadNetAutoReleaseSec.GetValueOnGameThread();
		const bool bBusy = bDirtyDuringDrag || bMarquee || DraftPoints.Num() > 0;
		if (IdleSec <= 0.0)
		{
			bAutoReleasePending = false;
		}
		else if (!bBusy && FPlatformTime::Seconds() - AutoReleaseArmTime >= IdleSec)
		{
			bAutoReleasePending = false;
			AutoRelease(ViewportClient);
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

	TickTerrainConform();
}

// Every authoring edit reshapes the road footprint, so the landscape underneath
// it has to be re-ramped -- not just the Draw tool's commit. Rather than adding
// a notify next to each of the ~17 Net->Rebuild() call sites in this file (and
// relying on every future one remembering), watch the network's rebuild serial:
// one place that cannot be bypassed by a new edit path.
//
// Debounced because the conform is a heightmap write: holding '-' to strip lanes
// or dragging a point rebuilds repeatedly, and only the settled shape matters.
void FEdModeRoadNet::TickTerrainConform()
{
	URoadNetwork* Net = GetNetwork();
	if (!Net) { return; }

	const uint32 Serial = Net->GetRebuildSerial();
	if (Serial != LastConformSerial)
	{
		LastConformSerial = Serial;
		ConformLastRebuildTime = FPlatformTime::Seconds();
		bConformPending = true;
	}
	if (!bConformPending) { return; }

	// Never sculpt mid-gesture: a draft in progress or a live widget drag will
	// rebuild again in a moment, and a queued smoothing rebuild owns its own
	// debounce (its rebuild re-arms this one).
	if (DraftPoints.Num() > 0 || bDirtyDuringDrag || bMarquee || bSmoothingRebuildPending) { return; }

	constexpr double kConformDebounceSec = 0.4;
	if (FPlatformTime::Seconds() - ConformLastRebuildTime < kConformDebounceSec) { return; }

	bConformPending = false;
	// No-op unless OSMRoadCore is loaded and registered the sculpt handler.
	RoadNetEditorBridge::NotifyRoadSegmentPlaced();
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
	bAutoReleasePending = false;
}

void FEdModeRoadNet::AutoRelease(FEditorViewportClient* ViewportClient)
{
	// A queued smoothing rebuild is owned by the junction we are about to let go
	// of, so commit it rather than strand it.
	FlushPendingSmoothing(ViewportClient);
	ClearSelection();
	if (ViewportClient) { ViewportClient->Invalidate(); }
}

void FEdModeRoadNet::ArmAutoRelease()
{
	bAutoReleasePending = true;
	AutoReleaseArmTime = FPlatformTime::Seconds();
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

void FEdModeRoadNet::SelectLaneFromPanel(int32 LaneLtoR)
{
	if (SelRoad == INDEX_NONE) { return; }
	SelectLaneOnRoad(SelRoad, LaneLtoR);
	if (GEditor) { GEditor->RedrawLevelEditingViewports(); }
}

void FEdModeRoadNet::CommitLaneEditFromPanel(int32 RoadIdx)
{
	URoadNetwork* Net = GetNetwork();
	if (!Net || RoadIdx == INDEX_NONE) { return; }

	const int32 Modified[] = { RoadIdx };
	Net->Rebuild(Modified);
	// The rebuild can reorder lanes (a width change moves every offset), so the
	// highlight has to come back from the id rather than from the stale index.
	ResolveSelLaneFromId();
	// Same settle path a viewport edit takes, so the terrain conform runs after
	// a cross-section change too rather than only after a drag in the viewport.
	RoadNetEditorBridge::NotifyRoadSegmentPlaced();
	if (GEditor) { GEditor->RedrawLevelEditingViewports(); }
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
		const TArray<int32>& Pts = ByRoad[r];
		// Deleting a run of points BREAKS the road at that gap (two roads) rather
		// than bridging across it — this is the "cut a hole here" gesture. New
		// pieces are appended, so processing roads highest-index-first keeps the
		// pending lower-index deletes valid.
		Removed += Pts.Num();
		Net->DeleteRoadPointsSplitting(r, Pts);
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
	// Resolve the actor first — NetActorPtr can be stale until GetOrCreate runs.
	if (ARoadNetActor* Actor = GetOrSpawnNetActor())
	{
		Actor->Modify();
		if (URoadNetwork* Net = Actor->GetNetwork())
		{
			Net->BeginAuthoringEdit();
			Net->Modify();
		}
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
				// Fixed-click primitives commit automatically on their last anchor:
				// Roundabout/Curve = 2 clicks (centre/start + radius/end), FreeCurve
				// = 3 clicks (origin + destination + apex that sets the bow).
				const ERoadNetDrawShape Shape = ActiveShape();
				const int32 Need =
					(Shape == ERoadNetDrawShape::Roundabout || Shape == ERoadNetDrawShape::Curve) ? 2 :
					(Shape == ERoadNetDrawShape::FreeCurve) ? 3 : MAX_int32;
				if (DraftPoints.Num() >= Need)
				{
					FinalizeDraft();
				}
				else if (InViewportClient) { InViewportClient->Invalidate(); }
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
			UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet][DEL] Delete pressed: Tool=%d ShowAllPoints=%d SelPoints=%d SelRoad=%d SelPoint=%d"),
				(int32)Tool, bShowAllPoints ? 1 : 0, SelPoints.Num(), SelRoad, SelPoint);
			if (Tool != ERoadNetDrawTool::Points) { return true; }
			// Multi-point delete: if a set of control points is selected (marquee
			// or Shift+click), delete them all in one undoable step.
			if (SelPoints.Num() > 0)
			{
				if (URoadNetwork* Net = GetNetwork())
				{
					const int32 RoadsBefore = Net->GetRoads().Num();
					const FScopedTransaction Transaction(LOCTEXT("RoadNetDeletePoints", "Delete RoadNet Points"));
					ModifyForEdit();
					const int32 N = DeleteSelectedPoints();
					const int32 RoadsAfter = Net->GetRoads().Num();
					UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet][DEL] Multi-point delete: removed %d point(s); roads %d -> %d; FULL rebuild"),
						N, RoadsBefore, RoadsAfter);
					Net->Rebuild();
					if (GEngine)
					{
						GEngine->AddOnScreenDebugMessage(-1, 2.5f, FColor::Cyan,
							FString::Printf(TEXT("RoadNet: cut %d point(s) — roads %d -> %d (a run of interior points breaks the road here)"),
								N, RoadsBefore, RoadsAfter));
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
						UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet][DEL] Idle-pick: trace hit; nearest point r=%d i=%d (d2=%.0f), nearest road r=%d (d2=%.0f) -> DelRoad=%d DelPoint=%d"),
							PtRoad, PtIdx, BestPtD2, RdRoad, BestRdD2, DelRoad, DelPoint);
					}
				}
				else
				{
					UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet][DEL] Idle-pick: LineTraceCursor MISS (no ground/road under cursor)"));
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
						const bool bDeleted = Net->DeleteRoadPoint(DelRoad, DelPoint, bRoadRemoved);
						UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet][DEL] Single-point delete r=%d p=%d -> %s (roadRemoved=%d); %s rebuild"),
							DelRoad, DelPoint, bDeleted ? TEXT("OK") : TEXT("FAILED"), bRoadRemoved ? 1 : 0,
							bRoadRemoved ? TEXT("FULL") : TEXT("WINDOWED"));
						if (bDeleted)
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
						UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet][DEL] Whole-road delete r=%d (no point resolved); FULL rebuild"), DelRoad);
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

			UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet][DEL] NOTHING resolved to delete (SelPoints=0, no pick). ShowAllPoints=%d"), bShowAllPoints ? 1 : 0);
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

			// Standard parking bay (Edge tool, draft not in progress):
			//   'P'       → centre a bay on the selected edge point, LEFT and RIGHT.
			//               Layout from roadnet.ParkingLayout (0=Parallel 1=Perp 2=Angled).
			//   'Shift+P' → clear all standard parking bays from the road.
			if (Tool == ERoadNetDrawTool::Edge && Key == EKeys::P)
			{
				URoadNetwork* Net = GetNetwork();
				if (!Net) { return true; }

				int32 Target = SelRoad;
				if (Target == INDEX_NONE)
				{
					if (GEngine)
					{
						GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Orange,
							TEXT("RoadNet: Edge tool → select an edge point → P to add parking bays left+right"));
					}
					return true;
				}

				const bool bShiftP = Viewport &&
					(Viewport->KeyState(EKeys::LeftShift) || Viewport->KeyState(EKeys::RightShift));

				// Resolve everything BEFORE opening a transaction so Ctrl+Z only
				// undoes a completed parking-bay edit.
				double CenterArc = 0.0;
				ERoadNetParkingLayout Layout = ERoadNetParkingLayout::Parallel;
				const TCHAR* LayoutName = TEXT("Parallel");
				if (!bShiftP)
				{
					if (SelEdgeKnot == INDEX_NONE)
					{
						if (GEngine)
						{
							GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Orange,
								TEXT("RoadNet: select an edge point, then P (or Add Parking Bay)"));
						}
						return true;
					}
					TArray<FRoadNetEdgeKnot> Profile;
					Net->GetOuterEdgeForDisplay(SelRoad,
						bSelEdgeRight ? ERoadNetSide::Right : ERoadNetSide::Left, Profile);
					if (!Profile.IsValidIndex(SelEdgeKnot))
					{
						if (GEngine)
						{
							GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Orange,
								TEXT("RoadNet: select a valid edge point, then P"));
						}
						return true;
					}
					CenterArc = Profile[SelEdgeKnot].Distance;

					int32 LayoutInt = 0;
					if (IConsoleVariable* CV = IConsoleManager::Get().FindConsoleVariable(TEXT("roadnet.ParkingLayout")))
					{
						LayoutInt = FMath::Clamp(CV->GetInt(), 0, 2);
					}
					Layout = (ERoadNetParkingLayout)(uint8)LayoutInt;
					LayoutName =
						(Layout == ERoadNetParkingLayout::Parallel)      ? TEXT("Parallel") :
						(Layout == ERoadNetParkingLayout::Perpendicular) ? TEXT("Perpendicular") : TEXT("Angled");
				}

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
					Net->AddStandardParkingBay(Target, ERoadNetSide::Left,  Layout, CenterArc);
					Net->AddStandardParkingBay(Target, ERoadNetSide::Right, Layout, CenterArc);
					Msg = FString::Printf(TEXT("RoadNet: added %s parking bays left+right on road %d at arc %.0f cm"),
						LayoutName, Target, CenterArc);
				}

				{ const int32 M = Target; Net->Rebuild(MakeArrayView(&M, 1)); }
				if (GEngine) { GEngine->AddOnScreenDebugMessage(-1, 2.5f, FColor::Cyan, Msg); }
				// The bay is placed; let go of the edge point it was anchored to.
				AutoRelease(ViewportClient);
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
				ArmAutoRelease();
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
				// One-shot: the preset is applied, so drop any road context. The
				// junction itself is picked under the cursor, not held.
				AutoRelease(ViewportClient);
				return true;
			}

			// Mid-block pedestrian crossing (Junctions tool):
			//   'C' → add a zebra crossing wherever the cursor is on a road, or
			//         remove the one already there. Junction zebras come from the
			//         junction preset; this is for the crossing outside a school
			//         that sits nowhere near an intersection.
			if (Tool == ERoadNetDrawTool::Junctions && Key == EKeys::C)
			{
				URoadNetwork* Net = GetNetwork();
				if (!Net) { return true; }

				FVector Hit;
				if (!ViewportClient || !LineTraceCursor(ViewportClient, Hit))
				{
					if (GEngine)
					{
						GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Orange,
							TEXT("RoadNet: hover a road, then press C to place a pedestrian crossing"));
					}
					return true;
				}

				const FScopedTransaction Transaction(LOCTEXT("RoadNetCrossing", "Edit RoadNet Crossing"));
				ModifyForEdit();

				bool bAdded = false;
				const int32 Road = Net->ToggleCrossingNear(FVector2D(Hit.X, Hit.Y), 2000.0, bAdded);
				if (Road == INDEX_NONE)
				{
					if (GEngine)
					{
						GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Orange,
							TEXT("RoadNet: no road under cursor — hover a road, then press C"));
					}
					return true;
				}

				{ const int32 M = Road; Net->Rebuild(MakeArrayView(&M, 1)); }
				if (GEngine)
				{
					GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan,
						FString::Printf(TEXT("RoadNet: crossing %s on road %d  ( press C on it again to remove )"),
							bAdded ? TEXT("added") : TEXT("removed"), Road));
				}
				// One-shot placement: nothing to keep hold of.
				AutoRelease(ViewportClient);
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
				AutoRelease(ViewportClient);
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

				// Keep the road so ',' / '.' can be tapped again, but start the
				// idle timer that hands it back.
				SelRoad = Target;
				SelPoint = INDEX_NONE;
				ArmAutoRelease();
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
				ArmAutoRelease();
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
		HoverRaw = Hit;
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

			// Ghost insert marker (Points tool, Ctrl held): show exactly where a
			// Ctrl+click would drop a new point. Inserting mid-span has worked
			// since the tool shipped, but nothing on screen said so, so nobody
			// found it — this is the discoverability, not the feature.
			if (Tool == ERoadNetDrawTool::Points && bHasHover && Viewport &&
				(Viewport->KeyState(EKeys::LeftControl) || Viewport->KeyState(EKeys::RightControl)))
			{
				FVector Best = FVector::ZeroVector;
				FVector SegA = FVector::ZeroVector, SegB = FVector::ZeroVector;
				double BestD2 = FMath::Square(2000.0);   // 20 m pick radius, as elsewhere
				for (const FRoadDef& Road : Roads)
				{
					for (int32 i = 0; i + 1 < Road.Ref.Num(); ++i)
					{
						const FVector C = FMath::ClosestPointOnSegment(HoverRaw, Road.Ref[i], Road.Ref[i + 1]);
						const double D2 = FVector::DistSquaredXY(HoverRaw, C);
						if (D2 < BestD2) { BestD2 = D2; Best = C; SegA = Road.Ref[i]; SegB = Road.Ref[i + 1]; }
					}
				}
				if (BestD2 < FMath::Square(2000.0))
				{
					const FVector Lift(0, 0, 25.f);
					PDI->DrawLine(SegA + Lift, SegB + Lift, kColorSnap, SDPG_Foreground, 3.f);
					PDI->DrawPoint(Best + Lift, kColorSnap, kPointSize + 6.f, SDPG_Foreground);
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

	// Build the preview centreline for the active shape/stage, then draw it as a
	// road-width ghost so the draft shows the footprint of the finished road.
	const ERoadNetDrawShape Shape = ActiveShape();
	TArray<FVector> Preview;
	if (Shape == ERoadNetDrawShape::Roundabout && DraftPoints.Num() == 1 && bHasHover)
	{
		const FVector2D C(DraftPoints[0].X, DraftPoints[0].Y);
		const double R = FVector2D::Distance(C, FVector2D(HoverPoint.X, HoverPoint.Y));
		RoadNetMath::SampleCircle(C, R, DraftPoints[0].Z, 48, Preview);
	}
	else if (Shape == ERoadNetDrawShape::Curve && DraftPoints.Num() == 1 && bHasHover)
	{
		RoadNetMath::SampleArc(DraftPoints[0], HoverPoint, DrawAngleRad(), /*bBulgeLeft*/ true, Preview);
	}
	else if (Shape == ERoadNetDrawShape::FreeCurve && bHasHover)
	{
		if (DraftPoints.Num() == 1) { Preview = { DraftPoints[0], HoverPoint }; }
		else if (DraftPoints.Num() >= 2)
		{
			// Stage 3 "set curve": the cursor is the apex the Bezier passes through.
			const FVector A = DraftPoints[0], B = DraftPoints[1];
			const FVector Ctrl = 2.0 * HoverPoint - 0.5 * (A + B);
			RoadNetMath::SampleQuadBezier(A, Ctrl, B, Preview);
		}
	}
	else if (Shape == ERoadNetDrawShape::Freehand)
	{
		Preview = DraftPoints;
		if (DraftPoints.Num() > 0 && bHasHover) { Preview.Add(HoverPoint); }
	}

	// Anchor dots so placed clicks stay visible over the ghost.
	for (const FVector& P : DraftPoints)
	{
		PDI->DrawPoint(P, kColorPoint, kPointSize, SDPG_Foreground);
	}
	if (Preview.Num() >= 2) { DrawRoadGhost(PDI, Preview); }
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

	// Resolve the shape into a centreline. Freehand uses the clicked points as-is;
	// Roundabout builds a closed circle (centre + radius clicks); Curve builds a
	// circular arc bending by the panel angle between the two clicks.
	const ERoadNetDrawShape Shape = ActiveShape();
	TArray<FVector> Ref;
	if (Shape == ERoadNetDrawShape::Roundabout)
	{
		const FVector2D C(DraftPoints[0].X, DraftPoints[0].Y);
		const double Radius = FVector2D::Distance(C, FVector2D(DraftPoints[1].X, DraftPoints[1].Y));
		if (Radius < 200.0) { DraftPoints.Reset(); return; }   // too small to be a road
		const int32 Segs = FMath::Clamp(FMath::RoundToInt(Radius / 100.0), 16, 96);
		RoadNetMath::SampleCircle(C, Radius, DraftPoints[0].Z, Segs, Ref);
	}
	else if (Shape == ERoadNetDrawShape::Curve)
	{
		RoadNetMath::SampleArc(DraftPoints[0], DraftPoints[1], DrawAngleRad(), /*bBulgeLeft*/ true, Ref);
	}
	else if (Shape == ERoadNetDrawShape::FreeCurve)
	{
		const FVector A = DraftPoints[0];
		const FVector B = DraftPoints[1];
		if (DraftPoints.Num() >= 3)
		{
			// Control point placed so the Bezier passes through the apex click at t=0.5.
			const FVector Ctrl = 2.0 * DraftPoints[2] - 0.5 * (A + B);
			RoadNetMath::SampleQuadBezier(A, Ctrl, B, Ref);
		}
		else { Ref = { A, B }; }   // apex not set (Enter early) -> straight span
	}
	else
	{
		Ref = DraftPoints;
	}
	if (Ref.Num() < 2) { DraftPoints.Reset(); return; }

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
	R.Ref    = MoveTemp(Ref);

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

	UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] Draw: committed a hand-drawn road (shape %d) with %d points."),
		(int32)Shape, R.Ref.Num());
	DraftPoints.Reset();
	// The terrain conform is picked up from the rebuild serial in TickTerrainConform.
}

bool FEdModeRoadNet::AddParkingBayToActiveSelection(uint8 LayoutInt, FString& OutMsg)
{
	URoadNetwork* Net = GetNetwork();
	if (!Net)
	{
		// Mode may be active before the actor pointer is cached.
		if (ARoadNetActor* Actor = GetOrSpawnNetActor()) { Net = Actor->GetNetwork(); }
	}
	if (!Net)
	{
		OutMsg = TEXT("No RoadNet network in the level — import or draw a road first.");
		return false;
	}
	if (ActiveTool() != ERoadNetDrawTool::Edge)
	{
		OutMsg = TEXT("Switch to the Edge tool (key 5), select an edge point, then Add Parking Bay.");
		return false;
	}
	if (SelRoad == INDEX_NONE || SelEdgeKnot == INDEX_NONE)
	{
		OutMsg = TEXT("Select an Edge-tool point first — the bay is centred on that point, left and right.");
		return false;
	}

	// Arc length of the selected edge handle — bay window is centred here.
	TArray<FRoadNetEdgeKnot> Profile;
	Net->GetOuterEdgeForDisplay(SelRoad,
		bSelEdgeRight ? ERoadNetSide::Right : ERoadNetSide::Left, Profile);
	if (!Profile.IsValidIndex(SelEdgeKnot))
	{
		OutMsg = TEXT("Selected edge point is invalid — click an edge handle again.");
		return false;
	}
	const double CenterArc = Profile[SelEdgeKnot].Distance;

	const ERoadNetParkingLayout Layout = (ERoadNetParkingLayout)(uint8)FMath::Clamp((int32)LayoutInt, 0, 2);
	const TCHAR* LayoutName =
		(Layout == ERoadNetParkingLayout::Parallel)      ? TEXT("Parallel") :
		(Layout == ERoadNetParkingLayout::Perpendicular) ? TEXT("Perpendicular") : TEXT("Angled");

	// Validate first, THEN open the transaction so Ctrl+Z only undoes a real add.
	const FScopedTransaction Transaction(LOCTEXT("RoadNetParkingBayPanel", "Add RoadNet Parking Bay"));
	ModifyForEdit();
	// One bay on each kerb, both centred on the selected edge point.
	Net->AddStandardParkingBay(SelRoad, ERoadNetSide::Left,  Layout, CenterArc);
	Net->AddStandardParkingBay(SelRoad, ERoadNetSide::Right, Layout, CenterArc);
	{ const int32 M = SelRoad; Net->Rebuild(MakeArrayView(&M, 1)); }   // windowed

	OutMsg = FString::Printf(TEXT("Added %s parking bays left+right on road %d at arc %.0f cm."),
		LayoutName, SelRoad, CenterArc);
	return true;
}

namespace
{
	// The RoadNet edit mode when it is the active one, else null. Every bridge
	// entry point goes through this, so "the mode is not open" degrades to an
	// empty answer rather than to a crash.
	FEdModeRoadNet* ActiveRoadNetMode()
	{
		FEdMode* Mode = GLevelEditorModeTools().GetActiveMode(FEdModeRoadNet::GetModeID());
		return Mode ? static_cast<FEdModeRoadNet*>(Mode) : nullptr;
	}
}

bool RoadNetEditorBridge::AddParkingBayToActiveSelection(uint8 LayoutInt, FString& OutMsg)
{
	FEdModeRoadNet* Mode = ActiveRoadNetMode();
	if (!Mode)
	{
		OutMsg = TEXT("Activate the RoadNet edit mode and select a road first.");
		return false;
	}
	return Mode->AddParkingBayToActiveSelection(LayoutInt, OutMsg);
}

URoadNetwork* RoadNetEditorBridge::GetActiveNetwork()
{
	FEdModeRoadNet* Mode = ActiveRoadNetMode();
	return Mode ? Mode->GetNetworkForPanel() : nullptr;
}

int32 RoadNetEditorBridge::GetSelectedRoad()
{
	FEdModeRoadNet* Mode = ActiveRoadNetMode();
	return Mode ? Mode->GetSelectedRoadForPanel() : INDEX_NONE;
}

int32 RoadNetEditorBridge::GetSelectedLane()
{
	FEdModeRoadNet* Mode = ActiveRoadNetMode();
	return Mode ? Mode->GetSelectedLaneForPanel() : INDEX_NONE;
}

void RoadNetEditorBridge::SelectLaneInViewport(int32 LaneLtoR)
{
	if (FEdModeRoadNet* Mode = ActiveRoadNetMode())
	{
		Mode->SelectLaneFromPanel(LaneLtoR);
	}
}

void RoadNetEditorBridge::ModifyNetworkForEdit()
{
	if (FEdModeRoadNet* Mode = ActiveRoadNetMode())
	{
		Mode->ModifyForEdit();
	}
}

void RoadNetEditorBridge::CommitLaneEdit(int32 RoadIdx)
{
	if (FEdModeRoadNet* Mode = ActiveRoadNetMode())
	{
		Mode->CommitLaneEditFromPanel(RoadIdx);
	}
}

static TFunction<void()> GPostPlaceConformHandler;

void RoadNetEditorBridge::SetPostPlaceConformHandler(TFunction<void()>&& Handler)
{
	GPostPlaceConformHandler = MoveTemp(Handler);
}

void RoadNetEditorBridge::ClearPostPlaceConformHandler()
{
	GPostPlaceConformHandler = nullptr;
}

void RoadNetEditorBridge::NotifyRoadSegmentPlaced()
{
	if (GPostPlaceConformHandler)
	{
		GPostPlaceConformHandler();
	}
}

#undef LOCTEXT_NAMESPACE
