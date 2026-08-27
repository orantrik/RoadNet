#pragma once
#include "CoreMinimal.h"

// ===========================================================================
// RoadNetEditorBridge — a tiny public surface so sibling editor tooling (the
// OSM Roads control panel) can drive the RoadNet edit mode without reaching
// into its private headers. Keeps the OSMRoadCore → RoadNetEditor dependency
// one-directional and header-clean.
// ===========================================================================
namespace RoadNetEditorBridge
{
	// Add a standard parking bay to the road currently selected in the active
	// RoadNet edit mode, using LayoutInt (0=Parallel, 1=Perpendicular, 2=Angled)
	// and the selected lane's side. Returns true on success; OutMsg receives a
	// status/error string suitable for logging or an on-screen toast.
	ROADNETEDITOR_API bool AddParkingBayToActiveSelection(uint8 LayoutInt, FString& OutMsg);

	// OSMRoadCore registers a handler that runs the same path as the panel's
	// "Conform Terrain" button. RoadNetEditor fires it (debounced, off the
	// network's rebuild serial) after ANY authoring edit settles, without taking
	// a dependency on OSMRoadCore. The network is already rebuilt when it fires.
	ROADNETEDITOR_API void SetPostPlaceConformHandler(TFunction<void()>&& Handler);
	ROADNETEDITOR_API void ClearPostPlaceConformHandler();
	ROADNETEDITOR_API void NotifyRoadSegmentPlaced();

	// ---- cross-section editor -------------------------------------------
	// The live network and what the edit mode has picked, so a panel widget can
	// mirror and edit that selection without owning it. Null / INDEX_NONE when
	// the mode is not active or nothing is selected.
	//
	// Deliberately pull, not push: the panel re-reads these each tick and
	// compares against the network's rebuild serial. A delegate would have to
	// fire from every path that touches SelRoad — click, marquee, undo, road
	// delete, tool switch — and the one that gets missed leaves the panel
	// editing a road that is no longer there.
	ROADNETEDITOR_API class URoadNetwork* GetActiveNetwork();
	ROADNETEDITOR_API int32 GetSelectedRoad();
	ROADNETEDITOR_API int32 GetSelectedLane();

	// Move the mode's lane highlight, so picking a lane in 2D lights the same
	// ribbon up in the viewport.
	ROADNETEDITOR_API void SelectLaneInViewport(int32 LaneLtoR);

	// Mark the network and its actor modified. Call INSIDE your own
	// FScopedTransaction and BEFORE mutating, so a panel edit lands in undo
	// exactly like a viewport one.
	ROADNETEDITOR_API void ModifyNetworkForEdit();

	// Rebuild just the edited road and redraw. Call after the lane setters.
	ROADNETEDITOR_API void CommitLaneEdit(int32 RoadIdx);
}
