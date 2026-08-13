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
	// "Conform Terrain" button. RoadNetEditor fires it after a hand-drawn road
	// is committed (FinalizeDraft) without taking a dependency on OSMRoadCore.
	ROADNETEDITOR_API void SetPostPlaceConformHandler(TFunction<void()>&& Handler);
	ROADNETEDITOR_API void ClearPostPlaceConformHandler();
	ROADNETEDITOR_API void NotifyRoadSegmentPlaced();
}
