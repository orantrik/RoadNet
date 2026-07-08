#pragma once
#include "CoreMinimal.h"

// ---------------------------------------------------------------------------
// OSMRoadNetBridge — feeds parsed OSM ways into the independent RoadNet engine
// (Pipeline 4). This is the ONLY coupling point: OSMRoadCore depends on RoadNet,
// never the reverse. RoadNet stays free of any OSM / RoadBLD dependency.
// ---------------------------------------------------------------------------
struct FOSMRoadWay;
class UWorld;

namespace OSMRoadNetBridge
{
	// Map the draped OSM ways to FRoadDef, build a URoadNetwork and run its
	// staged rebuild. Returns false + OutError on failure. OutNumRoads = roads
	// registered. (Geometry commit stages are still under construction — this
	// currently runs curve + topology derivation and logs the pipeline.)
	OSMROADCORE_API bool BuildRoads(
		UWorld* World,
		const TArray<FOSMRoadWay>& Ways,
		int32& OutNumRoads,
		FString& OutError);
}
