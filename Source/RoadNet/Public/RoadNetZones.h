#pragma once
#include "CoreMinimal.h"

// ===========================================================================
// RoadNetZones — grade-separation layering (§10.12).
//
// Before the boolean union we must NOT merge roads that only cross in 2-D but
// are vertically separated (overpass / underpass). We partition roads into
// "zones" (connected components) where an edge exists between two roads iff:
//   * they share a topology node (endpoint joint), OR
//   * they cross in 2-D at an at-grade point (|dZ| <= MaxZGap and same layer).
// A 2-D crossing with a large dZ (or a bridge/tunnel over a surface road)
// creates NO edge — the two roads land in different zones and are unioned +
// meshed independently, so they visually pass over/under each other.
// ===========================================================================
struct FRoadCurves;
struct FRoadDef;
struct FRoadNetJoint;
struct FRoadNetCrossing;

namespace RoadNetZones
{
	// Partition RoadIndices into grade-separated groups. Curves/Roads are keyed
	// by global road index; Joints supply shared-node connectivity; Crossings are
	// the precomputed 2-D centerline crossings (grade-separation edges).
	ROADNET_API void PartitionLayers(
		const TArray<int32>& RoadIndices,
		const TMap<int32, FRoadCurves>& Curves,
		const TArray<FRoadDef>& Roads,
		const TArray<FRoadNetJoint>& Joints,
		const TArray<FRoadNetCrossing>& Crossings,
		double MaxZGapCm,
		TArray<TArray<int32>>& OutGroups);
}
