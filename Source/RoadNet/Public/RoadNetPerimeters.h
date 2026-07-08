#pragma once
#include "CoreMinimal.h"
#include "Curve/GeneralPolygon2.h"

// ===========================================================================
// RoadNetPerimeters — per-zone perimeter loops for PCG export (§10.11 / §8.4).
//
// The merged boolean-union surface already encodes the network boundary: each
// FGeneralPolygon2d's outer ring is the outline of a connected road+junction
// component, and each hole ring is an enclosed block ("island") boundary.
// We lift those rings to world 3-D (Z sampled from road centerlines) and hand
// them to the commit stage, which realises them as closed USplineComponents —
// the standard, well-supported seam a PCG graph samples (road edges + block
// loops for scattering, façades, medians, etc.).
//
// This is the pragmatic product of the §10.11 half-edge design: the loops PCG
// actually consumes, without the full per-road face graph (a later refinement).
// ===========================================================================
struct FRoadCurves;

// One closed boundary loop in world centimetres.
struct FRoadNetLoop
{
	TArray<FVector> Points;   // closed ring (last != first; the spline closes it)
	bool bOuter = true;       // true = network outline, false = enclosed block hole
	int32 Zone = INDEX_NONE;  // source grade-separation zone
};

namespace RoadNetPerimeters
{
	// Extract loops from per-zone merged surface polygons. CenterLines are looked
	// up per zone from Curves (Zones[z] lists the road indices in that zone) so Z
	// matches the meshed surface. Appends to OutLoops.
	ROADNET_API void ExtractLoops(
		const TArray<TArray<UE::Geometry::FGeneralPolygon2d>>& ZonePolys,
		const TArray<TArray<int32>>& Zones,
		const TMap<int32, FRoadCurves>& Curves,
		double ZLiftCm,
		TArray<FRoadNetLoop>& OutLoops);
}
