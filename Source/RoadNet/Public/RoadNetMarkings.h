#pragma once
#include "CoreMinimal.h"
#include "Curve/GeneralPolygon2.h"

// ===========================================================================
// RoadNetMarkings — baked lane markings (§8.10 structured markings).
//
// Emits thin ribbon polygons along a road's centerline, split by paint colour
// so they can be meshed as two separately-coloured raised layers:
//   * YELLOW — the centre line dividing opposing traffic (two-way roads only).
//   * WHITE  — edge (shoulder) lines + dashed interior lane dividers.
// ===========================================================================
struct FRoadCurves;
struct FRoadDef;

namespace RoadNetMarkings
{
	// Append this road's marking ribbons (closed polygons), routed by colour:
	// centre line -> OutYellow, edge + lane-divider lines -> OutWhite.
	ROADNET_API void BuildRoadMarkings(
		const FRoadDef& Road,
		const FRoadCurves& Curves,
		TArray<UE::Geometry::FGeneralPolygon2d>& OutWhite,
		TArray<UE::Geometry::FGeneralPolygon2d>& OutYellow);
}
