#pragma once
#include "CoreMinimal.h"
#include "Curve/GeneralPolygon2.h"

// ===========================================================================
// RoadNetMarkings — baked lane markings (§8.10 structured markings).
//
// Emits thin ribbon polygons along a road's centerline, split by paint colour
// so they can be meshed as two separately-coloured raised layers:
//   * YELLOW — the centre line dividing opposing traffic, right-hand traffic only.
//   * WHITE  — edge (shoulder) lines, dashed interior lane dividers, and the
//              centre line itself under left-hand (UK) traffic.
// ===========================================================================
struct FRoadCurves;
struct FRoadDef;

namespace RoadNetMarkings
{
	// Append this road's marking ribbons (closed polygons), routed by colour.
	//
	// bDriveOnLeft decides both which side of the road each direction runs on
	// and which colour the centre line takes.
	ROADNET_API void BuildRoadMarkings(
		const FRoadDef& Road,
		const FRoadCurves& Curves,
		bool bDriveOnLeft,
		TArray<UE::Geometry::FGeneralPolygon2d>& OutWhite,
		TArray<UE::Geometry::FGeneralPolygon2d>& OutYellow);
}
