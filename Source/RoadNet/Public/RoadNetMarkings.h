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
	// bJointAtStart / bJointAtEnd say whether that end of the road meets a junction.
	// A crossing at a junction mouth must not stripe the direction that is LEAVING
	// that junction — the junction's own approach paint already covers that traffic,
	// and a second bar a few metres past the boundary reads as a double stop line.
	ROADNET_API void BuildRoadMarkings(
		const FRoadDef& Road,
		const FRoadCurves& Curves,
		bool bDriveOnLeft,
		bool bJointAtStart,
		bool bJointAtEnd,
		TArray<UE::Geometry::FGeneralPolygon2d>& OutWhite,
		TArray<UE::Geometry::FGeneralPolygon2d>& OutYellow);

	// Dash an arbitrary polyline into ribbon polygons of total width 2*HalfWCm,
	// cutting DashLenCm of paint every DashLenCm+GapLenCm. Appends to Out.
	//
	// The road markings above dash lines they derive themselves from a lane
	// offset; this takes a path somebody else built. The junction turn guide
	// (LOT"M 809) is the reason it exists: its path is a lane-graph turn
	// movement, which has no lane offset to be derived from.
	ROADNET_API void DashPolyline(
		const TArray<FVector>& Path,
		double DashLenCm, double GapLenCm, double HalfWCm,
		TArray<UE::Geometry::FGeneralPolygon2d>& Out);

	// Drive DashPolyline over the cases that would break a junction turn guide:
	// degenerate inputs that must refuse rather than hang, the dash count over a
	// known length, and paint area roughly matching dash/(dash+gap) of the strip.
	// Run via RoadNet.DashSelfCheck.
	ROADNET_API bool DashSelfCheck(FString& OutReport);
}
