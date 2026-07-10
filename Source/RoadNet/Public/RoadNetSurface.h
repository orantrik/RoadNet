#pragma once
#include "CoreMinimal.h"
#include "Curve/GeneralPolygon2.h"

// ===========================================================================
// RoadNetSurface — the boolean-union junction surface (§10.9).
//
// Each road contributes a 2-D outline polygon (between its two outer edges).
// A Clipper2 boolean UNION of all overlapping outlines produces one continuous
// surface where roads meet — junctions become filled automatically, with no
// bespoke junction mesh and no map-spanning "blobs". Uses the engine's
// GeometryAlgorithms (which bundles Clipper2); no RoadBLD dependency.
// ===========================================================================
struct FRoadCurves;

namespace RoadNetSurface
{
	// Build a single road's closed outline polygon from its left/right outer
	// edges. Returns false if degenerate. Output outer ring is CCW.
	ROADNET_API bool BuildRoadOutline(const FRoadCurves& Curves, UE::Geometry::FGeneralPolygon2d& Out);

	// Build a CCW regular-polygon disc approximating a circle (§10.8 fillet).
	ROADNET_API void MakeDisc(const FVector2D& Center, double Radius, int32 Segments,
		UE::Geometry::FGeneralPolygon2d& Out);

	// Union all road outlines (+ any ExtraPolys, e.g. junction fillet discs) into
	// merged surface polygons. InflateEpsilonCm bridges micro-gaps between abutting
	// arms (a "close" morphological op). Returns false only on a hard failure.
	ROADNET_API bool BuildMergedSurface(
		const TArray<const FRoadCurves*>& Curves,
		TArray<UE::Geometry::FGeneralPolygon2d>& OutMerged,
		double InflateEpsilonCm = 5.0,
		const TArray<UE::Geometry::FGeneralPolygon2d>* ExtraPolys = nullptr);

	// Build a closed ribbon polygon between two lateral offsets of a centerline
	// (§8.12 per-road sidewalk side). InnerOff/OuterOff are signed lateral offsets
	// (left = +, right = -) with |OuterOff| > |InnerOff|. Returns false if degenerate.
	ROADNET_API bool BuildSideRibbon(
		const TArray<FVector>& Sampled,
		double InnerOff, double OuterOff,
		UE::Geometry::FGeneralPolygon2d& Out);

	// Build clean strip polygon(s) by thickening an (open) centreline by HalfWidth
	// on each side through Clipper (round joins, butt ends). Unlike BuildSideRibbon
	// — which loops two raw miter offsets and can self-cross on the inside of a
	// bend — Clipper dissolves self-intersections, so tight curves and junction
	// approaches never emit folded/spiky triangles. May return >1 polygon if the
	// strip pinches. Returns false if degenerate.
	// bRoundEnds: cap the strip ends with a semicircle (a rounded "nose",
	// e.g. a raised median island) instead of a flat butt.
	ROADNET_API bool BuildPathRibbon(
		const TArray<FVector>& Center, double HalfWidth,
		TArray<UE::Geometry::FGeneralPolygon2d>& Out,
		double MaxStepsPerRadian = 16.0,
		bool bRoundEnds = false);

	// Build sidewalk bands from per-road, per-side ribbons (§8.12): UNION the
	// ribbons, then subtract the carriageway (RoadPolys) so sidewalks never sit on
	// the road or cross a junction. Only the sides each road actually requests are
	// present. Returns false on hard failure; OutBands filled on success.
	ROADNET_API bool BuildSidewalkBands(
		const TArray<UE::Geometry::FGeneralPolygon2d>& SideRibbons,
		const TArray<UE::Geometry::FGeneralPolygon2d>& RoadPolys,
		TArray<UE::Geometry::FGeneralPolygon2d>& OutBands);

	// Subject MINUS Clip (boolean difference). Used to clear lane markings out of
	// junction areas. Returns false on hard failure (Out left empty).
	ROADNET_API bool Difference(
		const TArray<UE::Geometry::FGeneralPolygon2d>& Subject,
		const TArray<UE::Geometry::FGeneralPolygon2d>& Clip,
		TArray<UE::Geometry::FGeneralPolygon2d>& Out);

	// Boolean union of a polygon set (dissolves overlaps into merged outlines +
	// holes). Used to trace a median island's OUTER perimeter (grass + concrete
	// band) as a single ring so kerbs wrap only the outside, never an inner seam.
	ROADNET_API bool Union(
		const TArray<UE::Geometry::FGeneralPolygon2d>& Polys,
		TArray<UE::Geometry::FGeneralPolygon2d>& Out);

	// Sum of |outer area| across polygons (cm^2) — for logging/QA.
	ROADNET_API double TotalArea(const TArray<UE::Geometry::FGeneralPolygon2d>& Polys);
}
