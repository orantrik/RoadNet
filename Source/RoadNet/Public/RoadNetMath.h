#pragma once
#include "CoreMinimal.h"

// ===========================================================================
// RoadNetMath — pure, engine-only geometry math for the road pipeline.
// (ROADBLD_REPLICATION_PLAN.md §10.1–§10.8)
//
// All functions are free/static and side-effect free so they can be unit tested
// headlessly. Units are centimetres; 2-D operations work in the XY plane unless
// noted. No RoadBLD / WorldBLD dependency.
// ===========================================================================
namespace RoadNetMath
{
	// ---- §10.1 Frame -------------------------------------------------------
	// Planar tangent at a polyline vertex (uses neighbouring segments). Falls
	// back to a unit +X if degenerate.
	ROADNET_API FVector2D TangentAt(const TArray<FVector>& Poly, int32 Index);

	// Right axis R = normalize(T x Z) in 2-D → rotate tangent -90° (UE LH, Z up):
	// R = (T.Y, -T.X).
	ROADNET_API FVector2D RightAxis(const FVector2D& Tangent);

	// ---- §10.3 Arc-length --------------------------------------------------
	// Cumulative arc length at each vertex (Length[0] == 0).
	ROADNET_API void CumulativeLength(const TArray<FVector>& Poly, TArray<double>& OutLen);
	ROADNET_API double TotalLength(const TArray<FVector>& Poly);

	// Resample a polyline at fixed spacing (cm). Keeps first & last points and,
	// optionally, high-curvature knots (split where heading change > MaxTurnRad).
	ROADNET_API void ResampleByArcLength(
		const TArray<FVector>& In, double Spacing, TArray<FVector>& Out,
		double MaxTurnRad = 0.0 /* 0 = disabled */);

	// Smooth a polyline with a centripetal Catmull-Rom spline that PASSES THROUGH
	// every input point (so OSM/hand-drawn vertices are preserved as knots), then
	// subdivides each span so no output chord exceeds MaxChordCm. Turns the
	// piecewise-linear centreline into a smooth curve before offsetting/meshing,
	// removing the faceting at every vertex. Z is interpolated the same way.
	// NOTE: Catmull-Rom is only C1 (continuous tangent, DIScontinuous curvature),
	// so curvature jumps at every knot — enough to still facet a wide carriageway.
	ROADNET_API void SmoothCatmullRom(
		const TArray<FVector>& In, double MaxChordCm, TArray<FVector>& Out);

	// Smooth a polyline with a natural cubic spline (C2 → continuous curvature,
	// the clothoid-equivalent for shading) that PASSES THROUGH every input knot.
	// Parametrised by chord length; X/Y/Z fitted independently with natural
	// (zero second-derivative) end conditions, then subdivided so no output chord
	// exceeds MaxChordCm. Preferred over Catmull-Rom: curvature no longer jumps at
	// knots, so offset edges and the meshed surface read smooth on curves.
	ROADNET_API void SmoothG2Spline(
		const TArray<FVector>& In, double MaxChordCm, TArray<FVector>& Out);

	// Smooth the LONGITUDINAL profile (Z vs arc-length) of a polyline into a
	// clean grade, IN PLACE. At each vertex a line z = a + b·s is fit by weighted
	// least squares to every vertex within ±HalfWindowCm of arc length (triangular
	// weights), and Z is replaced by the fitted value at that vertex.
	//
	// Why a *linear* fit: on a constant slope it returns that slope EXACTLY (the
	// bed stays a straight ramp), while averaging out the terrain micro-bumps a
	// draped centreline inherits and the up/down OVERSHOOT a cubic spline adds
	// between sparse knots — those are the "steps"/washboard under the road.
	// X/Y are left untouched (plan geometry unchanged); only the height ramps.
	ROADNET_API void SmoothProfileZ(TArray<FVector>& Poly, double HalfWindowCm);

	// ---- §10.4 Offset ------------------------------------------------------
	// Offset a polyline laterally by SignedOffset (cm, +right). Uses miter joins
	// clamped to MiterLimit*|offset|, falling back to the plain per-vertex offset
	// beyond the limit. 2-D (Z carried from the source vertex).
	ROADNET_API void OffsetPolyline(
		const TArray<FVector>& In, double SignedOffset, TArray<FVector>& Out,
		double MiterLimit = 4.0);

	// ---- §10.5 Projection --------------------------------------------------
	struct FProjectResult
	{
		double Distance   = TNumericLimits<double>::Max(); // |perp| to nearest seg
		double AlongDist  = 0.0;   // arc length from polyline start to the hit
		double Offset     = 0.0;   // signed lateral (+right of travel direction)
		int32  Segment    = INDEX_NONE;
		FVector2D Point   = FVector2D::ZeroVector;
	};
	// Nearest point on a segment [A,B] to Q; t clamped to [0,1] (§10.5).
	ROADNET_API FVector2D ClosestOnSegment(
		const FVector2D& A, const FVector2D& B, const FVector2D& Q, double& OutT);

	// Stable projection of Q onto a polyline (seam-safe tie-break by AlongDist).
	ROADNET_API FProjectResult ProjectToPolyline(const TArray<FVector>& Poly, const FVector2D& Q);

	// ---- §10.6 Segment intersection ---------------------------------------
	// Returns true and fills OutPoint/OutT/OutU if [P1,P2] crosses [P3,P4]
	// within both segments' [0,1] ranges. Parallel/degenerate → false.
	ROADNET_API bool SegmentIntersect2D(
		const FVector2D& P1, const FVector2D& P2,
		const FVector2D& P3, const FVector2D& P4,
		FVector2D& OutPoint, double& OutT, double& OutU, double Eps = 1e-6);

	// ---- §10.8 Corner fillet ----------------------------------------------
	// Given the apex and the two OUTGOING unit directions of the arms, compute the
	// fillet setback t = r / tan(phi/2) and the arc centre for radius r.
	// Returns false if arms are (anti)parallel (phi ~ 0 or pi).
	ROADNET_API bool CornerFillet(
		const FVector2D& Apex, const FVector2D& DirA, const FVector2D& DirB,
		double Radius, double& OutSetback, FVector2D& OutArcCenter, double& OutTurnAngleRad);

	// ---- helpers -----------------------------------------------------------
	// Signed area of a closed 2-D loop (CCW > 0). Does not require last==first.
	ROADNET_API double SignedArea(const TArray<FVector2D>& Loop);

	// 2-D cross product (z of the 3-D cross).
	FORCEINLINE double Cross2D(const FVector2D& A, const FVector2D& B)
	{
		return A.X * B.Y - A.Y * B.X;
	}
}
