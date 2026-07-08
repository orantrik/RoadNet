#pragma once
#include "CoreMinimal.h"
#include "Curve/GeneralPolygon2.h"

// ===========================================================================
// RoadNetMesh — triangulate the merged 2-D surface into a 3-D FDynamicMesh3
// (§10.15). Elevation is re-projected from the road centerlines: each vertex
// is projected onto the nearest centerline and its Z interpolated ALONG the
// hit segment (§10.10), so the surface follows road grade smoothly instead of
// snapping to the nearest vertex.
//
// AppendSurfaceMesh is called once per grade-separation zone (§10.12) so each
// zone samples only its own centerlines; the caller finalizes normals after.
// ===========================================================================
namespace UE::Geometry { class FDynamicMesh3; }

namespace RoadNetMesh
{
	// Triangulate Polys and APPEND into OutMesh (does not clear). Each output
	// vertex takes the interpolated Z of the nearest CenterLine (XY projection),
	// plus ZLiftCm. Returns the number of triangles appended.
	ROADNET_API int32 AppendSurfaceMesh(
		const TArray<UE::Geometry::FGeneralPolygon2d>& Polys,
		const TArray<const TArray<FVector>*>& CenterLines,
		double ZLiftCm,
		UE::Geometry::FDynamicMesh3& OutMesh);

	// Enable attributes and compute per-vertex normals (call once, after all
	// zones have been appended). No-op on an empty mesh.
	ROADNET_API void FinalizeNormals(UE::Geometry::FDynamicMesh3& Mesh);

	// Interpolated elevation at (X,Y): project onto every centerline, keep the
	// nearest, lerp Z along that segment (§10.10). Shared by the mesher and the
	// perimeter-loop exporter so both agree on road grade.
	ROADNET_API double SampleHeight(
		const TArray<const TArray<FVector>*>& CenterLines, double X, double Y, double Fallback);

	// Z of the first centerline point (a sane Fallback for SampleHeight).
	ROADNET_API double FirstCenterlineZ(const TArray<const TArray<FVector>*>& CenterLines);
}
