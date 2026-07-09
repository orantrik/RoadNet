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
	// Grid-accelerated nearest-centerline height sampler (§10.10). Build once
	// from a zone's centerlines, then query per vertex/loop-point in ~O(1). This
	// replaces the former O(vertices × centerline-points) linear scan that made
	// city-scale imports (100s of roads, 10,000s of points) take minutes.
	class ROADNET_API FCenterlineHeightField
	{
	public:
		// CellCm should be a few × the polyline sample spacing (default 10 m).
		void Build(const TArray<const TArray<FVector>*>& CenterLines, double CellCm = 1000.0);

		// Interpolated Z at (X,Y): nearest centerline segment, lerped along it.
		// Returns Fallback if the field is empty or nothing is found nearby.
		double SampleHeight(double X, double Y, double Fallback) const;

		double FirstZ() const { return FallbackZ; }
		bool IsEmpty() const { return Segs.Num() == 0; }

	private:
		struct FSeg { FVector2D A, B; double Za, Zb; };
		TArray<FSeg> Segs;
		TMultiMap<FIntPoint, int32> Grid;
		double CellCm = 1000.0;
		double FallbackZ = 0.0;
	};

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

	// Z of the first centerline point (a sane Fallback for SampleHeight).
	ROADNET_API double FirstCenterlineZ(const TArray<const TArray<FVector>*>& CenterLines);
}
