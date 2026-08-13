#pragma once
#include "CoreMinimal.h"
#include "Curve/GeneralPolygon2.h"
#include "RoadNetMath.h"

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

		// Nearest-centerline projection at (X,Y) off the SAME grid, matching
		// RoadNetMath::ProjectToPolyline's conventions (AlongDist is arc length
		// from that centerline's start, Offset is signed +right of travel).
		// Segment is the hit's index within its own centerline. Distance stays
		// TNumericLimits<double>::Max() if the field is empty.
		RoadNetMath::FProjectResult ProjectNearest(double X, double Y) const;

		double FirstZ() const { return FallbackZ; }
		bool IsEmpty() const { return Segs.Num() == 0; }

	private:
		// AlongA/Len carry the arc length so a projection can be answered from a
		// single segment without walking its whole polyline.
		struct FSeg { FVector2D A, B; double Za, Zb; int32 Road; double AlongA; double Len; int32 Local; };
		TArray<FSeg> Segs;
		TMultiMap<FIntPoint, int32> Grid;
		double CellCm = 1000.0;
		double FallbackZ = 0.0;
	};

	// Triangulate Polys and APPEND into OutMesh (does not clear). Each output
	// vertex takes the interpolated Z of the nearest CenterLine (XY projection),
	// plus ZLiftCm. Returns the number of triangles appended.
	//
	// If VertexColorFn is provided, the mesh's per-vertex colour overlay is
	// enabled and each vertex is coloured by VertexColorFn(X,Y) — used to bake
	// per-lane shading into the SINGLE carriageway mesh (so lanes no longer need
	// a separate lifted overlay that z-intersects the road). Linear colour.
	//
	// If bComputeUVs, a primary-UV overlay is written from each vertex's nearest
	// centreline: U = lateral offset, V = arc length along the road, both scaled
	// by 1/UVUnitCm (default 100 → 1 UV unit = 1 m) so a tiling material maps
	// straight/along the road instead of stretching over the boolean footprint.
	//
	// If bGradientNormals, per-vertex normals are taken from the finite-difference
	// gradient of the (blended) height field rather than averaged from the jittery
	// Delaunay triangles — smooth, grade-following normals with no facet blotches.
	// When set, the caller must NOT also run FinalizeNormals (it would overwrite).
	//
	// If bWorldUVs, UVs are world-planar (U = X, V = Y, scaled by 1/UVUnitCm)
	// instead of centreline offset/arc-length. Use for grass islands (medians)
	// so a tiling ground texture reads world-aligned with no mirror seam down the
	// centre (offset UVs flip sign across the reference line = a mirrored look).
	ROADNET_API int32 AppendSurfaceMesh(
		const TArray<UE::Geometry::FGeneralPolygon2d>& Polys,
		const TArray<const TArray<FVector>*>& CenterLines,
		double ZLiftCm,
		UE::Geometry::FDynamicMesh3& OutMesh,
		const TFunction<FVector3f(double, double)>* VertexColorFn = nullptr,
		bool bComputeUVs = false,
		double UVUnitCm = 100.0,
		bool bGradientNormals = false,
		bool bWorldUVs = false);

	// Enable attributes and compute per-vertex normals (call once, after all
	// zones have been appended). No-op on an empty mesh.
	ROADNET_API void FinalizeNormals(UE::Geometry::FDynamicMesh3& Mesh);

	// Z of the first centerline point (a sane Fallback for SampleHeight).
	ROADNET_API double FirstCenterlineZ(const TArray<const TArray<FVector>*>& CenterLines);
}
