#pragma once
#include "CoreMinimal.h"
#include "Curve/GeneralPolygon2.h"
#include "RoadNetMesh.h"   // FCenterlineHeightField

// ===========================================================================
// RoadNetCurbs — kerb-line instance placement (§8.12 companion).
//
// The kerb is the boundary between the merged carriageway and the sidewalk
// band. Because we derive it from the FINAL, already-merged surface polygons +
// the sidewalk polygons of a rebuild, the kerb line automatically tracks lane
// additions, sidewalk-width changes and curved junction corners — it is always
// rebuilt in lock-step with the sidewalks (no separate authoring).
//
// This module only produces PLACEMENTS (world transforms + a per-piece length
// so the caller can stretch one kerb segment mesh to tile gaplessly). The
// caller instances them into a HierarchicalInstancedStaticMeshComponent.
// INDEPENDENCE: engine geometry types only; no RoadBLD dependency.
// ===========================================================================
struct FRoadCurves;

namespace RoadNetCurbs
{
	// One kerb instance placement, world centimetres. Yaw aligns the kerb mesh's
	// long (travel) axis to the kerb tangent; the sidewalk is on the instance's
	// LEFT (mesh local +Y), so a kerb mesh authored with its raised face toward
	// −Y (road side) reads correctly all the way round.
	struct FCurbInstance
	{
		FVector Location = FVector::ZeroVector;  // road-top height at the kerb line
		float   YawDeg   = 0.f;                  // travel direction (degrees)
		float   LengthCm = 100.f;                // along-run length this piece spans
	};

	// Derive kerb placements for ONE grade zone from its merged carriageway
	// surface polygons + sidewalk band polygons. An edge of the carriageway
	// boundary is a kerb edge when a probe just off the edge lands inside the
	// sidewalk band; contiguous kerb edges are chained and sampled every
	// ~SpacingCm. Z is taken from the zone's centreline height field (+ZLiftCm).
	// Instances are APPENDED to OutInstances.
	ROADNET_API void BuildCurbInstancesForZone(
		const TArray<UE::Geometry::FGeneralPolygon2d>& SurfacePolys,
		const TArray<UE::Geometry::FGeneralPolygon2d>& SidewalkPolys,
		const RoadNetMesh::FCenterlineHeightField& Height,
		double SpacingCm,
		double ZLiftCm,
		TArray<FCurbInstance>& OutInstances);

	// Place kerb pieces along an OPEN polyline (e.g. a median edge). Same
	// adaptive, curve-hugging chunking as the boundary path; Z from the height
	// field. Sidewalk-side orientation matches the polyline's travel LEFT.
	// Instances are APPENDED to OutInstances.
	ROADNET_API void BuildCurbInstancesAlongLine(
		const TArray<FVector>& Line,
		const RoadNetMesh::FCenterlineHeightField& Height,
		double SpacingCm,
		double ZLiftCm,
		TArray<FCurbInstance>& OutInstances);
}
