#pragma once
#include "CoreMinimal.h"
#include "Curve/GeneralPolygon2.h"

// One place a centreline crosses a junction clip polygon. Markings and the lane
// graph both read this, so the stop line and the lane mouth are the same point.
namespace RoadNetMouths
{
	struct FCrossing
	{
		FVector2D Boundary = FVector2D::ZeroVector;
		FVector2D Stop = FVector2D::ZeroVector;
		FVector2D Outward = FVector2D(1, 0);
		FVector2D OutN = FVector2D(1, 0);
		double Z = 0.0;
		bool bForwardEnters = false;
	};

	void ClipCrossings(const TArray<FVector>& Sampled, const UE::Geometry::FGeneralPolygon2d& Poly,
		double HalfWidthCm, TArray<FCrossing>& Out);
}
