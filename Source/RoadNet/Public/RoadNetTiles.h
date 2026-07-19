#pragma once
#include "CoreMinimal.h"

// ===========================================================================
// RoadNetTiles — spatial-grid tiling helpers (§ tiling).
//
// RoadNet geometry is partitioned into a fixed world-aligned grid of square
// cells (TileSizeCm on a side) so that (a) each cell can be its own actor for
// World-Partition streaming and (b) an edit only re-commits the cells it
// touches instead of the whole (single, city-wide) grade zone. A tile is keyed
// by its integer cell coordinate FIntPoint(cx, cy) where
//   cx = floor(WorldX / TileSizeCm), cy = floor(WorldY / TileSizeCm).
// Z is ignored for tiling (grade separation is handled upstream by zones).
//
// Pure math only — no engine actor/UObject dependency — so it is cheap to
// include anywhere in the pipeline.
// ===========================================================================
namespace RoadNetTiles
{
	// Windowed-rebuild safety margin (cm): how far one road's geometry can reach
	// beyond its own centreline into a neighbour cell (junction fillet discs,
	// smoothing, stop-line clearance, half-widths, curb/furniture offsets). Any
	// road whose bounds are within this margin of a dirty cell participates in
	// that cell's recompute, so junction geometry at a cell border is correct.
	// Deliberately generous (junction influence is only tens of metres).
	inline constexpr double kTileMarginCm = 30000.0; // 300 m

	// World XY -> integer tile coordinate.
	inline FIntPoint WorldToTile(double WorldX, double WorldY, double TileSizeCm)
	{
		const double S = FMath::Max(1.0, TileSizeCm);
		return FIntPoint(
			(int32)FMath::FloorToDouble(WorldX / S),
			(int32)FMath::FloorToDouble(WorldY / S));
	}

	inline FIntPoint WorldToTile(const FVector& P, double TileSizeCm)
	{
		return WorldToTile(P.X, P.Y, TileSizeCm);
	}

	inline FIntPoint WorldToTile(const FVector2D& P, double TileSizeCm)
	{
		return WorldToTile(P.X, P.Y, TileSizeCm);
	}

	// World-space XY bounds of a tile (Z left at +/- large so callers can ignore).
	inline FBox2D TileBounds2D(const FIntPoint& Tile, double TileSizeCm)
	{
		const double S = FMath::Max(1.0, TileSizeCm);
		const FVector2D Min((double)Tile.X * S, (double)Tile.Y * S);
		return FBox2D(Min, Min + FVector2D(S, S));
	}

	// Centre of a tile in world XY (Z = 0).
	inline FVector TileCenterWorld(const FIntPoint& Tile, double TileSizeCm)
	{
		const FBox2D B = TileBounds2D(Tile, TileSizeCm);
		const FVector2D C = B.GetCenter();
		return FVector(C.X, C.Y, 0.0);
	}

	// Append every tile coordinate whose cell overlaps [MinXY, MaxXY] (an XY
	// bounding box, optionally dilated by ExpandCm) into OutTiles (uniqued by
	// the caller via a set).
	template <typename SetType>
	void TilesOverlappingBox(const FVector2D& MinXY, const FVector2D& MaxXY,
		double TileSizeCm, double ExpandCm, SetType& OutTiles)
	{
		const FIntPoint Lo = WorldToTile(MinXY.X - ExpandCm, MinXY.Y - ExpandCm, TileSizeCm);
		const FIntPoint Hi = WorldToTile(MaxXY.X + ExpandCm, MaxXY.Y + ExpandCm, TileSizeCm);
		for (int32 y = Lo.Y; y <= Hi.Y; ++y)
		{
			for (int32 x = Lo.X; x <= Hi.X; ++x)
			{
				OutTiles.Add(FIntPoint(x, y));
			}
		}
	}
}
