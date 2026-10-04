// RoadNetSurface.cpp — Clipper2 boolean-union junction surface (§10.9).
#include "RoadNetSurface.h"
#include "RoadNetwork.h"
#include "RoadNetMath.h"
#include "RoadNetLog.h"
#include "Async/ParallelFor.h"
#include "Polygon2.h"
#include "CompGeom/ConvexHull2.h"
#include "Curve/PolygonIntersectionUtils.h"
#include "Curve/PolygonOffsetUtils.h"

using namespace UE::Geometry;

namespace RoadNetSurface
{
	bool BuildRoadOutline(const FRoadCurves& C, FGeneralPolygon2d& Out)
	{
		const int32 N = C.LeftEdge.Num();
		if (N < 2 || C.RightEdge.Num() != N) { return false; }

		// Closed loop: left edge forward, then right edge reversed.
		TArray<FVector2d> Loop;
		Loop.Reserve(N * 2);
		for (int32 i = 0; i < N; ++i)        { Loop.Emplace(C.LeftEdge[i].X,  C.LeftEdge[i].Y); }
		for (int32 i = N - 1; i >= 0; --i)   { Loop.Emplace(C.RightEdge[i].X, C.RightEdge[i].Y); }

		FPolygon2d Poly(Loop);
		if (Poly.VertexCount() < 3) { return false; }
		if (FMath::Abs(Poly.SignedArea()) < 1.0) { return false; } // degenerate sliver

		// Outer ring convention: CCW (counter-clockwise, positive area).
		if (Poly.IsClockwise()) { Poly.Reverse(); }

		Out.SetOuter(Poly);
		return true;
	}

	void MakeDisc(const FVector2D& Center, double Radius, int32 Segments, FGeneralPolygon2d& Out)
	{
		Segments = FMath::Clamp(Segments, 6, 96);
		TArray<FVector2d> Ring;
		Ring.Reserve(Segments);
		for (int32 i = 0; i < Segments; ++i)
		{
			const double A = (2.0 * PI * i) / Segments; // CCW
			Ring.Emplace(Center.X + Radius * FMath::Cos(A), Center.Y + Radius * FMath::Sin(A));
		}
		FPolygon2d Poly(Ring);
		if (Poly.IsClockwise()) { Poly.Reverse(); }
		Out.SetOuter(Poly);
	}

	bool MakeHull(const TArray<FVector2D>& Pts, FGeneralPolygon2d& Out)
	{
		if (Pts.Num() < 3) { return false; }

		FConvexHull2d Hull;
		if (!Hull.Solve(Pts.Num(),
				[&Pts](int32 i) { return FVector2d(Pts[i].X, Pts[i].Y); },
				[](int32) { return true; }))
		{
			return false;   // collinear or degenerate
		}

		const TArray<int32>& Idx = Hull.GetPolygonIndices();
		if (Idx.Num() < 3) { return false; }

		TArray<FVector2d> Ring;
		Ring.Reserve(Idx.Num());
		for (int32 i : Idx) { Ring.Emplace(Pts[i].X, Pts[i].Y); }

		FPolygon2d Poly(Ring);
		if (Poly.VertexCount() < 3 || FMath::Abs(Poly.SignedArea()) < 1.0) { return false; }
		if (Poly.IsClockwise()) { Poly.Reverse(); }

		Out.SetOuter(Poly);
		return true;
	}

	// Morphological close (dilate +e then erode -e): rounds concave corners and
	// bridges micro-gaps without changing the overall outer size. In-place on Io.
	// Join defaults to Round for visual fillets; the 5 cm city weld should pass
	// Square — Round is wasted there (it is not a fillet) and explodes vertex count.
	static void MorphClose(TArray<FGeneralPolygon2d>& Io, double e,
		EPolygonOffsetJoinType Join = EPolygonOffsetJoinType::Round)
	{
		if (e <= 0.0 || Io.Num() == 0) { return; }
		TArray<FGeneralPolygon2d> Closed;
		// The arc-density arguments are NOT optional in practice. Clipper's default
		// steps-per-radian is, in the engine's own words, "extremely large", so a
		// round join emits thousands of vertices where tens will do. Every other
		// Round offset in this plugin already passes these two.
		if (PolygonsOffsets(
				+e, -e, Io, Closed, /*bCopyInputOnFailure*/true,
				/*MiterLimit*/2.0, Join, EPolygonOffsetEndType::Polygon,
				/*MaxStepsPerRadian*/16.0, /*DefaultStepsPerRadianScale*/1.0e-3))
		{
			Io = MoveTemp(Closed);
		}
	}

	// Clipper's union of N non-overlapping subjects returns ~N polygons, so a
	// single Execute() over 79k city-wide junction patches does not reduce and
	// never returns. Union nearby patches first (they actually overlap), then
	// the leftover count is ~occupied cells, which the final overlay can finish.
	static bool UnionSpatial(TArray<FGeneralPolygon2d> In, TArray<FGeneralPolygon2d>& Out, double CellCm)
	{
		Out.Reset();
		if (In.Num() == 0) { return true; }
		if (In.Num() == 1) { Out = MoveTemp(In); return true; }
		if (In.Num() <= 64)
		{
			return PolygonsUnion(In, Out, /*bCopyInputOnFailure*/true);
		}

		const int32 InN = In.Num();
		TMap<FIntPoint, TArray<FGeneralPolygon2d>> Cells;
		for (FGeneralPolygon2d& P : In)
		{
			const FAxisAlignedBox2d B = P.Bounds();
			const FVector2d C = (B.Min + B.Max) * 0.5;
			Cells.FindOrAdd(FIntPoint(
				FMath::FloorToInt32(C.X / CellCm),
				FMath::FloorToInt32(C.Y / CellCm))).Add(MoveTemp(P));
		}
		In.Reset();

		// One cell's union cannot see another's — that is the whole point of the
		// spatial split — so run them in parallel. Flattened to an array first
		// because a TMap cannot be indexed, and each cell writes only its own
		// output slot.
		TArray<TArray<FGeneralPolygon2d>*> CellList;
		CellList.Reserve(Cells.Num());
		for (auto& KV : Cells) { CellList.Add(&KV.Value); }

		TArray<TArray<FGeneralPolygon2d>> CellOut;
		CellOut.SetNum(CellList.Num());
		ParallelFor(CellList.Num(), [&CellList, &CellOut](int32 i)
		{
			TArray<FGeneralPolygon2d>& Cell = *CellList[i];
			if (Cell.Num() == 1) { CellOut[i].Add(MoveTemp(Cell[0])); return; }
			TArray<FGeneralPolygon2d> U;
			if (PolygonsUnion(Cell, U, /*bCopyInputOnFailure*/true)) { CellOut[i] = MoveTemp(U); }
			else { CellOut[i] = MoveTemp(Cell); }
		});

		TArray<FGeneralPolygon2d> Reduced;
		Reduced.Reserve(CellList.Num());
		for (TArray<FGeneralPolygon2d>& C : CellOut) { Reduced.Append(MoveTemp(C)); }

		UE_LOG(LogRoadNet, Log, TEXT("[RoadNet]   surface: spatial union %d polys -> %d in %.0f m cells"),
			InN, Reduced.Num(), CellCm / 100.0);

		// Grow the cell until the leftover is a size Clipper can overlay, or the
		// cell is a few km and further grouping stops helping.
		if (Reduced.Num() > 512 && CellCm < 400000.0)
		{
			return UnionSpatial(MoveTemp(Reduced), Out, CellCm * 2.0);
		}
		return PolygonsUnion(Reduced, Out, /*bCopyInputOnFailure*/true);
	}

	// A fillet between two arms reaches (half-width + radius) / sin(angle/2)
	// from the junction. A width-sized disc clips that corner off before the
	// close can round it, particularly on sparse hand-drawn T/X junctions.
	static double JunctionWorkRadius(const FJunctionClose& J, const TArray<const FRoadCurves*>& Curves)
	{
		TArray<double> Bearings;
		double Reach = J.FillRadiusCm;
		for (const FRoadCurves* C : Curves)
		{
			if (!C || C->Sampled.Num() < 2) { continue; }
			const auto P = RoadNetMath::ProjectToPolyline(C->Sampled, J.Center);
			if (P.Segment == INDEX_NONE || P.Distance > J.FillRadiusCm + 5.0) { continue; }
			const FVector D = C->Sampled[P.Segment + 1] - C->Sampled[P.Segment];
			if (D.SizeSquared2D() < 1.e-6) { continue; }
			const double A = FMath::Atan2(D.Y, D.X);
			const double Length = RoadNetMath::TotalLength(C->Sampled);
			if (P.AlongDist > 1.0) { Bearings.Add(FMath::Fmod(A + 3.0 * PI, 2.0 * PI)); }
			if (Length - P.AlongDist > 1.0) { Bearings.Add(FMath::Fmod(A + 2.0 * PI, 2.0 * PI)); }
			Reach = FMath::Max(Reach, Length + P.Distance);
		}
		Bearings.Sort();
		double Radius = J.FillRadiusCm + J.CloseCm + 50.0;
		for (int32 i = 0; i < Bearings.Num(); ++i)
		{
			const double Next = i + 1 < Bearings.Num() ? Bearings[i + 1] : Bearings[0] + 2.0 * PI;
			const double Angle = Next - Bearings[i];
			if (Angle < 1.e-4 || Angle >= PI) { continue; }
			const double CornerReach = (J.FillRadiusCm + J.CloseCm) / FMath::Sin(0.5 * Angle);
			Radius = FMath::Max(Radius, FMath::Min(CornerReach, Reach) + J.CloseCm + 50.0);
		}
		return Radius;
	}

	bool BuildMergedSurface(
		const TArray<const FRoadCurves*>& Curves,
		TArray<FGeneralPolygon2d>& OutMerged,
		double InflateEpsilonCm,
		const TArray<FGeneralPolygon2d>* ExtraPolys,
		const TArray<FJunctionClose>* PerJunction)
	{
		OutMerged.Reset();

		TArray<FGeneralPolygon2d> Outlines;
		// 1:1 with Outlines — the centreline each outline was built around, null for
		// ExtraPolys. The per-junction pass needs both views of a road: the centreline
		// to size the work disc, the outline to clip against it.
		TArray<const FRoadCurves*> OutlineCurve;
		Outlines.Reserve(Curves.Num() + (ExtraPolys ? ExtraPolys->Num() : 0));
		OutlineCurve.Reserve(Curves.Num() + (ExtraPolys ? ExtraPolys->Num() : 0));
		for (const FRoadCurves* C : Curves)
		{
			if (!C) { continue; }
			FGeneralPolygon2d GP;
			if (BuildRoadOutline(*C, GP)) { Outlines.Add(MoveTemp(GP)); OutlineCurve.Add(C); }
		}
		if (ExtraPolys)
		{
			Outlines.Append(*ExtraPolys);
			OutlineCurve.AddZeroed(ExtraPolys->Num());
		}
		if (Outlines.Num() == 0) { return true; }

		// A 35k-road import sat in this function for nine hours with nothing in the
		// log; at city scale each phase gets a timestamped line so a stall names its
		// phase itself.
		// Threshold was 5000, which is above what the sidewalk pass feeds in — so
		// the slowest phases of a city import logged nothing at all. A few lines
		// on a medium network is a far smaller cost than a silent stall.
		const bool bTracePhases = Outlines.Num() >= 1000;
		const double tUnion0 = FPlatformTime::Seconds();

		// Announce the phase BEFORE the work, not after. Every line below reports
		// a phase that has already finished, so a slow one is indistinguishable
		// from a hang — which is exactly how this function kept being reported as
		// frozen. The sidewalk-band pass in particular starts here in silence.
		if (bTracePhases)
		{
			UE_LOG(LogRoadNet, Log, TEXT("[RoadNet]   surface: starting union of %d outline(s)..."),
				Outlines.Num());
		}

		// Boolean union of all arm outlines -> filled, seamless junctions (§10.9).
		//
		// Spatially, for the same reason the junction patches are: Clipper's cost
		// on one global Execute() over thousands of overlapping city-wide ribbons
		// is superlinear, and the band pass (wider outlines, far more mutual
		// overlap than the carriageway) is where that bites. UnionSpatial unions
		// neighbours first and finishes with a small overlay, giving the identical
		// region. Outlines is COPIED because the per-junction pass below still
		// needs the individual arms.
		const bool bUnionOk = (Outlines.Num() > 64)
			? UnionSpatial(Outlines, OutMerged, /*CellCm=*/20000.0)
			: PolygonsUnion(Outlines, OutMerged, /*bCopyInputOnFailure*/true);
		if (!bUnionOk)
		{
			UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet] Surface union failed; using unmerged outlines."));
			return true; // OutMerged holds the copied input (bCopyInputOnFailure)
		}
		if (bTracePhases)
		{
			UE_LOG(LogRoadNet, Log, TEXT("[RoadNet]   surface: union of %d outlines -> %d ring(s) in %.1f s"),
				Outlines.Num(), OutMerged.Num(), FPlatformTime::Seconds() - tUnion0);
		}

		const bool bPerJunction = (PerJunction && PerJunction->Num() > 0);
		if (!bPerJunction)
		{
			// Original path: one global morphological "close" bridges micro-gaps
			// between arms meeting at oblique angles without changing overall size.
			MorphClose(OutMerged, InflateEpsilonCm);
			return true;
		}

		// ---- per-junction smoothing -----------------------------------------
		// Weld the whole surface with only a hairline epsilon (so abutting arms
		// still connect), then round EACH junction locally with its own radius.
		// A junction's rounding is computed on the local surface patch (surface âˆ©
		// disc) and unioned back, so junctions carry independent smoothing while
		// straight road runs are untouched.
		// The weld is NOT a rounding radius: it exists only to close the hairline
		// between two abutting arm outlines. Deriving it from InflateEpsilonCm
		// meant dialling junction smoothing down to 0 also dropped the weld from
		// 5 cm to 2 cm, so the merged carriageway came apart into dozens of
		// fragments — same pavement area, many more rings — and every downstream
		// pass that scales with ring count (offsets, sidewalk bands, kerbs) went
		// with it. Sharp corners and connected pavement are separate requests.
		constexpr double WeldCm = 5.0;
		const double tWeld0 = FPlatformTime::Seconds();
		// Square: this weld only closes hairline gaps. Round here is not a fillet
		// and is what bloated the 1115-ring city surface into a union that never
		// returned.
		MorphClose(OutMerged, WeldCm, EPolygonOffsetJoinType::Square);
		if (bTracePhases)
		{
			UE_LOG(LogRoadNet, Log, TEXT("[RoadNet]   surface: %.0f cm weld in %.1f s"),
				WeldCm, FPlatformTime::Seconds() - tWeld0);
		}

		// Each junction's region is computed from the PRE-union outlines near it, not
		// from OutMerged. Union is pointwise — a point is inside the union iff it is
		// inside some outline — so (union of everything) ∩ disc equals (union of the
		// outlines touching the disc) ∩ disc, and Clipper computes that union
		// implicitly when handed several subjects. The previous code clipped against
		// the MERGED surface: a connected city zone welds into ONE ring whose
		// bounding box spans the whole net, so its "reject rings by bbox" filter
		// rejected nothing and all ~60k junctions paid a Clipper clip against a
		// million-vertex polygon — while JunctionWorkRadius projected every junction
		// onto every curve in the zone. Two O(junctions × zone) passes; a 35k-road
		// import stalled in them for hours. One uniform grid over the outline bounds
		// now serves both queries, making each junction's cost local.
		// The only behavioural delta: hairline weld slivers inside the disc are not
		// in the clipped region, but the region's own close (CloseCm > WeldCm)
		// re-bridges anything the weld could have, and the patch is unioned onto the
		// welded base anyway. The curb-smoothing automation test pins this.
		auto BoxesOverlap = [](const FAxisAlignedBox2d& A, const FAxisAlignedBox2d& B)
		{
			return !(A.Max.X < B.Min.X || A.Min.X > B.Max.X ||
			         A.Max.Y < B.Min.Y || A.Min.Y > B.Max.Y);
		};
		constexpr double GridCellCm = 5000.0; // 50 m: a junction work disc spans a few cells
		TArray<FAxisAlignedBox2d> OutlineBounds;
		OutlineBounds.Reserve(Outlines.Num());
		TMap<FIntPoint, TArray<int32>> Grid;
		for (int32 i = 0; i < Outlines.Num(); ++i)
		{
			const FAxisAlignedBox2d B = Outlines[i].Bounds();
			OutlineBounds.Add(B);
			const int32 X0 = FMath::FloorToInt32(B.Min.X / GridCellCm);
			const int32 X1 = FMath::FloorToInt32(B.Max.X / GridCellCm);
			const int32 Y0 = FMath::FloorToInt32(B.Min.Y / GridCellCm);
			const int32 Y1 = FMath::FloorToInt32(B.Max.Y / GridCellCm);
			for (int32 y = Y0; y <= Y1; ++y)
			{
				for (int32 x = X0; x <= X1; ++x)
				{
					Grid.FindOrAdd(FIntPoint(x, y)).Add(i);
				}
			}
		}
		// Junctions are independent of each other: Grid, OutlineBounds, Outlines
		// and OutlineCurve are all read-only from here on, and each junction
		// produces its own patches. The only mutable state is the query scratch,
		// so every worker gets its own — one bit per outline, allocated once per
		// worker rather than once per junction.
		struct FJunctionScratch
		{
			TBitArray<>                Seen;
			TArray<int32>              Hits;
			TArray<const FRoadCurves*> NearCurves;
			TArray<FGeneralPolygon2d>  Out;
		};

		const double tJunc0 = FPlatformTime::Seconds();
		TArray<FJunctionScratch> Scratch;
		// Capture by reference: ParallelFor is synchronous, so every captured local
		// outlives the call, and all of them are read-only inside it.
		ParallelForWithTaskContext(Scratch, PerJunction->Num(),
			[&](FJunctionScratch& S, int32 ji)
		{
			const FJunctionClose& J = (*PerJunction)[ji];
			if (J.CloseCm <= WeldCm) { return; } // no extra rounding at this junction

			if (S.Seen.Num() != Outlines.Num()) { S.Seen.Init(false, Outlines.Num()); }

			auto QueryBox = [&](const FAxisAlignedBox2d& Box)
			{
				S.Hits.Reset();
				const int32 X0 = FMath::FloorToInt32(Box.Min.X / GridCellCm);
				const int32 X1 = FMath::FloorToInt32(Box.Max.X / GridCellCm);
				const int32 Y0 = FMath::FloorToInt32(Box.Min.Y / GridCellCm);
				const int32 Y1 = FMath::FloorToInt32(Box.Max.Y / GridCellCm);
				for (int32 y = Y0; y <= Y1; ++y)
				{
					for (int32 x = X0; x <= X1; ++x)
					{
						if (const TArray<int32>* Cell = Grid.Find(FIntPoint(x, y)))
						{
							for (int32 i : *Cell)
							{
								if (!S.Seen[i] && BoxesOverlap(OutlineBounds[i], Box))
								{
									S.Seen[i] = true;
									S.Hits.Add(i);
								}
							}
						}
					}
				}
				for (int32 i : S.Hits) { S.Seen[i] = false; } // stamp reset, no reallocation
			};

			// Only arms whose centreline enters the fill disc can shape the work
			// radius (JunctionWorkRadius discards the rest by exact distance), and
			// an outline's bounds contain its centreline, so this box catches them.
			const double Pad = J.FillRadiusCm + 5.0;
			QueryBox(FAxisAlignedBox2d(
				FVector2d(J.Center.X - Pad, J.Center.Y - Pad),
				FVector2d(J.Center.X + Pad, J.Center.Y + Pad)));
			S.NearCurves.Reset();
			for (int32 i : S.Hits)
			{
				if (OutlineCurve[i]) { S.NearCurves.Add(OutlineCurve[i]); }
			}

			const double R = JunctionWorkRadius(J, S.NearCurves);
			FGeneralPolygon2d Disc;
			MakeDisc(J.Center, R, /*Segments*/48, Disc);
			const TArray<FGeneralPolygon2d> DiscArr = { MoveTemp(Disc) };

			QueryBox(FAxisAlignedBox2d(
				FVector2d(J.Center.X - R, J.Center.Y - R),
				FVector2d(J.Center.X + R, J.Center.Y + R)));
			if (S.Hits.Num() == 0) { return; }
			TArray<FGeneralPolygon2d> Near;
			Near.Reserve(S.Hits.Num());
			for (int32 i : S.Hits) { Near.Add(Outlines[i]); }

			TArray<FGeneralPolygon2d> Region;
			if (!PolygonsIntersection(Near, DiscArr, Region) || Region.Num() == 0) { return; }
			MorphClose(Region, J.CloseCm);
			S.Out.Append(MoveTemp(Region));
		});

		TArray<FGeneralPolygon2d> Patches;
		for (FJunctionScratch& S : Scratch) { Patches.Append(MoveTemp(S.Out)); }

		if (bTracePhases)
		{
			UE_LOG(LogRoadNet, Log, TEXT("[RoadNet]   surface: %d junction patch(es) from %d junction(s) in %.1f s"),
				Patches.Num(), PerJunction->Num(), FPlatformTime::Seconds() - tJunc0);
		}

		if (Patches.Num() > 0)
		{
			const double tFinal0 = FPlatformTime::Seconds();
			TArray<FGeneralPolygon2d> PatchUnion;
			UnionSpatial(MoveTemp(Patches), PatchUnion, /*CellCm=*/20000.0);
			if (PatchUnion.Num() > 0)
			{
				TArray<FGeneralPolygon2d> All = MoveTemp(OutMerged);
				All.Append(MoveTemp(PatchUnion));
				TArray<FGeneralPolygon2d> Merged2;
				if (PolygonsUnion(All, Merged2, /*bCopyInputOnFailure*/true))
				{
					OutMerged = MoveTemp(Merged2);
				}
				else
				{
					OutMerged = MoveTemp(All);
				}
			}
			if (bTracePhases)
			{
				UE_LOG(LogRoadNet, Log, TEXT("[RoadNet]   surface: patches unioned back in %.1f s"),
					FPlatformTime::Seconds() - tFinal0);
			}
		}
		return true;
	}

	// Close two already-offset rails into one ribbon loop. Shared by the fixed-
	// and variable-width entry points so they cannot drift apart.
	static bool CloseRibbon(const TArray<FVector>& Inner, const TArray<FVector>& Outer,
		FGeneralPolygon2d& Out)
	{
		if (Inner.Num() < 2 || Outer.Num() != Inner.Num()) { return false; }

		TArray<FVector2d> Loop;
		Loop.Reserve(Inner.Num() * 2);
		for (int32 i = 0; i < Inner.Num(); ++i)      { Loop.Emplace(Inner[i].X, Inner[i].Y); }
		for (int32 i = Outer.Num() - 1; i >= 0; --i) { Loop.Emplace(Outer[i].X, Outer[i].Y); }

		FPolygon2d Poly(Loop);
		if (Poly.VertexCount() < 3 || FMath::Abs(Poly.SignedArea()) < 1.0) { return false; }
		if (Poly.IsClockwise()) { Poly.Reverse(); }
		Out.SetOuter(Poly);
		return true;
	}

	bool BuildSideRibbon(const TArray<FVector>& Sampled, double InnerOff, double OuterOff,
		FGeneralPolygon2d& Out)
	{
		if (Sampled.Num() < 2) { return false; }
		TArray<FVector> Inner, Outer;
		RoadNetMath::OffsetPolyline(Sampled, InnerOff, Inner);
		RoadNetMath::OffsetPolyline(Sampled, OuterOff, Outer);
		return CloseRibbon(Inner, Outer, Out);
	}

	bool BuildSideRibbonVariable(const TArray<FVector>& Sampled, double InnerOff,
		const TArray<double>& OuterOffs, FGeneralPolygon2d& Out)
	{
		if (Sampled.Num() < 2 || OuterOffs.Num() != Sampled.Num()) { return false; }
		TArray<FVector> Inner, Outer;
		RoadNetMath::OffsetPolyline(Sampled, InnerOff, Inner);
		RoadNetMath::OffsetPolylineVariable(Sampled, OuterOffs, Outer);
		return CloseRibbon(Inner, Outer, Out);
	}

	bool BuildPathRibbon(const TArray<FVector>& Center, double HalfWidth,
		TArray<FGeneralPolygon2d>& Out, double MaxStepsPerRadian, bool bRoundEnds)
	{
		Out.Reset();
		if (Center.Num() < 2 || HalfWidth < 1.0) { return false; }

		// Drop near-coincident vertices first: zero-length input segments are the
		// usual source of offset spikes.
		TArray<FVector2d> Pts;
		Pts.Reserve(Center.Num());
		for (const FVector& P : Center)
		{
			const FVector2d Q(P.X, P.Y);
			if (Pts.Num() == 0 || FVector2d::DistSquared(Pts.Last(), Q) > 1.0) { Pts.Add(Q); }
		}
		if (Pts.Num() < 2) { return false; }

		// Butt end-type => Clipper treats the ring as an OPEN path and thickens
		// both sides by Offset; Round joins keep bends smooth. The internal
		// NonZero union collapses any self-overlap into a simple boundary.
		FOffsetPolygon2d Off;
		Off.Polygons.Add(TArrayView<FVector2d>(Pts));
		Off.Offset = HalfWidth;
		Off.MiterLimit = 2.0;
		Off.JoinType = EPolygonOffsetJoinType::Round;
		Off.EndType = bRoundEnds ? EPolygonOffsetEndType::Round : EPolygonOffsetEndType::Butt;
		Off.MaxStepsPerRadian = (MaxStepsPerRadian > 0.0) ? MaxStepsPerRadian : 16.0;
		if (!Off.ComputeResult()) { return false; }

		for (FGeneralPolygon2d& GP : Off.Result)
		{
			if (FMath::Abs(GP.GetOuter().SignedArea()) >= 1.0) { Out.Add(MoveTemp(GP)); }
		}
		return Out.Num() > 0;
	}

	bool BuildSidewalkBands(
		const TArray<FGeneralPolygon2d>& SideRibbons,
		const TArray<FGeneralPolygon2d>& RoadPolys,
		TArray<FGeneralPolygon2d>& OutBands)
	{
		OutBands.Reset();
		if (SideRibbons.Num() == 0) { return true; }

		// This function was completely silent and, on a city, accounted for five
		// of a seven-minute import — the carriageway pass above at least named its
		// phases. Both halves get a line now.
		const bool bTrace = SideRibbons.Num() >= 1000;

		// Union the per-road side strips so co-linear neighbours merge cleanly.
		// Spatially, for the same reason the carriageway union is: one global
		// Execute() over ~13k overlapping city-wide ribbons is superlinear.
		const double tUnion0 = FPlatformTime::Seconds();
		TArray<FGeneralPolygon2d> Merged;
		const bool bUnionOk = (SideRibbons.Num() > 64)
			? UnionSpatial(SideRibbons, Merged, /*CellCm=*/20000.0)
			: PolygonsUnion(SideRibbons, Merged, /*bCopyInputOnFailure*/true);
		if (!bUnionOk && Merged.Num() == 0)
		{
			Merged = SideRibbons;
		}
		if (bTrace)
		{
			UE_LOG(LogRoadNet, Log, TEXT("[RoadNet]   bands: union of %d side ribbon(s) -> %d in %.1f s"),
				SideRibbons.Num(), Merged.Num(), FPlatformTime::Seconds() - tUnion0);
		}
		if (Merged.Num() == 0) { return true; }

		// Subtract the carriageway so a sidewalk never overlaps the road or a
		// junction fill (the discs are already part of RoadPolys).
		if (RoadPolys.Num() == 0) { OutBands = MoveTemp(Merged); return true; }

		// Per band, in parallel, against only the road polygons that can actually
		// cut it. A single global difference makes every band pay the vertex count
		// of the whole carriageway.
		const double tDiff0 = FPlatformTime::Seconds();
		TArray<FAxisAlignedBox2d> RoadBounds;
		RoadBounds.Reserve(RoadPolys.Num());
		for (const FGeneralPolygon2d& R : RoadPolys) { RoadBounds.Add(R.Bounds()); }

		TArray<TArray<FGeneralPolygon2d>> PerBand;
		PerBand.SetNum(Merged.Num());
		ParallelFor(Merged.Num(), [&](int32 i)
		{
			const FAxisAlignedBox2d B = Merged[i].Bounds();
			TArray<FGeneralPolygon2d> Cutters;
			for (int32 j = 0; j < RoadPolys.Num(); ++j)
			{
				if (RoadBounds[j].Intersects(B)) { Cutters.Add(RoadPolys[j]); }
			}
			TArray<FGeneralPolygon2d> Subject = { Merged[i] };
			if (Cutters.Num() == 0) { PerBand[i] = MoveTemp(Subject); return; }
			if (!PolygonsDifference(Subject, Cutters, PerBand[i])) { PerBand[i] = MoveTemp(Subject); }
		});
		for (TArray<FGeneralPolygon2d>& P : PerBand) { OutBands.Append(MoveTemp(P)); }

		if (bTrace)
		{
			UE_LOG(LogRoadNet, Log,
				TEXT("[RoadNet]   bands: carriageway subtracted from %d band(s) -> %d in %.1f s"),
				Merged.Num(), OutBands.Num(), FPlatformTime::Seconds() - tDiff0);
		}
		return true;
	}

	bool Difference(
		const TArray<FGeneralPolygon2d>& Subject,
		const TArray<FGeneralPolygon2d>& Clip,
		TArray<FGeneralPolygon2d>& Out)
	{
		Out.Reset();
		if (Subject.Num() == 0) { return true; }
		if (Clip.Num() == 0) { Out = Subject; return true; }
		return PolygonsDifference(Subject, Clip, Out);
	}

	bool Union(const TArray<FGeneralPolygon2d>& Polys, TArray<FGeneralPolygon2d>& Out)
	{
		Out.Reset();
		if (Polys.Num() == 0) { return true; }
		if (Polys.Num() == 1) { Out = Polys; return true; }
		// Same spatial split the carriageway and sidewalk unions use, for the same
		// reason: one global Execute() over thousands of overlapping city-wide
		// polygons is superlinear. It matters HERE because the parcel stage unions
		// every plot with the entire carriageway to build its obstacle set — the
		// callers with a handful of pieces fall through to the plain union below,
		// since UnionSpatial only splits above 64.
		if (Polys.Num() > 64) { return UnionSpatial(Polys, Out, /*CellCm=*/20000.0); }
		return PolygonsUnion(Polys, Out, /*bCopyInputOnFailure*/true);
	}

	// See the header for why chopping is the only thing that helps here.
	static void ChopPolyToCell(FGeneralPolygon2d&& Poly, double CellCm,
		TArray<FGeneralPolygon2d>& Out, int32 Depth = 0)
	{
		const FAxisAlignedBox2d B = Poly.Bounds();
		const double W = B.Max.X - B.Min.X;
		const double H = B.Max.Y - B.Min.Y;
		// ponytail: depth 12 caps a pathological input at 4096 pieces; real input
		// bottoms out far earlier (an 8 km zone needs ~6 splits per axis to 200 m).
		if ((W <= CellCm && H <= CellCm) || Depth >= 12)
		{
			Out.Add(MoveTemp(Poly));
			return;
		}
		const bool bSplitX = W >= H;
		const double Mid = bSplitX ? 0.5 * (B.Min.X + B.Max.X) : 0.5 * (B.Min.Y + B.Max.Y);
		auto HalfRect = [&](bool bLow) -> FGeneralPolygon2d
		{
			double x0 = B.Min.X - 10.0, x1 = B.Max.X + 10.0;
			double y0 = B.Min.Y - 10.0, y1 = B.Max.Y + 10.0;
			if (bSplitX) { (bLow ? x1 : x0) = Mid; }
			else         { (bLow ? y1 : y0) = Mid; }
			FPolygon2d R;
			R.AppendVertex(FVector2d(x0, y0)); R.AppendVertex(FVector2d(x1, y0));
			R.AppendVertex(FVector2d(x1, y1)); R.AppendVertex(FVector2d(x0, y1));
			return FGeneralPolygon2d(R);
		};
		const TArray<FGeneralPolygon2d> Subject = { Poly };
		const TArray<FGeneralPolygon2d> ClipLo  = { HalfRect(true) };
		const TArray<FGeneralPolygon2d> ClipHi  = { HalfRect(false) };
		TArray<FGeneralPolygon2d> Lo, Hi;
		// BOTH halves must clip cleanly or the piece is kept whole — losing half
		// the asphalt to a Clipper failure is worse than one slow polygon.
		if (!PolygonsIntersection(Subject, ClipLo, Lo) ||
			!PolygonsIntersection(Subject, ClipHi, Hi))
		{
			Out.Add(MoveTemp(Poly));
			return;
		}
		for (FGeneralPolygon2d& P : Lo) { ChopPolyToCell(MoveTemp(P), CellCm, Out, Depth + 1); }
		for (FGeneralPolygon2d& P : Hi) { ChopPolyToCell(MoveTemp(P), CellCm, Out, Depth + 1); }
	}

	void ChopToCells(const TArray<FGeneralPolygon2d>& In, double CellCm,
		TArray<FGeneralPolygon2d>& Out)
	{
		Out.Reset();
		Out.Reserve(In.Num());
		for (const FGeneralPolygon2d& P : In)
		{
			ChopPolyToCell(FGeneralPolygon2d(P), CellCm, Out);
		}
	}

	double TotalArea(const TArray<FGeneralPolygon2d>& Polys)
	{
		double A = 0.0;
		for (const FGeneralPolygon2d& P : Polys)
		{
			A += FMath::Abs(P.GetOuter().SignedArea());
			for (const FPolygon2d& H : P.GetHoles()) { A -= FMath::Abs(H.SignedArea()); }
		}
		return A;
	}
}
