// RoadNetSurface.cpp — Clipper2 boolean-union junction surface (§10.9).
#include "RoadNetSurface.h"
#include "RoadNetwork.h"
#include "RoadNetMath.h"
#include "RoadNetLog.h"
#include "Polygon2.h"
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

	// Morphological close (dilate +e then erode -e): rounds concave corners and
	// bridges micro-gaps without changing the overall outer size. In-place on Io.
	static void MorphClose(TArray<FGeneralPolygon2d>& Io, double e)
	{
		if (e <= 0.0 || Io.Num() == 0) { return; }
		TArray<FGeneralPolygon2d> Closed;
		if (PolygonsOffsets(
				+e, -e, Io, Closed, /*bCopyInputOnFailure*/true,
				/*MiterLimit*/2.0, EPolygonOffsetJoinType::Round, EPolygonOffsetEndType::Polygon))
		{
			Io = MoveTemp(Closed);
		}
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
		Outlines.Reserve(Curves.Num() + (ExtraPolys ? ExtraPolys->Num() : 0));
		for (const FRoadCurves* C : Curves)
		{
			if (!C) { continue; }
			FGeneralPolygon2d GP;
			if (BuildRoadOutline(*C, GP)) { Outlines.Add(MoveTemp(GP)); }
		}
		if (ExtraPolys) { Outlines.Append(*ExtraPolys); }
		if (Outlines.Num() == 0) { return true; }

		// Boolean union of all arm outlines → filled, seamless junctions (§10.9).
		if (!PolygonsUnion(Outlines, OutMerged, /*bCopyInputOnFailure*/true))
		{
			UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet] Surface union failed; using unmerged outlines."));
			return true; // OutMerged holds the copied input (bCopyInputOnFailure)
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
		// A junction's rounding is computed on the local surface patch (surface ∩
		// disc) and unioned back, so junctions carry independent smoothing while
		// straight road runs are untouched.
		const double WeldCm = FMath::Min(InflateEpsilonCm, 5.0);
		MorphClose(OutMerged, WeldCm);

		TArray<FGeneralPolygon2d> Patches;
		Patches.Reserve(PerJunction->Num());
		for (const FJunctionClose& J : *PerJunction)
		{
			if (J.CloseCm <= WeldCm) { continue; } // no extra rounding at this junction
			FGeneralPolygon2d Disc;
			MakeDisc(J.Center, J.FillRadiusCm + J.CloseCm + 50.0, /*Segments*/48, Disc);
			const TArray<FGeneralPolygon2d> DiscArr = { MoveTemp(Disc) };

			TArray<FGeneralPolygon2d> Region;
			if (!PolygonsIntersection(OutMerged, DiscArr, Region) || Region.Num() == 0) { continue; }
			MorphClose(Region, J.CloseCm);
			Patches.Append(MoveTemp(Region));
		}

		if (Patches.Num() > 0)
		{
			TArray<FGeneralPolygon2d> All = MoveTemp(OutMerged);
			All.Append(MoveTemp(Patches));
			TArray<FGeneralPolygon2d> Merged2;
			if (PolygonsUnion(All, Merged2, /*bCopyInputOnFailure*/true))
			{
				OutMerged = MoveTemp(Merged2);
			}
			else
			{
				OutMerged = MoveTemp(All); // union failed → keep base + patches
			}
		}
		return true;
	}

	bool BuildSideRibbon(const TArray<FVector>& Sampled, double InnerOff, double OuterOff,
		FGeneralPolygon2d& Out)
	{
		if (Sampled.Num() < 2) { return false; }
		TArray<FVector> Inner, Outer;
		RoadNetMath::OffsetPolyline(Sampled, InnerOff, Inner);
		RoadNetMath::OffsetPolyline(Sampled, OuterOff, Outer);
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

		// Union the per-road side strips so co-linear neighbours merge cleanly.
		TArray<FGeneralPolygon2d> Merged;
		if (!PolygonsUnion(SideRibbons, Merged, /*bCopyInputOnFailure*/true))
		{
			Merged = SideRibbons;
		}
		if (Merged.Num() == 0) { return true; }

		// Subtract the carriageway so a sidewalk never overlaps the road or a
		// junction fill (the discs are already part of RoadPolys).
		if (RoadPolys.Num() == 0) { OutBands = MoveTemp(Merged); return true; }
		return PolygonsDifference(Merged, RoadPolys, OutBands);
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
		return PolygonsUnion(Polys, Out, /*bCopyInputOnFailure*/true);
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
