// RoadNetCurbs.cpp — kerb-line instance placement (see RoadNetCurbs.h).
#include "RoadNetCurbs.h"
#include "Algo/Reverse.h"

using namespace UE::Geometry;
using namespace RoadNetMesh;

namespace RoadNetCurbs
{
	namespace
	{
		// How far off an edge to probe when classifying it as a kerb edge (cm).
		constexpr double kProbeCm = 40.0;
		// Ignore boundary edges shorter than this (Clipper micro-segments, cm).
		constexpr double kMinEdgeCm = 1.0;

		// Bounds[i] is Polys[i]'s outer-ring box, so a box miss is a definite miss
		// and the expensive ring walk is skipped. Without this reject the kerb pass
		// is O(surface edges x every vertex of every sidewalk polygon) — on a
		// city-sized network that is the single most expensive thing in a rebuild.
		bool PointInAny(const TArray<FGeneralPolygon2d>& Polys,
			const TArray<FAxisAlignedBox2d>& Bounds, const FVector2D& P)
		{
			const FVector2d Q(P.X, P.Y);
			for (int32 i = 0; i < Polys.Num(); ++i)
			{
				if (!Bounds[i].Contains(Q)) { continue; }
				if (Polys[i].Contains(Q)) { return true; }
			}
			return false;
		}

		// Walk a chained kerb run (>=2 points) and append pieces. Points are
		// pre-oriented so the sidewalk is on the LEFT of travel.
		//
		// Constant length on straights, deform only at corners: pieces are tiled
		// at EXACTLY SpacingCm (the standard kerb length) by ARC LENGTH — cutting
		// segments mid-way instead of ending at boundary vertices — so every
		// straight stone is the same size (mesh scale 1, no stretch). A piece is
		// cut SHORT only where the boundary bends away from the piece's start
		// heading before a full standard length, so the mesh compresses to follow
		// the arc just at corners/curves.
		void EmitRun(const TArray<FVector2D>& Pts,
			const FCenterlineHeightField& Height, double SpacingCm, double ZLiftCm,
			TArray<FCurbInstance>& Out)
		{
			const int32 N = Pts.Num();
			if (N < 2) { return; }
			const double Fallback = Height.FirstZ();

			// Cumulative arc length of the boundary polyline.
			TArray<double> S; S.SetNumUninitialized(N);
			S[0] = 0.0;
			for (int32 i = 1; i < N; ++i) { S[i] = S[i - 1] + FVector2D::Distance(Pts[i - 1], Pts[i]); }
			const double Total = S[N - 1];
			if (Total < 1.0) { return; }

			const double StdLenCm = FMath::Max(20.0, SpacingCm);   // standard piece length
			constexpr double kMaxPieceTurnRad = 0.20;              // ~11.5° → cut on curves

			// Pieces are cut on the PLAN shape only. A kerb stone is a rigid
			// precast block: a layer sets it to the grade and lets the joints take
			// up the curvature, so the stone never subdivides to trace a vertical
			// curve. The code used to halve a piece until its chord matched the
			// draped grade at its midpoint, which meant every crest and sag in the
			// height field spawned a run of stunted stones — and at a genuine step
			// it bottomed out at the minimum length and produced the shattered,
			// tilted chains around junctions.

			// Position along the polyline at arc length s (forward-only hint).
			auto PosAt = [&](double s, int32& Hint) -> FVector2D
			{
				s = FMath::Clamp(s, 0.0, Total);
				while (Hint + 1 < N && S[Hint + 1] < s) { ++Hint; }
				const double SegLen = S[Hint + 1] - S[Hint];
				const double a = SegLen > 1e-6 ? (s - S[Hint]) / SegLen : 0.0;
				return FMath::Lerp(Pts[Hint], Pts[Hint + 1], a);
			};
			// Unit heading of the segment that contains arc length s.
			auto HeadingAt = [&](double s) -> FVector2D
			{
				int32 k = 0;
				while (k + 1 < N - 1 && S[k + 1] <= s) { ++k; }
				return (Pts[k + 1] - Pts[k]).GetSafeNormal();
			};

			auto Emit = [&](const FVector2D& A, const FVector2D& B)
			{
				const double L = FVector2D::Distance(A, B);
				if (L < 1.0) { return; }
				FVector2D Dir = B - A;
				if (!Dir.Normalize()) { return; }
				const FVector2D M = 0.5 * (A + B);

				// Sample terrain at BOTH ends so the piece follows the longitudinal
				// grade (seat at mean height, pitch over its horizontal length).
				const double Zs = Height.SampleHeight(A.X, A.Y, Fallback) + ZLiftCm;
				const double Ze = Height.SampleHeight(B.X, B.Y, Fallback) + ZLiftCm;

				FCurbInstance CI;
				CI.Location = FVector(M.X, M.Y, 0.5 * (Zs + Ze));
				CI.YawDeg   = (float)FMath::RadiansToDegrees(FMath::Atan2(Dir.Y, Dir.X));
				CI.PitchDeg = (float)FMath::RadiansToDegrees(FMath::Atan2(Ze - Zs, L));
				CI.LengthCm = (float)L;
				Out.Add(CI);
			};

			double s = 0.0;
			int32  HintS = 0, HintE = 0, Guard = 0;
			while (s < Total - 1.0 && Guard++ < 200000)
			{
				double e = s + FMath::Min(StdLenCm, Total - s);

				// Cut early at the first boundary vertex whose
				// heading has turned away from the piece's start heading — that is
				// the corner/curve where the stone must shorten to follow the arc.
				const FVector2D Dir0 = HeadingAt(s);
				for (int32 k = 1; k + 1 < N; ++k)
				{
					// A short remainder before a corner is still a real piece. Skipping
					// it makes the next stone cut diagonally across the junction.
					if (S[k] <= s + 1.e-6) { continue; }
					if (S[k] >= e) { break; }
					const FVector2D Hk = (Pts[k + 1] - Pts[k]).GetSafeNormal();
					const double Turn = FMath::Acos(FMath::Clamp((double)FVector2D::DotProduct(Dir0, Hk), -1.0, 1.0));
					if (Turn > kMaxPieceTurnRad) { e = S[k]; break; }
				}

				// Do not extend a corner cut or fold a tail across it: that would
				// undo the heading check above. Short end pieces are intentional.

				HintE = HintS;
				Emit(PosAt(s, HintS), PosAt(e, HintE));
				s = e;
			}
		}

		// Classify + chain kerb edges on one closed ring, then emit pieces.
		void ProcessRing(const TArray<FVector2d>& V,
			const TArray<FGeneralPolygon2d>& Sidewalks,
			const TArray<FAxisAlignedBox2d>& SidewalkBounds,
			const FCenterlineHeightField& Height, double SpacingCm, double ZLiftCm,
			TArray<FCurbInstance>& Out)
		{
			const int32 N = V.Num();
			if (N < 2) { return; }

			// Per-edge: is it a kerb edge, and is the sidewalk on its left?
			TArray<uint8> IsCurb;   IsCurb.SetNumZeroed(N);
			TArray<uint8> SwLeft;   SwLeft.SetNumZeroed(N);
			for (int32 i = 0; i < N; ++i)
			{
				const FVector2D A(V[i].X, V[i].Y);
				const FVector2D B(V[(i + 1) % N].X, V[(i + 1) % N].Y);
				FVector2D Dir = B - A;
				if (Dir.Size() < kMinEdgeCm) { continue; }
				Dir.Normalize();
				const FVector2D Mid = 0.5 * (A + B);
				const FVector2D NLeft(-Dir.Y, Dir.X);      // +90° (CCW) of travel
				const bool bLeft  = PointInAny(Sidewalks, SidewalkBounds, Mid + NLeft * kProbeCm);
				const bool bRight = PointInAny(Sidewalks, SidewalkBounds, Mid - NLeft * kProbeCm);
				if (bLeft || bRight)
				{
					IsCurb[i] = 1;
					SwLeft[i]  = bLeft ? 1 : 0;   // prefer left if both
				}
			}

			// Turn a list of consecutive edge indices into an oriented polyline
			// (sidewalk forced onto the left) and emit it.
			auto Flush = [&](const TArray<int32>& Edges)
			{
				if (Edges.Num() == 0) { return; }
				TArray<FVector2D> Pts;
				Pts.Reserve(Edges.Num() + 1);
				for (int32 e : Edges) { Pts.Add(FVector2D(V[e].X, V[e].Y)); }
				const int32 Last = Edges.Last();
				Pts.Add(FVector2D(V[(Last + 1) % N].X, V[(Last + 1) % N].Y));

				// If the sidewalk is on the right of the run's travel, reverse so
				// it lands on the left (mesh +Y) consistently.
				if (SwLeft[Edges[0]] == 0) { Algo::Reverse(Pts); }
				EmitRun(Pts, Height, SpacingCm, ZLiftCm, Out);
			};

			bool bAll = true;
			for (int32 i = 0; i < N; ++i) { if (!IsCurb[i]) { bAll = false; break; } }

			if (bAll)
			{
				TArray<int32> Edges; Edges.Reserve(N);
				for (int32 i = 0; i < N; ++i) { Edges.Add(i); }
				Flush(Edges);
				return;
			}

			// Start at a non-kerb edge so runs never wrap the seam, then collect
			// maximal consecutive kerb runs.
			int32 S0 = 0;
			while (S0 < N && IsCurb[S0]) { ++S0; }
			TArray<int32> Cur;
			for (int32 off = 1; off <= N; ++off)
			{
				const int32 i = (S0 + off) % N;
				if (IsCurb[i]) { Cur.Add(i); }
				else if (Cur.Num() > 0) { Flush(Cur); Cur.Reset(); }
			}
			if (Cur.Num() > 0) { Flush(Cur); }
		}
	}

	void BuildCurbInstancesForZone(
		const TArray<FGeneralPolygon2d>& SurfacePolys,
		const TArray<FGeneralPolygon2d>& SidewalkPolys,
		const FCenterlineHeightField& Height,
		double SpacingCm,
		double ZLiftCm,
		TArray<FCurbInstance>& OutInstances)
	{
		if (SurfacePolys.Num() == 0 || SidewalkPolys.Num() == 0) { return; }
		SpacingCm = FMath::Max(50.0, SpacingCm);

		// Once per zone, not once per probe: the sidewalk set is constant for the
		// whole ring walk and TPolygon2::Bounds() is itself a full vertex scan.
		TArray<FAxisAlignedBox2d> SidewalkBounds;
		SidewalkBounds.Reserve(SidewalkPolys.Num());
		for (const FGeneralPolygon2d& GP : SidewalkPolys)
		{
			SidewalkBounds.Add(GP.Bounds());
		}

		for (const FGeneralPolygon2d& GP : SurfacePolys)
		{
			ProcessRing(GP.GetOuter().GetVertices(), SidewalkPolys, SidewalkBounds,
				Height, SpacingCm, ZLiftCm, OutInstances);
			for (const TPolygon2<double>& Hole : GP.GetHoles())
			{
				ProcessRing(Hole.GetVertices(), SidewalkPolys, SidewalkBounds,
					Height, SpacingCm, ZLiftCm, OutInstances);
			}
		}
	}

	void BuildCurbInstancesAlongLine(
		const TArray<FVector>& Line,
		const FCenterlineHeightField& Height,
		double SpacingCm,
		double ZLiftCm,
		TArray<FCurbInstance>& OutInstances)
	{
		if (Line.Num() < 2) { return; }
		SpacingCm = FMath::Max(50.0, SpacingCm);
		TArray<FVector2D> Pts;
		Pts.Reserve(Line.Num());
		for (const FVector& P : Line) { Pts.Emplace(P.X, P.Y); }
		EmitRun(Pts, Height, SpacingCm, ZLiftCm, OutInstances);
	}
}
