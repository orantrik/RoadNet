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

		bool PointInAny(const TArray<FGeneralPolygon2d>& Polys, const FVector2D& P)
		{
			const FVector2d Q(P.X, P.Y);
			for (const FGeneralPolygon2d& GP : Polys)
			{
				if (GP.Contains(Q)) { return true; }
			}
			return false;
		}

		// Walk a chained kerb run (>=2 points) and append evenly spaced pieces.
		// Points are pre-oriented so the sidewalk is on the LEFT of travel.
		void EmitRun(const TArray<FVector2D>& Pts,
			const FCenterlineHeightField& Height, double SpacingCm, double ZLiftCm,
			TArray<FCurbInstance>& Out)
		{
			const int32 N = Pts.Num();
			if (N < 2) { return; }
			const double Fallback = Height.FirstZ();

			// Adaptive chunking: each piece is a CHORD [S..E] of the boundary
			// polyline, so consecutive pieces meet end-to-end and hug the curve.
			// A piece grows up to SpacingCm on straights but is cut early on bends
			// (when the chord deviates from the piece's start heading), so corners
			// get short pieces that follow the arc — the mesh is then stretched to
			// each chord's length (§ "kerbs adjust in size for nice corners").
			constexpr double kMaxPieceTurnRad = 0.20;                 // ~11.5° per piece
			const double MinPieceCm = FMath::Max(20.0, SpacingCm * 0.15);

			auto Emit = [&](const FVector2D& S, const FVector2D& E)
			{
				const double L = FVector2D::Distance(S, E);
				if (L < 1.0) { return; }
				const FVector2D M = 0.5 * (S + E);
				FVector2D Dir = E - S;
				if (!Dir.Normalize()) { return; }
				FCurbInstance CI;
				CI.Location = FVector(M.X, M.Y, Height.SampleHeight(M.X, M.Y, Fallback) + ZLiftCm);
				CI.YawDeg   = (float)FMath::RadiansToDegrees(FMath::Atan2(Dir.Y, Dir.X));
				CI.LengthCm = (float)L;
				Out.Add(CI);
			};

			FVector2D S = Pts[0];          // current piece start
			FVector2D Dir0 = FVector2D::ZeroVector;
			bool bHaveDir0 = false;
			for (int32 i = 1; i < N; ++i)
			{
				if (!bHaveDir0)
				{
					Dir0 = (Pts[i] - S);
					if (Dir0.Normalize()) { bHaveDir0 = true; }
					else { continue; }
				}
				const double ChordLen = FVector2D::Distance(S, Pts[i]);
				FVector2D Chord = (Pts[i] - S).GetSafeNormal();
				const double Turn = FMath::Acos(FMath::Clamp((double)FVector2D::DotProduct(Dir0, Chord), -1.0, 1.0));

				const bool bLong = ChordLen >= SpacingCm;
				const bool bBend = (Turn >= kMaxPieceTurnRad) && (ChordLen >= MinPieceCm);
				if (bLong || bBend)
				{
					Emit(S, Pts[i]);
					S = Pts[i];
					bHaveDir0 = false;
				}
			}
			// Tail: fold a short remainder into the last piece rather than slivering.
			if (FVector2D::Distance(S, Pts.Last()) >= MinPieceCm || Out.Num() == 0)
			{
				Emit(S, Pts.Last());
			}
		}

		// Classify + chain kerb edges on one closed ring, then emit pieces.
		void ProcessRing(const TArray<FVector2d>& V,
			const TArray<FGeneralPolygon2d>& Sidewalks,
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
				const bool bLeft  = PointInAny(Sidewalks, Mid + NLeft * kProbeCm);
				const bool bRight = PointInAny(Sidewalks, Mid - NLeft * kProbeCm);
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

		for (const FGeneralPolygon2d& GP : SurfacePolys)
		{
			ProcessRing(GP.GetOuter().GetVertices(), SidewalkPolys, Height, SpacingCm, ZLiftCm, OutInstances);
			for (const TPolygon2<double>& Hole : GP.GetHoles())
			{
				ProcessRing(Hole.GetVertices(), SidewalkPolys, Height, SpacingCm, ZLiftCm, OutInstances);
			}
		}
	}
}
