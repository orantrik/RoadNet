// Parcel access paths — branch the sidewalk off the band and land it on each
// parcel's road-facing edge.
//
// WHY THIS IS NOT ADDED TO THE SIDEWALK BAND
//
// The obvious implementation is to append the spur polygon to Ctx.ZoneSidewalkPolys before the
// commit and let Clipper2's union weld it on. That does produce the mesh — but the band is not a
// mesh input, it is a shared fact about the world with FOUR readers:
//
//   1. CommitLayer("Sidewalks")   RoadNetwork.cpp — meshes it. The one we want.
//   2. the outboard cycle track   RoadNetwork.cpp — SUBTRACTS the band, so appending here would
//                                 carve every spur out of the bike lane it crosses.
//   3. CommitCurbs                RoadNetwork.cpp — generates the kerb line from the boundary
//                                 between carriageway and band, so a spur would grow a kerb down
//                                 BOTH its sides. Right for a driveway crossing, wrong for a path.
//   4. RoadNetFurniture           uses the band as a hard placement guard (OnSidewalk), so
//                                 appending makes the access path eligible ground for benches.
//
// So the spurs stay in their own array and are injected one stage later, straight into the
// "Sidewalks" TILE buckets that BuildTilePartition fills. Those buckets are read only by
// CommitLayer, which is exactly reader 1 and nothing else. The spur still inherits the material,
// the 15 cm kerb lift, the terrain conform and the ground skirt, because all four of those are
// properties of the LAYER, not of the band.
//
// WHAT IT BUILDS
//
// A spur perpendicular from the outer edge of the band to the parcel's nearest edge, plus an apron
// laid along that edge where it lands — the two strokes in the original drawing. Both are clipped
// against the parcels themselves, so a path stops AT a boundary and never covers a seated pad.

#include "RoadNetwork.h"

#include "RoadNetLog.h"
#include "RoadNetMath.h"
#include "RoadNetSurface.h"   // FGeneralPolygon2d, plus the Union/Difference wrappers

#include "Components/SplineComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"   // RoadNet.ParcelAccessSelfCheck

// Max slope (degrees) between NEARBY spline points of DIFFERENT elements —
// free parcel knots against road centrelines, and against each other. Enforced
// on the latent splines (BindParcelsToStreet's height-field pass) before the
// terrain deform and before any mesh, so no two splines can pass close together
// with a cliff of height difference between them.
static TAutoConsoleVariable<float> CVarRoadNetSplineFieldMaxSlopeDeg(
	TEXT("roadnet.SplineFieldMaxSlopeDeg"),
	8.0f,
	TEXT("Max slope (degrees) between nearby spline points of different elements (parcel vs road, parcel vs parcel), reconciled on the latent splines before terrain deform. Default 8."),
	ECVF_Default);

// § ease heights rulers — defined in RoadNetwork.cpp (same module), read here
// so the parcel field pass enforces the SAME step/radius contract the road
// centrelines and sidewalk rings get.
extern TAutoConsoleVariable<int32> CVarRoadNetEaseHeights;
extern TAutoConsoleVariable<float> CVarRoadNetEaseMaxStepCm;
extern TAutoConsoleVariable<float> CVarRoadNetEaseRadiusCm;

namespace
{
	using namespace UE::Geometry;

	// The contract with the parcel importer, which lives in a plugin RoadNet does not depend on.
	// Parcels are plain AActors carrying this tag and a closed spline of this name, so the two
	// sides meet over the level rather than over a build dependency.
	const FName kParcelTag(TEXT("OSMParcel"));
	const FName kFootprintName(TEXT("Footprint"));

	/** One parcel, reduced to what an access path needs: its ring and where it sits. */
	struct FParcelRing
	{
		AActor*           Actor = nullptr;
		USplineComponent* Spline = nullptr;   // the Footprint itself, for vertex re-seating
		TArray<FVector2D> Pts;
		TArray<double>    PtZ;                // per-vertex Z, kept in lockstep with Pts
		FBox2D            Box = FBox2D(ForceInit);
		FVector2D         Centroid = FVector2D::ZeroVector;
	};

	// ---- Phase 3: parcel-vertex bindings ------------------------------------
	// A parcel vertex near a road is not cadastral truth any more — it belongs
	// to the street. The binding records WHERE on the street it belongs (road
	// GUID + station + signed offset), stored as a tag on the parcel actor so
	// the contract stays tag/level like everything else. On every rebuild the
	// bound vertices are re-projected from the CURRENT curve, so editing a road
	// drags its parcels' street edges along instead of stranding them.
	//
	// Tag: osm:bind=<vertexIdx>,<roadGuid>,<stationCm>,<offsetCm>

	struct FVertexBinding
	{
		FGuid  RoadId;
		double StationCm = 0.0;
		double OffsetCm  = 0.0;   // sign matches FProjectResult::Offset (Cross2D convention)
	};

	void ParseBindings(const AActor* Actor, TMap<int32, FVertexBinding>& Out)
	{
		for (const FName& T : Actor->Tags)
		{
			const FString S = T.ToString();
			if (!S.StartsWith(TEXT("osm:bind="))) { continue; }
			TArray<FString> Parts;
			S.Mid(9).ParseIntoArray(Parts, TEXT(","));
			FVertexBinding B;
			if (Parts.Num() == 4 && FGuid::Parse(Parts[1], B.RoadId))
			{
				B.StationCm = FCString::Atod(*Parts[2]);
				B.OffsetCm  = FCString::Atod(*Parts[3]);
				Out.Add(FCString::Atoi(*Parts[0]), B);
			}
		}
	}

	/**
	 * Point on the polyline at arc length S, with unit tangent and interpolated
	 * Z. The inverse of ProjectToPolyline: Pos + (-Tan.Y, Tan.X) * Offset
	 * reconstructs the query point (checked in the self-check below).
	 */
	void EvalPolylineAtArc(const TArray<FVector>& Poly, double S,
		FVector2D& OutPos, FVector2D& OutTan, double& OutZ)
	{
		OutPos = FVector2D(Poly[0].X, Poly[0].Y);
		OutTan = FVector2D(1.0, 0.0);
		OutZ   = Poly[0].Z;
		double Acc = 0.0;
		for (int32 i = 0; i + 1 < Poly.Num(); ++i)
		{
			const FVector2D A(Poly[i].X, Poly[i].Y);
			const FVector2D B(Poly[i + 1].X, Poly[i + 1].Y);
			const double Len = FVector2D::Distance(A, B);
			if (Len < KINDA_SMALL_NUMBER) { continue; }
			OutTan = (B - A) / Len;
			if (S <= Acc + Len || i + 2 == Poly.Num())
			{
				const double T = FMath::Clamp((S - Acc) / Len, 0.0, 1.0);
				OutPos = FMath::Lerp(A, B, T);
				OutZ   = FMath::Lerp(Poly[i].Z, Poly[i + 1].Z, T);
				return;
			}
			Acc += Len;
		}
	}

	/** CCW polygon from an ordered ring, or false if it is too small to mean anything. */
	bool RingToPolygon(const TArray<FVector2D>& Ring, FGeneralPolygon2d& Out)
	{
		if (Ring.Num() < 3) { return false; }
		TArray<FVector2d> Loop;
		Loop.Reserve(Ring.Num());
		for (const FVector2D& P : Ring) { Loop.Emplace(P.X, P.Y); }
		FPolygon2d Poly(Loop);
		if (Poly.VertexCount() < 3) { return false; }
		if (FMath::Abs(Poly.SignedArea()) < 1.0) { return false; } // degenerate sliver
		if (Poly.IsClockwise()) { Poly.Reverse(); }                // outer rings are CCW here
		Out.SetOuter(Poly);
		return true;
	}

	/** CCW rectangle centred at C, extending HalfU along U and HalfV across it. */
	FGeneralPolygon2d MakeRect(const FVector2D& C, const FVector2D& U, double HalfU, double HalfV)
	{
		const FVector2D V(-U.Y, U.X);
		TArray<FVector2d> Loop;
		Loop.Reserve(4);
		Loop.Emplace(C.X - U.X * HalfU - V.X * HalfV, C.Y - U.Y * HalfU - V.Y * HalfV);
		Loop.Emplace(C.X + U.X * HalfU - V.X * HalfV, C.Y + U.Y * HalfU - V.Y * HalfV);
		Loop.Emplace(C.X + U.X * HalfU + V.X * HalfV, C.Y + U.Y * HalfU + V.Y * HalfV);
		Loop.Emplace(C.X - U.X * HalfU + V.X * HalfV, C.Y - U.Y * HalfU + V.Y * HalfV);
		FPolygon2d P(Loop);
		if (P.IsClockwise()) { P.Reverse(); }
		FGeneralPolygon2d G;
		G.SetOuter(P);
		return G;
	}

	/**
	 * Closest point to Q on the ring, treating it as a closed polyline.
	 *
	 * Edges, not vertices: a long straight frontage may have a vertex only at each end, and
	 * snapping the path to the nearer corner would walk it diagonally across the plot boundary
	 * instead of meeting the frontage square on.
	 *
	 * OutEdgeDir is the unit direction of the edge the point landed on, which is what the apron
	 * is laid along.
	 */
	FVector2D ClosestOnRing(const TArray<FVector2D>& Ring, const FVector2D& Q, FVector2D& OutEdgeDir)
	{
		FVector2D Best = Ring[0];
		OutEdgeDir = FVector2D(1.0, 0.0);
		double BestD2 = TNumericLimits<double>::Max();
		for (int32 i = 0, j = Ring.Num() - 1; i < Ring.Num(); j = i++)
		{
			const FVector2D A = Ring[j];
			const FVector2D B = Ring[i];
			const FVector2D AB = B - A;
			const double Len2 = AB.SizeSquared();
			if (Len2 < 1.0) { continue; }
			const double T = FMath::Clamp(FVector2D::DotProduct(Q - A, AB) / Len2, 0.0, 1.0);
			const FVector2D P = A + AB * T;
			const double D2 = FVector2D::DistSquared(P, Q);
			if (D2 < BestD2) { BestD2 = D2; Best = P; OutEdgeDir = AB / FMath::Sqrt(Len2); }
		}
		return Best;
	}

	double RingArcToPoint(const TArray<FVector2D>& Ring, const FVector2D& Q)
	{
		double Acc = 0.0, BestAcc = 0.0, BestD2 = TNumericLimits<double>::Max();
		for (int32 i = 0, j = Ring.Num() - 1; i < Ring.Num(); j = i++)
		{
			const FVector2D A = Ring[j];
			const FVector2D B = Ring[i];
			const FVector2D AB = B - A;
			const double Len2 = AB.SizeSquared();
			if (Len2 < 1.0) { continue; }
			const double Len = FMath::Sqrt(Len2);
			const double T = FMath::Clamp(FVector2D::DotProduct(Q - A, AB) / Len2, 0.0, 1.0);
			const double D2 = FVector2D::DistSquared(Q, A + AB * T);
			if (D2 < BestD2) { BestD2 = D2; BestAcc = Acc + T * Len; }
			Acc += Len;
		}
		return BestAcc;
	}

	/**
	 * The geometry above, checked without a level, a road or a parcel.
	 *
	 * ClosestOnRing is the piece worth pinning: snapping to the nearest VERTEX instead of the
	 * nearest point on an EDGE is the plausible-looking mistake, it still produces a path, and the
	 * path runs diagonally to a corner rather than meeting the frontage square on — which reads as
	 * "the router is a bit off" rather than as a bug.
	 */
	void RunParcelAccessSelfCheck()
	{
		// A square plot, 1000 cm on a side, with vertices only at its corners.
		const TArray<FVector2D> Square = {
			FVector2D(0.0, 0.0), FVector2D(1000.0, 0.0),
			FVector2D(1000.0, 1000.0), FVector2D(0.0, 1000.0) };

		// A road running below it: the landing point must be the middle of the bottom EDGE.
		FVector2D Dir;
		const FVector2D Landing = ClosestOnRing(Square, FVector2D(500.0, -800.0), Dir);
		checkf(FVector2D::Distance(Landing, FVector2D(500.0, 0.0)) < 1.0,
			TEXT("the path must land on the frontage edge, not walk to a corner"));
		// ...and the apron lies ALONG that edge, so its direction is horizontal either way round.
		checkf(FMath::Abs(Dir.Y) < 1e-6 && FMath::IsNearlyEqual(FMath::Abs(Dir.X), 1.0, 1e-6),
			TEXT("the apron direction must follow the edge that was landed on"));

		// A query nearest an actual corner still resolves to that corner.
		const FVector2D Corner = ClosestOnRing(Square, FVector2D(-500.0, -500.0), Dir);
		checkf(FVector2D::Distance(Corner, FVector2D(0.0, 0.0)) < 1.0,
			TEXT("a query off a corner resolves to the corner"));

		// The spur rectangle: right area, and wound CCW like every other polygon fed to Clipper2.
		const FGeneralPolygon2d Rect =
			MakeRect(FVector2D(0.0, 0.0), FVector2D(1.0, 0.0), /*HalfU*/ 200.0, /*HalfV*/ 50.0);
		checkf(!Rect.GetOuter().IsClockwise(), TEXT("outer rings must be CCW"));
		checkf(FMath::IsNearlyEqual(Rect.GetOuter().Area(), 400.0 * 100.0, 1.0),
			TEXT("the spur rectangle must be 2*HalfU by 2*HalfV"));

		// A rectangle built along a diagonal keeps its area — this catches a swapped or
		// un-normalised axis, which would silently taper every spur that is not axis-aligned.
		const FVector2D Diag = FVector2D(1.0, 1.0).GetSafeNormal();
		const FGeneralPolygon2d Tilted = MakeRect(FVector2D(500.0, 500.0), Diag, 200.0, 50.0);
		checkf(FMath::IsNearlyEqual(Tilted.GetOuter().Area(), 400.0 * 100.0, 1.0),
			TEXT("a rotated spur must not change size"));

		// Binding round-trip: EvalPolylineAtArc must be the exact inverse of
		// ProjectToPolyline, or every re-seated parcel vertex drifts sideways a
		// little more on each rebuild.
		{
			TArray<FVector> Road;
			for (int32 i = 0; i <= 10; ++i) { Road.Add(FVector(i * 500.0, 0.0, i * 10.0)); }
			const FVector2D Q(1730.0, 260.0);
			const RoadNetMath::FProjectResult PR = RoadNetMath::ProjectToPolyline(Road, Q);
			FVector2D Pos, Tan; double Z = 0.0;
			EvalPolylineAtArc(Road, PR.AlongDist, Pos, Tan, Z);
			const FVector2D Rebuilt = Pos + FVector2D(-Tan.Y, Tan.X) * PR.Offset;
			checkf(FVector2D::Distance(Rebuilt, Q) < 1.0,
				TEXT("a binding's station+offset must reconstruct the vertex it recorded"));
			checkf(FMath::IsNearlyEqual(Z, 34.6, 0.5),
				TEXT("EvalPolylineAtArc must interpolate Z along the arc"));
		}

		UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] ParcelAccessSelfCheck: PASS"));
	}

	static FAutoConsoleCommand GParcelAccessSelfCheckCmd(
		TEXT("RoadNet.ParcelAccessSelfCheck"),
		TEXT("Assert the parcel access-path geometry helpers (no level needed) and log PASS."),
		FConsoleCommandDelegate::CreateStatic(&RunParcelAccessSelfCheck));

	/** Every OSMParcel footprint in the level, in world XY. */
	void GatherParcels(UWorld* World, TArray<FParcelRing>& Out)
	{
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			AActor* Actor = *It;
			if (!IsValid(Actor) || !Actor->Tags.Contains(kParcelTag)) { continue; }

			USplineComponent* Spline = nullptr;
			Actor->ForEachComponent<USplineComponent>(/*bIncludeFromChildActors*/false,
				[&Spline](USplineComponent* S)
				{
					if (!Spline && S->GetFName() == kFootprintName) { Spline = S; }
				});
			if (!Spline) { continue; }

			const int32 N = Spline->GetNumberOfSplinePoints();
			if (N < 3) { continue; }

			FParcelRing R;
			R.Actor = Actor;
			R.Spline = Spline;
			R.Pts.Reserve(N);
			R.PtZ.Reserve(N);
			for (int32 i = 0; i < N; ++i)
			{
				const FVector W = Spline->GetLocationAtSplinePoint(i, ESplineCoordinateSpace::World);
				R.Pts.Emplace(W.X, W.Y);
				R.PtZ.Add(W.Z);
				R.Box += FVector2D(W.X, W.Y);
			}
			for (const FVector2D& P : R.Pts) { R.Centroid += P; }
			R.Centroid /= (double)R.Pts.Num();
			Out.Add(MoveTemp(R));
		}
	}
}

// ---------------------------------------------------------------------------
// § parcel-street snap — the latent-space seam, parcel side.
//
// A parcel vertex near a road is not cadastral truth any more: it belongs to
// the street. This pass MOVES it onto the road's sidewalk OUTER edge (XY and
// Z), records the binding (road GUID + station + snapped offset) as an
// osm:bind tag, and then cleans the ring — packed knots and collinear noise
// are dropped BEFORE anything downstream (conform, surfaces, fences) reads the
// ring. Bindings re-seat from the CURRENT curve on every rebuild, so editing a
// road drags its parcels' street edges along.
//
// Runs in BOTH rebuild modes: latent (import / Build Street stage 1, before
// the landscape conform) and full (idempotent — the vertices are already on
// the edge, so the snap is a no-op and only the re-seat matters).
// ---------------------------------------------------------------------------
void URoadNetwork::BindParcelsToStreet(FRoadNetRebuildContext& Ctx)
{
	UWorld* World = WorldPtr.Get();
	if (!World || Ctx.Curves.Num() == 0) { return; }

	TArray<FParcelRing> Parcels;
	GatherParcels(World, Parcels);
	if (Parcels.Num() == 0) { return; }

	TMap<int32, FBox2D> RoadBox;
	for (const TPair<int32, FRoadCurves>& KV : Ctx.Curves)
	{
		if (KV.Value.Sampled.Num() < 2) { continue; }
		FBox2D B(ForceInit);
		for (const FVector& P : KV.Value.Sampled) { B += FVector2D(P.X, P.Y); }
		RoadBox.Add(KV.Key, B);
	}

	// Sidewalk OUTER edge distance from the centreline: where a street-facing
	// parcel vertex belongs. Same terms the deform corridor uses for pavement.
	auto BandOuterOf = [this](int32 RoadIdx) -> double
	{
		const FRoadNetLaneSpec& RL = Roads[RoadIdx].Lanes;
		return FMath::Max(50.0, (double)RL.HalfWidthCm())
			+ ((RL.bSidewalkLeft || RL.bSidewalkRight) ? (double)FMath::Max(0.f, RL.SidewalkWidth) : 0.0);
	};

	// A vertex "belongs to the street" within the pavement plus a margin.
	constexpr double kBindMarginCm    = 300.0;
	constexpr double kMinSpacingCm    = 80.0;   // packed parcel knots below this triangulate as spikes
	constexpr double kCollinearTolCm  = 5.0;    // cadastral noise on a straight edge
	int32 Bound = 0, Reseated = 0, Orphaned = 0, Dropped = 0;

	// Per-parcel state shared by ALL the passes below (snap, weld, height
	// field): final bindings, and which parcels were touched. Nothing is written
	// back to the splines until every pass has had its say — one write, one
	// tangent fix, one tag re-stamp per parcel, at the very end.
	TArray<TMap<int32, FVertexBinding>> LiveAll;
	LiveAll.SetNum(Parcels.Num());
	TArray<int32> ParsedCounts;
	ParsedCounts.Init(0, Parcels.Num());
	TBitArray<> Touched(false, Parcels.Num());   // Modify() already called
	TBitArray<> Dirty(false, Parcels.Num());     // needs write-back + re-stamp
	auto Touch = [&](int32 pi)
	{
		Dirty[pi] = true;
		if (!Touched[pi])
		{
			Touched[pi] = true;
			Parcels[pi].Actor->Modify();
			Parcels[pi].Spline->Modify();
		}
	};

	for (int32 pi = 0; pi < Parcels.Num(); ++pi)
	{
		FParcelRing& Parcel = Parcels[pi];
		if (!Parcel.Actor || !Parcel.Spline) { continue; }

		TMap<int32, FVertexBinding> Bindings;
		ParseBindings(Parcel.Actor, Bindings);
		ParsedCounts[pi] = Bindings.Num();
		TMap<int32, FVertexBinding>& Live = LiveAll[pi];   // final snapped bindings

		auto EnsureModify = [&]() { Touch(pi); };

		const int32 NumV = FMath::Min(Parcel.Pts.Num(), Parcel.Spline->GetNumberOfSplinePoints());
		for (int32 i = 0; i < NumV; ++i)
		{
			if (const FVertexBinding* B = Bindings.Find(i))
			{
				// Re-seat from the stored station on the CURRENT curve — this is
				// what makes a parcel follow a road edit. The offset is re-snapped
				// to the current sidewalk outer edge, so a widened road pushes its
				// parcels back instead of paving over them.
				const int32 RoadIdx = FindRoadById(B->RoadId);
				const FRoadCurves* C = (RoadIdx != INDEX_NONE) ? Ctx.Curves.Find(RoadIdx) : nullptr;
				if (!C || C->Sampled.Num() < 2)
				{
					++Orphaned;   // road gone: vertex reverts to cadastral truth (tag not re-stamped)
					continue;
				}
				const double Off = (B->OffsetCm >= 0.0 ? 1.0 : -1.0) * BandOuterOf(RoadIdx);
				FVector2D Pos, Tan; double BedZ = 0.0;
				EvalPolylineAtArc(C->Sampled, B->StationCm, Pos, Tan, BedZ);
				const FVector2D N(-Tan.Y, Tan.X);   // Cross2D(Tan, N) = +1, matches Offset's sign
				const FVector NewW(Pos.X + N.X * Off, Pos.Y + N.Y * Off, BedZ + SidewalkTopLiftCm);
				EnsureModify();
				Parcel.Pts[i] = FVector2D(NewW.X, NewW.Y);
				Parcel.PtZ[i] = NewW.Z;
				Live.Add(i, { B->RoadId, B->StationCm, Off });
				++Reseated;
				continue;
			}

			// No binding yet: does this vertex belong to the street? Nearest
			// eligible road within (pavement + margin) claims it, and the vertex
			// is SNAPPED onto that road's sidewalk outer edge — XY and Z. This is
			// the "road sidewalk splines snapped with parcel splines" rule: after
			// this pass the two lines are the same line.
			int32  BestR = INDEX_NONE;
			double BestD = TNumericLimits<double>::Max();
			RoadNetMath::FProjectResult BestVPR;
			double BestBandOuter = 0.0;
			for (const TPair<int32, FRoadCurves>& KV : Ctx.Curves)
			{
				const int32 r = KV.Key;
				if (!Roads.IsValidIndex(r) || KV.Value.Sampled.Num() < 2) { continue; }
				if (Roads[r].bBridge || Roads[r].bTunnel || Roads[r].Layer != 0) { continue; }
				const double BandOuter = BandOuterOf(r);
				if (const FBox2D* Bx = RoadBox.Find(r))
				{
					if (!Bx->ExpandBy(BandOuter + kBindMarginCm).IsInside(Parcel.Pts[i])) { continue; }
				}
				const RoadNetMath::FProjectResult PR =
					RoadNetMath::ProjectToPolyline(KV.Value.Sampled, Parcel.Pts[i]);
				if (PR.Distance < BestD)
				{
					BestD = PR.Distance; BestR = r; BestVPR = PR; BestBandOuter = BandOuter;
				}
			}
			if (BestR == INDEX_NONE || BestD > BestBandOuter + kBindMarginCm) { continue; }

			FVector2D Pos, Tan; double BedZ = 0.0;
			EvalPolylineAtArc(Ctx.Curves.FindChecked(BestR).Sampled, BestVPR.AlongDist, Pos, Tan, BedZ);
			const FVector2D N(-Tan.Y, Tan.X);
			double Sign = (BestVPR.Offset >= 0.0) ? 1.0 : -1.0;
			if (FMath::Abs(BestVPR.Offset) < 1.0)
			{
				// Dead on the centreline: push toward the parcel's own side.
				Sign = (FVector2D::DotProduct(N, Parcel.Centroid - Pos) >= 0.0) ? 1.0 : -1.0;
			}
			const double Off = Sign * BestBandOuter;
			const FVector NewW(Pos.X + N.X * Off, Pos.Y + N.Y * Off, BedZ + SidewalkTopLiftCm);
			EnsureModify();
			Parcel.Pts[i] = FVector2D(NewW.X, NewW.Y);
			Parcel.PtZ[i] = NewW.Z;
			Live.Add(i, { Roads[BestR].Id, BestVPR.AlongDist, Off });
			++Bound;
		}

		// ---- post-snap ring hygiene ---------------------------------------
		// Snapping can pile vertices up (two cadastral corners projecting to
		// nearly the same station), and cadastral rings carry noise of their
		// own. Drop packed and collinear knots NOW, before any surface, fence
		// or conform reads this ring — this is where "no vertex spikes and bad
		// topology" is enforced for parcels. Bound vertices are only dropped
		// when packed against another BOUND vertex (they sit on the same edge
		// line, so the ring reads identical without one of them).
		{
			int32 N = Parcel.Pts.Num();
			bool bAny = true;
			while (bAny && N > 3)
			{
				bAny = false;
				for (int32 i = 0; i < N && N > 3; )
				{
					const int32 Prev = (i + N - 1) % N;
					const int32 Next = (i + 1) % N;
					const double DPrev = FVector2D::Distance(Parcel.Pts[i], Parcel.Pts[Prev]);
					bool bRemove = false;
					if (!Live.Contains(i))
					{
						if (DPrev < kMinSpacingCm)
						{
							bRemove = true;
						}
						else
						{
							const FVector2D AC = Parcel.Pts[Next] - Parcel.Pts[Prev];
							const double L = AC.Size();
							if (L > 1e-6)
							{
								const FVector2D D = AC / L;
								const FVector2D AB = Parcel.Pts[i] - Parcel.Pts[Prev];
								if (FMath::Abs(AB.X * D.Y - AB.Y * D.X) < kCollinearTolCm)
								{
									bRemove = true;
								}
							}
						}
					}
					else if (Live.Contains(Prev) && DPrev < kMinSpacingCm)
					{
						bRemove = true;   // two bound knots on the same edge line
					}
					if (!bRemove) { ++i; continue; }

					EnsureModify();
					Parcel.Spline->RemoveSplinePoint(i, /*bUpdateSpline*/false);
					Parcel.Pts.RemoveAt(i);
					Parcel.PtZ.RemoveAt(i);
					Live.Remove(i);
					TMap<int32, FVertexBinding> Shifted;
					for (TPair<int32, FVertexBinding>& KV : Live)
					{
						Shifted.Add(KV.Key > i ? KV.Key - 1 : KV.Key, KV.Value);
					}
					Live = MoveTemp(Shifted);
					--N;
					++Dropped;
					bAny = true;
				}
			}
		}

	}

	// -----------------------------------------------------------------------
	// Weld pass — SPLINES CONNECT. Two parcels that share a boundary must share
	// the corner POINT, not merely pass near it: two knots 40 cm apart with
	// independent heights are exactly the "too close, huge dZ" defect. Every
	// cluster of corners within the weld radius collapses onto one position —
	// a street-bound member wins (it sits on the sidewalk edge, which is law),
	// otherwise the cluster averages. XY and Z both.
	// -----------------------------------------------------------------------
	int32 Welded = 0;
	{
		constexpr double kWeldCm   = 50.0;
		constexpr double kWeldCell = 200.0;
		struct FVRef { int32 P; int32 V; };
		TMultiMap<FIntPoint, FVRef> VGrid;
		auto CellOf = [kWeldCell](const FVector2D& P)
		{
			return FIntPoint((int32)FMath::FloorToInt(P.X / kWeldCell),
			                 (int32)FMath::FloorToInt(P.Y / kWeldCell));
		};
		for (int32 pi = 0; pi < Parcels.Num(); ++pi)
		{
			if (!Parcels[pi].Actor || !Parcels[pi].Spline) { continue; }
			for (int32 v = 0; v < Parcels[pi].Pts.Num(); ++v)
			{
				VGrid.Add(CellOf(Parcels[pi].Pts[v]), { pi, v });
			}
		}
		TSet<uint64> Done;
		TArray<FVRef> Bucket, Cluster;
		for (int32 pi = 0; pi < Parcels.Num(); ++pi)
		{
			if (!Parcels[pi].Actor || !Parcels[pi].Spline) { continue; }
			for (int32 v = 0; v < Parcels[pi].Pts.Num(); ++v)
			{
				const uint64 Key = ((uint64)pi << 32) | (uint32)v;
				if (Done.Contains(Key)) { continue; }

				const FVector2D P = Parcels[pi].Pts[v];
				const FIntPoint C = CellOf(P);
				Cluster.Reset();
				for (int32 dx = -1; dx <= 1; ++dx)
				{
					for (int32 dy = -1; dy <= 1; ++dy)
					{
						Bucket.Reset();
						VGrid.MultiFind(FIntPoint(C.X + dx, C.Y + dy), Bucket);
						for (const FVRef& R : Bucket)
						{
							if (FVector2D::Distance(Parcels[R.P].Pts[R.V], P) <= kWeldCm)
							{
								Cluster.Add(R);
							}
						}
					}
				}
				if (Cluster.Num() < 2)
				{
					Done.Add(Key);
					continue;
				}

				// Target: the first street-bound member, else the average.
				FVector2D TXY = FVector2D::ZeroVector;
				double TZ = 0.0;
				bool bBoundTarget = false;
				for (const FVRef& R : Cluster)
				{
					if (LiveAll[R.P].Contains(R.V))
					{
						TXY = Parcels[R.P].Pts[R.V];
						TZ  = Parcels[R.P].PtZ[R.V];
						bBoundTarget = true;
						break;
					}
				}
				if (!bBoundTarget)
				{
					for (const FVRef& R : Cluster) { TXY += Parcels[R.P].Pts[R.V]; TZ += Parcels[R.P].PtZ[R.V]; }
					TXY /= (double)Cluster.Num();
					TZ  /= (double)Cluster.Num();
				}
				for (const FVRef& R : Cluster)
				{
					Done.Add(((uint64)R.P << 32) | (uint32)R.V);
					if (LiveAll[R.P].Contains(R.V)) { continue; }   // bound knots do not move
					if (!Parcels[R.P].Pts[R.V].Equals(TXY, 0.1)
						|| !FMath::IsNearlyEqual(Parcels[R.P].PtZ[R.V], TZ, 0.1))
					{
						Touch(R.P);
						Parcels[R.P].Pts[R.V] = TXY;
						Parcels[R.P].PtZ[R.V] = TZ;
						++Welded;
					}
				}
			}
		}
	}

	// -----------------------------------------------------------------------
	// Height-field pass — NO TWO SPLINE POINTS NEAR EACH OTHER MAY DISAGREE IN
	// HEIGHT faster than the field cap. The pinned truth is the street: every
	// road centreline sample (at sidewalk-top height) and every street-bound
	// parcel knot. Free parcel knots are pulled into the allowed wedge of every
	// pinned and free neighbour around them — Gauss-Seidel, a few sweeps — so a
	// back corner can no longer sit metres above the road beside it, and two
	// abutting parcels cannot disagree across their shared boundary. This runs
	// on SPLINES, before the terrain deform and before any mesh.
	// -----------------------------------------------------------------------
	int32 FieldMoves = 0;
	{
		const double TanCap = FMath::Tan(FMath::DegreesToRadians(FMath::Clamp(
			(double)CVarRoadNetSplineFieldMaxSlopeDeg.GetValueOnAnyThread(), 0.5, 45.0)));
		constexpr double kFieldCellCm = 1500.0;   // = the reconcile radius
		// § ease heights: inside the ease radius the allowance is also capped
		// ABSOLUTELY (max step cm), not just by slope — the user's two rulers.
		// ponytail: the grid search only reaches kFieldCellCm, so an ease
		// radius above 1500 cm is clamped to it; widen the cell if ever needed.
		const bool bEase = CVarRoadNetEaseHeights.GetValueOnAnyThread() != 0;
		const double EaseStep = FMath::Max(1.0, (double)CVarRoadNetEaseMaxStepCm.GetValueOnAnyThread());
		const double EaseRad  = FMath::Clamp((double)CVarRoadNetEaseRadiusCm.GetValueOnAnyThread(), 10.0, kFieldCellCm);
		auto CellOf = [](const FVector2D& P)
		{
			return FIntPoint((int32)FMath::FloorToInt(P.X / kFieldCellCm),
			                 (int32)FMath::FloorToInt(P.Y / kFieldCellCm));
		};

		struct FPinned { FVector2D XY; double Z; };
		TArray<FPinned> Pins;
		TMultiMap<FIntPoint, int32> PinGrid;
		for (const TPair<int32, FRoadCurves>& KV : Ctx.Curves)
		{
			const int32 r = KV.Key;
			if (!Roads.IsValidIndex(r) || Roads[r].bBridge || Roads[r].bTunnel
				|| Roads[r].Layer != 0)
			{
				continue;
			}
			for (const FVector& P : KV.Value.Sampled)
			{
				const int32 Idx = Pins.Add({ FVector2D(P.X, P.Y), P.Z + SidewalkTopLiftCm });
				PinGrid.Add(CellOf(Pins[Idx].XY), Idx);
			}
		}
		struct FFree { int32 P; int32 V; };
		TArray<FFree> Frees;
		TMultiMap<FIntPoint, int32> FreeGrid;
		for (int32 pi = 0; pi < Parcels.Num(); ++pi)
		{
			if (!Parcels[pi].Actor || !Parcels[pi].Spline) { continue; }
			for (int32 v = 0; v < Parcels[pi].Pts.Num(); ++v)
			{
				if (LiveAll[pi].Contains(v))
				{
					const int32 Idx = Pins.Add({ Parcels[pi].Pts[v], Parcels[pi].PtZ[v] });
					PinGrid.Add(CellOf(Pins[Idx].XY), Idx);
				}
				else
				{
					const int32 Idx = Frees.Add({ pi, v });
					FreeGrid.Add(CellOf(Parcels[pi].Pts[v]), Idx);
				}
			}
		}

		TArray<int32> Bucket;
		constexpr int32 kSweeps = 4;
		for (int32 Sweep = 0; Sweep < kSweeps; ++Sweep)
		{
			bool bAny = false;
			for (int32 f = 0; f < Frees.Num(); ++f)
			{
				FParcelRing& Pr = Parcels[Frees[f].P];
				const FVector2D XY = Pr.Pts[Frees[f].V];
				double& Z = Pr.PtZ[Frees[f].V];
				const FIntPoint C = CellOf(XY);
				double Lo = -1.0e18, Hi = 1.0e18;
				for (int32 dx = -1; dx <= 1; ++dx)
				{
					for (int32 dy = -1; dy <= 1; ++dy)
					{
						const FIntPoint Cell(C.X + dx, C.Y + dy);
						Bucket.Reset();
						PinGrid.MultiFind(Cell, Bucket);
						for (const int32 Idx : Bucket)
						{
							const double D = FVector2D::Distance(Pins[Idx].XY, XY);
							if (D > kFieldCellCm) { continue; }
							double Allow = TanCap * D + 2.0;
							if (bEase && D <= EaseRad) { Allow = FMath::Min(Allow, EaseStep); }
							Lo = FMath::Max(Lo, Pins[Idx].Z - Allow);
							Hi = FMath::Min(Hi, Pins[Idx].Z + Allow);
						}
						Bucket.Reset();
						FreeGrid.MultiFind(Cell, Bucket);
						for (const int32 Idx : Bucket)
						{
							if (Idx == f) { continue; }
							const FParcelRing& Qr = Parcels[Frees[Idx].P];
							const double D = FVector2D::Distance(Qr.Pts[Frees[Idx].V], XY);
							if (D > kFieldCellCm) { continue; }
							double Allow = TanCap * D + 2.0;
							if (bEase && D <= EaseRad) { Allow = FMath::Min(Allow, EaseStep); }
							Lo = FMath::Max(Lo, Qr.PtZ[Frees[Idx].V] - Allow);
							Hi = FMath::Min(Hi, Qr.PtZ[Frees[Idx].V] + Allow);
						}
					}
				}
				const double NewZ = (Lo > Hi) ? 0.5 * (Lo + Hi) : FMath::Clamp(Z, Lo, Hi);
				if (!FMath::IsNearlyEqual(NewZ, Z, 0.5))
				{
					Touch(Frees[f].P);
					Z = NewZ;
					++FieldMoves;
					bAny = true;
				}
			}
			if (!bAny) { break; }
		}
	}

	// -----------------------------------------------------------------------
	// Write-back — once, at the very end: every touched parcel gets its points
	// set from the reconciled (Pts, PtZ), ALL points forced to LINEAR (a
	// cadastral ring is a polygon; curve tangents between snapped knots are the
	// mangled loops in the viewport), tags re-stamped, spline updated.
	// -----------------------------------------------------------------------
	for (int32 pi = 0; pi < Parcels.Num(); ++pi)
	{
		FParcelRing& Parcel = Parcels[pi];
		if (!Parcel.Actor || !Parcel.Spline) { continue; }
		if (!Dirty[pi] && LiveAll[pi].Num() == ParsedCounts[pi]) { continue; }
		Touch(pi);

		const int32 N = FMath::Min(Parcel.Pts.Num(), Parcel.Spline->GetNumberOfSplinePoints());
		for (int32 i = 0; i < N; ++i)
		{
			Parcel.Spline->SetLocationAtSplinePoint(i,
				FVector(Parcel.Pts[i].X, Parcel.Pts[i].Y, Parcel.PtZ[i]),
				ESplineCoordinateSpace::World, /*bUpdateSpline*/false);
			Parcel.Spline->SetSplinePointType(i, ESplinePointType::Linear, /*bUpdateSpline*/false);
		}
		Parcel.Actor->Tags.RemoveAll([](const FName& T)
		{
			return T.ToString().StartsWith(TEXT("osm:bind="));
		});
		for (const TPair<int32, FVertexBinding>& KV : LiveAll[pi])
		{
			Parcel.Actor->Tags.Add(*FString::Printf(TEXT("osm:bind=%d,%s,%.0f,%.0f"),
				KV.Key, *KV.Value.RoadId.ToString(EGuidFormats::DigitsWithHyphens),
				KV.Value.StationCm, KV.Value.OffsetCm));
		}
		Parcel.Spline->UpdateSpline();
		Parcel.Box = FBox2D(ForceInit);
		Parcel.Centroid = FVector2D::ZeroVector;
		for (const FVector2D& P : Parcel.Pts) { Parcel.Box += P; Parcel.Centroid += P; }
		Parcel.Centroid /= (double)FMath::Max(1, Parcel.Pts.Num());
	}

	if (Bound + Reseated + Orphaned + Dropped + Welded + FieldMoves > 0)
	{
		UE_LOG(LogRoadNet, Log,
			TEXT("[RoadNet] SplineField: %d vertex(es) snapped to sidewalk edges (new), %d re-seated, %d orphaned, %d packed/collinear knot(s) dropped, %d corner(s) welded across parcels, %d height(s) reconciled against neighbouring splines."),
			Bound, Reseated, Orphaned, Dropped, Welded, FieldMoves);
	}
}

void URoadNetwork::BuildParcelAccessPaths(FRoadNetRebuildContext& Ctx)
{
	using namespace UE::Geometry;

	Ctx.ParcelAccessPolys.Reset();
	PlanParcelFrontages.Reset();   // street plan API: rebuilt below alongside the tags
	if (!bBuildParcelAccessPaths || ParcelAccessWidthCm <= 0.f) { return; }

	UWorld* World = WorldPtr.Get();
	if (!World) { return; }

	// Snap + clean FIRST (idempotent if the latent pass already ran), so every
	// ring this stage measures is the ring the level actually has.
	BindParcelsToStreet(Ctx);

	TArray<FParcelRing> Parcels;
	GatherParcels(World, Parcels);
	if (Parcels.Num() == 0) { return; } // no parcels in the level: this stage costs nothing

	// The band's own tile buckets. Absent means BuildTilePartition emitted no sidewalk at all,
	// in which case there is no pavement for a path to branch off — but we still stamp
	// osm:access so parcel fences can cut a gate on the road-facing edge.
	TArray<TMap<FIntPoint, TArray<FGeneralPolygon2d>>>* WalkBuckets =
		Ctx.ZoneTileLayers.Find(FName(TEXT("Sidewalks")));

	// Which zone each road belongs to, so a spur is filed under the same zone as the road it
	// leaves — zones are grade-separated, and a path must not join an overpass's pavement.
	TMap<int32, int32> RoadZone;
	for (int32 z = 0; z < Ctx.Zones.Num(); ++z)
	{
		for (int32 r : Ctx.Zones[z]) { RoadZone.Add(r, z); }
	}

	// Road bounding boxes, so the per-parcel search can reject most roads without walking their
	// polylines.
	//
	// ponytail: still O(parcels x roads) in the worst case, with the box test as the only guard.
	// Fine at the few-hundred-parcel scale this imports; a city-wide run would want the roads in
	// the same coarse grid TopoKeyOf uses.
	const double ReachCm = FMath::Max(1.0, (double)ParcelAccessMaxReachM * 100.0);
	TMap<int32, FBox2D> RoadBox;
	for (const TPair<int32, FRoadCurves>& KV : Ctx.Curves)
	{
		if (KV.Value.Sampled.Num() < 2) { continue; }
		FBox2D B(ForceInit);
		for (const FVector& P : KV.Value.Sampled) { B += FVector2D(P.X, P.Y); }
		RoadBox.Add(KV.Key, B);
	}

	// Everything a path must not cover, unioned once rather than per parcel: the plots themselves
	// (the path stops AT a boundary, and a seated pad must stay untouched) and the carriageway
	// (a footpath may reach the kerb, never cross it). Built AFTER the binding pass above, so
	// the obstacles are the rings the level actually has now.
	TArray<FGeneralPolygon2d> Obstacles;
	for (const FParcelRing& P : Parcels)
	{
		FGeneralPolygon2d GP;
		if (RingToPolygon(P.Pts, GP)) { Obstacles.Add(MoveTemp(GP)); }
	}
	for (const TArray<FGeneralPolygon2d>& ZoneSurf : Ctx.ZoneSurfacePolys) { Obstacles.Append(ZoneSurf); }
	{
		TArray<FGeneralPolygon2d> Merged;
		if (RoadNetSurface::Union(Obstacles, Merged)) { Obstacles = MoveTemp(Merged); }
	}

	int32 Built = 0, TooFar = 0, AlreadyReached = 0, Degenerate = 0;

	for (const FParcelRing& Parcel : Parcels)
	{
		// 1) Which road fronts this plot. Nearest by the ring, not by the centroid: a long thin
		// plot's centroid can sit closer to a road its frontage does not face.
		int32     BestRoad = INDEX_NONE;
		double    BestDist = TNumericLimits<double>::Max();
		FVector2D BestOnRing = FVector2D::ZeroVector;
		FVector2D BestEdgeDir = FVector2D(1.0, 0.0);
		RoadNetMath::FProjectResult BestPR;

		for (const TPair<int32, FRoadCurves>& KV : Ctx.Curves)
		{
			const int32 r = KV.Key;
			if (!Roads.IsValidIndex(r) || KV.Value.Sampled.Num() < 2) { continue; }
			if (!RoadZone.Contains(r)) { continue; }
			// A bridge deck or a tunnel bore has no kerb at ground level to branch from.
			if (Roads[r].bBridge || Roads[r].bTunnel || Roads[r].Layer != 0) { continue; }

			// Reach is measured frontage-to-KERB, but the box is around the CENTRELINE, so the
			// margin has to carry the road's own half-width and pavement as well. One margin on
			// one box: expanding both would quietly double the setting.
			const FRoadNetLaneSpec& RL = Roads[r].Lanes;
			const double RBandOuter = FMath::Max(50.0, (double)RL.HalfWidthCm())
				+ ((RL.bSidewalkLeft || RL.bSidewalkRight) ? (double)FMath::Max(0.f, RL.SidewalkWidth) : 0.0);
			if (const FBox2D* B = RoadBox.Find(r))
			{
				if (!B->ExpandBy(ReachCm + RBandOuter).Intersect(Parcel.Box)) { continue; }
			}

			// Project the plot's centroid to find roughly where the road runs past it, then take
			// the ring point nearest THAT, then re-project. Two cheap steps instead of testing
			// every ring vertex against every road.
			const RoadNetMath::FProjectResult Coarse =
				RoadNetMath::ProjectToPolyline(KV.Value.Sampled, Parcel.Centroid);
			FVector2D EdgeDir;
			const FVector2D OnRing = ClosestOnRing(Parcel.Pts, Coarse.Point, EdgeDir);
			const RoadNetMath::FProjectResult PR =
				RoadNetMath::ProjectToPolyline(KV.Value.Sampled, OnRing);
			if (PR.Distance < BestDist)
			{
				BestDist = PR.Distance; BestRoad = r;
				BestOnRing = OnRing; BestEdgeDir = EdgeDir; BestPR = PR;
			}
		}

		if (BestRoad == INDEX_NONE) { ++TooFar; continue; }

		if (Parcel.Actor)
		{
			const double ArcCm = RingArcToPoint(Parcel.Pts, BestOnRing);
			Parcel.Actor->Tags.RemoveAll([](const FName& T)
			{
				const FString S = T.ToString();
				return S.StartsWith(TEXT("osm:access=")) || S.StartsWith(TEXT("osm:frontage="));
			});
			Parcel.Actor->Tags.Add(*FString::Printf(TEXT("osm:access=%.0f,%.0f"),
				ArcCm, (double)ParcelAccessWidthCm));

			// Street plan: the sidewalk-TOP height where this parcel meets the
			// street. Interpolated from the reconciled curve (the same profile
			// the pavement is meshed from), so a parcel plate or fence base that
			// reads this tag lands exactly on the pavement, not near it.
			const TArray<FVector>& CL = Ctx.Curves.FindChecked(BestRoad).Sampled;
			double BedZ = CL[0].Z;
			if (CL.IsValidIndex(BestPR.Segment) && CL.IsValidIndex(BestPR.Segment + 1))
			{
				const FVector& A = CL[BestPR.Segment];
				const FVector& B = CL[BestPR.Segment + 1];
				const double SegLen = FVector::Dist2D(A, B);
				const double T = (SegLen > 1.0)
					? FMath::Clamp(FVector2D::Distance(FVector2D(A.X, A.Y), BestPR.Point) / SegLen, 0.0, 1.0)
					: 0.0;
				BedZ = FMath::Lerp(A.Z, B.Z, T);
			}
			const double FrontZTop = BedZ + SidewalkTopLiftCm;
			Parcel.Actor->Tags.Add(*FString::Printf(TEXT("osm:frontage=%.0f,%.0f"),
				ArcCm, FrontZTop));

			FRoadNetParcelFrontage& F = PlanParcelFrontages.AddDefaulted_GetRef();
			F.Parcel      = Parcel.Actor;
			F.RoadId      = Roads[BestRoad].Id;
			F.StationCm   = BestPR.AlongDist;
			F.ArcCm       = ArcCm;
			F.FrontZTopCm = FrontZTop;
		}

		if (!WalkBuckets) { continue; }

		// 2) Where the band's outer edge is on that side. The spur departs from there, not from
		// the centreline, so it starts where the pavement actually ends.
		const FRoadNetLaneSpec& L = Roads[BestRoad].Lanes;
		const double Half = FMath::Max(50.0, (double)L.HalfWidthCm());
		const double WalkW = (L.bSidewalkLeft || L.bSidewalkRight)
			? (double)FMath::Max(0.f, L.SidewalkWidth) : 0.0;
		const double BandOuter = Half + WalkW;

		if (BestDist - BandOuter > ReachCm) { ++TooFar; continue; }
		// Frontage already under the pavement: the plot is reachable without a path, and building
		// one anyway would lay a second coplanar slab over the band and z-fight with it.
		if (BestDist <= BandOuter) { ++AlreadyReached; continue; }

		// Direction from the centreline out to the frontage. Degenerate only when the frontage
		// lies exactly on the centreline, which means the plot overlaps the road.
		FVector2D Out = BestOnRing - BestPR.Point;
		if (!Out.Normalize()) { ++Degenerate; continue; }

		// 3) The spur. It starts INSIDE the band by a whisker so the two meet with no sliver of
		// terrain between them, and ends at the frontage; the obstacle subtraction below trims
		// whatever of it lands on pavement or inside a plot.
		constexpr double kWeldCm = 40.0;
		const FVector2D Start = BestPR.Point + Out * FMath::Max(0.0, BandOuter - kWeldCm);
		const FVector2D End   = BestOnRing;
		FVector2D Along = End - Start;
		const double SpurLen = Along.Size();
		if (SpurLen < 1.0 || !Along.Normalize()) { ++AlreadyReached; continue; }

		TArray<FGeneralPolygon2d> Pieces;
		Pieces.Add(MakeRect((Start + End) * 0.5, Along, SpurLen * 0.5, ParcelAccessWidthCm * 0.5));

		// 4) The apron along the frontage. Centred on the landing point and laid along the edge
		// it landed on; the half that falls inside the plot is removed with the obstacles, which
		// is what makes it hug the boundary from outside rather than straddle it.
		if (ParcelAccessApronCm > 0.f)
		{
			Pieces.Add(MakeRect(End, BestEdgeDir,
				(double)ParcelAccessApronCm * 0.5, (double)ParcelAccessWidthCm * 0.5));
		}

		TArray<FGeneralPolygon2d> Shape;
		if (!RoadNetSurface::Union(Pieces, Shape) || Shape.Num() == 0) { ++Degenerate; continue; }

		TArray<FGeneralPolygon2d> Trimmed;
		if (!RoadNetSurface::Difference(Shape, Obstacles, Trimmed) || Trimmed.Num() == 0)
		{
			++AlreadyReached; // entirely swallowed by pavement or plots: nothing left to add
			continue;
		}

		// 5) File it under the tile that owns the ground it crosses, the same way every other
		// piece of geometry here is filed — by construction, not by a later spatial guess.
		const int32* ZonePtr = RoadZone.Find(BestRoad);
		if (!ZonePtr || !WalkBuckets->IsValidIndex(*ZonePtr)) { ++Degenerate; continue; }
		const FVector2D Mid = (Start + End) * 0.5;
		const FIntPoint Key = TopoKeyOf(FVector(Mid.X, Mid.Y, 0.0), Ctx);
		if (Key.X == INDEX_NONE) { ++TooFar; continue; }

		// The band's own polygons in this tile are already there and are coplanar with the spur,
		// so overlapping them would z-fight rather than weld. Subtract what is present, exactly
		// as BuildTilePartition's own EmitPieces does for the pieces it emits.
		TArray<FGeneralPolygon2d>& Bucket = (*WalkBuckets)[*ZonePtr].FindOrAdd(Key);
		TArray<FGeneralPolygon2d> Final;
		if (Bucket.Num() > 0)
		{
			if (!RoadNetSurface::Difference(Trimmed, Bucket, Final)) { Final = MoveTemp(Trimmed); }
		}
		else
		{
			Final = MoveTemp(Trimmed);
		}
		if (Final.Num() == 0) { ++AlreadyReached; continue; }

		Ctx.ParcelAccessPolys.Append(Final);
		Bucket.Append(MoveTemp(Final));
		++Built;
	}

	// "Strive to" is the spec: a plot that could not be reached says so and is left alone, rather
	// than being given a route across its neighbours.
	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet] ParcelAccess: %d parcel(s) -> %d path(s) built; %d out of reach (>%.0f m), "
			 "%d already on pavement, %d degenerate. Spurs are meshed with the sidewalks but are NOT "
			 "part of the band, so they grow no kerbs and take no street furniture."),
		Parcels.Num(), Built, TooFar, (double)ParcelAccessMaxReachM, AlreadyReached, Degenerate);
}
