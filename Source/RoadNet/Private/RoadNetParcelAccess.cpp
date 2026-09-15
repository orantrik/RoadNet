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
			for (int32 i = 0; i < N; ++i)
			{
				const FVector W = Spline->GetLocationAtSplinePoint(i, ESplineCoordinateSpace::World);
				R.Pts.Emplace(W.X, W.Y);
				R.Box += FVector2D(W.X, W.Y);
			}
			for (const FVector2D& P : R.Pts) { R.Centroid += P; }
			R.Centroid /= (double)R.Pts.Num();
			Out.Add(MoveTemp(R));
		}
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

	// -----------------------------------------------------------------------
	// Phase 3 — vertex bindings: record which parcel vertices belong to the
	// street, and re-seat previously bound vertices from the CURRENT curves.
	// Runs BEFORE the access/frontage pass so arcs are measured on the ring
	// the parcel will actually have.
	// -----------------------------------------------------------------------
	{
		// A vertex "belongs to the street" within the pavement plus a margin.
		constexpr double kBindMarginCm = 300.0;
		int32 Bound = 0, Reseated = 0, Orphaned = 0;

		for (FParcelRing& Parcel : Parcels)
		{
			if (!Parcel.Actor || !Parcel.Spline) { continue; }

			TMap<int32, FVertexBinding> Bindings;
			ParseBindings(Parcel.Actor, Bindings);

			bool bMovedAny = false;
			bool bModified = false;
			auto EnsureModify = [&]()
			{
				if (!bModified)
				{
					bModified = true;
					Parcel.Actor->Modify();
					Parcel.Spline->Modify();
				}
			};

			const int32 NumV = FMath::Min(Parcel.Pts.Num(), Parcel.Spline->GetNumberOfSplinePoints());
			for (int32 i = 0; i < NumV; ++i)
			{
				if (const FVertexBinding* B = Bindings.Find(i))
				{
					// Re-seat from the stored (station, offset) on the current
					// curve — this is what makes a parcel follow a road edit.
					const int32 RoadIdx = FindRoadById(B->RoadId);
					const FRoadCurves* C = (RoadIdx != INDEX_NONE) ? Ctx.Curves.Find(RoadIdx) : nullptr;
					if (!C || C->Sampled.Num() < 2)
					{
						// The road is gone: the vertex reverts to cadastral truth.
						++Orphaned;
						EnsureModify();
						Parcel.Actor->Tags.RemoveAll([i](const FName& T)
						{
							return T.ToString().StartsWith(FString::Printf(TEXT("osm:bind=%d,"), i));
						});
						continue;
					}
					FVector2D Pos, Tan; double BedZ = 0.0;
					EvalPolylineAtArc(C->Sampled, B->StationCm, Pos, Tan, BedZ);
					const FVector2D N(-Tan.Y, Tan.X);   // Cross2D(Tan, N) = +1, matches Offset's sign
					const FVector NewW(
						Pos.X + N.X * B->OffsetCm,
						Pos.Y + N.Y * B->OffsetCm,
						BedZ + SidewalkTopLiftCm);
					EnsureModify();
					Parcel.Spline->SetLocationAtSplinePoint(i, NewW,
						ESplineCoordinateSpace::World, /*bUpdateSpline*/false);
					Parcel.Pts[i] = FVector2D(NewW.X, NewW.Y);
					bMovedAny = true;
					++Reseated;
					continue;
				}

				// No binding yet: does this vertex belong to the street? Nearest
				// eligible road within (pavement + margin) claims it. XY stays
				// cadastral on first contact — the binding just remembers the
				// relationship — but Z snaps to the sidewalk top immediately, so
				// fences and sourcez seating read street height, not drape noise.
				int32  BestR = INDEX_NONE;
				double BestD = TNumericLimits<double>::Max();
				RoadNetMath::FProjectResult BestVPR;
				double BestBandOuter = 0.0;
				for (const TPair<int32, FRoadCurves>& KV : Ctx.Curves)
				{
					const int32 r = KV.Key;
					if (!Roads.IsValidIndex(r) || KV.Value.Sampled.Num() < 2) { continue; }
					if (Roads[r].bBridge || Roads[r].bTunnel || Roads[r].Layer != 0) { continue; }
					const FRoadNetLaneSpec& RL = Roads[r].Lanes;
					const double BandOuter = FMath::Max(50.0, (double)RL.HalfWidthCm())
						+ ((RL.bSidewalkLeft || RL.bSidewalkRight) ? (double)FMath::Max(0.f, RL.SidewalkWidth) : 0.0);
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

				EnsureModify();
				Parcel.Actor->Tags.Add(*FString::Printf(TEXT("osm:bind=%d,%s,%.0f,%.0f"),
					i, *Roads[BestR].Id.ToString(EGuidFormats::DigitsWithHyphens),
					BestVPR.AlongDist, BestVPR.Offset));

				FVector2D Pos, Tan; double BedZ = 0.0;
				EvalPolylineAtArc(Ctx.Curves.FindChecked(BestR).Sampled, BestVPR.AlongDist, Pos, Tan, BedZ);
				const FVector OldW = Parcel.Spline->GetLocationAtSplinePoint(i, ESplineCoordinateSpace::World);
				Parcel.Spline->SetLocationAtSplinePoint(i,
					FVector(OldW.X, OldW.Y, BedZ + SidewalkTopLiftCm),
					ESplineCoordinateSpace::World, /*bUpdateSpline*/false);
				bMovedAny = true;
				++Bound;
			}

			if (bMovedAny)
			{
				Parcel.Spline->UpdateSpline();
				// Post-snap hygiene inputs: the ring the access pass reads next
				// must be the ring the level now has.
				Parcel.Box = FBox2D(ForceInit);
				Parcel.Centroid = FVector2D::ZeroVector;
				for (const FVector2D& P : Parcel.Pts) { Parcel.Box += P; Parcel.Centroid += P; }
				Parcel.Centroid /= (double)FMath::Max(1, Parcel.Pts.Num());
			}
		}

		if (Bound + Reseated + Orphaned > 0)
		{
			UE_LOG(LogRoadNet, Log,
				TEXT("[RoadNet] ParcelBind: %d vertex(es) newly bound to road splines, %d re-seated from bindings, %d orphaned (road deleted)."),
				Bound, Reseated, Orphaned);
		}
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
