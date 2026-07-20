// RoadNetFurniture.cpp — street-furniture placement + commit (street features).
//
// BuildFurniture lays out world transforms for each enabled FRoadNetFurnitureType.
// Two distinct behaviours keep furniture off the carriageway:
//   • Spaced items (bench / bus stop / kiosk): sampled along the road centreline,
//     but a candidate is KEPT ONLY IF its ground point lands inside the road's
//     junction-clipped sidewalk band. That is the hard guard against benches in
//     the middle of / beside the street or inside a junction.
//   • Continuous items (guard rails): NOT sampled from the road at all. They ride
//     the KERB LINE, derived (per grade zone) from the merged carriageway + the
//     sidewalk band via the same builder the kerb meshes use. The kerb line is
//     already junction-clipped, so a rail hugs the curb and never crosses a
//     junction — exactly like a real guard rail. Each piece carries its own
//     length (baked into the transform's X scale) so tiles join gaplessly around
//     corners; CommitFurniture stretches the mesh/placeholder to that length.
// CommitFurniture then resolves each bucket placeholder-first: a Blueprint/actor
// class spawns real actors, else a Static Mesh override instances via a HISM,
// else a grey engine-cube placeholder is instanced so the layout is visible
// before real assets are wired up.
#include "RoadNetwork.h"
#include "RoadNetMath.h"
#include "RoadNetMesh.h"   // FCenterlineHeightField
#include "RoadNetCurbs.h"  // BuildCurbInstancesForZone (kerb line for guard rails)
#include "RoadNetTileActor.h" // per-cell geometry container
#include "RoadNetLog.h"
#include "Curve/GeneralPolygon2.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

void URoadNetwork::BuildFurniture(FRoadNetRebuildContext& Ctx) const
{
	using UE::Geometry::FGeneralPolygon2d;

	Ctx.FurnitureBuckets.Reset();
	if (!bBuildFurniture) { return; }

	constexpr double kFurnitureZLiftCm = 12.0; // match kRoadZLiftCm (anti z-fight)

	// Road → grade zone, so a spaced item can be tested against the right
	// per-zone sidewalk band.
	TMap<int32, int32> RoadZone;
	for (int32 z = 0; z < Ctx.Zones.Num(); ++z)
	{
		for (int32 RoadIdx : Ctx.Zones[z]) { RoadZone.Add(RoadIdx, z); }
	}

	for (int32 ti = 0; ti < FurnitureTypes.Num(); ++ti)
	{
		const FRoadNetFurnitureType& FT = FurnitureTypes[ti];
		if (!FT.bEnabled || (!FT.bLeft && !FT.bRight)) { continue; }

		const double Spacing = FMath::Max(20.f, FT.SpacingCm);
		const bool bCont = (FT.Placement == ERoadNetFurniturePlacement::Continuous);

		FRoadNetFurnitureBucket Bucket;
		Bucket.TypeIndex = ti;

		// ---------------------------------------------------------------------
		// CONTINUOUS (guard rails): tile along the junction-clipped KERB LINE,
		// derived per zone from the merged carriageway + sidewalk band. Side
		// toggles don't select a curb side here (the curb wraps the walk); the
		// type just has to be enabled. SideOffset is ignored — the rail sits on
		// the curb, which is the whole point of "follow the curb line".
		// ---------------------------------------------------------------------
		if (bCont)
		{
			for (int32 z = 0; z < Ctx.Zones.Num(); ++z)
			{
				if (!Ctx.ZoneSurfacePolys.IsValidIndex(z) || !Ctx.ZoneSidewalkPolys.IsValidIndex(z)) { continue; }
				if (Ctx.ZoneSurfacePolys[z].Num() == 0 || Ctx.ZoneSidewalkPolys[z].Num() == 0)       { continue; }

				TArray<const TArray<FVector>*> CenterLines;
				for (int32 RoadIdx : Ctx.Zones[z])
				{
					if (const FRoadCurves* C = Ctx.Curves.Find(RoadIdx)) { CenterLines.Add(&C->Sampled); }
				}
				RoadNetMesh::FCenterlineHeightField Field;
				Field.Build(CenterLines);

				TArray<RoadNetCurbs::FCurbInstance> Insts;
				RoadNetCurbs::BuildCurbInstancesForZone(
					Ctx.ZoneSurfacePolys[z], Ctx.ZoneSidewalkPolys[z], Field,
					Spacing, kFurnitureZLiftCm, Insts);

				for (const RoadNetCurbs::FCurbInstance& CI : Insts)
				{
					// Bake the piece length into the X scale so CommitFurniture can
					// stretch one tile to span this kerb chunk (gapless on curves).
					const float YawDeg = CI.YawDeg + FT.YawOffsetDeg;
					const FRotator Rot(CI.PitchDeg, YawDeg, 0.f);
					const FVector  ScaleLen(FMath::Max(10.f, CI.LengthCm), 1.f, 1.f);
					Bucket.Instances.Add(FTransform(Rot, CI.Location, ScaleLen));
				}
			}

			if (Bucket.Instances.Num() > 0) { Ctx.FurnitureBuckets.Add(MoveTemp(Bucket)); }
			continue;
		}

		// ---------------------------------------------------------------------
		// SPACED (bench / bus stop / kiosk): sample the centreline, then keep a
		// point ONLY if it lands inside the sidewalk band (junction-clipped).
		// ---------------------------------------------------------------------
		for (const TPair<int32, FRoadCurves>& KV : Ctx.Curves)
		{
			const int32 RoadIdx = KV.Key;
			const FRoadCurves& C = KV.Value;
			if (!Roads.IsValidIndex(RoadIdx) || C.Sampled.Num() < 2) { continue; }
			const TArray<FVector>& P = C.Sampled;

			TArray<double> CL;
			RoadNetMath::CumulativeLength(P, CL);
			const double Len = CL.Last();
			if (Len < 1.0) { continue; }

			const FRoadNetLaneSpec& LS = Roads[RoadIdx].Lanes;
			const double Half = (double)LS.HalfWidthCm();

			// Furniture only follows SIDEWALKS, never a bare carriageway edge, so
			// nothing lands in the middle of / beside a walkless road. Geometry
			// mapping (see BuildSurfaceUnion): the +RightAxis side is driven by
			// bSidewalkLeft, the −RightAxis side by bSidewalkRight. Combine that
			// with the type's own side selection (bRight = +RightAxis side here).
			const bool bHasWalk = (LS.SidewalkWidth > 0.f);
			const bool bDoPlus  = FT.bRight && bHasWalk && LS.bSidewalkLeft;   // +RightAxis
			const bool bDoMinus = FT.bLeft  && bHasWalk && LS.bSidewalkRight;  // −RightAxis
			if (!bDoPlus && !bDoMinus) { continue; }

			// Sidewalk band(s) of this road's zone — the hard placement guard.
			const int32* ZonePtr = RoadZone.Find(RoadIdx);
			if (!ZonePtr || !Ctx.ZoneSidewalkPolys.IsValidIndex(*ZonePtr)) { continue; }
			const TArray<FGeneralPolygon2d>& Walks = Ctx.ZoneSidewalkPolys[*ZonePtr];
			if (Walks.Num() == 0) { continue; }
			auto OnSidewalk = [&Walks](const FVector& Loc) -> bool
			{
				const FVector2d Q(Loc.X, Loc.Y);
				for (const FGeneralPolygon2d& GP : Walks) { if (GP.Contains(Q)) { return true; } }
				return false;
			};

			// Position (with draped Z) + planar tangent at arc length S.
			auto SampleAt = [&](double S, FVector& OutPos, FVector2D& OutTan)
			{
				S = FMath::Clamp(S, 0.0, Len);
				int32 seg = 0;
				while (seg + 1 < CL.Num() - 1 && CL[seg + 1] < S) { ++seg; }
				const double segLen = FMath::Max(1e-3, CL[seg + 1] - CL[seg]);
				const double t = FMath::Clamp((S - CL[seg]) / segLen, 0.0, 1.0);
				OutPos = FMath::Lerp(P[seg], P[seg + 1], t);
				FVector2D Tan(P[seg + 1].X - P[seg].X, P[seg + 1].Y - P[seg].Y);
				if (!Tan.Normalize()) { Tan = FVector2D(1.0, 0.0); }
				OutTan = Tan;
			};

			for (double S = Spacing; S <= Len + 1e-3; S += Spacing)
			{
				FVector Pos; FVector2D Tan;
				SampleAt(S, Pos, Tan);
				const FVector2D Rt = RoadNetMath::RightAxis(Tan);

				auto Emit = [&](double Sign)
				{
					const double Lat = Sign * (Half + (double)FT.SideOffsetCm);
					const FVector Loc = Pos + FVector(Rt.X * Lat, Rt.Y * Lat, 0.0);

					// HARD GUARD: skip anything that isn't actually on the sidewalk
					// (carriageway, junction gap, or past the walk's outer edge).
					if (!OnSidewalk(Loc)) { return; }

					// Spaced items FACE the road: forward (+X) points inward toward
					// the carriageway (−outward normal), so a bench's front looks at
					// the street. YawOffsetDeg fine-tunes for a mesh's front convention.
					const FVector2D Inward(-Sign * Rt.X, -Sign * Rt.Y);
					float YawDeg = FMath::RadiansToDegrees(FMath::Atan2(Inward.Y, Inward.X));
					YawDeg += FT.YawOffsetDeg;
					Bucket.Instances.Add(FTransform(FRotator(0.f, YawDeg, 0.f), Loc, FVector::OneVector));
				};
				if (bDoPlus)  { Emit(+1.0); }
				if (bDoMinus) { Emit(-1.0); }
			}
		}

		if (Bucket.Instances.Num() > 0) { Ctx.FurnitureBuckets.Add(MoveTemp(Bucket)); }
	}
}

void URoadNetwork::CommitFurniture(FRoadNetRebuildContext& Ctx)
{
	UWorld* World = WorldPtr.Get();
	if (!World) { return; }

	// (Tiles were cleared in PrepareTilesForCommit; nothing to do on empty input.)
	if (!bBuildFurniture || Ctx.FurnitureBuckets.Num() == 0) { return; }

	UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));

	int32 TotalInstances = 0, TotalActors = 0, CubeTypes = 0;
	// Batch per HISM: one-by-one AddInstance rebuilds the HISM cluster tree each
	// call (O(n^2) for a guard-rail-dense city). AddInstances() rebuilds once.
	TMap<UHierarchicalInstancedStaticMeshComponent*, TArray<FTransform>> FurnBatches;
	for (const FRoadNetFurnitureBucket& Bucket : Ctx.FurnitureBuckets)
	{
		if (!FurnitureTypes.IsValidIndex(Bucket.TypeIndex) || Bucket.Instances.Num() == 0) { continue; }
		const FRoadNetFurnitureType& FT = FurnitureTypes[Bucket.TypeIndex];

		// Resolution order: Blueprint/actor class > Static Mesh override > cube.
		UClass* BPClass = FT.BlueprintClass.IsNull() ? nullptr : FT.BlueprintClass.LoadSynchronous();
		if (BPClass)
		{
			for (const FTransform& X : Bucket.Instances)
			{
				const FIntPoint Coord = TopoKeyOf(X.GetLocation(), Ctx);
				if (Coord.X == INDEX_NONE || !IsTileInCommitScope(Coord, Ctx)) { continue; }
				ARoadNetTileActor* Tile = GetOrCreateTile(Coord);
				if (!Tile) { continue; }

				FActorSpawnParameters SP;
				SP.ObjectFlags |= RF_Transient;
				SP.Owner = Tile;
				SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
				if (AActor* Child = World->SpawnActor<AActor>(BPClass, X, SP))
				{
					Child->AttachToActor(Tile, FAttachmentTransformRules::KeepWorldTransform);
					Tile->TrackChildActor(Child);
					++TotalActors;
				}
			}
			continue;
		}

		UStaticMesh* Mesh = FT.MeshOverride.IsNull() ? nullptr : FT.MeshOverride.LoadSynchronous();
		const bool bPlaceholder = (Mesh == nullptr);
		if (!Mesh) { Mesh = CubeMesh; }
		if (!Mesh) { continue; }
		if (bPlaceholder) { ++CubeTypes; }

		// Scaling rules (per instance, because continuous pieces carry their own
		// kerb-chunk length in the incoming transform's X scale — see BuildFurniture):
		//   • Placeholder cube: scale the (100 cm, pivot-centred) cube to the type's
		//     box extent and seat its base on the placement Z. A continuous run uses
		//     the carried length for its along-run extent so rails join gaplessly.
		//   • Real mesh, continuous: stretch only X to the carried length (keep Y/Z
		//     natural); pivot on the ground point.
		//   • Real mesh, spaced: placed at unit scale, pivot on the ground point.
		const bool bCont = (FT.Placement == ERoadNetFurniturePlacement::Continuous);
		const FVector Size = Mesh->GetBoundingBox().GetSize();
		const FName HISMKey(*FString::Printf(TEXT("Furniture_%d"), Bucket.TypeIndex));

		for (const FTransform& X : Bucket.Instances)
		{
			const FIntPoint Coord = TopoKeyOf(X.GetLocation(), Ctx);
			if (Coord.X == INDEX_NONE || !IsTileInCommitScope(Coord, Ctx)) { continue; }
			ARoadNetTileActor* Tile = GetOrCreateTile(Coord);
			if (!Tile) { continue; }
			UHierarchicalInstancedStaticMeshComponent* H = Tile->GetOrCreateHISM(HISMKey, Mesh);
			if (!H) { continue; }

			const double PieceLen = FMath::Max(10.0, X.GetScale3D().X); // carried tile length (continuous)
			FVector Scale = FVector::OneVector;
			double BaseLiftZ = 0.0;
			if (bPlaceholder)
			{
				FVector Ext = FT.PlaceholderExtentCm;
				if (bCont) { Ext.X = PieceLen; }
				Scale = FVector(Ext.X / FMath::Max(1.0, Size.X),
				                Ext.Y / FMath::Max(1.0, Size.Y),
				                Ext.Z / FMath::Max(1.0, Size.Z));
				BaseLiftZ = 0.5 * Ext.Z; // cube centre → base seated on the point
			}
			else if (bCont)
			{
				Scale = FVector(PieceLen / FMath::Max(1.0, Size.X), 1.0, 1.0);
			}

			FTransform Inst = X;
			Inst.SetScale3D(Scale);
			Inst.SetLocation(X.GetLocation() + FVector(0.0, 0.0, BaseLiftZ));
			FurnBatches.FindOrAdd(H).Add(Inst);
			++TotalInstances;
		}
	}
	for (TPair<UHierarchicalInstancedStaticMeshComponent*, TArray<FTransform>>& KV : FurnBatches)
	{
		KV.Key->AddInstances(KV.Value, /*bShouldReturnIndices*/false, /*bWorldSpace*/true);
	}

	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet] CommitFurniture: %d instances (%d cube-placeholder types) + %d spawned actors across %d buckets."),
		TotalInstances, CubeTypes, TotalActors, Ctx.FurnitureBuckets.Num());
}
