// RoadNetTileActor.cpp — per-cell geometry container (§ tiling).
#include "RoadNetTileActor.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "UDynamicMesh.h"
#include "Components/DecalComponent.h"
#include "Components/DynamicMeshComponent.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/SplineComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"

ARoadNetTileActor::ARoadNetTileActor()
{
	PrimaryActorTick.bCanEverTick = false;
	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	// Root MUST be Static: the mesh/HISM layers we attach are Static, and a Static
	// child cannot attach to a Movable parent (attach aborts). Splines are Movable,
	// which is still allowed under a Static parent (child mobility >= parent).
	Root->SetMobility(EComponentMobility::Static);
	SetRootComponent(Root);
}

void ARoadNetTileActor::Configure(const FIntPoint& InCoord, double InTileSizeCm, const FGuid& InOwnerId)
{
	TileCoord = InCoord;
	TileSizeCm = InTileSizeCm;
	OwningNetworkId = InOwnerId;
}

UDynamicMeshComponent* ARoadNetTileActor::GetOrCreateMeshLayer(FName LayerName, UMaterialInterface* Material,
	FColor FallbackColor, bool bBakeVertexColors)
{
	UDynamicMeshComponent* Comp = nullptr;
	if (TObjectPtr<UDynamicMeshComponent>* Found = MeshLayers.Find(LayerName))
	{
		Comp = Found->Get();
	}

	const bool bNew = (Comp == nullptr);
	if (bNew)
	{
		Comp = NewObject<UDynamicMeshComponent>(this, LayerName);
		Comp->SetupAttachment(GetRootComponent());
		Comp->SetMobility(EComponentMobility::Static);
		Comp->RegisterComponent();
		AddInstanceComponent(Comp);
		MeshLayers.Add(LayerName, Comp);
	}

	// Two-sided so thin ribbons / island edges never cull to a hole.
	Comp->SetTwoSided(true);

	// Only road-level surfaces take marking decals. A decal is a box projected
	// down, so without this a stop bar at a kerb also paints itself across the
	// sidewalk and up the median next to it.
	static const TSet<FName> kNoDecalLayers = {
		FName(TEXT("Sidewalks")), FName(TEXT("Median")), FName(TEXT("MedianWalk")) };
	Comp->SetReceivesDecals(!kNoDecalLayers.Contains(LayerName));

	if (Material)
	{
		Comp->SetMaterial(0, Material);
		// A real material always wins: clear any tint/vertex-colour override.
		Comp->SetColorOverrideMode(EDynamicMeshComponentColorOverrideMode::None);
	}
	else if (bNew)
	{
		if (bBakeVertexColors)
		{
			Comp->SetColorOverrideMode(EDynamicMeshComponentColorOverrideMode::VertexColors);
		}
		else
		{
			Comp->SetColorOverrideMode(EDynamicMeshComponentColorOverrideMode::Constant);
			Comp->SetConstantOverrideColor(FallbackColor);
		}
	}
	return Comp;
}

UHierarchicalInstancedStaticMeshComponent* ARoadNetTileActor::GetOrCreateHISM(FName Key, UStaticMesh* Mesh,
	UMaterialInterface* Mat0, UMaterialInterface* Mat1)
{
	UHierarchicalInstancedStaticMeshComponent* H = nullptr;
	if (TObjectPtr<UHierarchicalInstancedStaticMeshComponent>* Found = HISMs.Find(Key))
	{
		H = Found->Get();
	}
	if (!H)
	{
		H = NewObject<UHierarchicalInstancedStaticMeshComponent>(this, Key);
		H->SetupAttachment(GetRootComponent());
		H->SetMobility(EComponentMobility::Static);
		H->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		H->SetCanEverAffectNavigation(false);
		H->RegisterComponent();
		AddInstanceComponent(H);
		HISMs.Add(Key, H);
	}
	if (Mesh) { H->SetStaticMesh(Mesh); }
	// Set slot 0 unconditionally: components are REUSED across rebuilds, so
	// skipping the call when Mat0 is null left a previously-applied override in
	// place forever and clearing the asset slot in the panel appeared to do
	// nothing. Passing null resets to the mesh's own material, which is what an
	// empty slot means.
	H->SetMaterial(0, Mat0);
	if (Mat1 && H->GetStaticMesh() && H->GetStaticMesh()->GetStaticMaterials().Num() > 1)
	{
		H->SetMaterial(1, Mat1);
	}
	return H;
}

UHierarchicalInstancedStaticMeshComponent* ARoadNetTileActor::FindHISM(FName Key) const
{
	if (const TObjectPtr<UHierarchicalInstancedStaticMeshComponent>* Found = HISMs.Find(Key))
	{
		return Found->Get();
	}
	return nullptr;
}

USplineComponent* ARoadNetTileActor::AddSpline()
{
	USplineComponent* Sp = NewObject<USplineComponent>(this);
	if (!Sp) { return nullptr; }
	Sp->SetMobility(EComponentMobility::Movable);
	Sp->AttachToComponent(GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
	Sp->RegisterComponent();
	AddInstanceComponent(Sp);
	Splines.Add(Sp);
	return Sp;
}

UDecalComponent* ARoadNetTileActor::AddRoadDecal(UMaterialInterface* Material, const FVector& Location,
	float YawDeg, const FVector& HalfSize)
{
	if (!Material || Decals.Num() >= kMaxDecalsPerTile) { return nullptr; }

	UDecalComponent* D = NewObject<UDecalComponent>(this);
	if (!D) { return nullptr; }
	D->SetMobility(EComponentMobility::Static);
	D->SetDecalMaterial(Material);
	D->DecalSize = HalfSize;
	D->AttachToComponent(GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
	// Pitch -90 aims the decal's projection axis straight down (the pose
	// ADecalActor gives its own component); yaw then spins the painted rectangle
	// in the ground plane, with DecalSize.Z running along that yaw.
	D->SetWorldLocationAndRotation(Location, FRotator(-90.f, YawDeg, 0.f));
	D->RegisterComponent();
	AddInstanceComponent(D);
	Decals.Add(D);
	return D;
}

void ARoadNetTileActor::TrackChildActor(AActor* Child)
{
	if (Child) { ChildActors.Add(Child); }
}

void ARoadNetTileActor::ClearForRebuild()
{
	// Empty every dynamic-mesh layer (component reused; content dropped).
	for (TPair<FName, TObjectPtr<UDynamicMeshComponent>>& KV : MeshLayers)
	{
		if (UDynamicMeshComponent* Comp = KV.Value.Get())
		{
			Comp->SetMesh(UE::Geometry::FDynamicMesh3());
			Comp->NotifyMeshUpdated();
		}
	}

	// Clear all HISM instances (components reused).
	for (TPair<FName, TObjectPtr<UHierarchicalInstancedStaticMeshComponent>>& KV : HISMs)
	{
		if (UHierarchicalInstancedStaticMeshComponent* H = KV.Value.Get()) { H->ClearInstances(); }
	}

	// Destroy splines + tracked child actors (recreated fresh each rebuild).
	for (TObjectPtr<USplineComponent>& Sp : Splines)
	{
		if (USplineComponent* S = Sp.Get()) { S->DestroyComponent(); }
	}
	Splines.Reset();

	for (TObjectPtr<UDecalComponent>& D : Decals)
	{
		if (UDecalComponent* Dec = D.Get()) { Dec->DestroyComponent(); }
	}
	Decals.Reset();

	for (TWeakObjectPtr<AActor>& C : ChildActors)
	{
		if (AActor* A = C.Get()) { A->Destroy(); }
	}
	ChildActors.Reset();
}

bool ARoadNetTileActor::IsEmptyTile() const
{
	for (const TPair<FName, TObjectPtr<UDynamicMeshComponent>>& KV : MeshLayers)
	{
		if (UDynamicMeshComponent* Comp = KV.Value.Get())
		{
			if (UDynamicMesh* DM = Comp->GetDynamicMesh())
			{
				if (DM->GetMeshRef().TriangleCount() > 0) { return false; }
			}
		}
	}
	for (const TPair<FName, TObjectPtr<UHierarchicalInstancedStaticMeshComponent>>& KV : HISMs)
	{
		if (const UHierarchicalInstancedStaticMeshComponent* H = KV.Value.Get())
		{
			if (H->GetInstanceCount() > 0) { return false; }
		}
	}
	if (Splines.Num() > 0 || Decals.Num() > 0) { return false; }
	for (const TWeakObjectPtr<AActor>& C : ChildActors) { if (C.IsValid()) { return false; } }
	return true;
}
