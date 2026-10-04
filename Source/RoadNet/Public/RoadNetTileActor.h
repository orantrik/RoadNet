#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RoadNetTileActor.generated.h"

class UDecalComponent;
class UDynamicMeshComponent;
class UHierarchicalInstancedStaticMeshComponent;
class USplineComponent;
class UMaterialInterface;
class UStaticMesh;

// ===========================================================================
// ARoadNetTileActor — geometry container for ONE spatial grid cell (§ tiling).
//
// Replaces the former network-wide RoadNet_Surface / _Sidewalks / _Curbs / ...
// actors. Every rebuild bins its output by world position into the tile whose
// cell contains it (see RoadNetTiles.h) and commits it into THIS actor's
// components, so a windowed edit only re-commits the handful of tiles it
// touches and World Partition can stream the network by location.
//
// One actor per (network, TileCoord). Layers are addressed by a stable FName
// (e.g. "Surface", "Sidewalks", "MarkingsWhite", ...): each maps to one reused
// UDynamicMeshComponent. HISM instancing (curbs/signals/furniture) is addressed
// by FName too. Splines (perimeter / lane-graph / median) and Blueprint
// furniture children are recreated fresh each rebuild.
//
// INDEPENDENCE: engine + own module only.
// ===========================================================================
UCLASS(NotBlueprintable, hidecategories=(Input, Replication, Cooking, LOD))
class ROADNET_API ARoadNetTileActor : public AActor
{
	GENERATED_BODY()

public:
	ARoadNetTileActor();

	// Which grid cell this tile represents, and the cell size it was built with
	// (cm). OwningNetworkId ties the tile to a specific URoadNetwork so multiple
	// networks in one level don't collide when the registry is rebuilt on load.
	UPROPERTY(VisibleAnywhere, Category = "RoadNet|Tile")
	FIntPoint TileCoord = FIntPoint(0, 0);

	UPROPERTY(VisibleAnywhere, Category = "RoadNet|Tile")
	double TileSizeCm = 25600.0;

	UPROPERTY(VisibleAnywhere, Category = "RoadNet|Tile")
	FGuid OwningNetworkId;

	void Configure(const FIntPoint& InCoord, double InTileSizeCm, const FGuid& InOwnerId);

	// Get (creating on first use) the reusable dynamic-mesh component for a named
	// visual layer. Applies Material to slot 0 when set; otherwise, on first
	// creation only, applies a constant vertex-colour override so the layer is
	// visible without a material. bBake uses baked vertex colours instead.
	UDynamicMeshComponent* GetOrCreateMeshLayer(FName LayerName, UMaterialInterface* Material,
		FColor FallbackColor, bool bBakeVertexColors);

	// Get (creating on first use) a named HISM (curbs/signals/furniture). Sets
	// the mesh and applies the optional material overrides. Instances are added
	// by the caller; ClearForRebuild empties them first.
	UHierarchicalInstancedStaticMeshComponent* GetOrCreateHISM(FName Key, UStaticMesh* Mesh,
		UMaterialInterface* Mat0 = nullptr, UMaterialInterface* Mat1 = nullptr);

	// Existing HISM or null — does not create. Used by curb-paint rebucket.
	UHierarchicalInstancedStaticMeshComponent* FindHISM(FName Key) const;

	// Add a fresh spline component (perimeter / lane-graph / median centre). The
	// caller sets points + ComponentTags. Recreated every rebuild.
	USplineComponent* AddSpline();

	// Add a fresh deferred decal projecting straight down onto the carriageway.
	// HalfSize is (depth, half-width, half-length) in the decal's own frame, so
	// the painted rectangle is 2*Y by 2*Z with YawDeg pointing along its length.
	// Recreated every rebuild, like splines. Returns null past the per-tile cap.
	UDecalComponent* AddRoadDecal(UMaterialInterface* Material, const FVector& Location,
		float YawDeg, const FVector& HalfSize);

	// Register a spawned child actor (Blueprint furniture) so ClearForRebuild
	// destroys it on the next rebuild of this tile.
	void TrackChildActor(AActor* Child);

	// Empty every layer's mesh, clear all HISM instances, and destroy all
	// splines + tracked child actors, so the tile can be freshly repopulated.
	// (Mesh + HISM components themselves are reused.)
	void ClearForRebuild();

	// True when the tile holds no geometry at all (all mesh layers empty, all
	// HISMs instance-less, no splines, no child actors) — safe to retire.
	bool IsEmptyTile() const;

	// ---- authoring-time streaming (§ scale) -------------------------------
	// Release/restore this tile's RENDER STATE. Unregistering a primitive frees
	// its render proxy and the GPU resource views behind it; hiding it or giving
	// it a cull distance frees neither, which is why neither can cure the D3D12
	// descriptor-heap exhaustion a city's worth of resident tiles causes. The
	// geometry itself is never touched, so streaming a tile back in restores it
	// pixel-for-pixel — this costs no visual fidelity, it only stops paying for
	// tiles that are kilometres away.
	//
	// World Partition cannot do this job for us here: these tiles are RF_Transient
	// authoring output, and WP only streams saved (OFPA) actors.
	void SetStreamedIn(bool bIn);
	bool IsStreamedIn() const { return bStreamedIn; }

	// Where this tile is, for the streamer to measure against the camera. Cached
	// while the render state exists because an unregistered component reports no
	// bounds — a streamed-out tile must still know where it is to come back.
	FBox GetStreamBounds() const { return CachedBounds; }
	void RefreshStreamBounds();

private:
	UPROPERTY()
	TMap<FName, TObjectPtr<UDynamicMeshComponent>> MeshLayers;

	UPROPERTY()
	TMap<FName, TObjectPtr<UHierarchicalInstancedStaticMeshComponent>> HISMs;

	UPROPERTY()
	TArray<TObjectPtr<USplineComponent>> Splines;

	// One component per painted mark when markings are drawn as decals. Capped:
	// a city's worth of dashes is tens of thousands of primitives, and each decal
	// is a separate draw the deferred pass has to sort.
	// ponytail: a flat cap silently drops marks past the limit on a very dense
	// tile. Upgrade path is one merged decal per continuous run rather than per
	// polygon, which needs run identity the polygon banks do not carry.
	static constexpr int32 kMaxDecalsPerTile = 4096;

	UPROPERTY()
	TArray<TObjectPtr<UDecalComponent>> Decals;

	UPROPERTY()
	TArray<TWeakObjectPtr<AActor>> ChildActors;

	UPROPERTY(Transient)
	bool bStreamedIn = true;

	UPROPERTY(Transient)
	FBox CachedBounds = FBox(ForceInit);
};
