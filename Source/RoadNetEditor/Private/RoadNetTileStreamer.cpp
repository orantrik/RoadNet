// RoadNetTileStreamer.cpp — authoring-time distance streaming for road tiles (§ scale).
//
// THE PROBLEM THIS SOLVES
// A city import commits tens of thousands of ARoadNetTileActor primitives. They
// are RF_Transient authoring output, so World Partition will not stream them —
// WP only streams saved (OFPA) actors. Every tile therefore keeps a live render
// proxy no matter where the camera is, and each proxy holds GPU resource views.
// Past ~1,000,000 live views D3D12 cannot grow the descriptor heap and the
// editor dies with "Hit D3D12 device limits on descriptors".
//
// WHY THIS AND NOT A CULL DISTANCE
// LDMaxDrawDistance and SetVisibility(false) stop a primitive from DRAWING but
// leave it registered, so its render proxy and descriptors stay allocated. They
// cannot fix a descriptor-count problem. Unregistering the component frees both,
// and is fully reversible, so nothing is lost — near geometry is untouched at
// full detail, and distant tiles come back exactly as they were when approached.
// This is what WP does for saved actors; here we do the same by hand.
#include "RoadNetTileActor.h"
#include "RoadNetLog.h"
#include "EngineUtils.h"
#include "Editor.h"
#include "EditorViewportClient.h"
#include "Components/PrimitiveComponent.h"
#include "HAL/IConsoleManager.h"
#include "TickableEditorObject.h"

namespace
{
// 0 disables streaming entirely (every tile stays resident, pre-fix behaviour).
// The default is generous: 4 km keeps a whole district live around the camera,
// which is far past the distance road detail is readable, while cutting a
// city-scale import's resident primitive count by an order of magnitude.
TAutoConsoleVariable<float> CVarTileStreamRadiusM(
	TEXT("roadnet.TileStreamRadiusM"),
	4000.0f,
	TEXT("Authoring-time streaming radius for RoadNet tiles, in metres, measured\n")
	TEXT("from the editor camera to the tile's bounds. Tiles outside it release\n")
	TEXT("their render state (freeing GPU descriptors) and restore it on return.\n")
	TEXT("0 = never stream out. Does not change how any visible tile looks."),
	ECVF_Default);

// Hysteresis: a tile must be this much further out than the radius before it is
// released. Without it a camera parked exactly on the boundary would register
// and unregister the same tile every sweep, which is far more expensive than
// simply keeping it.
TAutoConsoleVariable<float> CVarTileStreamHysteresisM(
	TEXT("roadnet.TileStreamHysteresisM"),
	500.0f,
	TEXT("Extra distance (m) past roadnet.TileStreamRadiusM before a tile is\n")
	TEXT("streamed out, so a camera on the boundary cannot thrash."),
	ECVF_Default);

TAutoConsoleVariable<float> CVarTileStreamIntervalSec(
	TEXT("roadnet.TileStreamIntervalSec"),
	0.5f,
	TEXT("Seconds between RoadNet tile streaming sweeps."),
	ECVF_Default);

// Registering thousands of components in one frame is its own hitch, so a sweep
// only changes this many tiles and the rest follow next sweep. Streaming IN is
// prioritised over streaming OUT: a visible hole is worse than a late release.
constexpr int32 kMaxChangesPerSweep = 64;

class FRoadNetTileStreamer final : public FTickableEditorObject
{
public:
	virtual void Tick(float DeltaTime) override
	{
		const float RadiusM = CVarTileStreamRadiusM.GetValueOnAnyThread();
		if (RadiusM <= 0.0f)
		{
			// Streaming was turned off: make sure nothing is left hidden.
			if (bAnyStreamedOut) { RestoreAll(); }
			return;
		}

		Accum += DeltaTime;
		const float Interval = FMath::Max(0.05f, CVarTileStreamIntervalSec.GetValueOnAnyThread());
		if (Accum < Interval) { return; }
		Accum = 0.0f;

		UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
		if (!World) { return; }

		FVector Camera;
		if (!TryGetCamera(Camera)) { return; }

		const double InCm  = (double)RadiusM * 100.0;
		const double OutCm = InCm + (double)FMath::Max(0.0f, CVarTileStreamHysteresisM.GetValueOnAnyThread()) * 100.0;

		int32 Changes = 0, In = 0, Out = 0, Resident = 0;
		bAnyStreamedOut = false;

		// Two passes so a full sweep budget can never be spent on releases while
		// a tile the camera is standing in is still waiting to come back.
		for (int32 Pass = 0; Pass < 2 && Changes < kMaxChangesPerSweep; ++Pass)
		{
			const bool bWantIn = (Pass == 0);
			for (TActorIterator<ARoadNetTileActor> It(World); It; ++It)
			{
				ARoadNetTileActor* Tile = *It;
				if (!Tile) { continue; }

				if (Pass == 0)
				{
					// A tile that has never been measured (freshly committed) has
					// no cached bounds yet; take them while it is still resident.
					if (Tile->IsStreamedIn() && !Tile->GetStreamBounds().IsValid)
					{
						Tile->RefreshStreamBounds();
					}
					if (Tile->IsStreamedIn()) { ++Resident; } else { bAnyStreamedOut = true; }
				}

				const FBox B = Tile->GetStreamBounds();
				// Unmeasurable tile (empty, or bounds never captured): leave it
				// alone rather than guess it is far away and blank it.
				if (!B.IsValid) { continue; }

				const double DistCm = FMath::Sqrt(B.ComputeSquaredDistanceToPoint(Camera));
				const bool bShouldBeIn = Tile->IsStreamedIn() ? (DistCm <= OutCm) : (DistCm <= InCm);
				if (bShouldBeIn == Tile->IsStreamedIn()) { continue; }
				if (bShouldBeIn != bWantIn) { continue; }

				Tile->SetStreamedIn(bShouldBeIn);
				if (bShouldBeIn) { ++In; }
				// Pass 0 counted what was ALREADY out; releases happen in pass 1,
				// so record them here too or turning streaming off afterwards
				// would think there is nothing to restore.
				else { ++Out; bAnyStreamedOut = true; }
				if (++Changes >= kMaxChangesPerSweep) { break; }
			}
		}

		if (In > 0 || Out > 0)
		{
			UE_LOG(LogRoadNet, Verbose,
				TEXT("[RoadNet] tile stream: +%d in, -%d out (%d resident, radius %.0f m)"),
				In, Out, Resident, RadiusM);
		}
	}

	virtual TStatId GetStatId() const override
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FRoadNetTileStreamer, STATGROUP_Tickables);
	}

private:
	static bool TryGetCamera(FVector& Out)
	{
		if (!GEditor) { return false; }
		// Prefer the viewport the user is actually looking through.
		for (FEditorViewportClient* VC : GEditor->GetAllViewportClients())
		{
			if (VC && VC->IsPerspective() && VC->GetWorld() == GEditor->GetEditorWorldContext().World())
			{
				Out = VC->GetViewLocation();
				return true;
			}
		}
		return false;
	}

	void RestoreAll()
	{
		UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
		if (!World) { return; }
		int32 n = 0;
		for (TActorIterator<ARoadNetTileActor> It(World); It; ++It)
		{
			if (ARoadNetTileActor* Tile = *It)
			{
				if (!Tile->IsStreamedIn()) { Tile->SetStreamedIn(true); ++n; }
			}
		}
		bAnyStreamedOut = false;
		if (n > 0)
		{
			UE_LOG(LogRoadNet, Log, TEXT("[RoadNet] tile stream disabled; restored %d tile(s)."), n);
		}
	}

	float Accum = 0.0f;
	bool bAnyStreamedOut = false;
};

TUniquePtr<FRoadNetTileStreamer> GTileStreamer;

// The one property everything here rests on: streaming out must free render
// state and streaming back in must restore it EXACTLY, or a commit bakes holes.
void TileStreamSelfCheck()
{
	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!World) { return; }

	ARoadNetTileActor* Tile = nullptr;
	for (TActorIterator<ARoadNetTileActor> It(World); It; ++It)
	{
		if (*It && !(*It)->IsEmptyTile()) { Tile = *It; break; }
	}
	if (!Tile)
	{
		UE_LOG(LogRoadNet, Display, TEXT("TileStreamSelfCheck: SKIP (no non-empty road tile in the level)."));
		return;
	}

	auto CountRegisteredPrims = [Tile]()
	{
		int32 n = 0;
		for (const UActorComponent* C : Tile->GetComponents())
		{
			if (C && C->IsA<UPrimitiveComponent>() && C->IsRegistered()) { ++n; }
		}
		return n;
	};

	const bool bWasIn = Tile->IsStreamedIn();
	Tile->SetStreamedIn(true);
	const int32 Before = CountRegisteredPrims();

	Tile->SetStreamedIn(false);
	const int32 Out = CountRegisteredPrims();
	const bool bBoundsKept = Tile->GetStreamBounds().IsValid != 0;

	Tile->SetStreamedIn(true);
	const int32 After = CountRegisteredPrims();

	// Splines stay registered on purpose, so "out" is not necessarily zero — the
	// contract is that it DROPS and that coming back is lossless.
	const bool bOK = (Before > 0) && (Out < Before) && (After == Before) && bBoundsKept;
	UE_LOG(LogRoadNet, Display,
		TEXT("TileStreamSelfCheck: %s (registered primitives %d -> %d streamed out -> %d back; bounds %s)"),
		bOK ? TEXT("PASS") : TEXT("FAIL"), Before, Out, After,
		bBoundsKept ? TEXT("kept") : TEXT("LOST"));

	Tile->SetStreamedIn(bWasIn);
}

FAutoConsoleCommand GTileStreamSelfCheck(
	TEXT("RoadNet.TileStreamSelfCheck"),
	TEXT("Verify a road tile's render state is released on stream-out and fully restored on stream-in."),
	FConsoleCommandDelegate::CreateStatic(&TileStreamSelfCheck));
} // namespace

namespace RoadNetTileStreamer
{
void Startup()  { if (!GTileStreamer) { GTileStreamer = MakeUnique<FRoadNetTileStreamer>(); } }
void Shutdown() { GTileStreamer.Reset(); }
}
