// RoadNetEditorModule.cpp — registers the click-to-draw road EdMode (§9.3).
#include "Modules/ModuleManager.h"
#include "EditorModeRegistry.h"
#include "EditorModeManager.h"
#include "Styling/AppStyle.h"
#include "Textures/SlateIcon.h"
#include "EdModeRoadNet.h"
#include "RoadNetActor.h"
#include "RoadNetwork.h"
#include "RoadNetMesh.h"
#include "RoadNetStandards.h"
#include "RoadNetMarkings.h"
#include "RoadNetLog.h"
#include "Editor.h"
#include "EditorViewportClient.h"
#include "EngineUtils.h"
#include "HAL/IConsoleManager.h"

namespace RoadNetTileStreamer { void Startup(); void Shutdown(); }

namespace
{
// ---------------------------------------------------------------------------
// RoadNet.EditRegion [<size in metres> | off]
//
// Confine every subsequent rebuild — including the panel's whole-network
// buttons and the terrain conform — to a square around the editor camera. The
// rest of the city keeps the tile actors and the terrain it already has, so an
// edit costs the district it touched instead of the whole map.
//
// No size argument reports the current scope.
// ---------------------------------------------------------------------------
void EditRegionCmd(const TArray<FString>& Args, UWorld* World)
{
	if (!World) { return; }

	URoadNetwork* Net = nullptr;
	for (TActorIterator<ARoadNetActor> It(World); It; ++It)
	{
		if (*It) { Net = (*It)->GetNetwork(); break; }
	}
	if (!Net)
	{
		UE_LOG(LogRoadNet, Warning, TEXT("RoadNet.EditRegion: no RoadNet actor in this level."));
		return;
	}

	auto Report = [Net]()
	{
		if (Net->EditRegion.bIsValid)
		{
			UE_LOG(LogRoadNet, Display,
				TEXT("RoadNet.EditRegion: active — %.0f x %.0f m centred (%.0f, %.0f) m. "
				     "Rebuilds and terrain conform touch only this district."),
				Net->EditRegion.GetSize().X / 100.0, Net->EditRegion.GetSize().Y / 100.0,
				Net->EditRegion.GetCenter().X / 100.0, Net->EditRegion.GetCenter().Y / 100.0);
		}
		else
		{
			UE_LOG(LogRoadNet, Display,
				TEXT("RoadNet.EditRegion: off — rebuilds cover the whole network. "
				     "'RoadNet.EditRegion 1000' scopes them to 1 km around the camera."));
		}
	};

	if (Args.Num() == 0) { Report(); return; }

	if (Args[0].Equals(TEXT("off"), ESearchCase::IgnoreCase) || Args[0] == TEXT("0"))
	{
		Net->EditRegion = FBox2D(ForceInit);
		Net->MarkPackageDirty();
		Report();
		return;
	}

	const double SizeM = FCString::Atod(*Args[0]);
	if (!(SizeM > 0.0))
	{
		UE_LOG(LogRoadNet, Warning, TEXT("RoadNet.EditRegion: expected a size in metres, or 'off'."));
		return;
	}

	FVector Camera = FVector::ZeroVector;
	bool bGotCamera = false;
	for (FEditorViewportClient* VC : GEditor->GetAllViewportClients())
	{
		if (VC && VC->IsPerspective() && VC->GetWorld() == World)
		{
			Camera = VC->GetViewLocation();
			bGotCamera = true;
			break;
		}
	}
	if (!bGotCamera)
	{
		UE_LOG(LogRoadNet, Warning, TEXT("RoadNet.EditRegion: no perspective viewport to centre on."));
		return;
	}

	const double HalfCm = SizeM * 100.0 * 0.5;
	const FVector2D C(Camera.X, Camera.Y);
	Net->EditRegion = FBox2D(C - FVector2D(HalfCm, HalfCm), C + FVector2D(HalfCm, HalfCm));
	Net->MarkPackageDirty();
	Report();
}

void SurfaceCacheSelfCheckCmd()
{
	// Needs no level: the check builds its own two-road context. Uses the CDO so
	// it runs the same code an imported network would.
	const URoadNetwork* Net = GetDefault<URoadNetwork>();
	FString Report;
	const bool bOk = Net->SurfaceCacheSelfCheck(Report);
	if (bOk) { UE_LOG(LogRoadNet, Log,     TEXT("[RoadNet] %s"), *Report); }
	else     { UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet] %s"), *Report); }
}

FAutoConsoleCommand GSurfaceCacheSelfCheck(
	TEXT("RoadNet.SurfaceCacheSelfCheck"),
	TEXT("Verify the surface-plan cache fingerprint is stable, blind to height changes, and sensitive to footprint changes."),
	FConsoleCommandDelegate::CreateStatic(&SurfaceCacheSelfCheckCmd));

void SidewalkKnotSelfCheckCmd()
{
	FString Report;
	const bool bOk = URoadNetwork::SidewalkKnotSelfCheck(Report);
	if (bOk) { UE_LOG(LogRoadNet, Log,     TEXT("[RoadNet] %s"), *Report); }
	else     { UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet] %s"), *Report); }
}

FAutoConsoleCommand GSidewalkKnotSelfCheck(
	TEXT("RoadNet.SidewalkKnotSelfCheck"),
	TEXT("Verify per-point sidewalk widths interpolate and clamp, that MaxSidewalkWidth covers a knotted bulge, and that the surface cache notices a knot edit."),
	FConsoleCommandDelegate::CreateStatic(&SidewalkKnotSelfCheckCmd));

void MeshRepairSelfCheckCmd()
{
	FString Report;
	const bool bOk = RoadNetMesh::RepairSelfCheck(Report);
	if (bOk) { UE_LOG(LogRoadNet, Log,     TEXT("[RoadNet] %s"), *Report); }
	else     { UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet] %s"), *Report); }
}

FAutoConsoleCommand GMeshRepairSelfCheck(
	TEXT("RoadNet.MeshRepairSelfCheck"),
	TEXT("Verify the tile/bake normal + winding repair fixes unset, zero and mis-wound triangles and leaves a healthy mesh untouched."),
	FConsoleCommandDelegate::CreateStatic(&MeshRepairSelfCheckCmd));

void StandardsSelfCheckCmd()
{
	FString Report;
	const bool bOk = RoadNetStandards::SelfCheck(Report);
	if (bOk) { UE_LOG(LogRoadNet, Log,     TEXT("[RoadNet] %s"), *Report); }
	else     { UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet] %s"), *Report); }
}

FAutoConsoleCommand GStandardsSelfCheck(
	TEXT("RoadNet.StandardsSelfCheck"),
	TEXT("Verify the turn-lane geometry tables (decel length, grade correction, taper) interpolate monotonically and honour their floors, and that every LOT\"M paint spec is usable."),
	FConsoleCommandDelegate::CreateStatic(&StandardsSelfCheckCmd));

void DashSelfCheckCmd()
{
	FString Report;
	const bool bOk = RoadNetMarkings::DashSelfCheck(Report);
	if (bOk) { UE_LOG(LogRoadNet, Log,     TEXT("[RoadNet] %s"), *Report); }
	else     { UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet] %s"), *Report); }
}

FAutoConsoleCommand GDashSelfCheck(
	TEXT("RoadNet.DashSelfCheck"),
	TEXT("Verify the dashed-polyline helper behind the junction turn guides: dash count, painted duty cycle, and that degenerate inputs refuse instead of looping forever."),
	FConsoleCommandDelegate::CreateStatic(&DashSelfCheckCmd));

FAutoConsoleCommandWithWorldAndArgs GEditRegionCmd(
	TEXT("RoadNet.EditRegion"),
	TEXT("Scope rebuilds and terrain conform to a square around the camera: 'RoadNet.EditRegion 1000' (metres), 'off', or no argument to report."),
	FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(&EditRegionCmd));
} // namespace

#define LOCTEXT_NAMESPACE "RoadNetEditor"

class FRoadNetEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		FEditorModeRegistry::Get().RegisterMode<FEdModeRoadNet>(
			FEdModeRoadNet::GetModeID(),
			LOCTEXT("RoadNetMode", "RoadNet Draw"),
			FSlateIcon(FAppStyle::GetAppStyleSetName(), "LevelEditor.BrushEdit"),
			true,
			7100);

		// Releases the render state of tiles the camera is nowhere near, so a
		// city-scale import cannot exhaust the D3D12 descriptor heap. Visual
		// quality is unchanged: see RoadNetTileStreamer.cpp.
		RoadNetTileStreamer::Startup();
	}

	virtual void ShutdownModule() override
	{
		RoadNetTileStreamer::Shutdown();
		FEditorModeRegistry::Get().UnregisterMode(FEdModeRoadNet::GetModeID());
	}
};

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FRoadNetEditorModule, RoadNetEditor)
