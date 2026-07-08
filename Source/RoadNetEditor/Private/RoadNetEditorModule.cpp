// RoadNetEditorModule.cpp — registers the click-to-draw road EdMode (§9.3).
#include "Modules/ModuleManager.h"
#include "EditorModeRegistry.h"
#include "EditorModeManager.h"
#include "Styling/AppStyle.h"
#include "Textures/SlateIcon.h"
#include "EdModeRoadNet.h"

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
	}

	virtual void ShutdownModule() override
	{
		FEditorModeRegistry::Get().UnregisterMode(FEdModeRoadNet::GetModeID());
	}
};

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FRoadNetEditorModule, RoadNetEditor)
