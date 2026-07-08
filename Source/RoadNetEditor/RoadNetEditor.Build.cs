// RoadNetEditor.Build.cs — editor-only module: the click-to-draw road mode (§9.3).
//
// INDEPENDENCE MANDATE: no RoadBLD / WorldBLD / CityBLD dependencies.
using UnrealBuildTool;

public class RoadNetEditor : ModuleRules
{
	public RoadNetEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"RoadNet",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"InputCore",
			"Slate",
			"SlateCore",
			"UnrealEd",
			"EditorFramework",
			"LevelEditor",
		});
	}
}
