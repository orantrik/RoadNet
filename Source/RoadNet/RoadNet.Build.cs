// RoadNet.Build.cs
//
// INDEPENDENCE MANDATE (see ROADBLD_REPLICATION_PLAN.md §10.0):
//   This module MUST NOT depend on RoadBLD / WorldBLD / CityBLD in any form.
//   Only engine modules + our own plugins are allowed here. A CI grep guard
//   should fail the build if any RoadBLD-family symbol appears in RoadNet.
using UnrealBuildTool;

public class RoadNet : ModuleRules
{
	public RoadNet(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			// Geometry stack for offsets, boolean surface, triangulation (§10.4/§10.9/§10.15).
			"GeometryCore",
			"GeometryAlgorithms",
			// PCG emission path (§8.4) — declared now, wired in a later phase.
			"PCG",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"DynamicMesh",
			"GeometryFramework",
		});

		if (Target.bBuildEditor)
		{
			PrivateDependencyModuleNames.Add("UnrealEd");
		}
	}
}
