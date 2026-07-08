// OSMRoadCore.Build.cs
using UnrealBuildTool;

public class OSMRoadCore : ModuleRules
{
	public OSMRoadCore(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core", "CoreUObject", "Engine",
			// Migrated PCG road nodes (LandscapeSplineToPoints, StitchPaths, TrimPathsByGap,
			// MovingAverage, FilterPathByDistance, GroovedSidewalk) — self-contained, replacing
			// the third-party RoadGenerator (Yazan) runtime module.
			"PCG",
			"Landscape",
			"GeometryFramework",
			"DynamicMesh",
			"GeometryScriptingCore",
			"GeometryAlgorithms",
			"GeometryCore",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"Json", "HTTP", "GeoReferencing",
			"Kismet",
			"Foliage",                   // LandscapeEdit.h pulls in InstancedFoliageActor.h (Foliage module)
			"ProceduralMeshComponent",   // road-surface + sidewalk geometry along the centerlines
			// Pipeline 2: reuse the RoadBuilder plugin's lane model + road-mesh engine
			// (ARoadScene/ARoadActor/URoadStyle/ULaneShape). We feed it OSM-derived
			// centrelines + per-class styles and let it triangulate the carriageway,
			// curbs, sidewalks, lane markings and CDT junctions.
			"RoadBuilder",
			// Pipeline 4: our own independent road engine (clean-room, no RoadBLD).
			// OSMRoadCore -> RoadNet is the only allowed dependency direction.
			"RoadNet",
		});

		if (Target.bBuildEditor)
		{
			PrivateDependencyModuleNames.Add("UnrealEd");
			// Landscape-spline bake backend (OSMRoadSplineDeform): editor-only
			// spline control-point/segment helpers + ApplySplines.
			PrivateDependencyModuleNames.Add("LandscapeEditor");
			PrivateDependencyModuleNames.Add("GeometryScriptingEditor");
		}
	}
}
