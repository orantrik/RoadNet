# Integration files (OSM → RoadNet)

These files do **not** belong to the RoadNet plugin. They live in the separate
`OSMRoadCore` plugin and are mirrored here only as a **restore point** so the full
OSM import pipeline can be recovered from this repository.

The directory layout under `Integration/OSMRoadCore/` mirrors the original relative
paths, so restoring is a straight copy back into your project's `Plugins/OSMRoadCore/`:

| File | Original location | Role |
| --- | --- | --- |
| `OSMRoadCore.uplugin` | `Plugins/OSMRoadCore/OSMRoadCore.uplugin` | Declares the `RoadNet` plugin dependency. |
| `Source/OSMRoadCore/OSMRoadCore.Build.cs` | same | Adds `RoadNet` to module dependencies (OSMRoadCore → RoadNet is the only allowed direction). |
| `Source/OSMRoadCore/Public/OSMRoadNetBridge.h` | same | Bridge API: OSM ways → `URoadNetwork`. |
| `Source/OSMRoadCore/Private/OSMRoadNetBridge.cpp` | same | Bridge impl: finds/spawns `ARoadNetActor`, refreshes only OSM-sourced roads, rebuilds. |
| `Source/OSMRoadCore/Private/OSMOverpassRoadImport.cpp` | same | OSM importer. Contains the `osm.RoadPipeline=4` branch that routes parsed/draped ways into RoadNet. |

> Note: `OSMOverpassRoadImport.cpp` is a large, general OSM-import file; only its
> Pipeline-4 branch is RoadNet-specific. It is included whole so the exact call site
> is preserved.

The RoadNet plugin itself (in the repo root) is self-contained and does not require
any of these files to compile or to be used for hand-drawn authoring.
