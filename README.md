# RoadNet

An independent, procedural road-network engine for Unreal Engine 5.8 (Pipeline 4).

Clean-room implementation of a staged rebuild pipeline with Clipper2 boolean-union
junctions. Fully independent of RoadBLD / WorldBLD / CityBLD.

## Features

- **Dual-input authoring** — build roads from OSM data or hand-draw them in the editor.
- **Boolean-union junctions** — road outlines are merged so intersections and
  end-to-end seams fill automatically (no bespoke junction meshes, no map-wide blobs).
- **Grade separation** — a union-find zoning pass keeps overpasses/underpasses on
  separate layers so they never merge.
- **Sidewalks** — derived from the merged carriageway (dilate + subtract) for clean,
  flap-free bands, with per-side control.
- **Lane markings** — baked centre lines, edge lines, and dashed interior dividers
  (white/yellow), suppressed inside junctions.
- **Interpolated elevation** — mesh vertices follow the nearest centreline segment.
- **Interactive editing** — click-to-draw with snapping, point move/delete, mid-span
  split, and whole-road selection/deletion.
- **PCG export** — perimeter loops (outer outlines + inner block holes) emitted as
  closed spline components for PCG graphs.
- **Material overrides** — per-layer materials for road, sidewalk, and markings.

## Modules

- `RoadNet` (Runtime) — data model, geometry math, surface/mesh/zone/marking builders.
- `RoadNetEditor` (Editor) — the click-to-draw editor mode.

## Installation

Copy this folder into your project's `Plugins/` directory and enable **RoadNet** in the
plugin browser. Requires the **PCG** and **GeometryScripting** plugins.
