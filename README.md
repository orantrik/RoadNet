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
  (white/yellow), suppressed inside junctions. Divider positions come from the
  resolved lane stack rather than uniform division, so unequal lane widths paint
  correctly, and stripe thickness scales with the lane it borders.
- **Lane direction** — every lane carries an explicit direction (forward, backward,
  both, or none) instead of inferring it from which side of the centreline it sits.
  The centre line is drawn only when traffic actually runs both ways.
- **Traffic handedness** — `bDriveOnLeft` (also the `roadnet.DriveOnLeft` CVar and a
  panel toggle) flips lane stacking, the lane graph, stop-bar and zebra placement,
  and paints the centre line white instead of yellow, so a UK street reads as one.
- **Mid-block crossings** — zebras are no longer junction-only; click anywhere on a
  road in the Junctions tool to place one, with an optional stop bar.
- **Interpolated elevation** — mesh vertices follow the nearest centreline segment.
- **Interactive editing** — click-to-draw with snapping, point move/delete, mid-span
  split, `Ctrl+click` insert on a segment (with a ghost marker while Ctrl is held),
  and whole-road selection/deletion.
- **Auto-release** — a discrete placement (junction preset, corner islands, parking
  bay, lane insert, crossing) clears the selection and flushes pending smoothing on
  the spot. Repeat adjustments keep their target and release after
  `roadnet.AutoReleaseSec` of no further input, so a value can still be nudged twice.
- **Junction conditioning** — `DeclusterNearJunctions` thins points bunched at an
  intersection and `StraightenJunctionApproaches` blends the first few points onto the
  approach tangent, so splines enter a junction cleanly instead of kinking. Both run
  from Smooth Roads and from the OSM import, behind `roadnet.JunctionConditioning`.
- **Cross-section editor** — a Streetmix-style head-on view of the selected road's lane
  stack, in the "Cross-section" tab of the OSM Roads panel. Click a lane to select it
  (the pick is mirrored in the viewport and back), drag the line between two lanes to
  trade width between them, drag an outer edge to widen the road, and set type and
  travel direction from the palettes. Edits go through `SetLaneWidth` / `SetLaneType` /
  `SetLaneDirection` / `RemoveLaneAt`, which materialise `DetailedLanes` and relayout,
  then rebuild that one road. `RoadNet.LaneSelfCheck` asserts their invariants, and the
  junction rules alongside them (lane continuity, the rightmost drop, taper lengths,
  and the junction grade bounds — a `Table 8.1` ceiling of 4–6% on each arm, and the
  §8.2.3 *floor* of 1% that a junction needs in order to drain).
- **Rounded junctions out of the box** — `JunctionSmoothingCm` defaults to 150 (the 15
  `]` steps a new junction used to need by hand), and rounding is always applied
  *locally* per junction rather than as one close over the whole zone, so a larger radius
  buys rounder corners without welding roads that merely run near each other. Junction
  paint and signals set themselves back by the smoothing radius on top of
  `JunctionClearanceCm`, since the fillet carries pavement further up each approach.
- **Standards-based junctions** — junction geometry follows the Israeli interurban
  design guidelines (`docs/ISRAELI_JUNCTION_STANDARDS.md`). An arm keeps its full width
  right up to the node: where two arms disagree on lane count, the surplus lane is
  carried *out* of the junction and tapered away beyond it on the rightmost lane over
  the `Table 5.1` length, and a single-lane approach is widened into a turn bay rather
  than having a through lane repurposed. Every arm carries a bearing, lane count and
  design speed, arms are ordered around the node, and the main axis is elected by road
  class then lane count so it keeps its alignment and level through the junction.
  Behind `bChannelizeJunctions`.
- **Lane matching, not a cross-product** — the connectivity graph pairs through lanes by
  position counting out from the kerb, and takes each turn off the lane it is signed
  from, instead of joining every incoming lane to every outgoing one.
- **PCG export** — perimeter loops (outer outlines + inner block holes) emitted as
  closed spline components for PCG graphs.
- **Material overrides** — per-layer materials for road, sidewalk, and markings.

## Modules

- `RoadNet` (Runtime) — data model, geometry math, surface/mesh/zone/marking builders.
- `RoadNetEditor` (Editor) — the click-to-draw editor mode.

## Installation

Copy this folder into your project's `Plugins/` directory and enable **RoadNet** in the
plugin browser. Requires the **PCG** and **GeometryScripting** plugins.
