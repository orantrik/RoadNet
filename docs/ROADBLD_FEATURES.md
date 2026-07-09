# RoadBLD — Feature & UX Capture (from `MyProject5`)

Running record of RoadBLD's observed features/UI while exploring the tool, so we
can decide parity for our RoadNet editor mode + pipeline. Grouped by feature set;
appended to as we walk through each.

---

## 1. Road Draw Tool (panel)

Top‑level toggles:

| Option | Default | Meaning / notes |
|---|---|---|
| **Parallel Draw** | off | Draw a road parallel to an existing one (offset carriageway / divided‑road authoring). |
| **Enable Snapping** | on | Cursor snaps to existing road points/segments (endpoints, mid‑span, other roads). |
| **Align Landscape** | on | The terrain‑conform toggle we studied — mirrors the road into a landscape spline + conforms the road profile to terrain (`bEnableLandscapeSplineMirror` / `bEffectiveAlignLandscape`). |
| **Draw Mode** | `Straight` | Dropdown: **Straight / Curve / Circle** — the geometry primitive being placed. |
| **Advanced** | collapsed | (contents TBD — expand next pass) |
| **Show Controls** | — | Expander that lists the input controls (below). |

### Draw Mode options
- **Straight** — polyline segments (click to place points).
- **Curve** — arc/curve placement (a Left Click "set curve center" step, per the
  controls list → likely center + endpoints arc authoring).
- **Circle** — closed circular road (roundabout‑style loop).

### Controls (verbatim from the panel)
- **Left Click** — Place road point / add segment / set curve center / select merge lane
- **Ctrl + Scroll Wheel** — Adjust brush vertical offset, or merged lane count in Merge Tool mode
- **Mouse Move** — Update road preview and snap targets
- **Escape** — Cancel current drawing state or exit tool
- **R** — Toggle unrestricted placement mode
- **Alt (hold)** — Enable angle snapping while drawing
- **Shift** — Blend two connected roads together

### Curved Merge (Shift‑blend) — live capture
The **Shift** "blend two connected roads" / "select merge lane" controls drive a
**smooth curved merge** of a new road into an existing one:
```
RoadController: Enabled smooth curved merge mode - MergeSourceRoad: DynamicRoad_…1128796338,
  Side: 0, MergeFromInnerEdge: Valid, CurvePoint: (-70098.95, 567.56, 627.29)
RoadController: Applied backwards merge offset in preview (NormalLaneWidthSum=315.00)
RoadController: Mid-road snap found at distance 315.10 cm from preview endpoint
RoadController: Disabled curve alignment after placing merge point
```
Mechanics:
- Merge is **relative to a source road + side** (`Side: 0`, `MergeFromInnerEdge`)
  — you attach to one **side/inner edge** of the target, not its centerline.
- **Lane‑width aware:** the join is offset by `NormalLaneWidthSum` (315 cm here),
  so the merging road lands at the correct **lane edge** (this is the
  divided‑road / ramp‑taper behaviour).
- **Mid‑road snapping:** snaps the merge point to a distance along the target
  road ("Mid‑road snap found at distance … from preview endpoint").
- **Curve alignment** is toggled on during placement then "Disabled … after
  placing merge point" → it auto‑curves the tie‑in tangent, then releases.
- The pick trace here hit a **`RoadGeo`** (road mesh), `bEffectiveAlignLandscape=0`
  — merging onto a road aligns to the road, not the terrain.

→ RoadNet parity: a lane‑aware merge that offsets the tie‑in by the target's
lane‑width sum, snaps to a point along the target's side edge, and curves the
tangent in — richer than our plain endpoint weld.

### Parity notes for our RoadNet `FEdModeRoadNet`
- We already have: click‑to‑place, snapping (endpoints + mid‑span), point
  move/delete, mid‑span split, whole‑road select/delete.
- **Gaps vs RoadBLD to consider:**
  - Draw‑mode **Curve** (center+arc) and **Circle** (roundabout) primitives.
  - **Parallel Draw** (offset a new road from an existing one).
  - **Vertical offset** via Ctrl+Scroll (brush Z), for over/underpasses.
  - **Angle snapping** while drawing (Alt‑hold).
  - **Blend/merge** two connected roads (Shift) + a **Merge Tool** mode with a
    Ctrl+Scroll merged‑lane‑count adjuster (divided‑road / lane‑count authoring).
  - **Unrestricted placement** toggle (R) — bypass snapping/validity for free placement.
- **Align Landscape** maps directly to the two‑way conform work we're bringing
  back to McpPcg (see `ROADBLD_TERRAIN_DEFORM.md`).

---

## 2. Main toolbar + Edit tools

The top‑level RoadBLD toolbar is organised into tool **categories**:

| Category | Purpose |
|---|---|
| **Draw** | Place new roads (the Road Draw Tool panel in §1). |
| **Edit** | "Road editing tools" — a submenu (see below). |
| **Junctions** | "Road junction editing tools" — submenu (see §3). |
| **Lanes** | Lane editing (add/remove/assign lanes) (§ TBD). |
| **Markings** | Lane/road markings authoring (§ TBD). |
| **Details** | Props / detail meshes along roads (§ TBD). |
| **Tools** | Dropdown of extra tools/utilities (§ TBD). |

### Edit → "Road editing tools" (submenu)

| Tool | Icon | Function (observed / inferred) |
|---|---|---|
| **Move Roads** | ↔ over dashes | Select + translate whole roads (and/or their points); the mirror spline + conform re‑sync on release. |
| **Delete Roads** | ✕ | Remove selected road(s) from the network. |
| **Chop Roads** | dashed crop box | Split/cut a road at a point (mid‑span chop into two roads) — like our `InsertRoadPoint`/split but producing separate road entities. |

### Edit Roads Tool (panel)

Options:

| Option | Default | Meaning / notes |
|---|---|---|
| **Enable Snapping** | off | Master snap toggle for edit‑time dragging. |
| **Snap To Individual Lanes** | off | Snap to per‑lane offsets (not just centerline) — divided‑road / lane‑level alignment. |
| **Keep Endpoint Snap** | off | Preserve an endpoint's snapped connection while editing (don't break the junction on drag). |
| **Snap Alignment Distance** | `4.0` | Snap radius (world units, likely m) for alignment targets. |
| **Turn Radius** | `1000000.0` | Min turn/corner radius; the huge default ≈ "no forced smoothing" (straight corners) until lowered. |
| **Landscape Height Offset** | `5.0` | Vertical offset of the road above the conformed landscape — the small lift that keeps the surface above the deformed terrain (our `kRoadZLiftCm` analogue). |

Controls (verbatim):
- **Left Click** — Select road or corner (via actor/hit proxy selection)
- **Mouse Move** — Highlight hovered road, track cursor for corner proximity
- **Gizmo Drag** — Move spline control points with endpoint snapping and preview
- **Escape** — Deselect road or exit tool (when no road selected)

Notes:
- Uses **hit‑proxy selection** for roads *and* corners (matches our
  `HRoadNetPointProxy`/`HRoadNetSegmentProxy` approach).
- **Gizmo Drag** = transform‑widget move of control points with live preview +
  endpoint snapping (same as our translate‑widget edit).
- **Landscape Height Offset (5.0)** is the counterpart to our road Z‑lift — worth
  exposing as a tunable in RoadNet too, now that we conform to terrain.
- **Keep Endpoint Snap** is a nicety we lack: hold a junction connection stable
  while moving interior points.

### Chop Roads (detail — live capture)

`ChopController` splits one road into two independent `DynamicRoad`s at a picked
spline location:
```
ChopController: Tool started
ChopController: No valid road to split                     ← when not over a road
ChopController: Splitting road 'DynamicRoad_…1842929389'
  at distance 142873.42 (InputKey: 7.96, SplitAfterIndex: 7)
ChopController: Copied spline range [0-7] with no start, added end   → dest (9 points)
ChopController: Copied spline range [8-10] with added start, no end  → dest (4 points)
  -> No landscape spline owner, calling InitializeMirrorSpline…   (×2, one per new road)
```
Mechanics:
- Pick point = a **spline distance / input key** (`7.96`), resolved to
  `SplitAfterIndex = 7`.
- **Half A** = original points `[0..7]` **+ an added end point** at the cut
  (9 pts). **Half B** = **an added start point** at the cut **+** points `[8..10]`
  (4 pts). → the **cut vertex is duplicated into both** halves so they stay
  coincident (a shared endpoint that will weld back as a junction).
- Each new road **initializes its own landscape mirror spline**
  (`InitializeMirrorSpline` ×2) — i.e. chop = two fresh roads, each re‑drives its
  own terrain deform.

→ RoadNet parity: a `SplitRoadAtPoint(road, param)` that produces two `FRoadDef`s
sharing the split vertex (so endpoint welding reconnects them), rather than our
current mid‑span *insert* (which keeps one road).

### Parity notes for RoadNet
- We have equivalents for **Move** (point translate widget), **Delete**
  (point + whole‑road delete), and **split** (Ctrl+click mid‑span
  `InsertRoadPoint`). RoadBLD separates these into explicit **modal tools** with
  their own toolbar buttons rather than modifier‑key gestures.
- **Chop Roads** is subtly different from our mid‑span *insert*: it appears to
  **sever** a road into two independent roads (topology split), not just add a
  vertex. Worth a dedicated `SplitRoadAtPoint` that creates two `FRoadDef`s.
- Toolbar is **category → tool** (Draw / Edit / Markings / Details / Tools); our
  single `FEdModeRoadNet` could expose sub‑tools similarly (modal buttons in the
  mode toolbar) for discoverability.

---

## 3. Junctions — "Road junction editing tools"

A top‑level category (between Edit and Markings) with a submenu:

| Tool | Icon | Function (observed / inferred) |
|---|---|---|
| **Corners** | road + corner marks | Edit the corner/fillet geometry at intersections (rounding radius, corner shaping where arms meet). |
| **Freeway Ramps** | curved off‑ramp | Author freeway ramps / slip lanes / interchange connectors between roads. |

### Corners (detail)

Per‑corner parameters (panel reuses the "Edit Roads Tool" header):

| Param | Example | Meaning |
|---|---|---|
| **Radius** | `1356.4` | Corner **fillet radius** — the rounding arc where two arms meet. |
| **Start Offset** | `560.4` | Distance along the **first** arm from the junction where the corner blend begins. |
| **End Offset** | `560.4` | Distance along the **second** arm where the corner blend ends. |

Controls: same as Edit Roads Tool — Left Click select road/**corner** (hit
proxy), Mouse Move highlight + corner proximity, **Gizmo Drag** adjust, Escape.

Storage / live behaviour (captured):
- Corners live in an indexed array **`CornerEditData[i]`** on the controller;
  the tool logs `RoadNetworkController::SetCornerParameters: Updated
  CornerEditData[6] - Radius: …, StartOffset: 500.00, EndOffset: 500.00` while
  dragging (radius streams live as you drag; offsets held at 500 here).
- Entering the tool logs "Corner editing is enabled, **skipping RoadGeo
  selection redirection**" → corner mode swaps the hit‑proxy selection target
  from the road mesh to the **corner** entity.

Interpretation:
- A corner is an **editable entity per arm‑pair at a junction**, defined by a
  fillet **radius** + **start/end offsets** along each incident arm (an offset
  arc/curve blend, not a single disc). Selectable via hit proxy and tweakable.
- This is richer than our implicit fillet **disc** (single radius centred on the
  junction). To reach parity we'd model each corner as `{radius, startOffset,
  endOffset}` between two arms and blend an arc between the offset points.
- Relates to RoadBLD's `BooleanIntersection` + intersection **Landscape Texture
  Patch** (plane‑fit pad) — see `ROADBLD_TERRAIN_DEFORM.md` §3.

### Freeway Ramps (detail — confirmed)
- Tooltip: **"Create freeway merge ramps"**.
- **Reuses the exact same curved‑merge path as the Shift‑blend** (§1): logs the
  identical `RoadController: Enabled smooth curved merge mode` with
  `MergeSourceRoad` / `Side` / `MergeFromInnerEdge` / `Applied backwards merge
  offset` / `Disabled curve alignment after placing merge point`. There is **no
  separate ramp code path** — a freeway ramp is authored as a lane‑width‑offset
  curved merge onto the target road's side edge.
- New nuance: the merge offset can be **toggled in preview** — `Applied backwards
  merge offset in preview` ↔ `Removed backwards merge offset in preview` (flips
  which side/direction the ramp tapers from).
- **No grade‑separation/layer logging observed** for ramps in this session — the
  ramps here were at‑grade merges (over/under handling, if any, not exercised).

### Parity notes for RoadNet
- Junction editing is a **dedicated modal category** in RoadBLD, distinct from
  road Edit. We currently derive junctions implicitly (endpoint welding +
  boolean union). A **Corners** tool implies explicit, per‑junction editable
  parameters (radius/shape), which we'd need to surface as editable junction
  entities rather than purely implicit geometry.

---

## 4. Lanes (edit lanes + ZoneGraph)

### Lane data model (from reflection: `DynamicRoad/DynamicRoadLane.h`)

**`UDynamicRoadLane`** — a lane is a first‑class object, not just an index:

| Field | Type | Meaning |
|---|---|---|
| `LaneID` | `FGuid` | Stable per‑lane id (used for connections/graph). |
| `LeftEdgeCurve` / `RightEdgeCurve` | `UEdgeCurve*` | The lane's two boundary curves (lane is the ribbon between them). |
| `LaneWidth` | double | Nominal width. |
| `ActiveSegments` | `FLaneWidthSegment[]` | **Variable‑width** sections along the lane. |
| `LaneSections` | `FLaneSection[]` | Longitudinal lane sections (per‑range attributes). |
| `SidewalkProfile` | `UCurveFloat*` | Cross‑section profile for an attached sidewalk. |
| `SidewalkMaterial` / `LaneOverlayMaterial` | `UMaterialInterface*` | Per‑lane materials. |
| `TextureUScale` / `TextureVScale` | double | UV tiling for the lane surface. |

**`FLaneWidthSegment`**: `StartDistance`, `EndDistance`, **`TransitionIn`**,
**`TransitionOut`** → lanes **taper** between widths over a transition distance.
This is exactly the mechanism for **turn bays / ramp tapers / lane add‑drop**:
a lane whose width ramps 0→full (or full→0) over `TransitionIn/Out`.

Implications for RoadNet: our `FRoadNetLaneSpec` is currently a flat per‑road
lane count. To match, lanes need to become **per‑lane entities** with boundary
curves + variable‑width segments (taper in/out), enabling turn bays and ramp
merges rather than a uniform carriageway.

### Edit Lanes option (`RoadLaneController`) — live capture
The Lanes tool is a **state machine** driven by `RoadLaneController`:
```
RoadLaneController: Tool Begin in SelectLane mode
RoadLaneController: Switching from state 0 to state 1
RoadLaneController: Entered SelectEdge mode with 2 edges
RoadLaneController: Transitioned to SelectEdge mode with 2 edges
```
- **State 0 = `SelectLane`** → pick a lane.
- **State 1 = `SelectEdge`** → then operate on the lane's **edges** ("with 2
  edges" = the lane's left/right boundary, i.e. the `LeftEdgeCurve` /
  `RightEdgeCurve` from the data model). So lane editing is **edge‑curve based**
  (adjust the boundary curves), not centerline‑offset based.
- **State 2 = `EditOffsets`** ⭐ → click an **EdgeCurve** (`EdgeCurve clicked
  (Index: 0)`) to edit its shape. The edge is a list of **OffsetPoints** =
  `(distanceAlongRoad, lateralOffset)`; the tool seeds them from the curve
  (`Initialized 2 points from EdgeCurve OffsetPoints`) and you **add/move offset
  points**:
  ```
  Adding new offset point at distance 900.00  with offset 915.00   → 3 points
  Adding new offset point at distance 900.00  with offset 1069.33  → 4 points
  Adding new offset point at distance 1700.00 with offset 1096.07  → 5 points
  Adding new offset point at distance 2100.00 with offset 1095.85  → 6 points
  ```
  → **A lane boundary (`UEdgeCurve`) is a piecewise `distance → lateral‑offset`
  polyline.** Lane width/shape (widenings, turn bays, tapers) is authored by
  editing these offset points on the left/right edge curves — *not* a single
  scalar width. This is the ground truth behind `FLaneWidthSegment`.
- **State 3 = `AddLane`** → `Entered AddLane mode (RangeMode: OFF, Range:
  0.0 - 0.0)`. Adds a lane either **full‑length** (RangeMode OFF) or over a
  **sub‑range** `[StartDist, EndDist]` (RangeMode ON) — the partial‑range case is
  the **turn bay / merge lane** (a lane that exists only along part of the road).
- **State machine:** `0 SelectLane → 1 SelectEdge → 2 EditOffsets` (per edge) and
  `→ 3 AddLane`; returns to `0` after a commit (each edit triggers a normal
  rebuild: mesh + `MirrorSpline` conform + junction patch bake).
- Per‑commit rebuild also re‑conforms the modified road — captured
  `ConformReason_ModifiedRoad: 1` + `MirrorSplineRefresh: 1` +
  `Landscape slowest: Conform_GlobalActorScan` on each commit (two‑way conform is
  driven per lane edit, tagged by reason).
- Junction connections / routing graph: **resolved — see "ZoneGraph creation"
  below** (RoadBLD does not build one).

### Lane Editing panel

| Option | Values | Meaning |
|---|---|---|
| **Lane Preset** | `None`, … | Preset bundle of lane settings. |
| **Lane Behavior** | see enum ↓ | Semantic type of the lane. |
| **Lock Lane Widths** | checkbox | Keep other lanes' widths fixed while editing one edge. |
| **Mode** | `Select Lane` / `Add Lane` (/ `Form Lane`) | Current tool sub‑mode (matches the state machine). |

**Lane Behavior enum** — confirmed from reflection as **`ELaneType`** (`enum
class : uint8`, in `Public/DynamicRoad/DynamicRoadData.h`). Panel label "Lane
Behavior" = this enum:

| `ELaneType` | Display name | Notes |
|---|---|---|
| `Normal` | Normal Driving Lane | default drivable lane |
| `Parking` | Street‑side parking area | |
| `Border` | Border Lane | |
| `Restricted` | Restricted Lane | |
| `Shoulder` | Shoulder Lane | drives the shoulder EdgeCurve / mask logic |
| `CenterTurn` | Center Turn Lane | |
| `Median` | Median Lane | |
| `None` | Empty lane data (**Deprecated**) | hidden; "Use `USidewalk` instead" |

→ RoadNet parity: replicate `ELaneType` minus the deprecated `None` (sidewalks
are their own object, not a lane type).

### Lane direction — confirmed: it's **side‑based**, not a per‑lane flag
There is **no `bReverse`/`Forward`/`Direction` field on a lane.** Direction is
encoded by which **side of the reference line** the lane sits on, via enum
**`ERoadSide { Left, Right, None }`** (reflection comment: *"Helper enum to make
selecting a side more clear than remembering 0 equals left and 1 equals right"*).
- Confirmed live: `AddLane: … Added a new right side lane. Total number of lanes
  on this road is 5` — lanes are added **per side**.
- Matches the viewport arrows: left‑side lanes point one way (yellow), right‑side
  the other (purple). Travel direction = side of centerline.
- `DynamicRoad` exposes "all lanes (both **left and right** lanes, excluding
  sidewalks)".

→ RoadNet parity: model direction as `ERoadSide` (left/right of the road frame),
not a per‑lane forward/back bool. Our road frame already has a signed lateral
axis, so left = −offset, right = +offset; forward/back follows the side.

Controls (verbatim):
- **Left Click** — Select lane (Select Lane) / choose outer edge to preview/add lane (Add Lane/Form Lane)
- **Right Click** — Commit lane add on hovered edge (Add Lane/Form Lane)
- **Drag Gizmo** — Edit selected lane edge offsets
- **Delete** — Delete selected lane while editing offsets
- **Escape** — Step back to previous lane tool state

Commit log (from the edit before the crash):
```
RoadLaneController: Switching from state 2 to state 0
RoadLaneController: Rebuilt road network with modified road
RoadLaneController: Applied 8 offset points to EdgeCurve and recalculated lane shapes
```
→ committing EditOffsets writes the offset points back to the `EdgeCurve` and
**recalculates lane shapes**, then triggers the standard rebuild.

Parity for RoadNet: model a lane as `{behavior enum, leftEdge OffsetPoints[],
rightEdge OffsetPoints[]}` where each edge is a `distance→offset` polyline;
support the behavior taxonomy above (driving / parking / border / shoulder /
center‑turn / median / restricted).

### ⚠️ RoadBLD crash observed (their bug, note only)
Committing the lane edit (during the network **store serialization**) crashed:
```
Unhandled Exception: EXCEPTION_ACCESS_VIOLATION reading address 0x0000000000060004
UnrealEditor-RoadBLDRuntime.dll!`anonymous namespace'::SerializeObjectPath()
  [ …\RoadBLD\Source\RoadBLDRuntime\Private\DynamicRoad\Store\RoadNetworkStore.cpp:70 ]
```
- Occurred in the commit's store‑persist step (`StageStoreChunks` /
  `FlushStoreToBulkData`), consistent with a **`RoadNetworkStore`** that
  serializes the network to bulk‑data chunks via object‑path references — a
  dangling/`nullptr` object path at `RoadNetworkStore.cpp:70`.
- This is a **RoadBLD 1.5.6 bug**, not our code. Lesson for RoadNet: if we
  persist a network store with object‑path refs, **null‑guard the serializer**.
- **Repro status:** on the follow‑up session the crash **did not reproduce** —
  the exact same lane‑edit sequence (SelectEdge → EditOffsets → add 3/6 offset
  points → commit, repeated) serialized cleanly every time
  (`StageStoreChunks` + `FlushStoreToBulkData` OK). So it's a **transient /
  non‑deterministic** dangling object‑path in their store, not a reliable
  reproducer. Nothing for us to fix; noted for defensiveness only.

### ZoneGraph creation — CONFIRMED FINDING (from reflection + logs)
**RoadBLD does not build a lane‑routing ZoneGraph inside `RoadBLDRuntime`.**
Verified two ways:
1. **Reflection**: there is **no `AZoneShape`/`UZoneGraph*` UCLASS** authored by
   RoadBLD. The *only* mention of ZoneShape is a comment on the sidewalk
   partition data — *"Whether pedestrian **ZoneShapes** and other walkability
   consumers should use this partition"* (`USidewalkPartition::bWalkable`). So
   RoadBLD **exposes** lane/partition data that an external ZoneGraph *could*
   consume; it does not generate the graph itself.
2. **Logs**: across the whole lane‑editing session, **zero** `Zone/ZoneGraph/
   ZoneShape/Connection/Successor` lines were emitted. Every commit only ran the
   geometry rebuild (`StageStoreChunks` → `FlushStoreToBulkData`) + conform +
   mirror‑spline refresh.

What RoadBLD actually builds at rebuild (from the reflected **Rebuild* builders**):
`Curve → EndpointJoint → Corner → **PairwiseZone** → SurfaceBoolean →
OverlapMask/ShoulderMask/SidewalkOverlapMask → Perimeter → Mesh → Committer`.
- **`RebuildPairwiseZoneBuilder`** = grade‑separation / overlap **zones**
  (pairwise road‑vs‑road), i.e. RoadBLD's analogue of our `PartitionLayers` — a
  *geometry* zone, **not** a routing graph.
- No `Lane*Connection` / `Successor` / `Predecessor` builder exists → **no turn
  connectivity graph is authored.**

**The "graph" integration is via PCG, not ZoneGraph.** Reflection shows PCG
nodes + a preset hook:
- `UPCGRoadBLDGetRoadEdgesSettings` — exposes road **edge curves** to a PCG graph.
- `UPCGRoadBLDGetSidewalkPartitionsSettings` — exposes **sidewalk partitions**
  (with `bWalkable`) to PCG.
- `UDynamicRoadDrawPreset::RoadPCGGraph` (`UPCGGraph*`) — *"Optional PCG graph
  executed on each generated `ARoadGeo`."*

→ **Answer for our replication:** to get lane routing / a ZoneGraph we build it
**ourselves** from the lane geometry we already have (per‑side `ELaneType` lanes
+ `EdgeCurve` offset polylines + welded endpoint joints). RoadBLD gives us the
*inputs* (edges, sides, partitions, walkable flags) but not a routing graph — so
RoadNet's ZoneGraph is net‑new work: connect lane ends across each welded joint
(match by side + adjacency), which we can emit as engine `ZoneShape`s or our own
graph for PCG.

---

## 5. (next feature set — pending: Markings / Details / Tools)
