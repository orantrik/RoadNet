# RoadBLD Replication Plan — Building an Independent Road-Network Engine

**Author:** reverse-engineering notes + implementation plan
**Target engine:** Unreal Engine 5.8
**Goal:** Replicate RoadBLD's road/junction generation *logic* inside our own plugins
(`OSMRoadCore` + a new `RoadNet` runtime), **fully independent** of WorldBLD / RoadBLD /
CityBLD. The engine must support **two authoring modes that feed the same rebuild
pipeline**: (1) **OSM import** — ways become road definitions automatically, and
(2) **hand-drawing** — an interactive editor tool to draw, snap, and edit roads
exactly like RoadBLD. Both modes produce the same `FRoadDef` data; the *rebuild
pipeline* and *junction model* are shared and are the parts worth copying. See §9 for
the dual-input authoring design.

> **Provenance / honesty note.** RoadBLD ships **no source** — only precompiled DLLs.
> Everything below was reconstructed from (a) the UHT-generated reflection headers in
> `MyProject5\Plugins\RoadBLD\Intermediate\...\UHT\`, (b) the doc-comments compiled
> into `UnrealEditor-RoadBLDRuntime.dll` and `UnrealEditor-RoadBLDEditorToolkit.dll`
> (extracted as strings), and (c) mangled C++ symbol signatures. Doc-comment quotes
> are verbatim. Struct field *layouts* that aren't UHT-exported are inferred from
> symbol names and are marked **(inferred)**. Nothing in RoadBLD was modified.

---

## 0. Why replicate RoadBLD at all

Our current OSM road path (Pipeline 2 → RoadBuilder) fights three recurring problems:
map-spanning junction blobs, sidewalks crossing through intersections, and terrain
fighting the road at crossings. RoadBLD solves the same class of problems with a
fundamentally cleaner architecture that we can adopt:

1. **A staged, compute-only rebuild pipeline** feeding a single **commit** step — no
   mid-pipeline world mutation, so it is deterministic and re-runnable.
2. **Two decoupled connection systems** — explicit *endpoint links* (topology we author)
   vs discovered *edge crossings* (geometry) — that never fight each other.
3. **Junctions as a Clipper2 boolean union** of road-outline polygons + corner-fillet
   wedges, not a bespoke junction mesh. This is the single biggest quality lever.
4. **Incremental scoping + stable GUIDs** so edits are cheap and preserved.

The rest of this document is the blueprint to build that ourselves.

---

## 1. RoadBLD architecture (as reconstructed)

### 1.1 Three-layer separation

| Layer | RoadBLD type | Responsibility | Our equivalent |
|-------|--------------|----------------|----------------|
| **Authoring / persistence** | `ADynamicRoad` | Control points, splines, lanes, endpoint links | OSM way → `FRoadDef` (data, not an actor) |
| **Network / orchestration** | `ADynamicRoadNetwork` | Road list, rebuild store, corner-edit overrides, intersection patch overrides, octree | `URoadNetwork` (subsystem or actor) |
| **Generated geometry** | `ARoadGeo` (disposable) | Triangulated road/intersection surface, PCG host, landscape patch | `ARoadGeoChunk` (disposable) |

Key principle: **roads persist, geometry is disposable**. `ARoadGeo` actors are
destroyed and recreated every rebuild; the source-of-truth is the road definitions +
the network store.

### 1.2 Curve hierarchy

```
UCurveObject                 // base: TArray<FPolylinePoint> Points, OffsetPoints, CurvatureRate, TotalLength
  ├─ UReferenceLine          // the design centerline that drives everything
  ├─ UEdgeCurve              // a lane boundary / road outer edge; has FLaneMarkingSegment[] + Side
  └─ UDynamicRoadLane        // width profile between two UEdgeCurve; FLaneWidthSegment[] + FLaneSection[]
```

Three distinct "centerlines" exist and must not be confused:
- **`ReferenceLine`** — the authored/design centerline (input to generation).
- **`GeometricCenterline`** — rebuild-derived average of the two outer edges (physical center; used for snapping/queries).
- **`LaneMidpoint`** — midpoint of the lane structure.

Clothoid math flow: `ControlPoints → RoadControlSplineComponent (+ ClothoidSplineComponent) → CalculateRefLine() → UReferenceLine polyline` with per-section straight/arc/spiral decomposition (`CalculateCurveSections`, `CurvatureRate`).

### 1.3 Connection topology — the crucial part

RoadBLD has **two independent connection mechanisms** plus a legacy one:

**(A) Explicit endpoint links (`FRoadEndpointLink`)** — authored topology, persisted on
the **first/last control point** of a road (in `FRoadControlSplinePointData.EndpointLinks`):

| Field | Type | Meaning (verbatim doc) |
|-------|------|------------------------|
| `LinkedRoad` | `TSoftObjectPtr<ADynamicRoad>` | target road |
| `bLinkedToFirstPoint` | `bool` | "Which end of the linked road: true = first point (index 0), false = last point." |
| `RightAxisOffset` | `double` | "Signed lateral offset from the linked road's ControlSpline endpoint, measured along its right axis (positive = right, negative = left)." |

An endpoint can hold **multiple** links → supports T/Y-junctions, splits, merges. Doc:
*"Persistent links to other road endpoints at this control point. Multiple links are
supported (e.g. a road splitting into several smaller roads)."*

**(B) Discovered edge crossings (`FRoadNetworkCorner`)** — geometry found at rebuild
time where two road *edges* actually cross (mid-road intersections). Not stored on the
road; lives in the network store.

**(C) Legacy simple snap (`FRoadSnap`: `SnappedRoad` + `Endpoint` index)** — kept for
back-compat only.

> **There is no `ConnectedParents`/`ConnectedChildren` graph** in RoadBLD 1.5.6 (verified
> absent from all symbols). Our current RoadBuilder plugin uses those; RoadBLD does not.

The rebuild deliberately **skips endpoint-snap intersections** when detecting edge
crossings, so (A) and (B) never produce duplicate junctions. This decoupling is exactly
what our OSM pipeline lacks — we infer *everything* from shared node IDs.

---

## 2. The rebuild pipeline (the heart of it)

RoadBLD's rebuild is a chain of **compute-only** `URebuild*Builder` stages that populate a
single `FRebuildContext`, followed by **one** `URebuildCommitter` that does all world
mutation. Entry point: `ADynamicRoadNetwork::RebuildRoadNetworkIncremental(...)`.

### 2.1 Stage order and I/O

| # | Stage | Reads | Writes | Verbatim intent (abbrev.) |
|---|-------|-------|--------|---------------------------|
| 0 | `DeterminePendingRoads` | `ModifiedRoads`, topology | `PendingRoads`, `RoadsToTestAgainst` | seed + expand scope |
| 1 | `EndpointJointBuilder` (pass 1) | endpoint links | `EndpointJointModel`, expanded scope | "Build canonical endpoint-joint groups… Classify groups into **seam/split/intersection** outcomes… Expand pending/test scopes with endpoint-linked roads" |
| 2 | `CurveBuilder` | `ModifiedRoads`, `PendingRoads` | curves, `RoadSnapshots`, octree boxes | "Recalculate curves only for ModifiedRoads… Repair missing/invalid polylines… Snapshot… Build octree query boxes" |
| 3 | `CornerBuilder` | `PendingRoads` × `RoadsToTestAgainst`, `CornerEditData` | `IntersectionModel` | "Detect edge intersections… preserve user-edited radii/offsets (stable GUID reuse)… **Skip endpoint-snap intersections**… walkways use smaller offsets/radii" |
| 4 | `EndpointJointBuilder::RefineFromCornerSignals` | `IntersectionModel` | refined `EndpointJointModel` | reconcile authored joints with discovered corners |
| 5 | `OverlapMaskBuilder` | all outer edges + geometric centerlines | `OverlapMasks` (+ `PairwiseOverlapCache`) | "per-road overlap masks… reference-line distance intervals where a road overlaps another… before SurfaceBoolean/Mesh" |
| 6 | `PairwiseZoneBuilder` | `PairwiseOverlapCache`, `OverlapMasks` | `PairwiseZones` (mutates `OverlapMasks`) | "**height-layer classification of overlapping road pairs**… back-propagates zone classifications" |
| 7 | `ShoulderMaskBuilder` | `PendingRoads`, intersection masks | `ShoulderMasks` | "Detect shoulder overlap regions inside intersection masks (marking suppression)" |
| 8 | `SidewalkOverlapMaskBuilder` | sidewalk inner edges | `SidewalkOverlapMaskModel` | "overlap masks on sidewalk inner edges for flat sidewalk trimming" |
| 9 | `SurfaceBooleanBuilder` | `IntersectionModel`, `OverlapMasks`, polylines | `MergedSurfacePlan` | see §2.2 — the Clipper2 union |
| 10 | `PerimeterBuilder` | `IntersectionModel` | `PerimeterLoops`, `CutModel` | "deterministic perimeter cuts (stable GUIDs)… quantized **half-edge graph**… extract closed perimeter loops" |
| 11 | `MeshBuilder` | `PerimeterLoops`, `OverlapMasks`, scoped roads | `MeshPlan` (`RoadMeshes`, `RoadGeoPlans`) | "Populate MeshPlan.RoadMeshes… RoadGeoPlans from PerimeterLoops… no spawning" (ParallelFor here) |
| 12 | `DetailsBuilder` | `PerimeterLoops`, scoped roads | `DetailsPlan` | "Plan road-module application per perimeter segment" |
| 13 | `Committer` | all plans + snapshots | **world actors**, network arrays, store, landscape | "mostly spawning and destroying actors"; frame-budgeted progressive commit |

### 2.2 How a junction becomes geometry (`SurfaceBooleanBuilder`)

Verbatim:
> 1. Extract each road's 2D outline polygon from its outer edge polylines
> 2. Extract **corner fillet polygons** from the IntersectionModel
> 3. Build **corner-authoritative merge groups** (roads connected by corners are merged)
> 4. Perform a **Clipper2 boolean union** per merge group
> 5. Re-project vertices to 3D using only same-group source polygons

So there is **no junction actor and no junction mesh algorithm** — a junction is simply
the boolean union of the arm rectangles plus the fillet wedges, producing one continuous
surface. Micro-gaps between polygons are bridged by an inflate epsilon
(`BooleanSurfaceInflateEpsilon`) before the union. `PerimeterBuilder` then walks a
quantized half-edge graph to extract the closed interior loops that become the
intersection `ARoadGeo` actors.

### 2.3 Incremental scoping model

Three road sets in `FRebuildContext`:
- **`ModifiedRoads`** — what the user/importer changed. Only these get curve recompute.
- **`PendingRoads`** — the work scope that receives outputs (masks, meshes, snapshots).
- **`RoadsToTestAgainst`** — broadphase partners for edge-crossing tests (superset of Pending).

Scope expands two ways: (1) **endpoint-linked roads** (a modified road pulls in everything
it's linked to), and (2) **octree broadphase** (`GetPotentialOverlaps` on outer-edge boxes).
Curve building can be split (`BuildCurvesOnly` → expand scope → `SnapshotRoadCurves` →
`BuildOctrees`) so the octree sees the final scope.

### 2.4 Stable-GUID edit preservation

Every artifact that a user can edit carries a GUID so incremental rebuilds don't lose
edits and don't rely on array indices:
- `FPerimeterCut.CutID`, `FRoadNetworkCorner.CornerID`, `ARoadGeo` rebuild-loop signature.
- **`FRoadCornerEditData`** — a lightweight persisted record (edge refs + distances +
  offsets + radius) that preserves corner edits *while keeping the live corner array empty*.
- The committer matches planned-vs-existing loop signatures (`RebuildCommitter_GuidSignatures`)
  to **reuse unchanged loops** unless `bForceRebuildAffectedRoads`.

### 2.5 Concurrency & robustness

- `bEnableAsyncRebuild`: all compute stages can run on a background thread; only the
  committer touches the game thread. `AbortAndWaitForPendingRebuild` cancels safely.
- `ParallelFor` in the mesh planning stage.
- Polyline repair for post-load corruption (`CalculateRefLine`/`CalculateLaneShapes`),
  `ValidateRoadNetwork`, `FRebuildIssue` aggregation, segment-projection distance query
  that's "stable near sharp corners and closed-loop seams".

### 2.6 Key numeric tunables (from `URoadBLDRuntimeSettings` / config)

| Setting | Default | Purpose |
|---------|---------|---------|
| `DefaultCornerOffset` | 500 cm | how far back each arm is trimmed to open a corner |
| `DefaultCornerRadius` | 500 cm | fillet radius |
| `IntersectionHeightThreshold` | 450 cm | max vertical gap for two edges to count as crossing (grade separation) |
| `IntersectionHeightMatchBlendDistanceMultiplier` | 3.0 | height-match blends over `narrowRoadWidth × this` along the narrower road |
| `IntersectionPatchMarginCm` | 75 | landscape patch margin around intersection |
| `IntersectionPatchTexelCm` / `FalloffCm` | 100 / 10 | patch bake quality / blend |
| `PolylineDensity` | 200 cm | polyline sampling |
| `LowQualityPolylineDensityMultiplier` | ~4× | coarse polylines during interactive editing |
| `bMatchIntersectionReferenceLineHeights` | true | force arms to a shared elevation at crossings |
| `bUseClipperForSidewalkOffset` | true | Clipper2 offset to avoid sidewalk self-intersections |
| `BooleanSurfaceInflateEpsilon` | small | bridge micro-gaps before union |
| `CommitFrameBudgetMs` | ~16 | progressive commit budget |

---

## 3. Mapping RoadBLD → our OSM/PCG context

We support **both** OSM import **and** hand-drawing (§9). The two authoring layers differ,
but they converge on the same `FRoadDef` data and the pipeline below is identical for both:

| RoadBLD concept | Our source |
|-----------------|------------|
| `ADynamicRoad` control points | OSM way node polyline (`FOSMRoadWay.PointsCm`) |
| `FRoadEndpointLink` (authored) | **derived from shared OSM node IDs** — a node shared by 2 way-ends = seam; shared by ≥3 arms = intersection |
| endpoint-joint seam/split/intersection classification | our continuity-merge + junction-census pre-passes (already prototyped in `OSMRoadBuilderBridge`) |
| `CornerBuilder` edge crossings | needed for crossings **not** on a shared node (rare in OSM, but real for bridges/overlaps) |
| `PairwiseZoneBuilder` height layers | OSM `layer`/`bridge`/`tunnel` tags → which road is on top |
| `bMatchIntersectionReferenceLineHeights` | our junction vertex-snap + width-scaled plate/landing (already implemented in `OSMOverpassRoadImport.cpp`) |
| `SurfaceBooleanBuilder` | **the big new piece to build** (Clipper2 union of arms + fillets) |
| `ARoadGeo` disposable geometry | our per-chunk generated mesh actors |
| editor draw/snap tools (`RoadController` etc.) | **needed for hand-draw mode** — port the controller/FSM layer (§9); OSM mode bypasses it |

The upshot: because OSM already gives us node-level topology, the OSM path gets the
*endpoint-joint* graph almost for free (shared node IDs). The hand-draw path instead
**authors** that topology interactively (endpoint snap → `FEndpointLink`). Both then hand
off to the same pipeline, so the core engineering effort stays on the **surface boolean
union** and **height/mask reconciliation** that make junctions clean — shared by both modes.

---

## 4. Implementation roadmap

A new runtime module `RoadNet` (independent of WorldBLD family), driven by `OSMRoadCore`.

### Phase 0 — Data model & scaffolding
- `FRoadDef` (POD): reference polyline (cm, world), per-point radius/elevation, lane
  spec (count fwd/back, widths), road class, `layer`/bridge/tunnel flags, `NodeIds[]`.
- `FEndpointLink { int32 OtherRoad; bool bOtherFirst; double RightAxisOffset; }`.
- `URoadNetwork` container: `TArray<FRoadDef>`, the derived stores below, and a
  `Rebuild(TArrayView<int32> ModifiedRoads)` entry point.
- Curve utilities: polyline resample at `PolylineDensity`, arc-length param, offset
  polyline (Clipper2), segment-projection distance/offset query (the "stable near seams" one).
- **Milestone:** load OSM ways → `FRoadDef[]`, visualize reference lines.

### Phase 1 — Topology builder (endpoint joints)
- Build `EndsAtNode` map from OSM node IDs. Classify each node:
  - 2 way-ends, compatible class/name → **seam** (continuity merge).
  - 1 end into another way's interior, or ≥3 arms → **intersection**.
  - one end fanning to several → **split** (lane-disparity case).
- Emit `FEndpointLink[]` per road end + a per-node `FJunction { arms[], classKing, Z }`.
- Reuse the existing continuity-merge + junction-census logic; formalize the
  seam/split/intersection enum from RoadBLD.
- **Milestone:** junction census with correct arm counts; seams merged into through-roads.

### Phase 2 — Curve + edge generation
- For each road: reference line → left/right outer `EdgeCurve` at half-width
  (Clipper2 offset, round joins), lane edges, sidewalk inner/outer edges.
- Repair/guard degenerate inputs (min segment length, finite checks, max segment count)
  — we already have these safeguards in `RoadCurve.cpp`; port them.
- Build an **octree** of outer-edge segment boxes for broadphase.
- **Milestone:** every road has clean offset edges; octree queries work.

### Phase 3 — Corner detection (edge crossings)
- For scoped road pairs (octree broadphase), find edge intersections
  (`GetCurveIntersections` equivalent). **Skip** pairs already handled as endpoint joints.
- Apply `IntersectionHeightThreshold` (reuse our `MaxJunctionZGap`, ~350–450 cm).
- Produce `FRoadCorner { StartEdge, EndEdge, StartDist, EndDist, StartOffset, EndOffset,
  IntersectionPoint, CornerRadius, DirA, DirB, GUID }`. Offsets default from
  `DefaultCornerOffset`/`Radius`, scaled down for walkways.
- Preserve edits by GUID (`FRoadCornerEditData` analog) — optional for OSM (no manual edits yet).
- **Milestone:** corners generated at every junction with correct trim distances.

### Phase 4 — Overlap + height-layer masks
- `OverlapMaskBuilder`: per-road reference-line intervals where it overlaps any other road.
- `PairwiseZoneBuilder`: for each overlapping pair, classify **which is on top** from
  `layer`/bridge/tunnel tags (fallback: higher class on top). Back-propagate to masks so
  the lower road's surface/markings are suppressed under the overpass.
- `ShoulderMaskBuilder` + `SidewalkOverlapMaskBuilder`: suppress markings/trim sidewalks
  inside intersection masks — **this is what stops sidewalks crossing through junctions.**
- **Milestone:** overpasses render correctly; no markings/sidewalks inside junction area.

### Phase 5 — Surface boolean union (the key quality piece)
- Extract each road's 2D outline polygon (between its two outer edges over its non-masked span).
- Extract **corner fillet wedge polygons** from `FRoadCorner` (arc between the two trimmed
  arm edges at `CornerRadius`).
- Build **corner-authoritative merge groups** (union-find: roads sharing a corner merge).
- **Clipper2 boolean union** per merge group (inflate by epsilon first to bridge gaps).
- Re-project the 2D union boundary back to 3D using only same-group source polygons
  (barycentric/nearest-segment height sampling).
- **Milestone:** junctions are a single seamless surface — no blobs, no gaps, no z-fight.

### Phase 6 — Perimeter loops + mesh plan
- `PerimeterBuilder`: quantize corner/cut/edge points into a half-edge graph; extract
  closed loops = intersection interiors. Assign stable loop GUIDs.
- `MeshBuilder`: plan per-road strip meshes (respecting masks) + per-loop intersection
  meshes. Triangulate loops (CDT/Delaunay — we already have a CDT path in RoadBuilder;
  port the input-sanitization guards).
- **Milestone:** road strips + intersection polygons meshed as a plan (no spawning yet).

### Phase 7 — Commit
- Single commit step: destroy stale chunks, spawn new `ARoadGeoChunk` actors from the plan,
  write derived arrays back to `URoadNetwork`, drive the landscape deform/patch
  (reuse our `SculptRoadCorridors` + junction-disc stamping).
- Frame-budget the commit for large imports (progressive, ~16 ms/tick).
- **Milestone:** full OSM import → clean meshed network with junctions + conformed terrain.

### Phase 8 — Details & polish
- Road markings (centerline by oneway, lane borders), crossings at intersection masks
  (`FCrossingPlacementService` analog: sample edge spans inside masks), props/modules,
  sidewalk partitions. Height matching refinement.

### Phase 9 — Incremental + async (optional, for editor use)
- Scope expansion (endpoint links + octree), GUID reuse of unchanged loops, background-thread
  compute + game-thread progressive commit. Only worth it if we later allow interactive edits.

---

## 5. Data structures to define (concrete)

```
// Persistent
FRoadDef        { TArray<FVector> Ref; TArray<double> Radius, Elev; FLaneSpec Lanes;
                  uint8 Class; int32 Layer; bool bBridge,bTunnel; TArray<int64> NodeIds;
                  FGuid Id; TArray<FEndpointLink> StartLinks, EndLinks; }
FEndpointLink   { int32 OtherRoad; bool bOtherFirst; double RightAxisOffset; }

// Rebuild context (compute-only bus)
FRebuildContext { TArray<int32> Modified, Pending, TestAgainst;
                  TMap<int32,FRoadCurves> Curves; FEndpointJointModel Joints;
                  FIntersectionModel Corners; TArray<FOverlapMask> OverlapMasks;
                  TArray<FPairwiseZone> Zones; TArray<FShoulderMask> Shoulders;
                  FMergedSurfacePlan Surface; TArray<FPerimeterLoop> Loops;
                  FMeshPlan Mesh; FDetailsPlan Details; TArray<FRebuildIssue> Issues; }

FRoadCorner     { FGuid Id; int32 StartEdge,EndEdge; double StartDist,EndDist,
                  StartOffset,EndOffset,CornerRadius; FVector Point; int DirA,DirB; bool bStale; }
FEndpointJoint  { int64 Node; TArray<FArm> Arms; enum {Seam,Split,Intersection} Kind;
                  uint8 ClassKing; double Z; }
FOverlapMask    { int32 Road; double StartDist,EndDist; int32 OverTop; } // OverTop from PairwiseZone
FPerimeterLoop  { FGuid Id; TArray<FPerimeterPoint> Pts; } // Pts traverse Cut/Corner/Edge
```

---

## 6. Risks, gaps, and decisions

- **Clipper2 3D re-projection** is the trickiest step: the union is 2D, heights come from
  source polygons. Nearest-segment sampling within the merge group is the RoadBLD approach;
  budget time for seam artifacts on steep terrain.
- **CDT robustness**: degenerate/near-duplicate input crashes CDT. Port the input
  sanitization and `__try/__except` guards already present in the RoadBuilder plugin.
- **Height matching vs terrain**: we already snap junction Z + sculpt a plate/disc; keep
  that as the "reference-line height match" stage so the boolean surface is planar per junction.
- **The editor-controller layer (draw/snap/chop/crossing tools) is now in scope** for
  hand-draw mode (§9). It is a large, largely independent effort that sits *on top of* the
  same pipeline; OSM mode does not use it. Sequence it after the core pipeline (Phases 1–7)
  is proven headless with OSM data, so the interactive layer builds on a stable base.
- **Unknowns not in the dumps:** exact `FRebuildContext` field layout, exact builder call
  order in `DynamicRoadNetwork.cpp`, and some numeric defaults (`SnapDistanceThreshold`,
  precise commit phase names). These are inferred; validate empirically.
- **Licensing:** this is a clean-room reimplementation from observed *behavior/interfaces*,
  not copied source (none exists to copy). Keep it that way — implement from this plan, not
  by decompiling bodies.

---

## 7. Priority recommendation

If we build only one thing from this, build **Phase 5 (surface boolean union)** on top of
our existing topology pre-passes and junction height-snap. That single change replaces the
fragile per-road-mesh + separate-junction-mesh model (the blob source) with RoadBLD's
seamless boolean surface, and is the highest-leverage step toward clean procedural cities.

Suggested order for real impact: **Phase 1 (topology) → 2 (edges) → 4 (masks) → 5 (boolean
surface) → 6/7 (perimeter+commit)**, deferring corner-edge detection (Phase 3), details
(Phase 8), and incremental/async (Phase 9).

---

## 8. Live console findings (interactive draw session — MyProject5, 2026-07-08)

> **Provenance.** These come from *watching the editor log live* across a full feature tour in
> `MyProject5` (the untouched RoadBLD install): drawing roads, T/4-way junctions, the edit-road
> tool, the junction tool, a freeway ramp, the lane tool, parking bays, markings, and details.
> With the shipped default `bDebugMessages=false`, RoadBLD still emits a full `[RebuildTiming]`
> telemetry trace; with `bDebugMessages=true` it additionally logs `LogDynamicRoad` topology
> diagnostics, `RoadController`/`RoadLaneController`/`ULaneMarkingController` decisions, and
> degenerate-state warnings. Nothing was modified. This section **confirms and extends** the
> static reconstruction in §1–§2 with observed runtime behavior.

### 8.1 The pipeline, as actually logged

Each edit (`bRebuildRoadsAfterEveryEdit=true`) runs compute off-thread, then a
**frame-budgeted (16 ms/frame) commit** on the game thread. Observed order:

**Async (background thread) —** matches the §2.1 builders:
```
CurveBuilder → CornerBuilder → OverlapMaskBuilder → ShoulderMaskBuilder
  → SidewalkOverlapMaskBuilder → MeshBuilder → DetailsBuilder
  → "Total async (background thread): N ms"
```
`PairwiseZoneBuilder`, `PerimeterBuilder`, and `SurfaceBooleanBuilder` are **not** separately
timed in the async list — the perimeter/boolean work surfaces in the commit's
`SpawnNewActors` stage instead (see below), i.e. geometry union happens at spawn time.

**Commit (game thread, sliced) —**
```
DestroyOldActors
SpawnNewActors:  Setup | PerimeterLoopGeo (+_Reuse, +_PCG) | RoadModules
                 | BooleanSegment (+_PCG) | BooleanIntersection (+_LandscapePatch, +_PCG)
SpawnTactileMeshes → CommitShoulderMasks → BuildMeshPerRoad → ApplyLandscapePaint
  → UpdateNetworkArrays → RebuildFreehandMarkings → StageStoreChunks → FlushStoreToBulkData
```

**Landscape phase —**
```
ConformVerticalProfile | Conform_GlobalActorScan | Conform_HeightmapRead | Conform_TraceFallback
MirrorSpline_UpdateRoadSpline | MirrorSpline_RefreshSplineLayer | MirrorSpline_InitializeManagedSpline
Patch_Apply_ResolveLandscape | Patch_Apply_BakeTexture | Patch_Apply_ConfigureAndUpdate
counters: ConformReason_Default / ConformReason_ModifiedRoad / SkippedConform / MirrorSplineRefresh
```

### 8.2 The single most important runtime rule: crossing ≠ junction

Confirmed directly by drawing both cases:

- **Endpoint snap → junction.** Ending a road *on an existing road's endpoint/node* logs
  `CalculateSnapControlPoint: Created snap control point at (x,y,z) with radius 5000.00`
  (a **50 m** capture radius) and the commit produces `BooleanIntersection` actor(s) +
  `[IntersectionPatchDiag]` landscape patch(es). A completed 4-way network showed
  `BooleanIntersection: 4 actors`, `BooleanSegment: 14 actors`.
- **Overshoot / mid-road crossing → NO junction.** Dragging a road across the *flank* of
  another logs `Mid-road snap found at distance N cm` every frame **but** the resolver
  rejects it: `Warning: CalculateSnapControlPoint: No valid snap target`. That rebuild
  produced `BooleanSegment` only, **zero `BooleanIntersection`, no patch** — the centerlines
  simply overlap as independent segments.

**Implication for our OSM pipeline (updates Phase 1):** RoadBLD forms junctions from
**endpoint-to-node** topology, not from geometric crossing. Where OSM has a side street
dead-ending into the *interior* of a through-road, we must **split the through-road at the
contact point** so the contact becomes a shared node/endpoint — otherwise no intersection is
generated. OSM's shared-node IDs give us this for free at real nodes; only mid-edge contacts
need an explicit split.

### 8.3 Incremental reuse is real and dominant (confirms §2.3–§2.4)

- Live scope readout: `Total pre-async (game thread): N ms  [29 roads, 1 pending]` for a
  simple point-add vs `[27 roads, 15 pending]` for a topology change — the `Modified →
  Pending → TestAgainst` expansion in action.
- The GUID reuse cache is the perf backbone: one commit showed
  `PerimeterLoopGeo: 20 actors (rebuilt)` alongside `PerimeterLoopGeo_Reuse: 64 actors
  (0.1 ms total)`. **Only touched roads recompute outline geometry; the rest are served from
  cache.** For us this means the reuse cache (deferred to Phase 9) is actually central to
  batch-import perf, not just interactive editing.

### 8.4 Parallel PCG output path (new — directly relevant to us)

Every geometry class emits a PCG variant: `PerimeterLoopGeo_PCG`, `BooleanSegment_PCG`,
`BooleanIntersection_PCG`. RoadBLD produces **PCG data alongside the static-mesh actors** for
segments, perimeters, and intersections. Since our city system is PCG-based, this is the
natural integration seam — roads/junctions can feed a PCG graph, not only baked meshes.

### 8.5 Terrain: patch mechanism confirmed + a trace fallback

- `[IntersectionPatchDiag]` confirms §2.2/§2.6: per intersection it **fits a plane** through
  the intersection ring vertices, offsets down by **`BiasDown=25 cm`** (road sits proud),
  and bakes an `RGBA16F` `LandscapeTexturePatch` named
  `RoadBLD_IntersectionLandscapePatchHeight` at ~`IntersectionPatchTexelCm=100`.
  Example: `RingVerts=278 ... Res=25x24 ... Coverage=(2328,2296)`.
- `Conform_TraceFallback` (N calls): when a road lacks mirror-spline height data, RoadBLD
  **line-traces against the landscape** for Z. `DrawStart` also fires a vertical multi-trace
  (`Z=+1000 → −5000`) against `LandscapeStreamingProxy` to drape the first control point.
- `MirrorSpline_UpdateRoadSpline` is the dominant landscape cost (often >90 %) and scales
  with how many roads a single edit touches — a real cost knob for large networks.

### 8.6 Corner detection uses an edge-segment octree (confirms §2.1 CornerBuilder)

`[CornerBuilder] Octree boxes are created per polyline segment (NumSegments = NumPoints - 1).
3000 is bounds expansion, not spacing. TotalBoxes=524`. Each road exposes two boundary edges
(`EdgeCurve_2` = left, `EdgeCurve_4` = right), densified to ~200 cm (`PolylineDensity`), and
all edge segments populate a spatial octree for broadphase crossing tests. Note this feeds
**corner filleting**, which is separate from junction *creation* (§8.2).

### 8.7 Road creation & authoring details

- New road: `CreateReferenceLine: Successfully created new UReferenceLine object` followed by
  a default lane profile — observed **4 lanes (2 right + 2 left)** added at creation, driven
  by the selected preset (e.g. `RP_Sample_2Lane2Way_NoSidewalks_C`). Road profiles are
  **preset assets**, launched via `RoadBLDViewportToolbar: Launching RoadController ... preset '...'`.
- A **self-intersection guard** blocks self-crossing control polygons:
  `RoadController: Cannot draw - Self-intersection detected`.

### 8.8 Degenerate-state warnings worth designing around

Recurring during junction rebuilds:
```
Warning: CalculateLaneShapes: ReferenceLine is null or has no CurveSections
Warning: DynamicRoad::SyncLandscapeMirrorSplineFromCenterline - No polyline points to update!
Warning: CalculateGeometricCenterPolyline: Edge polylines have insufficient points - Left:1, Right:1
Warning: Build: Failed to create static mesh   ← fires on nearly every intersection commit
```
Junction splitting occasionally yields **collapsed / zero-length arm sub-roads** with empty
centerlines (no lanes, no terrain deform), and the intersection boolean sometimes emits a
degenerate mesh (`nearly zero tangents/bi-normals`). **Our replication needs a
minimum-arm-length guard and a degenerate-arm cull** at the split/topology stage (Phase 1/6),
or we inherit these empty roads and failed meshes.

### 8.9 Editing tools observed — a per-tool controller layer (updates §9)

The interactive layer is **not one monolith** — it's a set of focused controllers, each a
small state machine, all funnelling into the same rebuild. Observed live:

- **`RoadController`** — draw new roads. Emits `CreateReferenceLine`, `CalculateSnapControlPoint`
  (50 m endpoint capture), self-intersection guard (§8.7). Launched with a preset asset.
- **`RoadNetworkController`** — select/move/edit existing roads; scopes the incremental rebuild
  (`Modified → Pending`) to the touched road + neighbors.
- **`RoadLaneController`** — lane editing via a **3-state FSM**: `SelectLane → SelectEdge →
  EditOffsets`. Parking bays are made here (§8.11).
- **`ULaneMarkingController`** — structured lane-line markings (§8.10); has its own
  `RebuildTargetRoadOnly` fast path and `ToolExit`/ESC-deselect lifecycle.
- **Junction / freeway-ramp / details tools** — ramp merges via `BooleanSegment` +
  `BooleanIntersection` and activates the `ShoulderMaskBuilder` for gore-area suppression;
  details place prefab actors (§8.11).

**Implication for §9:** the hand-draw layer should be modeled as **independent tool controllers
(each an FSM) over a shared `URoadNetwork`**, not a single mega-tool. This matches RoadBLD and
keeps each interaction (draw, edit, lane, marking, detail) isolated and testable.

### 8.10 Markings are two distinct systems

1. **Structured lane markings** — `FLaneMarkingSegment` records carried on a road's
   `UEdgeCurve` (§1.2). Each segment observed as
   `{ Width, EdgeOffset, TextureVScale, SegmentBehavior (enum), Material }`, e.g.
   `Width: 204.70, EdgeOffset: 0.00, TextureVScale: 1.00, SegmentBehavior: 1,
   Material: M_Greybox_SolidWhiteLine`. Edited via `ULaneMarkingController::SetTargetSegmentParameters`.
   They are **baked into the road surface mesh** — a marking-only edit costs in
   `BuildMeshPerRoad` with `SpawnNewActors ≈ 0` (all perimeter/boolean geometry reused).
2. **Freehand markings** — standalone `URoadMarkingLine` actors (painted polylines
   independent of any lane edge). Observed `7 control points → densified to 1003-point strip
   mesh`, rebuilt in the dedicated **`RebuildFreehandMarkings`** commit stage (idle at 0 ms
   until a freehand line exists, then ~27 ms).

`FMeshDecalsVS/PS` shaders compiled during this work → overlay markings can render as **mesh
decals**, not only baked geometry.

**For our plan:** map OSM lane/edge/centerline lines to system (1) — parameterized segments on
edge curves, baked into the surface. Reserve system (2) (freehand strips / decals) for arbitrary
paint (stop lines, hatching, symbols, crossings) not tied to a lane edge.

### 8.11 Details = prefab Blueprint actors, spawned outside the rebuild

The **details** tool drops prefab `Island_*` Blueprint actors (e.g.
`Island_Sample_RaisedMedian_C`) by right-clicking a `RoadGeo` (context menu). Each detail actor
builds **two meshes**: a **constant-size curb/kerb template** (`StaticMesh_0`, identical
~0.39 MB every instance = reused geometry) plus a **footprint-conformed fill** (`StaticMesh_1`,
variable size = grows with the drawn island). Fill material is a **bundled Megascans tiling
surface** (`MSPresets/MSTextures/...segtrul_4K_Albedo/Normal_Tiling`, virtual-textured).
Crucially, **no `[RebuildTiming]` block fires for detail placement** — details are spawned
directly, decoupled from the async road rebuild, so they are cheap.

**For our plan:** details are the *road-furniture / traffic-island* layer — a **library of
prefab actors** (raised medians, splitter islands, roundabout centers, gore fills) that snap to
`RoadGeo` and emit a reused curb template + a conformed fill with a bundled PBR material. Maps to
OSM `highway=traffic_island`, roundabout centers, median strips. Keep them as instanced prefabs,
**not** boolean'd into the surface. (Note: the async pipeline also has a lightweight
`DetailsBuilder` stage, §2.1 #12 — that plans *road-module application per perimeter segment*
and is distinct from these placed prefab actors.)

### 8.12 Sidewalks — perimeter-loop bands + an overlap-mask builder (this is the clean-intersection mechanism)

Directly answering the recurring sidewalk problem from the old spline method: RoadBLD does **not**
run sidewalks as separate splines. Observed:

- **`GenerateSidewalkGeometryForPerimeterLoop`** — sidewalk geometry is generated **per road
  perimeter loop** as a band, via a **Clipper2 boolean union** of the sidewalk-band polygons.
- **`SidewalkOverlapMaskBuilder`** (async stage, §2.1 #8) — resolves where sidewalk bands
  **overlap** at junctions/islands so overlapping segments are trimmed/masked instead of crossing
  through the intersection or z-fighting. Its cost was **0 ms until details/junctions existed**,
  then scaled with geometry (`5.6 → 26.6 ms`) as more overlaps appeared.
- **Degenerate case to guard:** `Warning: GenerateSidewalkGeometryForPerimeterLoop: Clipper
  Union returned empty result` — when a detail/island consumes the whole sidewalk band the union
  is empty and a zero-area stub mesh is emitted. Our generator must **skip empty-union loops**.

**For our plan:** this is exactly the "sidewalks don't cross the junction" behavior we were
chasing. Model sidewalks as **perimeter-loop bands (Clipper2) + an overlap-mask trim stage**
(Phase 4's `SidewalkOverlapMaskBuilder`, feeding Phase 5/6), *not* as merged splines. This
supersedes the abandoned Yazan spline approach.

### 8.13 Final static read of the whole plugin folder (before project close)

> **Provenance.** A last pass over the entire `MyProject5\Plugins\RoadBLD` tree (308 files):
> the `.uplugin`, `Config\DefaultRoadBLD.ini`, and all 55 UHT reflection headers. This is
> static structure, not runtime — it *confirms* §1–§2 and adds the items below.

**Module & dependency shape (`RoadBLD.uplugin`, v1.5.6, UE 5.8.0).** Three modules:
`RoadBLDRuntime` (Runtime), `RoadBLDEditorToolkit` (Editor), and a **separate
`ClothoidsModule`** (Runtime, pure C++ — no UObjects, hence no UHT headers). It confirms the
clothoid math lives in its own dependency-free module (validates §10.2's split of curve math
into `RoadNetMath`). Plugin dependencies reveal the real toolset:
- **`WorldBLD`** — RoadBLD depends on its parent. *Our `RoadNet` must NOT* (independence guard, §10.0).
- **`LandscapePatch` + `Landmass`** — the intersection height patch (validates §10.14).
- **`PCG` + `PCGGeometryScriptInterop`** — the PCG output path (§8.4); confirmed by PCG nodes
  `PCGRoadBLDGetRoadEdges` and `PCGRoadBLDGetSidewalkPartitions`.
- **`GeometryScripting`** (dynamic-mesh booleans — `RoadBLDDynamicMeshUtils`), **`Text3D`**
  (road-sign text), **`ProceduralMeshComponent`** (interactive preview meshes).

**Config defaults (`DefaultRoadBLD.ini`) — confirms and *extends* §2.6.** Confirmed exactly:
`PolylineDensity=200`, `DefaultCornerOffset=500`, `DefaultCornerRadius=500`,
`IntersectionHeightThreshold=450`, `IntersectionHeightMatchBlendDistanceMultiplier=3.0`,
`IntersectionPatchTexelCm=100 / MarginCm=75 / FalloffCm=10`, `MarkingZOffset=2.5`,
`bMatchIntersectionReferenceLineHeights=true`. **New settings we hadn't captured:**

| Setting | Default | Meaning / plan impact |
|---------|---------|-----------------------|
| `bEnableRoadCrownGeneration` | True | **road crown / camber** — the surface is cross-sloped, not flat. Add a crown profile to §10.15 meshing (raise centerline vs edges). |
| `bEnableWorldDisplacement` / `WorldDisplacementMax` / `UpwardBias` | True / 10 / 3 cm | material **world-position displacement** for surface relief; small (≤10 cm) with a +3 cm bias. |
| `bEnableIntersectionTireWear` + `TireWearDecalMaterial` | True | **tire-wear decals** baked into intersections (a details-layer flourish). |
| `bEnableAbutmentGeneration` + `DefaultAbutmentMaterial` | False | **bridge abutments/supports** for elevated roads (off by default). |
| `bEnableRoadLandscapePaint` / `RoadPaintTexelCm` / `bAutoCreatePaintLayerInfo` | True / 50 / True | paints a **landscape material layer** under roads (50 cm/texel) — distinct from the height patch. |
| `GlobalLineTraceDistance` | 1e6 | drape/trace distance (confirms the `Conform_TraceFallback` traces, §8.5). |
| `RoadProcessorClass` | `BP_RoadNetworkProcessor_C` | the network processor is a **Blueprint** — logic is partly BP-driven, not all C++. |
| `DefaultMarkingWidth` | 50 cm | default lane-line width. |
| marking mats | Double-yellow / broken-white / solid-white / dotted-white greybox | confirms §8.10 material set. |
| `RoadCollisionType` | NoCollision | generated road geo ships collision-free by default. |

**Class inventory highlights (55 UHT headers).** All **12 rebuild builders** from §2.1 exist as
`URebuild*Builder` UObjects (Curve, Corner, EndpointJoint, OverlapMask, PairwiseZone,
ShoulderMask, SidewalkOverlapMask, SurfaceBoolean, Perimeter, Mesh, Details, Committer) —
static confirmation of the pipeline. Newly catalogued types:
- **`USidewalk : UDynamicRoadLane`** and **`USidewalkPartition : UDynamicRoadLane`** — a sidewalk
  **is a specialized lane**, chunked into partitions (feeds the `GetSidewalkPartitions` PCG node).
  Strongly reinforces §8.12/§10.16: model sidewalks as lane subclasses reusing the edge-curve
  machinery, not as a parallel system.
- **`ARoadIsland : AActor`** — spline-driven island/median with `RebuildIslandMesh` (BlueprintNativeEvent),
  `EnforceLinearSplinePoints`, `OnSplineModified`. This is the base of the `Island_*` prefabs (§8.11).
- **Road furniture:** `ARoadSign` (+ Text3D), `URoadStamp`, `URoadSupportData`, `URoadModuleObject`,
  `APropSpawner` / `UPropsGeneratorBase`, `USocketLightSettings` — a whole props/signage/lighting
  layer beyond markings & islands. Maps to our §8.11 details library (defer, but plan the hooks).
- **Core types:** `FRoadEndpointLink` (confirms §1.3), `FRoadSnap` (legacy), `FSpawnModuleDesc`,
  and enum `ERoadModulePosition {Left, Right, Center}` (road modules are placed per-side).
- **`URoadMeshGenerator`, `URoadBLDDynamicMeshUtils`** — meshing built on `GeometryScripting`
  dynamic-mesh ops (an alternative/adjunct to raw CDT for §10.15).

**Net additions to Pipeline 4 (§10):** add (1) a **crown/camber** cross-slope to the mesh
cross-section, (2) optional **world-displacement** material params, (3) sidewalks as
**`UDynamicRoadLane` subclasses** with partitioning, (4) a **landscape paint-layer** pass
alongside the height patch, and (5) deferred hooks for **signage/props/abutments/tire-wear**.
None change the core pipeline; they are additive layers.

### 8.14 Net effect on the plan

Nothing in §1–§7 is contradicted; the runtime evidence sharpens these points:
1. **Phase 1** must explicitly *split through-roads at interior contacts* (crossing alone is
   never a junction).
2. **Phase 9's reuse cache** is not optional polish — it is the reason per-edit and
   batch rebuilds stay affordable; plan for GUID-keyed geometry reuse early.
3. Add a **PCG emission path** (§8.4) as a first-class output, and **degenerate-arm culling**
   (§8.8) as a hard guard in the topology/mesh stages.
4. **Sidewalks (§8.12)** belong in the pipeline as perimeter-loop bands + overlap-mask trimming
   (Phase 4→5/6), with an empty-union guard — this is the clean-intersection fix.
5. **Markings (§8.10)** split into baked lane-segments (Phase 8) vs freehand/decal overlays;
   **details (§8.11)** are decoupled prefab actors placed post-rebuild.
6. The **hand-draw authoring layer is a set of per-tool FSM controllers** (§8.9) over the shared
   network — see §9.

---

## 9. Authoring modes: hand-drawn + OSM (dual input)

**Requirement:** the engine must accept roads from **both** an interactive hand-draw tool
**and** OSM import, interchangeably, in the same network. This is a first-class design goal,
not an afterthought.

### 9.1 The convergence point — one data model, one pipeline

Both modes produce the **same `FRoadDef`** (§5) and mutate the **same `URoadNetwork`**, then
call the identical `Rebuild(ModifiedRoads)` entry point. Nothing downstream of `FRoadDef` knows
or cares how a road was authored. This is the single most important rule: **the rebuild
pipeline (§2), junction model (§1.3), and mask/boolean stages (Phases 3–7) are mode-agnostic.**

```
Hand-draw tools ─┐
                 ├─▶  FRoadDef[] in URoadNetwork  ─▶  Rebuild(Modified)  ─▶ geometry + PCG
OSM import ──────┘         (shared source of truth)      (shared pipeline)
```

### 9.2 How the two modes differ (only the front end)

| Concern | OSM import mode | Hand-draw mode |
|---------|-----------------|----------------|
| Geometry source | way node polyline (`FOSMRoadWay.PointsCm`) | user-placed control points → clothoid spline (§1.2) |
| Topology / links | **derived** from shared node IDs (seam/split/intersection) | **authored** by endpoint snap → `FEndpointLink` (50 m capture, §8.2) |
| Lane profile | OSM `lanes`/`lanes:forward`/`lanes:backward` tags | selected **preset asset** (e.g. 2-lane-2-way) |
| Interior contacts | split through-road at shared node (Phase 1) | user snaps to endpoint, or explicit **chop/split tool** |
| Scope on change | whole import = one big `ModifiedRoads` batch | just the edited road(s) → incremental (§8.3) |
| Editor UI | none (headless) | per-tool FSM controllers (§8.9) |

### 9.3 Hand-draw layer to build (ports RoadBLD's controller design, §8.9)

Model as **independent tool controllers**, each a small FSM, over the shared `URoadNetwork`.
Minimum viable set, in priority order:

1. **DrawRoadTool** (`RoadController` analog) — click to place control points; live clothoid
   preview; **endpoint snap** within a capture radius (default 50 m → writes `FEndpointLink`);
   **self-intersection guard**; commit creates an `FRoadDef` from a chosen lane preset.
2. **Select/EditTool** (`RoadNetworkController` analog) — pick a road, drag control points /
   endpoints, re-snap; scopes an incremental rebuild to the touched road + linked neighbors.
3. **SplitTool** — split a road at a picked point (turns a mid-edge contact into a shared
   endpoint so a real junction forms — the hand-draw equivalent of Phase 1's OSM node split).
4. **LaneTool** (`RoadLaneController` analog) — `SelectLane → SelectEdge → EditOffsets` FSM;
   width/offset edits and **parking bays** (lateral offset points on an `UEdgeCurve`, §8.11).
5. **MarkingTool** (`ULaneMarkingController` analog) — edit baked lane-segment markings
   (§8.10 system 1) and paint freehand overlay lines (§8.10 system 2); uses a
   `RebuildTargetRoadOnly` fast path.
6. **DetailsTool** — drop prefab island/median actors on a `RoadGeo` (§8.11).

Each tool only edits `FRoadDef`/overrides and marks roads dirty; **it never triggers world
mutation itself** — the shared committer does, so hand-draw and OSM stay consistent.

### 9.4 Interop rules (both modes coexisting)

- **Hand-drawn roads can snap to OSM-imported roads and vice-versa** — endpoint snap writes an
  `FEndpointLink` regardless of how the target was authored (topology is unified in
  `URoadNetwork`, not per-source).
- **Provenance flag** on `FRoadDef` (`enum ESource { OSM, HandDrawn }`) so a re-import can
  refresh/replace OSM roads **without destroying hand-drawn edits**. Re-import replaces only
  `Source==OSM` roads; hand-drawn roads and their links survive.
- **Stable GUIDs** (§2.4) are shared across both — hand-drawn edits and OSM roads reuse the same
  GUID-keyed geometry cache, so mixed networks rebuild incrementally.

### 9.5 Sequencing

Build the **headless pipeline first** (Phases 0–7, driven by OSM) so junction quality is proven
without UI. Then layer the hand-draw controllers (§9.3) on top — they are additive and reuse the
entire pipeline. DrawRoadTool + Select/EditTool + SplitTool give a usable hand-draw MVP; Lane/
Marking/Details tools follow. This keeps the risky core (boolean surface, masks) decoupled from
the (large but mechanical) editor UI work.

---

## 10. Pipeline 4 — a from-scratch road plugin (math, logic & technical spec)

> **NOTE / DIRECTIVE.** Create a **new pipeline, `Pipeline 4`** (`osm.RoadPipeline=4`), that
> implements a road-generation plugin **from scratch** using logic *similar to* RoadBLD.
> **It MUST be fully independent of RoadBLD** — no link dependency, no `#include`, no module
> reference, no content reference to `RoadBLD` / `WorldBLD` / `CityBLD`. RoadBLD is used only as
> a *behavioral reference* (this document). Everything below is a clean-room specification:
> the equations, the build logic, and the technical wiring needed to build it ourselves.

### 10.0 Independence mandate & module layout

New runtime + editor modules, depending only on engine modules, our `OSMRoadCore`, and a
**vendored copy of Clipper2** (BSL-1.0, header-only friendly). **Zero** references to the
RoadBLD family.

```
Plugins/RoadNet/
  RoadNet.uplugin                       # Modules: RoadNet (Runtime), RoadNetEditor (Editor, optional)
  Source/RoadNet/
    Public/  RoadNetTypes.h             # FRoadDef, FEndpointLink, FLaneSpec, enums
             URoadNetwork.h             # UObject/Actor: road list + stores + Rebuild()
             RoadNetMath.h              # all math in §10.2–§10.10 (free functions, testable)
    Private/ Curve/     RefLineClothoid.cpp  OffsetPolyline.cpp  ProjectQuery.cpp
             Topology/  EndpointJoints.cpp   CornerDetect.cpp
             Surface/   OverlapMask.cpp  PairwiseZone.cpp  SurfaceBoolean.cpp  Reproject3D.cpp
             Perimeter/ HalfEdgeGraph.cpp    LoopExtract.cpp  Triangulate.cpp
             Mesh/      MeshBuilder.cpp      MarkingBuilder.cpp
             Commit/    Committer.cpp        LandscapeConform.cpp  PatchBake.cpp
             ThirdParty/Clipper2/...    # vendored, namespaced ridgorously
  Source/RoadNetEditor/                 # §9 hand-draw tools (only if editor build)
```

`Build.cs`: `PublicDependencyModuleNames = { Core, CoreUObject, Engine, GeometryCore,
GeometryFramework, Landscape, PCG }` (+ `UnrealEd, EditorSubsystem` for the editor module).
**Never** add `RoadBLDRuntime` etc. A CI/grep guard should fail the build if any RoadBLD symbol
appears in `RoadNet`.

Selection: extend the existing `osm.RoadPipeline` CVar to accept **`4`**, routing OSM ways into
`URoadNetwork::ImportFromOSM(...)` → `Rebuild()` instead of the RoadBuilder bridge.

### 10.1 Conventions & the road frame

- Units: **centimeters**, Unreal **left-handed, Z-up**. All math in world space unless noted.
- Along a reference line parameterized by arc length \(s\), define the **road frame** from the
  planar tangent:
\[
\mathbf{T}(s)=\frac{\mathbf{r}'(s)}{\lVert \mathbf{r}'(s)\rVert}, \qquad
\mathbf{R}(s)=\text{normalize}\big(\mathbf{T}(s)\times \hat{\mathbf{z}}\big), \qquad
\mathbf{U}(s)=\mathbf{R}(s)\times\mathbf{T}(s)
\]
  \(\mathbf{R}\) is the **right axis** (used by `FEndpointLink.RightAxisOffset` and all lateral
  offsets); \(\mathbf{U}\) is the surface up. A lateral offset \(o\) (signed, +right) maps a
  centerline point to \(\mathbf{P}_o(s)=\mathbf{r}(s)+o\,\mathbf{R}(s)\).

### 10.2 Reference line — clothoid (Euler-spiral) spline

Goal: a \(G^2\)-continuous centerline whose **curvature varies linearly with arc length** within
each segment (the clothoid property), so vehicles/markings transition smoothly. Between two
consecutive control points we fit a clothoid segment with curvature
\[
\kappa(s)=\kappa_0+c\,s,\qquad c=\frac{d\kappa}{ds}=\text{CurvatureRate (const per section)} .
\]
Heading integrates to a quadratic and position to the **Fresnel integrals**:
\[
\theta(s)=\theta_0+\kappa_0 s+\tfrac12 c\,s^2,
\]
\[
x(s)=x_0+\int_0^{s}\cos\theta(u)\,du,\qquad
y(s)=y_0+\int_0^{s}\sin\theta(u)\,du .
\]
For \(c\neq 0\), substitute \(u=\big(\kappa_0+c\,t\big)/\sqrt{\pi |c|}\) to reach the standard
Fresnel form
\[
C(z)=\int_0^{z}\cos\!\Big(\tfrac{\pi}{2}t^2\Big)dt,\quad
S(z)=\int_0^{z}\sin\!\Big(\tfrac{\pi}{2}t^2\Big)dt,
\]
evaluated by a rational/serial approximation (Boersma or the `A&S 7.3` series). **Practical
implementation:** rather than solve a global clothoid spline, decompose each section into
**straight | circular arc | spiral** pieces (`CalculateCurveSections`) and sample analytically:
- **Straight:** \(\mathbf{r}(s)=\mathbf{P}_0+s\,\mathbf{T}_0\).
- **Arc (radius \(\rho=1/\kappa\)):** \(\theta(s)=\theta_0+s/\rho\); center
  \(\mathbf{c}=\mathbf{P}_0+\rho\,\mathbf{R}_0\); \(\mathbf{r}(s)=\mathbf{c}+\rho(\cos,\sin)\).
- **Spiral:** Fresnel as above, scaled by \(\sqrt{\pi/|c|}\).

Elevation is a separate 1-D profile \(z(s)\): fit a **monotone cubic Hermite (PCHIP)** through
per-control-point heights so grades don't overshoot; sample alongside \((x,y)\).

If the clothoid fit is deferred, a correct interim is a **centripetal Catmull-Rom** through the
control points (tension \(\alpha=0.5\)) — \(C^1\), no cusps — upgraded to clothoids later. The
rest of the pipeline only consumes the **sampled polyline**, so this choice is swappable.

### 10.3 Arc-length resampling

Given the analytic curve, sample a polyline at fixed spacing `PolylineDensity` \(=\Delta\)
(200 cm; ×4 coarse during interactive drag). Cumulative length
\(L_i=\sum_{k<i}\lVert \mathbf{p}_{k+1}-\mathbf{p}_k\rVert\); emit points at
\(s_j=j\Delta\) by binary-searching \(L\) and linearly interpolating. Always include exact
segment endpoints and high-curvature knots (adaptive: also split where
\(\lVert\Delta\theta\rVert>\theta_{max}\), e.g. 5°).

### 10.4 Lateral offset & edge curves

Left/right **outer edges** at half-width \(w/2\): naïvely \(\mathbf{P}\pm\frac{w}{2}\mathbf{R}\),
but per-vertex offset self-intersects on concave bends. Use a **polyline offset with joins**:
- **Miter** at a vertex with incoming/outgoing right axes \(\mathbf{R}_a,\mathbf{R}_b\):
  miter direction \(\mathbf{m}=\text{normalize}(\mathbf{R}_a+\mathbf{R}_b)\), length
  \(\ell=\dfrac{w/2}{\cos(\phi/2)}\) where \(\phi\) is the turn angle; **clamp** \(\ell\) to a
  miter limit \(m_{lim}\,(w/2)\) and fall back to a **round join** (arc of radius \(w/2\))
  beyond it.
- For robustness at tight bends and variable width, run the offset through **Clipper2**
  `InflatePaths(w/2, JoinType::Round, EndType::Butt)` and take the boundary
  (`bUseClipperForSidewalkOffset=true` analog). This removes offset self-intersections
  automatically.

Lane edges and sidewalk inner/outer edges are additional offsets at cumulative lane widths
\(o_k=\sum_{i\le k} w_i - \tfrac{w}{2}\).

### 10.5 Stable point→polyline projection (needed everywhere)

For a query point \(\mathbf{q}\), the nearest point on segment
\([\mathbf{a},\mathbf{b}]\) is \(\mathbf{a}+t^\*(\mathbf{b}-\mathbf{a})\),
\[
t^\*=\operatorname{clamp}\!\Big(\frac{(\mathbf{q}-\mathbf{a})\cdot(\mathbf{b}-\mathbf{a})}
{\lVert \mathbf{b}-\mathbf{a}\rVert^2},\,0,\,1\Big).
\]
Signed lateral (right) offset uses the 2-D cross product:
\(o=\dfrac{(\mathbf{b}-\mathbf{a})\times(\mathbf{q}-\mathbf{a})}{\lVert\mathbf{b}-\mathbf{a}\rVert}\).
Return \((\text{dist}=|{\cdot}|, \text{alongDist}=L_{seg}+t^\*\lVert b-a\rVert, \text{offset}=o)\).
For stability **near seams/sharp corners**, evaluate all segments and keep the min-distance hit,
breaking ties by smallest `alongDist` (avoids flipping between two segments sharing a vertex).

### 10.6 Edge-crossing detection (corners) + broadphase

Two segments \([\mathbf{p}_1,\mathbf{p}_2]\), \([\mathbf{p}_3,\mathbf{p}_4]\) intersect where
\[
d=(\mathbf{p}_2-\mathbf{p}_1)\times(\mathbf{p}_4-\mathbf{p}_3),\quad
t=\frac{(\mathbf{p}_3-\mathbf{p}_1)\times(\mathbf{p}_4-\mathbf{p}_3)}{d},\quad
u=\frac{(\mathbf{p}_3-\mathbf{p}_1)\times(\mathbf{p}_2-\mathbf{p}_1)}{d},
\]
a crossing iff \(|d|>\varepsilon\) and \(t,u\in[0,1]\); point \(=\mathbf{p}_1+t(\mathbf{p}_2-\mathbf{p}_1)\).
**Broadphase:** insert each outer-edge segment's AABB (expanded by ~3000 cm) into a
`TOctree2`/uniform grid; only test pairs whose boxes overlap. **Skip** pairs already joined as
endpoint joints (§10.7) and pairs with \(|z_A-z_B|>\) `IntersectionHeightThreshold` (450 cm →
grade separation, §10.12).

### 10.7 Endpoint joints (topology) — OSM-derived or hand-authored

- **OSM:** build `Node → {road end}` multimap from shared node IDs. Classify each node:
  \(\deg=1\) terminal; \(\deg=2\) & compatible class/name → **seam** (merge into one through-road);
  \(\deg\ge 3\) → **intersection**; one end into another road's *interior* → **split** the
  through-road at the projected point (§10.5) so it becomes a shared endpoint.
- **Hand-draw:** endpoint snap within radius \(R_{snap}\) (5000 cm) writes an `FEndpointLink`
  \(\{OtherRoad, bOtherFirst, RightAxisOffset\}\), where `RightAxisOffset` is the §10.5 signed
  offset of this endpoint onto the target's endpoint frame.

Crucial rule (from §8.2): **a geometric crossing is not a junction** — only shared
endpoints/nodes are. Interior contacts must be *split* first.

### 10.8 Corner fillet geometry

At a convex corner where two trimmed arm edges meet with interior turn angle \(\phi\) (angle
between the two outgoing edge directions), a fillet of radius \(r\) (=`DefaultCornerRadius`,
500 cm) is tangent to both edges. **Trim/setback distance** from the corner apex along each edge:
\[
t=\frac{r}{\tan(\phi/2)}\;(=\,\text{how far back each arm is cut}),\qquad
\text{apex→arc-center distance }=\frac{r}{\sin(\phi/2)} .
\]
The arm is additionally pulled back by `DefaultCornerOffset` (500 cm) to *open* the corner
(`StartOffset/EndOffset`). The fillet arc (center \(\mathbf{c}\), from tangent point A to B) is
sampled into a small polygon wedge. Walkways/sidewalks use scaled-down \(r,\,\text{offset}\).
Emit `FRoadCorner{ StartEdge, EndEdge, StartDist=t_A, EndDist=t_B, StartOffset, EndOffset,
CornerRadius=r, Point=apex, GUID }`.

### 10.9 Surface boolean union (the quality core)

1. For each road, extract the 2-D **outline polygon** between its two outer edges over its
   non-masked span (§10.10) → CCW ring.
2. Extract each **corner fillet** as a wedge polygon (§10.8).
3. **Merge groups** by union-find: roads sharing a corner are in one group.
4. **Inflate** every polygon by \(\delta=\)`BooleanSurfaceInflateEpsilon` (a few cm) to bridge
   micro-gaps, then **Clipper2 `Union`** (even-odd/non-zero) per group → merged 2-D boundary
   (possibly with holes). Optionally deflate by \(\delta\) to restore size.
5. This single merged surface *is* the junction — no bespoke junction mesh.

### 10.10 Overlap masks & 3-D re-projection

**Overlap mask:** for each road, the set of reference-line **distance intervals**
\([s_0,s_1]\) where its outline overlaps another road's outline (found via §10.5 projection of
one edge onto another, or polygon intersection tests). Masked spans are excluded from the road's
own outline (step 1 above) and from marking/sidewalk emission → this is what removes double
surfaces and stops sidewalks crossing junctions.

**2-D → 3-D height:** each merged-boundary vertex \(\mathbf{v}_{2D}\) gets \(z\) by sampling
**only same-group source polygons**: find the nearest source **segment** \([\mathbf{a},\mathbf{b}]\)
(with per-vertex heights \(z_a,z_b\)) via §10.5 and interpolate \(z=(1-t^\*)z_a+t^\* z_b\); or,
inside a source triangle, use **barycentric** weights \((\lambda_1,\lambda_2,\lambda_3)\),
\(z=\lambda_1 z_1+\lambda_2 z_2+\lambda_3 z_3\). Blend across group seams by inverse-distance
weighting of the \(k\) nearest source samples to avoid Z discontinuities.

### 10.11 Perimeter loops via a quantized half-edge graph

To turn the merged boundary + interior cuts into closed faces (road strips vs. intersection
interiors):
1. **Quantize** all vertices to a grid of size \(q\) (e.g. 1 cm): \(\mathbf{v}\mapsto
   \operatorname{round}(\mathbf{v}/q)\) so coincident points from different sources merge exactly.
2. Build an undirected planar graph; split each undirected edge into **two half-edges**.
3. At each vertex, sort outgoing half-edges by angle \(\operatorname{atan2}(\Delta y,\Delta x)\).
4. **Face traversal:** for a half-edge, `next = twin.then most-clockwise outgoing`
   (\(\text{next}= \text{the outgoing half-edge whose angle is the next one clockwise from the
   reversed incoming direction}\)); follow `next` until returning to start → one closed loop.
5. Discard the outer (infinite) face by signed-area sign; keep interior loops. **Signed area**
   \(A=\tfrac12\sum_i (x_i y_{i+1}-x_{i+1}y_i)\): CCW \(>0\). Assign each loop a **stable GUID**
   from the sorted set of its source edge/corner IDs (for reuse, §10.15).

### 10.12 Height layering (grade separation)

For an overlapping pair, decide **which is on top** from tags: higher `layer`, then
`bridge` over none over `tunnel`, then higher road class. The lower road's surface & markings
are suppressed inside the overlap mask (drawn *under* the overpass). Pairs with vertical gap
\(>450\) cm never become a corner (§10.6) — they simply pass over.

### 10.13 Height matching & blend at intersections

All arms meeting at a junction are forced to a shared node elevation \(z^\*\) (mean or
king-arm height). The correction blends out along each arm over a distance
\(D=w_{narrow}\times\)`HeightMatchBlendMult` (3.0). With normalized along-distance
\(\hat s=\operatorname{clamp}(s/D,0,1)\), use **smoothstep**:
\[
z(s)=z^\*+\big(z_{orig}(s)-z^\*\big)\,\operatorname{smoothstep}(\hat s),\quad
\operatorname{smoothstep}(x)=3x^2-2x^3 .
\]

### 10.14 Landscape conform: plane fit + patch bake

Flatten terrain under each junction:
1. **Least-squares plane** through the junction ring vertices \(\{(x_i,y_i,z_i)\}\): solve
   \(\min_{a,b,d}\sum (a x_i+b y_i+d-z_i)^2\) via the \(3\times3\) normal equations
   \(\mathbf{A}^\top\mathbf{A}\,[a,b,d]^\top=\mathbf{A}^\top\mathbf{z}\).
2. Offset the plane **down** by `BiasDown` (25 cm) so the road sits proud (no z-fight).
3. Bake an `RGBA16F` **`LandscapeTexturePatch`** (height patch) covering the ring AABB + margin
   (75 cm) at `IntersectionPatchTexelCm` (100 cm/texel), with a `FalloffCm` (10 cm) blend to
   the surrounding heightmap. Road corridors use our existing `SculptRoadCorridors` +
   junction-disc stamping (already in `OSMOverpassRoadImport.cpp`) — **reuse, do not re-derive**.

### 10.15 Meshing

- **Road strips:** triangulate each perimeter loop. For a simple ribbon, connect the two edge
  polylines as a triangle strip. For arbitrary loops (junction interiors), run **constrained
  Delaunay triangulation** (`GeometryCore`/`Delaunay`), constraining loop boundary edges. **Guard
  inputs** (dedupe within \(q\), drop zero-area/collinear, finite checks) — degenerate input is
  the classic CDT crash and the source of RoadBLD's `Failed to create static mesh` (§8.8).
- **Vertices/normals/UVs:** position from §10.10; normal \(=\mathbf U\) (or triangle normal for
  junction faces); **UV** \(u=\)lateral offset\(/\)texture width, \(v=s/\)`TextureVScale` (along
  arc length). Compute tangents from \(\partial\mathbf p/\partial u\).
- **Empty-union guard (§8.12):** skip any loop whose Clipper union/area is empty rather than
  emitting a zero-area stub.

### 10.16 Markings, sidewalks, details

- **Baked lane markings:** `FLaneMarkingSegment{ Width, EdgeOffset, TextureVScale,
  SegmentBehavior, Material }` on each `UEdgeCurve`; realized as extra UV bands / thin quads on
  the road mesh (§8.10 system 1), suppressed inside shoulder/overlap masks.
- **Freehand markings / decals:** independent `URoadMarkingLine` strip meshes or mesh decals
  (§8.10 system 2), rebuilt in a separate stage.
- **Sidewalks:** perimeter-loop bands (§10.4 offsets) unioned via Clipper2, trimmed by the
  sidewalk overlap mask (§10.10) — the clean-intersection mechanism (§8.12).
- **Details:** prefab instanced actors (islands/medians) placed on generated geometry, **outside**
  the rebuild (§8.11).

### 10.17 Incremental scoping & GUID reuse

Sets \(Modified\subseteq Pending\subseteq TestAgainst\) (§2.3). Expand \(Pending\) by
endpoint-linked roads; expand \(TestAgainst\) by octree broadphase. A loop/geo is **reused** if
its GUID signature (§10.11) is unchanged and no source road is in \(Pending\); otherwise it is
destroyed and rebuilt. This keeps both interactive edits and batch OSM imports affordable
(§8.3). All compute (§10.2–§10.15 up to plans) runs off the game thread; only the committer
mutates the world, frame-budgeted at ~16 ms (§2.5).

### 10.18 Pipeline-4 entry point (pseudocode)

```cpp
void URoadNetwork::Rebuild(TArrayView<const int32> Modified)
{
    FRebuildContext Ctx;
    Ctx.Modified = Modified;
    DeterminePendingRoads(Ctx);                 // §10.17 scope expand
    BuildEndpointJoints(Ctx);                   // §10.7 (OSM node IDs or authored links)
    BuildCurves(Ctx);                           // §10.2–§10.4 refline, edges, snapshots, octree
    DetectCorners(Ctx);                         // §10.6 (skip joints & grade-separated)
    BuildOverlapMasks(Ctx);                     // §10.10
    ClassifyPairwiseZones(Ctx);                 // §10.12 height layering
    BuildShoulderAndSidewalkMasks(Ctx);         // §10.16 suppression
    BuildSurfaceUnion(Ctx);                     // §10.9 Clipper2 union + §10.10 reproject
    ExtractPerimeterLoops(Ctx);                 // §10.11 half-edge → loops (+GUIDs)
    PlanMeshes(Ctx);                            // §10.15 (ParallelFor; no spawning)
    PlanDetails(Ctx);                           // §10.16 modules per segment
    // ---- single commit (game thread, sliced) ----
    Commit(Ctx);                                // destroy stale, spawn geo, arrays, landscape §10.14
}
```

### 10.19 Test & acceptance

Unit-test the pure math (§10.2–§10.11) headlessly: Fresnel vs. reference tables; offset polyline
has no self-intersections; projection returns correct signed offset near a shared vertex;
segment-intersection edge cases (parallel, collinear, endpoint-touch); half-edge face extraction
on hand-built graphs; boolean union area conservation. Integration acceptance = **an OSM import
on Pipeline 4 produces seamless junctions (no blobs), no sidewalks through intersections, and
terrain conformed** — the three problems from §0 — with **zero RoadBLD dependency** (verified by
the build-time symbol grep in §10.0).

---

## Appendix A — History: how this was reconstructed & what was ported

> Preserved verbatim from the working notes that produced this plan and the companion fixes
> in `OSMRoadCore`. Kept for provenance.

**One thing you should know first.** RoadBLD ships no source code. The `Plugins\RoadBLD\Source`
folders contain only `Build.cs` files with `bUsePrecompiled = true` — everything else is
compiled DLLs and content assets. So there is literally nothing to copy verbatim. Instead, I
extracted the readable strings from `UnrealEditor-RoadBLDRuntime.dll`, which turned out to
contain the plugin's complete doc-comments, class names, and settings descriptions — enough to
reconstruct exactly how it handles junctions.

**How RoadBLD handles junctions (reconstructed from the binary + config).** It has no junction
actor at all. A staged rebuild pipeline (CurveBuilder → CornerBuilder → OverlapMaskBuilder →
PerimeterBuilder → SurfaceBooleanBuilder → MeshBuilder → Committer) treats intersections as
data, not objects:

- **Corners, not gates:** it detects edge-curve crossings, builds corner fillets between them
  (fixed 5 m offset/radius defaults), and explicitly skips endpoint-snap intersections and
  mismatched road types — the exact degenerate cases that caused our blobs.
- **Boolean union surface:** each road's outline polygon plus the corner fillets get merged
  with a Clipper2 boolean union into one continuous surface — no seams, no separate junction
  mesh to misbehave.
- **Height threshold 450 cm:** crossings with a larger Z gap never form intersections. This
  independently validates the `MaxJunctionZGap` (350 cm) guard I added to RoadBuilder.
- **Intersection height matching:** all roads meeting at an intersection have their centerline
  heights forced to match, blended out along the narrower road over narrow width × 3.
- **Intersection landscape patch:** a dedicated terrain patch per intersection (75 cm margin)
  flattens the ground under the whole junction polygon.

**What I ported into our pipeline (`OSMOverpassRoadImport.cpp`).** The last two items were our
real gaps, and both are now implemented:

- **Width-scaled junction plates.** Our existing junction snap used a fixed 3 m plate / 8 m
  landing regardless of road size. Now each junction sizes itself: the flat plate grows to
  cover the widest incident arm plus a 75 cm margin, and the landing extends by narrowest arm
  width × 3 (tunable via `osm.RoadJunctionBlendWidthMult`). An arterial crossing gets a
  properly large level area; an alley crossing stays compact. The old CVars act as minimums.
- **Junction disc terrain stamping.** The corridor sculpt only rasterized road rectangles, so
  the corner-fillet wedges of a junction polygon sat outside every arm's corridor — on slopes,
  terrain poked through the junction corners. The sculpt now stamps a flat disc at every
  junction node (radius = widest arm's flat half-width + 300 cm fillet allowance, bed Z = the
  node's snapped height), integrated into the same max-weight-wins rasterizer so it blends
  cleanly with the arm corridors.

No lint errors. This needs a full editor rebuild (it's in `OSMRoadCore`, alongside the earlier
RoadBuilder junction fixes which also need one).

**Worth noting:** RoadBLD's design independently confirms the direction of every fix we made
this week — Z-gap guarding, rejecting endpoint-degenerate corners, and keeping junction
fallbacks local. If you later want the next tier of their quality, the candidates are the
Clipper2 surface union (one merged road surface instead of per-road meshes) and marking
suppression inside junction masks — both are bigger builds, so I left them out for now.

### A.1 Build status / known blocker (open)

The full editor rebuild required to land the `OSMRoadCore` + RoadBuilder junction fixes is
currently **failing** with a stale/missing generated-header error, not a code error in our
changes:

```
cannot open source file "MassStateTreeProcessors.gen.cpp"
Build.bat McpPcgEditor Win64 Development ... exited with code 8
+ cascading: "expression preceding parentheses ... must have (pointer-to-) function type",
  "template parameter T is not used ... TIsContiguousContainer<<error type>>",
  "the size of an array must be greater than zero" (×several)
```

This signature (a missing `*.gen.cpp` for an unrelated engine module → cascading template /
zero-size-array errors) is a **stale UHT/Intermediate cache**, not a compile error in the road
work. Likely remedy (to try when we return to it): close the editor, delete the affected
`Intermediate/Build` (and regenerate project files), then rebuild `McpPcgEditor`. **Not yet
attempted / unresolved** — recorded here so it isn't lost.

---

## §11 Pipeline 4 (RoadNet) — implementation status

The independent `RoadNet` plugin (Plugins/RoadNet) is wired end-to-end from OSM import
(`osm.RoadPipeline 4` → `OSMRoadNetBridge` → `URoadNetwork::Rebuild`). All stages below
compile clean (`McpPcgEditor`, exit 0) and are **zero-dependency on RoadBLD/WorldBLD/CityBLD**.

**Implemented**
- **Data model** (`RoadNetTypes.h`): `FRoadDef`, `FRoadNetLaneSpec` (directional lanes,
  sidewalk flags), `FRoadNetEndpointLink`, provenance (`OSM`/`HandDrawn`), grade fields.
- **Math** (`RoadNetMath`): tangent/right-axis frame, arc-length resample (adaptive knot
  preserve), miter offset, stable point→polyline projection, segment intersection, corner
  fillet (§10.1–§10.8).
- **§10.2–10.4 Curves**: reference resample + left/right outer edges per road.
- **§10.7 Endpoint joints**: unified topology derivation — shared OSM node id (exact map) **plus
  spatial welding** (ends within `kEndpointWeldCm` = 400 cm collapse to one joint via a uniform
  grid). This lets hand-drawn roads (no NodeIds) and **mixed OSM+hand-drawn** ends form real
  junctions. Degree classification (Terminal/Seam/Intersection) as before.
- **§10.12 Grade separation** (`RoadNetZones`): union-find over shared nodes + at-grade
  crossings; overpasses/underpasses (`|dZ| > 350 cm`, differing `Layer`/bridge/tunnel) land in
  separate zones and are unioned + meshed independently → **no cross-level blobs**.
- **§10.9 Boolean-union surface** (`RoadNetSurface`, engine Clipper2 `PolygonsUnion`): per-zone
  union of road outlines → seamless filled junctions; morphological close bridges micro-gaps.
- **§10.8 Junction fillets**: rounded discs (radius = widest incident half-width) added at
  intersection joints + centerline crossings so junction corners read as clean rounds.
- **§8.12 Sidewalks**: **per-road, per-side** strips — each road emits a ribbon between its outer
  edge and edge+`SidewalkWidth` only on the side(s) it requests (`bSidewalkLeft`/`bSidewalkRight`).
  The strips are unioned, then the carriageway (incl. junction discs) is subtracted, so sidewalks
  hug only the requested sides, merge cleanly between neighbours, and never cross an intersection.
- **§8.10 Lane markings**: baked, split by paint colour into **two meshed layers** —
  **yellow** centre line (two-way roads) and **white** edge (shoulder) lines + **dashed interior
  lane dividers**. Both colours are **subtracted from the junction discs** so paint clears the
  intersection boxes (never crosses a junction). Committed as `RoadNet_Markings_White` /
  `RoadNet_Markings_Yellow` actors.
- **§10.10 Elevation reprojection**: each mesh vertex is projected onto the nearest centerline
  and its Z **interpolated along the hit segment** (smooth over slopes), not nearest-vertex.
- **§10.15 Meshing + commit** (`RoadNetMesh`): constrained-Delaunay triangulation, per-zone
  projection reprojection (+12 cm anti-z-fight lift), upward winding, computed normals. Spawns
  reusable `ADynamicMeshActor`s: `RoadNet_Surface` (asphalt), `RoadNet_Sidewalks` (concrete,
  +15 cm curb), `RoadNet_Markings_White` + `RoadNet_Markings_Yellow` (paint, +4 cm).
- **§10.16 Materials**: each layer has an optional `UMaterialInterface` override on `URoadNetwork`
  (`RoadMaterial` / `SidewalkMaterial` / `MarkingWhiteMaterial` / `MarkingYellowMaterial`, editable
  inline on `ARoadNetActor`). When set, the real material is applied to the layer's mesh; when
  unset, the layer falls back to a flat constant vertex colour so geometry is always visible.
- **§10.11 / §8.4 Perimeter loops → PCG export** (`RoadNetPerimeters`): the merged surface's
  outer rings (network outlines) and hole rings (enclosed block "islands") are lifted to world
  3-D (Z via the shared `RoadNetMesh::SampleHeight`) and realised as closed **`USplineComponent`s**
  on a reused `RoadNet_Perimeters` actor, tagged `RoadNetPerimeter` + `...Outer`/`...Hole`. This
  is the standard PCG-consumable seam (Spline Sampler / Get Spline Data) for scattering along road
  edges and filling blocks — the pragmatic product of the §10.11 half-edge design (full per-road
  face graph is a later refinement).
- Terrain: Pipeline 4 runs the existing `SculptRoadCorridors` before RoadNet, so corridors are
  already flattened into the landscape.
- **§9 Dual-input authoring**: `ARoadNetActor` is the level-persistent home of the
  `URoadNetwork` (its `Roads` array serializes with the actor). OSM import now finds/spawns this
  actor and refreshes only `Source==OSM` roads, so **hand-drawn roads survive OSM re-imports**
  (§9.4). Two hand-draw front-ends both converge on the same `FRoadDef` + rebuild pipeline (§9.1):
  1. **Spline-draft MVP** on `ARoadNetActor`: add editable `USplineComponent` drafts
     (`AddDraftSpline`), shape them in the viewport, then `RebuildFromDrafts` turns them into
     `Source==HandDrawn` roads and rebuilds; `ClearHandDrawn` removes them.
  2. **Click-to-draw EdMode** (`RoadNetEditor` editor module, `FEdModeRoadNet`, mode id
     `EM_RoadNet`, registered via `FEditorModeRegistry`): activate the "RoadNet Draw" mode, then
     **left-click** to line-trace reference points onto terrain (falls back to the Z=0 plane when
     no collision), **Enter / double-click / right-click** to commit the polyline into a
     `Source==HandDrawn` road (wrapped in an undo transaction) and rebuild, **Backspace/Delete**
     to drop the last point, **Escape** to cancel. Placed points + segments + a live rubber-band
     preview to the cursor are drawn in the viewport. **Snapping:** the cursor snaps (2-D, within
     `kSnapCm` = 600 cm) to existing road vertices — endpoints *and* interior vertices — and to the
     draft's own earlier points; the active snap target is highlighted red. Because the snap radius
     exceeds the joint weld radius, snapped ends reliably weld into junctions at rebuild (endpoint
     meets endpoint) or tee onto an existing centreline (endpoint meets interior vertex, picked up
     as a geometric crossing). **Editing (idle — no draft in progress):** left-click a hand-drawn
     road's point handle (via `HRoadNetPointProxy` hit proxies drawn over each `Ref` vertex) to
     select it; the standard **translate widget** moves it (mutations batched during the drag,
     one drag-spanning undo transaction, network rebuilt on release via `EndTracking`); **Delete**
     removes the selected point (and the whole road if it drops below 2 points); **Ctrl+click a
     segment** (via `HRoadNetSegmentProxy`, lower priority than the point proxy) inserts a new
     point projected onto that segment (mid-span **split**) and selects it. **Plain click on a
     segment** selects the **whole road** (highlighted, no widget); **Delete** then removes the
     entire road. Point/road mutation goes through `URoadNetwork::MoveRoadPoint / DeleteRoadPoint
     / InsertRoadPoint / RemoveRoad`. Modelled on RoadBuilder's `FEdModeRoad` (legacy `FEdMode`)
     to match the engine's proven editor pattern, but fully independent of it.
  Draft lane count / width / oneway / sidewalks / class are `ARoadNetActor` properties shared by
  both front-ends.

**Not yet implemented (refinements)**
- Draw mode now supports draw+snap/weld, point move/delete, Ctrl+click mid-span split, and
  whole-road select/delete. Remaining §9.3 tooling: editing OSM-sourced points and explicit
  `FRoadNetEndpointLink` authoring (vs. the current position weld).
- §10.10 overlap masks (barycentric multi-source blend) — current projection is nearest-centerline.
- §10.11 **full** half-edge per-road face graph (current export uses the merged-surface rings —
  enough for PCG loops, but not yet per-road faces for incremental rebuild scoping / GUID reuse).
- Details/props (§8.11), landscape paint-layer pass, per-road-material IDs. (Per-layer material
  overrides now exist; per-road/per-class material variation is the remaining refinement.)

---

## §12 Lanes & ZoneGraph — learnings from RoadBLD (drives the next phase)

Reverse-engineered from RoadBLD's reflection (`RoadBLDRuntime` UHT gen) + a live
`MyProject5` lane-editing session. Full capture in `ROADBLD_FEATURES.md §4`.
This section is the **implementation brief** for RoadNet's lane + graph work.

### 12.1 Lane data model to replicate
- A lane is a **first-class object** (`UDynamicRoadLane`), not a per-road count.
  Ours (`FRoadNetLaneSpec`) must grow from a flat lane count to per-lane entities.
- Each lane is the **ribbon between two boundary curves** (`Left/RightEdgeCurve`,
  `UEdgeCurve`). An `EdgeCurve` is a piecewise **`(distanceAlongRoad → lateralOffset)`
  polyline** ("OffsetPoints"). Lane shape (widenings, turn bays, tapers) is authored
  by editing these offset points — **not** a single scalar width.
- **Variable width** via `FLaneWidthSegment { StartDistance, EndDistance,
  TransitionIn, TransitionOut }` → a lane can ramp 0→full / full→0 over a transition
  (this is the turn-bay / ramp-taper / lane add-drop mechanism).
- **Lane type** = `ELaneType { Normal, Parking, Border, Restricted, Shoulder,
  CenterTurn, Median }` (drop deprecated `None`; sidewalks are their own object).
- **Direction is side-based**, not a per-lane flag: `ERoadSide { Left, Right, None }`.
  Left lanes travel one way, right the other. Confirmed by `Added a new right side
  lane` logging + the viewport arrows. RoadNet: left = −lateral, right = +lateral off
  the road frame; travel direction = side.

**RoadNet task:** replace `FRoadNetLaneSpec` count with `TArray<FRoadNetLane>` where
`FRoadNetLane = { ERoadNetLaneType Type, ERoadSide Side, FEdgeOffsets Left, FEdgeOffsets
Right, TArray<FLaneWidthSegment> WidthSegments, UMaterialInterface* Overlay }`, and
`FEdgeOffsets = TArray<FVector2D /*dist, offset*/>`. Build lane ribbons by offsetting the
reference curve per its edge-offset polylines (reuse `RoadNetMath::OffsetPolyline`).

### 12.2 ZoneGraph — RoadBLD does NOT build one (net-new for us)
**Confirmed** (reflection: no `AZoneShape`/`ZoneGraph` UCLASS authored by RoadBLD;
logs: zero zone/connection lines). RoadBLD only **exposes** data for downstream
consumers:
- `USidewalkPartition { bWalkable, Material }` — described as feeding *"pedestrian
  ZoneShapes and other walkability consumers."*
- PCG nodes `PCGRoadBLDGetRoadEdges`, `PCGRoadBLDGetSidewalkPartitions`, and a
  per-preset `RoadPCGGraph (UPCGGraph*)` run on each `ARoadGeo`.
- Its rebuild builds **geometry** zones only (`RebuildPairwiseZoneBuilder` =
  grade-sep/overlap, our `PartitionLayers` analogue) — **no routing/turn graph**.

**RoadNet task (net-new):** build our own lane-connectivity graph from what we
already have — per-side `ELaneType` lanes + `EdgeCurve` offset polylines + welded
endpoint joints. At each welded joint, connect lane ends by **side + adjacency +
type** (drivable-to-drivable), producing either engine `ZoneShape`s (if we take the
`ZoneGraph` plugin dependency) or our own graph struct exported to PCG. This is the
main remaining design piece; RoadBLD offers the *inputs*, not the algorithm.

### 12.4 IMPLEMENTED (compiles clean, `McpPcgEditor` exit 0)
- **Lane data model** (`RoadNetTypes.h`): `ERoadNetLaneType` (Normal/Parking/
  Border/Restricted/Shoulder/CenterTurn/Median), `ERoadNetSide` (Left/Right/
  Center), `FRoadNetEdgeKnot` (dist→offset), `FRoadNetLaneWidthSeg` (taper),
  `FRoadNetLane` (LaneId, type, side, centre offset, width, edge knots, width
  segments, overlay material). `FRoadNetLaneSpec::DetailedLanes` overrides the
  count model when authored; `ResolveLanes()` synthesizes a uniform per-side
  lane set from the counts otherwise, so OSM import is unchanged and both paths
  iterate real lanes.
- **Lane geometry** (`RoadNetLanes.h/.cpp`): `LaneOffsetAt` (uniform or edge-knot
  interpolated) + `BuildLaneCenterline` (per-vertex lateral offset of the
  resampled reference).
- **Lane graph** (`URoadNetwork::BuildLaneGraph`): at every joint with ≥2 arms,
  each drivable lane entering the joint connects to each drivable lane leaving it
  on a *different* arm (all turn movements); 2-arm seams also flagged `bThrough`.
  Direction is side-based (Right = +arc, Left = −arc). Output:
  `FRoadNetLaneConnection { From, To, Joint, Entry, Exit, bThrough }`.
- **PCG export** (`CommitLaneGraph`): one spline per connection on a
  `RoadNet_LaneGraph` actor — turns curve through the joint centre, seams stay
  linear — tagged `RoadNetLaneGraph` + `RoadNetLaneTurn`/`RoadNetLaneThrough`.

**Deferred refinement:** per-lane ribbon *rendering* (distinct meshed lane
strips + per-lane overlay materials). The carriageway still renders as the merged
boolean surface + baked markings; lane strips are a visual upgrade, not needed
for the graph.

### 12.3 Per-commit conform coupling (already in our terrain doc)
Each lane-edit commit re-runs the two-way conform on the modified road
(`ConformReason_ModifiedRoad` + `MirrorSplineRefresh`). Mirror in RoadNet: any lane
edit that changes the carriageway extent must re-drive the corridor sculpt for that
road only (dirty-tracked), per `ROADBLD_TERRAIN_DEFORM.md`.
