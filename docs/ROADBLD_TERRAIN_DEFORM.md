# RoadBLD — Terrain Deformation: Reverse‑Engineering Notes

Collected while studying the precompiled **RoadBLD** plugin in `MyProject5`
(only `Build.cs` + UHT‑generated reflection headers ship; no implementation
source). The reflection metadata (`Intermediate/.../UHT/*.gen.cpp`) exposes every
`UCLASS`/`USTRUCT`/`UPROPERTY` name, type, category and tooltip, which is enough
to recover the deformation **architecture** and the exact tunables. Runtime
behaviour is confirmed by listening to the editor log while hand‑drawing roads.

Goal: adopt whatever makes RoadBLD's terrain conform cleanly (no bumps, no
trenching) into our McpPcg road pipelines, then push the fix.

---

## 1. Headline finding — RoadBLD does NOT hand‑carve the heightmap

It deforms the terrain through the **engine's native Landscape Spline system**,
driven onto a **dedicated Landscape spline edit layer** (non‑destructive), with
smooth per‑control‑point width + falloff. It never writes raw heightmap cells the
way our `SculptRoadCorridors` does.

Evidence (from `Public/DynamicLandscape/LandscapeMirrorSpline.h`):

- **`ULandscapeMirrorSplineBase : USplineComponent`** — a road‑side spline that
  *mirrors* into the landscape spline graph. Properties:
  - `TScriptInterface<ILandscapeSplineInterface> LandscapeSplineOwner` — the
    landscape spline owner it feeds.
  - `TWeakObjectPtr<ALandscape> WeakLandscape` — the target landscape.
  - `bHasEverSynced`, `bIsRegistered` — lifecycle/sync state.
  - `bDeferSplineLayerRefresh`, `bPendingSplineLayerRefresh` — **deferred spline
    edit‑layer refresh** (batches heightmap updates; strong signal it writes to a
    Landscape *edit layer*, i.e. non‑destructive & revertible).
  - `ELandscapeSplineVisibility CachedVisibility { None, Mesh, Wireframe }`.
- **`ULandscapeMirrorSplineMetadata : USplineMetadata`** — mapping to the native
  graph: `ULandscapeSplineControlPoint* StartPoint`, `int NumControlPoints`,
  `bAlwaysRotateForward`, `bJoinSegmentsOnRemovePoint`,
  `bAutoChangeConnectionsOnMove` (topology auto‑maintained as you edit).
- **`FLandscapeControlPointData`** — per‑control‑point deform params (category
  *"Landscape Splines"*). This is the whole recipe:

  | Field | Meaning |
  |---|---|
  | `HalfWidth` | flattened bed half‑width at this point |
  | `HeightOffset` | raise/lower the bed vs the control point |
  | `LayerWidthRatio` | paint/material layer width vs the flatten width |
  | `SideFalloff` | lateral distance over which it blends back to natural terrain |
  | `LeftSideFalloffFactor` / `RightSideFalloffFactor` | asymmetric per‑side falloff |
  | `EndFalloff` | longitudinal blend at the spline ends |
  | `bRaiseTerrain` / `bLowerTerrain` | whether this segment may fill / cut (per‑segment!) |
  | `bEnableTangentOverride`, `TangentOverride` | grade continuity control |

  > The `bRaiseTerrain`/`bLowerTerrain` tooltip: *"Controls the segment between
  > the current point and the next point, if one exists."* → the raise/cut policy
  > is **per‑segment**, not global.

### Why this looks smooth where ours looks bumpy
- The native landscape‑spline deform applies a **smooth lateral falloff**
  (`SideFalloff` + per‑side factors) instead of our flat‑core + single cosine
  buffer, and a **longitudinal `EndFalloff`**, so the bed melts into terrain.
- Writing to a **spline edit layer** means the deform is re‑evaluated cleanly
  every edit (deferred/batched via the `bPendingSplineLayerRefresh` flag) rather
  than accumulating destructive stamps — no double‑carving, fully revertible.
- It is still ultimately heightmap‑resolution‑bound, but the falloff hides the
  quantisation far better than a hard flatten does.

---

## 2. How it maps onto what we already have (McpPcg)

- Our **Pipeline 1 (Yazan / `ApplySplines`)** already uses native landscape
  splines but was judged "trench‑prone on coarse terrain". RoadBLD shows the fix
  is **not** to abandon landscape splines but to drive them with proper
  per‑point `HalfWidth` + `SideFalloff` (+ left/right factors) + `EndFalloff` and
  a **dedicated spline edit layer**, plus per‑segment `bRaiseTerrain/bLowerTerrain`.
- Our **`SculptRoadCorridors`** (manual heightmap flatten + cosine buffer) is the
  cruder equivalent; the missing pieces are: (a) smooth per‑side falloff, (b)
  edit‑layer (non‑destructive) writes, (c) per‑segment raise/lower policy.

### Candidate fixes to push back to McpPcg
0. **★ Two‑way conform (the real fix for bumps).** Don't just drape the road or
   just flatten the terrain — do BOTH each rebuild, like RoadBLD:
   (a) conform the road's vertical profile to the terrain (heightmap read), and
   (b) deform the terrain to the road via a native landscape spline on a spline
   edit layer. The two meet in the middle, so no terrain bump rides over the
   road and the road never floats. This supersedes the one‑way `SculptRoadCorridors`.
1. Add asymmetric lateral falloff (`Left/RightSideFalloffFactor`) + longitudinal
   `EndFalloff` to the deform so the bed blends instead of stepping.
1b. **Junctions → Landscape Texture Patch.** For intersection pads, fit a plane
   through the junction ring and bake a small height texture applied via
   `ULandscapeTexturePatch` (with a small down‑bias), instead of a hard flatten.
   Keeps the pad planar + blended and is non‑destructive.
2. Move corridor writes onto a **Landscape edit layer** (revertible, no
   destructive accumulation), refreshed once per rebuild (deferred like RoadBLD).
3. Expose per‑road **raise vs lower** policy (cut‑only / fill‑only / both) like
   `bRaiseTerrain`/`bLowerTerrain`.
4. **Dirty‑track the conform** (`SkippedConform` / `ConformReason_ModifiedRoad`):
   only re‑conform roads that changed; run commit on a per‑frame time budget.
5. **Filter the height trace to `ALandscape`/`LandscapeStreamingProxy`** (ignore
   `WorldPartitionHLOD`) — fixes our Z‑pick when regions/HLOD overlap.
6. Keep the `quadCm` tessellation lever, but treat it as secondary — the two‑way
   conform + falloff is the primary smoothness win.

---

## 3. Runtime log observations (hand‑drawing session)

_Listening to `MyProject5/Saved/Logs/MyProject5.log` while roads are drawn/edited._

### Session 1 — drawing a `RP_Sample_2Lane2Way_NoSidewalks` road

Confirms the reflection‑derived architecture and names the exact runtime entry points:

- Toolset is **WorldBLD** (`LogWorldBLDEditor`): tools `CityBLD, RoadBLD, TwinBLD`.
  Drawing is driven by a `RoadController` + `RoadBrushActor`, launched from a
  preset (`RP_Sample_2Lane2Way_NoSidewalks_C`).
- **Terrain pick = downward multi‑trace** on mouse move:
  `RoadController [LandscapeTrace] Firing multi-trace From=(x,y,+1188) To=(x,y,-4811)`.
  Flags on the controller: `bEnableLandscapeSplineMirror=1`,
  `bEffectiveAlignLandscape=0`.
  - ⚠️ In this session the trace **hit `WorldPartitionHLOD`
    (`HLOD0_Instancing_…`), not the landscape** (`Z=203.7`). Same WP/HLOD trace
    pitfall we have in McpPcg — the road samples height off the HLOD proxy when
    the real landscape collision isn't the first hit. Worth guarding
    (filter trace hits to `ALandscape`/landscape collision).
- **The mirror step is the deform**: `DynamicRoad::SyncLandscapeMirrorSplineFromCenterline`
  is called continuously during the drag (once per centerline update). It pushes
  the road centerline polyline into the native landscape spline
  (warns `No polyline points to update!` before any point exists).
- **Batched / deferred, applied at Commit** (matches `bDeferSplineLayerRefresh`):
  the rebuild timing shows the landscape work happens in the *Commit* phase:
  ```
  [RebuildTiming] Commit ApplyLandscapePaint: 0.0 ms
  [RebuildTiming] Landscape breakdown (execution: 0.1 ms):
  [RebuildTiming]   MirrorSplineRefresh: 4
  [RebuildTiming] Landscape slowest: MirrorSpline_UpdateRoadSpline (0.0 ms)
  ```
  So per rebuild it: (a) syncs the mirror spline from the centerline, (b) runs
  `MirrorSpline_UpdateRoadSpline` (the landscape‑spline heightmap refresh), and
  (c) `ApplyLandscapePaint` (paint layers). All counted/timed as one "Landscape"
  bucket → **the terrain deform is a first‑class, batched stage of the road
  rebuild**, not a separate manual carve.

**Replication takeaways (add to §2 fixes):**
- Mirror the road centerline → native landscape spline control points
  (`SyncLandscapeMirrorSplineFromCenterline` is the exact operation to copy).
- Run the landscape deform as a **batched Commit stage** once per rebuild, not
  per edit.
- **Filter the height trace to the landscape** (ignore `WorldPartitionHLOD`
  hits) — this is a bug we should also fix in our own `SampleTerrainZ` path.

### Session 1 (cont.) — rebuild cadence & cost

- The landscape deform is **effectively free**: per rebuild it runs
  `MirrorSpline_UpdateRoadSpline` **once per control point** (5 calls for a
  5‑point road, ~75–87% of a **0.1–0.3 ms** total "Landscape" bucket).
  → mirroring into landscape splines costs nothing; the engine's spline‑layer
  heightmap eval is deferred/off the hot path. Our per‑rebuild deform budget is
  not the concern — correctness of the spline params is.
- During an active drag `SyncLandscapeMirrorSplineFromCenterline` is spammed
  ~60×/sec (one per mouse tick) but only the **Commit** does the actual
  `ApplyLandscapePaint` + `MirrorSplineRefresh` → confirms **sync‑often,
  apply‑once (batched at commit)**.
- New landscape sub‑step seen: **`Conform_GlobalActorScan`** (listed under the
  Landscape breakdown) — a scan/conform pass over global actors, likely how it
  finds the landscape (and possibly other roads) to conform to. Sometimes it is
  the "slowest" landscape step (still ~0 ms here).
- Debug hooks exist under a **`RoadBLD.Debug.*` CVar namespace**
  (e.g. `RoadBLD.Debug.LogRoadGeoSpawns`) — useful switches to turn on for
  deeper logging next session.
- HLOD‑trace pitfall reconfirmed on the 2nd road (hit
  `HLOD0_Instancing_…` `WorldPartitionHLOD`, `bEffectiveAlignLandscape=0`).

### Session 1 (cont.) — ⭐ the deform is a TWO‑WAY conform + full pipeline

Drawing over a **loaded** region the trace hits the real landscape and alignment
switches on:
```
[LandscapeTrace] hit actor='LandscapeStreamingProxy_…' class='LandscapeStreamingProxy' Z=256.8
[LandscapeTrace] Result - bEffectiveAlignLandscape=1  bEnableLandscapeSplineMirror=1
```
→ **align only works when the trace hits a `LandscapeStreamingProxy`, not the
HLOD.** (Directly validates filtering our `SampleTerrainZ` trace to landscape.)

**The terrain step is bidirectional** (this is the key to "no bumps"):
1. **Road → terrain (drape):** `ConformVerticalProfile` + `Conform_HeightmapRead`
   *read the landscape heightmap* and conform the road's **vertical profile** to
   the ground. (2–3 heightmap reads per rebuild.)
2. **Terrain → road (carve):** `MirrorSpline_UpdateRoadSpline` drives the native
   landscape spline (which deforms the heightmap via the spline edit layer).
   First time: `MirrorSpline_InitializeManagedSpline` ("No landscape spline
   owner, calling InitializeMirrorSpline…"); after that "owner exists, will sync
   on UpdateSpline".

So the road and the terrain **meet in the middle**: the road is graded to follow
terrain, and the terrain is flattened to the road — that two‑sided conform is
why there are no bumps riding over the surface. Our pipeline only does half
(drape) + a crude one‑way flatten.

**Incremental / dirty‑tracked:** the Landscape breakdown reports
`ConformReason_ModifiedRoad: 1` and `SkippedConform: 1` — only modified roads are
re‑conformed; unchanged ones are skipped. The managed landscape spline is
**cleared and re‑synced every rebuild** ("Delegating to parent Reset (will clear
landscape spline)").

**Full rebuild pipeline** (`[RebuildTiming]`), for reference when we restructure ours:
- *Pre‑async (game thread):* `CurveBuilder` → `Expand+Snapshot` → `FillNetworkSnapshot`.
- *Async (background thread):* `CornerBuilder` → `OverlapMaskBuilder` →
  `ShoulderMaskBuilder` → `SidewalkOverlapMaskBuilder` → `MeshBuilder` → `DetailsBuilder`.
- *Commit (game thread, budget 16 ms/frame, spread over 3–4 frames):*
  `DestroyOldActors` → `SpawnNewActors` (`PerimeterLoopGeo`, `RoadModules`,
  **`BooleanSegment`** = the dominant cost) → `SpawnTactileMeshes` →
  `CommitShoulderMasks` → `BuildMeshPerRoad` → **`ApplyLandscapePaint`** →
  `UpdateNetworkArrays` → `RebuildFreehandMarkings` → `StageStoreChunks` →
  `FlushStoreToBulkData`.
- *Landscape bucket:* `ConformVerticalProfile` + `Conform_HeightmapRead` +
  `MirrorSpline_UpdateRoadSpline` (+ `InitializeManagedSpline` first time).

### Session 1 (cont. 2) — ⭐ junctions deform via a LANDSCAPE TEXTURE PATCH

Intersections use a **different** terrain mechanism than straight segments — the
engine's **Landscape Texture (height) Patch**, not a spline:

```
[IntersectionPatchDiag] Geo=RoadGeo_…_1918514451 RingVerts=383
  RingZ[min=79.4 max=596.9 avg=297.8]
  Plane[A=-0.1120 B=-0.1155 C=-6440.0] Centroid=(-48014.1,-11514.4,272.8)
  LandscapeZ=100.0 LandscapeScaleZ=100.0 BiasDown=25.0
  Coverage=(3035.0, 2893.1) Res=32x30
LogTexture: Building texture … RoadGeo_…_1918514451.LandscapeTexturePatch_0.
  RoadBLD_IntersectionLandscapePatchHeight (RGBA16F, 32x30 …)
```

So per intersection it:
1. Collects the junction **ring vertices** (383 here), and **fits a plane**
   `Plane[A,B,C]` through them (least‑squares tilt of the junction pad).
2. Bakes a small **RGBA16F height texture** (`32x30` at ~1 texel/quad over the
   `Coverage` bbox) → `RoadBLD_IntersectionLandscapePatchHeight`.
3. Applies it as a **`ULandscapeTexturePatch`** (`Patch_Apply_ResolveLandscape`
   → `Patch_Apply_BakeTexture` → `Patch_Apply_ConfigureAndUpdate`), with a small
   **`BiasDown=25.0`** so the pad sits just under the road surface.
   The surface side has a matching `BooleanIntersection` + `BooleanIntersection_LandscapePatch`.

**→ The full deform strategy is three‑part:**
- **Straight segments:** native **Landscape Spline** (`MirrorSpline`) with
  per‑point width + falloff.
- **Junctions:** **Landscape Texture Patch** (plane‑fit height texture) — this is
  how the intersection pad stays flat/planar and blended while the arms taper in.
- **Road profile:** `ConformVerticalProfile` grades the road to the read‑back
  heightmap (the drape half of the two‑way conform).

Both landscape mechanisms are **non‑destructive engine features** (spline edit
layer + texture patch), which is why it's revertible and doesn't accumulate
damage.

**Full‑network rebuild seen here** (`ConformReason_Default: 20`, i.e. every road
re‑conformed, not just the modified one): Commit 2100 ms / 23 frames; Landscape
882 ms = `ConformVerticalProfile` 488 ms (21 calls) + `Conform_HeightmapRead`
288 ms (**168 calls**) + `MirrorSpline_UpdateRoadSpline` 100 ms (23) + patch bake
~4 ms (3). Heightmap reads dominate at scale → a read‑cache/height‑field would
be the optimization (mirrors our own `FCenterlineHeightField` work).

**Scaling note:** per‑edit cost climbs as the network grows —
`BooleanSegment` 33 → 57 → 66 → 96 → 136 → 141 ms and
`MirrorSpline_UpdateRoadSpline` 3.8 → 15.4 ms across six consecutive edits (whole
network rebuilt each time; conform is dirty‑tracked but the surface boolean is
not). Same O(N) growth we care about — a budgeted, multi‑frame commit
(16 ms/frame) is how they keep the editor responsive.

---

### Session 2 — moving points (edit path)

- **Edit dirty‑collection at the controller:**
  `RoadNetworkController::CollectRoadsToRebuild: Added edited road '…' to rebuild
  list` → moving a point marks just the edited road (+ its connected roads) for
  rebuild; the Landscape bucket then shows `ConformReason_ModifiedRoad: 3` /
  `SkippedConform: 6`. Confirms per‑road dirty tracking end‑to‑end.
- **⭐ `Conform_TraceFallback` — the conform is HYBRID.** The height sample is
  *heightmap‑read first, downward‑trace fallback second*:
  `Conform_HeightmapRead` (primary) + `Conform_TraceFallback: 6.0 ms (28 calls)`
  for points the heightmap read couldn't resolve. This is exactly the
  heightmap‑primary / trace‑fallback design I put in our `SampleTerrainZ` — good
  independent validation.
- **Per‑edit junction re‑bake:** moving a point re‑baked **5 intersection
  patches** in the affected region (`[IntersectionPatchDiag]` each with its own
  plane fit + `RGBA16F` texture sized to coverage, e.g. `51x94`, `37x67`,
  `29x33`, `24x25`, `29x30`; `BiasDown=25` throughout). Patch bake is cheap
  (`Patch_Apply_BakeTexture: 11.5 ms / 5 calls`); the landscape cost is dominated
  by `MirrorSpline_UpdateRoadSpline` (145 ms / 5 calls) on this edit.
- **CornerBuilder uses an octree** for broadphase: `[CornerBuilder] Octree boxes
  … per polyline segment (NumSegments = NumPoints-1). 3000 is bounds expansion …
  TotalBoxes=448` → corner/overlap detection is spatially accelerated (mirrors
  our uniform‑grid broadphase in `BuildCrossings`).

## 4. Open questions to resolve back in MyProject5
- Which **edit layer name** does RoadBLD create for the spline deform? (confirm
  it's a reserved Landscape layer, and whether it's per‑landscape.)
- Default values of `SideFalloff`, `EndFalloff`, `HalfWidth` vs road width.
- Does it re‑drive the deform on **every** point move, or only on commit? (the
  `bDeferSplineLayerRefresh`/`bPendingSplineLayerRefresh` pair suggests batching.)
- How junctions are handled in the landscape‑spline graph (shared control points
  via `bAutoChangeConnectionsOnMove` + `bJoinSegmentsOnRemovePoint`).
