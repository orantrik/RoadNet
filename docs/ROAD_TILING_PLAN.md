# Road Tiling — Plan v2 (ownership by construction)

> Goal (unchanged, 2 days running): **one tile = one road stretch between two
> intersections**, carrying its whole cross-section — carriageway + BOTH sidewalks +
> both curbs + its markings + furniture + centre/edge splines. Junctions are their
> own tiles. A road through a junction is TWO tiles. Delete a mid-road point → two
> tiles.

---

## 0. Post-mortem — why v1 failed for two days (do not retry these)

v1's Phase 3 was demoted from *per-segment generation* to *provenance routing*
(see the old "as-built note"). That shortcut kept the merged-blob pipeline and
tried to guess ownership afterwards. Every attempt since was a patch on that guess:

| # | Heuristic tried | Counterexample that killed it |
|---|---|---|
| 1 | centroid → nearest road | sidewalk centroid is offset from its own road; neighbour wins |
| 2 | footprint partition + distance buffer | any buffer either steals or loses a walk |
| 3 | inner-edge single-vertex owner | one corner near a neighbour flips the whole piece |
| 4 | majority ARM vote | fixed arm flips; cross-ROAD theft untouched |
| 5 | medial cut of "fused" pieces | detection heuristic over-fires (diagonal slices) and under-detects |
| 6 | majority ROAD vote | best guess yet — still `road 14 arm 0 spans 2 tiles` |

**Root pattern:** `BuildSurfaceUnion` fuses all roads per zone into anonymous
geometry (`ZoneSurfacePolys`, and the sidewalk `Band = dilate(union) − union`);
`CommitLayer` then partitions the blob spatially. Post-hoc spatial assignment of
anonymous geometry ALWAYS has a counterexample. **No more routing heuristics.**

Kept from the v1 work (generation-side, correct):
- `RoadSampleArm` / `RoadArmRuns` — centreline cut into inter-junction arm runs.
- **Junction discs in `ZoneJunctionCarve`** (2026-07-20 fix): a disc at every
  same-grade crossing and every ≥3-arm joint, so THROUGH roads break at T/X
  junctions (the carriageway-overlap "bite" alone never touched a through road's
  centreline — that's why one tile spanned 3 junctions). Overpasses (|ΔZ|>3 m)
  excluded; 2-arm continuation seams excluded (no over-segmentation).
- `SegAlias` divided-road pairing; `SegTileKey`/`JunTileKey`; `CommitSegmentSplines`
  (already per arm run — correct by construction).

---

## 1. The fix — tag at generation, never assign afterwards

Every polygon is built **from its own road's centreline slice** and carries
`(road, arm)` from birth. There is no assignment step left to get wrong.

Verified enablers already in the code:
- Per-road sidewalk ribbons ALREADY exist: `CommitLayer`'s side masking builds
  `BuildSideRibbon(Sampled, ±In, ±Out)` per road/side — identity is destroyed one
  line later when they are unioned into `MaskU`. We keep them separate instead.
- The zone band `Band = dilate(union) − union` already excludes ALL carriageways,
  so `ribbon ∩ Band` has no flaps over crossing roads and the exact outer edge the
  user sees today.

New context maps (in `FRoadNetRebuildContext`):

```cpp
TMap<FIntPoint, TArray<FGeneralPolygon2d>> TileSurfacePolys;   // key = tile
TMap<FIntPoint, TArray<FGeneralPolygon2d>> TileSidewalkPolys;  // key = tile
```

---

## 2. Phases

### A — Per-arm SIDEWALKS (the reported bug, first)
For each road `r`, each arm run `[lo..hi]` (from `RoadArmRuns`), each enabled side:
1. `Sub = Sampled[lo..hi]` (pad one sample past each end so caps land inside the carve).
2. `Ribbon = BuildSideRibbon(Sub, ±In, ±Out)` — same offsets as today's masks.
3. `Piece = Ribbon ∩ Band[z] − ZoneJunctionCarve[z]`.
4. `Piece −= already-emitted sidewalk polys` (deterministic road-index order): where
   two roads run closer than 2×walk width the shared strip seams mid-gap instead of
   double-covering. Neither road can lose a whole walk — its inner portion always
   borders its own carriageway.
5. `TileSidewalkPolys[SegTileKey(r.Id, arm)] += Piece` (alias-resolved → divided
   pairs share a tile). **Both sides, same key — theft impossible by construction.**

Junction sidewalk corners: `Band[z] ∩ ZoneJunctionCarve[z][i]` → `JunTileKey`.
Residual band slivers (weld/blend leftovers): tiny, cosmetic — merge into the
nearest already-populated neighbouring tile (slivers only; never a whole walk).

### B — Per-arm SURFACE + junction tiles
- Arm carriageway: `BuildRoadOutline(Sub)` − `ZoneJunctionCarve[z]` →
  `TileSurfacePolys[segKey]`.
- Junction: `ZoneSurfacePolys[z] ∩ ZoneJunctionCarve[z][i]` → `TileSurfacePolys[junKey]`
  (keeps today's disc/close blend look, owned by the junction tile).
- Residual (blend slivers outside carve): same nearest-populated-tile sweep as A.
- Merged `ZoneSurfacePolys` is NOT deleted — terrain conform + junction-clip
  derivation still read it. It just stops being the tiling source.

### C — Delete the guessing machinery
Remove from `CommitLayer`: `RouteForPiece`, `ArmForPieceOnRoad`, `RouteToRoad`,
`MedialCutPolys`, `DepositHalf`, the fused-piece vote, `NearestRoadAt` polygon
routing. Surface + Sidewalks layers consume the pre-bucketed `Tile*Polys` maps.
Meshing, per-tile accumulation, windowing (`IsTileInCommitScope`) unchanged.
Point layers (furniture, signals) keep `TopoKeyOf(point)` — correct for points.

### D — Downstream layers by arm, not by space
Markings (white/yellow), median, bike, parking are already generated per road:
route each generated poly by the arm run containing its source samples
(junction-clipped parts → junction tile). Curbs follow their sidewalk's tile.

### E — Self-check (ponytail rule: one runnable check)
`[TILECHK]` becomes an assertion of construction: every seg tile with a
carriageway whose road has walks enabled on both sides must contain sidewalk
polys on BOTH sides of its centreline (signed-offset test on poly centroids).
Log `[TILECHK] seg tiles: N, both-sides OK: N, missing-side: 0` — a nonzero
`missing-side` names the tile.

---

## 3. Acceptance (user-visible)
- Select any `RoadNet_Seg_*`: carriageway + BOTH its sidewalks + curbs + splines,
  cut perpendicular at each junction carve boundary.
- No road tile spans a junction (through roads at T/X split in two).
- Junctions are their own tiles; segments stop at their boundary.
- Divided road = one tile (both carriageways + median).
- Delete a mid-road point → two tiles.

## 4. Files touched
| File | Change |
|---|---|
| `RoadNetwork.h` | `TileSurfacePolys` / `TileSidewalkPolys` in ctx |
| `RoadNetwork.cpp` | Phase A/B generation; CommitLayer consumes buckets; delete heuristics; arm-routing for downstream layers; TILECHK v2 |
| `RoadNetSurface.*` | reuse only |
