# Junction and lane markings — תקו"ה 2020 chapter 8 compliance plan

Source text: `docs/research/takoh2020_full.txt` (PDF pages 350-401). Urban values
throughout; the generator has no urban/non-urban flag, so every road is treated
as urban (design speed still selects the arrow and 805 tables).

## Defects observed (2026-10-14 screenshots) and their causes

| Defect | Cause in code |
|---|---|
| 809 diamond / chords on plain crosses | `BuildTurnGuides` draws 809 for every far-side turn group |
| Grey squares instead of arrows | `CommitLaneMarks` falls back to `/Engine/BasicShapes/Cube` when no arrow mesh is assigned |
| Arrows in lanes that allow everything | no "approach divided by direction" test (813 rule 2ג) |
| Widened approaches unmarked, centre line off the widened road | turn bays widen the surface (`ArmWidenings`) but `BuildRoadMarkings` paints fixed offsets |
| Centre lines misaligned across a T | same: one arm's far-side bay shifts the opposing lane, nothing marks the jog (809 2ב) |
| Lines run over crosswalks | per-road lines are only cut by the junction clip, not at the stop line |
| Dash pattern 3 m / 5 m, 12 cm | `Paint(LaneDivider)` working values, not table 10 |
| White line along every kerb | edge stroke on the whole surface boundary; 807 is yellow and only where there is no kerb |
| Stop bar 50 cm, zebra 50/40 | `kBarHalfThickCm = 25`, zebra pitch 90 |
| Unmarked junctions | `ResolveJunctionPresetNear` defaults to `None` |

## Rules implemented (all junction types: cross, T, Y/fork, skew, multi-arm, roundabout, dual carriageway)

1. **Paint dimensions** (tables 4, 7, 10, 12, 13): 801 100:200 w10 between junctions; 100:100 on the
   continuing lane approaching a junction; 805 100:50 w10, length by speed (25 m ≤50 km/h … 100 m at 110);
   803 10-10-10; 808 short 100×30 @100:100 over 50 m, long 300×30 @300:150; 809 through 100:100, turn 200:100;
   810 w30; 811 bars 50 gaps 50; 815 frame 10, stripes 20 @ 50 gap, 45°.
2. **Per-road lines end at the stop line** (outer edge of 810, behind the crosswalk when there is one),
   at junctions and at mid-block crossings. Dash phase is measured on the reference line and anchored at
   the stop line, so dashes on parallel lanes line up (801 3ד).
3. **Centre line (opposing traffic)**: 803 on each junction/crossing approach over the arrow section
   (last arrow + 5 m), 805 before it, 801 mid-block; joined into one 803 where the 801 gap would be
   < 150 m. 803 is never painted between same-direction lanes.
4. **Bays and drops** follow `ArmWidenings`: a far-side bay moves the 803 and every line beyond it by the
   bulge; the bay/through boundary gets 808 short rectangles; a dropped lane gets 808 long rectangles and
   814 diagonal arrow clusters (3 arrows 3 m apart, clusters 15 m apart).
5. **Exclusive turn lanes** (lane-graph mask has a turn and no through) next to a through lane: the 801
   boundary is replaced by 808 short rectangles over 50 m from the stop line (808 2ד). Shared lanes keep 801 (808 2ב).
6. **813 arrows**: only on approaches whose lanes are divided by direction; none in a lane that allows
   every movement of its approach; tails at table 1 distances from the stop line (5/15/25/35 m ≤30 km/h,
   5/25/45/65 m ≤50, …), limited to the lane's usable length (bay: full-width part); 75% size under 3.2 m.
   Painted as glyph polygons from fig. 6 when no arrow mesh is assigned; an assigned mesh still wins.
7. **809 only where needed** (809 2א/2ב): turns made from 2+ lanes, any turn at a signalised junction,
   through movements without geometric continuity (lateral jog > 1.5 m or bend > 20°), and the two edges of
   a fork's gore. Paths are curves (quadratic, Hermite S-curve for parallel ends); exit ends on the lane edge.
8. **810 / 811 defaults (`Auto`, the preset of every junction without an override)**: crosswalk on arms with
   sidewalks on both sides; stop line on every entering arm that has a crosswalk or is a minor arm (lower class
   than the junction's best; all arms when classes are equal, Regulation 64). Stop line spans the entering
   lanes plus any turn bay. Explicit presets keep their meaning.
9. **815 painted islands**: unused junction pavement (junction area minus arm corridors, lane-movement
   corridors and built islands) at least 2 m wide and 8 m² gets frame + 45° stripes — the Y gore, skew
   corners and roundabout splitter areas.
10. **807 edge line**: yellow 10 cm, only on carriageway boundary that is not against a sidewalk or a
    built median/island kerb. Median edge lines next to a raised median are dropped (803 rule 2ג).

## Verification

- `RoadNet.StandardsSelfCheck` / `DashSelfCheck` cover the new specs; the 16 RoadNet automation tests run.
- LaneLock baseline re-recorded only if the lane graph changed (markings are not in it).
- Recapture the 34 scenario cameras and compare each junction type against these rules.

## Known simplifications

- Urban tables only. Turn-lane purpose comes from the lane graph (authored roles + `turn:lanes` tokens).
- 815 corner threshold (2 m wide, 8 m²) is a generator rule, not a spec value.
