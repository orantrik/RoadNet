# Israeli Interurban Junction Design Standards — Working Reference

Distilled from Netivei Israel / Ministry of Transport **הנחיות לתכנון גיאומטרי של דרכים בין-עירוניות**:

- **כרך 2 — תכן גיאומטרי של צמתים**, 2nd edition, November 2023, 355 pages. Primary source for everything junction-related.
- **כרך 1**, 04/2018. Source for cross-section, horizontal alignment, vertical alignment, and road classification values that the junction work depends on.

This file exists so that the 355-page junction volume does not have to be re-read. Every value below carries the section or table it came from. Values that the source extraction left ambiguous are marked **UNCERTAIN** rather than smoothed over — see [Appendix B](#appendix-b--index-of-uncertain-items-and-gaps) for the full list.

> **Provenance.** This document is mostly a synthesis of agent transcripts that read the PDFs, not of the PDFs directly, so where two transcripts disagree both readings are shown.
>
> **The raw extractions were NOT deleted** — an earlier note in this file said they were, which was wrong. They were moved out of `Saved/` (Unreal scratch, which gets wiped) to **`docs/research/`** beside this file: 43 files, including the per-chapter PyMuPDF dumps `chapter2/3/4/6/7/8/11_full.txt` and the scripts that produced them. Re-verify any **UNCERTAIN** item against those before acting on it — [Appendix B](#appendix-b--index-of-uncertain-items-and-gaps) item 1 has already been resolved that way.

---

## 0. The single most important finding

**Per כרך 2 §5.2.1, the through-lane count inside a junction must never fall below the upstream count, and lanes must NOT be narrowed at a junction.**

Verbatim (§5.2.1):

> מספר הנתיבים ההמשכיים בתוך הצומת **לא יפחת** ממספר הנתיבים הקיים בקטע הכביש **לפני הצומת** (כדי למנוע ניגודי התמזגות לקראת הצומת).

> **אין לבצע הצרת נתיבים בצומת** ויש לשמור על החתך הרגיל לרוחב גם באזור הצומת.

> רוחב הנתיבים ההמשכיים בקטע הישר בתחומי צומת לא מרומזר, **לא יפחת** מרוחב הנתיבים בזרוע הכביש לפני הצומת.

Three consequences that govern any generator:

1. **Lane count only changes outside the junction.** Extra through lanes added for capacity are added on the approach and dropped after the exit — never inside the conflict area.
2. **The lane added or dropped is always the rightmost** (§5.2.4), so that left-side lanes (left-turn storage in particular) are never severed:
   > בכל מקרה של הוספת נתיב המשכי לתנועה ישר לקראת הצומת המרומזר או הפחתתו אחריה, **הנתיב שייקטע או יתווסף עקב השינוי יהיה הנתיב הימני ביותר**
3. **Width mismatch between arms is resolved by channelization, not by blending.** Two arms of different width do not get merged into a tapered blob. Each arm keeps its own full cross-section into the junction; the difference is absorbed by islands, dedicated turn lanes, and turn-lane tapers (§5.2.4, §5.5, Ch. 6 §6.5, Ch. 7 §7.4). Chapter 5 contains **no** "4-lane arm meets 2-lane arm" merge formula, because the guideline does not treat that as a merge problem at all.

**Implementation notes:** the junction polygon should be built as the union of full-width arm ribbons plus channelizing geometry, never as a boolean blend of narrowed centrelines. Reject any generated junction whose interior through-lane width is less than `min(upstream lane widths)` on that arm.

> **UNCERTAIN — one transcript disagrees.** The Chapter 3 reader glossed §3.3.2 as saying that at signalised junctions the designer "may narrow through lanes slightly," while unsignalised junctions must not. That gloss directly contradicts the verbatim §5.2.1 Hebrew above, which carries no signalised exemption. Treat the §5.2.1 text as authoritative and the Chapter 3 gloss as a probable extraction error, but verify on internal page 3-16 if lane narrowing is ever implemented.

---

## Source map — which chapter holds what

| Chapter (כרך 2) | Internal pages | Subject | Covered in |
|---|---|---|---|
| 2 | 2-1 … 2-39 | Definitions, conflict points, control/channelization classes, **design vehicles** | [§6](#6-design-vehicles-כרך-2-פרק-2) |
| 3 | 3-1 … 3-53 | Junction types, **class-pair matrix**, stagger, split junctions | [§1](#1-junction-type-selection-stagger-and-angle) |
| 4 | 4-1 … 4-30 | Sight distances, sight triangles, critical gaps | [Appendix A](#appendix-a--sight-distance-and-sight-triangles-כרך-2-פרק-4) |
| 5 | 5-1 … 5-19 | **Lane continuity**, channelization principles, islands, kerbs | [§2](#2-lane-continuity-and-lane-addition--drop) |
| 6 | 6-1 … 6-22 | **Right-turn design** — radii, deceleration, acceleration, triangular islands | [§3](#3-right-turn-design-כרך-2-פרק-6) |
| 7 | 7-1 … 7-32 | **Left-turn design** — tapers, deceleration, storage, teardrop islands | [§4](#4-left-turn-design-כרך-2-פרק-7) |
| 8 | 8-1 … 8-20 | **Junction elevation**, grades, vertical/horizontal curves at junctions | [§5](#5-junction-elevation-design-כרך-2-פרק-8) |
| 11 | 11-1 … 11-39 | **Roundabouts** | [§7](#7-roundabouts-כרך-2-פרק-11) |

### Warning: table numbers collide between volumes and chapters

This bites hard. Three separate tables are called "Table 3.1" and two are called "Table 5.1":

| Citation used here | Actual content |
|---|---|
| `כרך 2 Table 3.1` | Class-pair junction-type matrix |
| `כרך 1 §3.2.2 Table 3.1` | Lane width by road class and design speed |
| `כרך 2 Table 5.1` | Minimum post-junction continuous-lane length before taper |
| `כרך 1 §5.2.1 Table 5.1` | Minimum horizontal curve radius, e_max, side friction |
| `כרך 2 Table 2.4` | Design vehicle dimensions |
| `כרך 1 §2.2 Table 2.4` | Design speed ranges by road type |

Every citation below states the volume when there is any risk of confusion.

---

## 1. Junction type selection, stagger, and angle

### 1.1 Table 3.1 — class-pair junction matrix (כרך 2 §3.1.2)

Caption: *טבלה 3.1: מידרג מפגשי הדרכים בין-עירוניות והממשק עם עורק עירוני ורחוב מאסף בשלבי הפיתוח הסופי*

The matrix is indexed by the functional class of both meeting roads and returns the permitted meeting form **at final development stage** (§3.1.2).

Row/column classes, high to low:

| Hebrew | English |
|---|---|
| מהירה | Freeway / expressway |
| מעויירת מהירה | Suburban freeway |
| ראשית | Primary (always dual carriageway at final stage — footnote 4) |
| אזורית | Regional / collector |
| מקומית וגישה לישוב | Local + settlement access |
| עורק עירוני | Urban arterial |
| רחוב מאסף | Urban collector street |

Cell abbreviations:

| Hebrew | English |
|---|---|
| מחלף מערכת | System interchange (freeway–freeway) |
| מחלף גישה | Access interchange |
| מחלפון זעיר | Mini-interchange (§3.1.5) |
| מיפרד | Grade separation, no connection |
| דרך שירות | Service road |
| צומת (מרומזר) | Signalised at-grade junction |
| צומת (מ.מ.) | Fully channelized at-grade |
| צומת (מ.ח.) | Partially channelized at-grade |
| צומת (מ.ב.מ.) | Unchannelized at-grade |
| מעגל | Roundabout |
| הסתעפות בפניות ימניות (מ.מ.) | Right-in / right-out only, fully channelized |

**Matrix** (higher class as row, lower as column; "—" = same class or upper triangle):

| Row \ Col | Suburban FW | Primary | Regional | Local/Access | Urban Art. | Collector |
|---|---|---|---|---|---|---|
| **Suburban FW** | System IC | — | — | — | — | — |
| **Primary** | System IC | System IC | — | — | — | — |
| **Regional** | System IC or Access IC | System IC or Access IC | Access IC | — | — | — |
| **Local/Access** | Grade sep. or service rd | Grade sep. or service rd | Access IC⁽⁵⁾ or grade sep. | Access IC⁽³⁾ or mini-IC⁽⁵⁾ or junction (sig. / מ.מ. / RIRO) | — | — |
| **Urban Art.** | Grade sep. | Access IC⁽³⁾ or grade sep. | Access IC⁽³⁾ or junction (sig. / מ.מ.) | Mini-IC or RIRO (מ.מ.) or service rd | Mini-IC or RIRO (מ.מ.) or service rd | — |
| **Collector** | Grade sep. | Grade sep. | Grade sep. | Access IC⁽³⁾ or junction (sig. / roundabout) | Access IC⁽³⁾ or mini-IC or junction (sig. / מ.מ. / roundabout) | Junction (מ.מ. / מ.ח. / מ.ב.מ. / roundabout) |

Local × Local and equivalent low-tier pairs resolve to `צומת (מ.מ., מ.ח., מ.ב.מ., מעגל)` — any standard at-grade control level per Table 2.1.

**Footnotes to Table 3.1:**

| # | Rule |
|---|---|
| 1 | Road class definitions are in כרך 1 §2.1.3 |
| 2 | Control/channelization types come from Table 2.1; island and lane layout from Chapter 5 |
| 3 | Interchange justification per §3.1.3–3.1.4 |
| 4 | **A primary road (דרך ראשית) in this table is always dual carriageway at final stage** |
| 5 | Mini-interchange on a dual primary per §3.1.5 |
| 6 | A primary-road meeting becomes an access interchange at final stage; a signalised junction is acceptable in interim stages |
| 7 | A **single-carriageway** primary may use a roundabout against an equal or lower class |
| 8 | Two regional roads are both dual carriageway at final stage |

Caveat in the section text (§3.1.2): the matrix is *principled, not absolute* — «חלוקת הצמתים לפי טבלה 3.1 הינה עקרונית, ומייצגת מקרים כלליים ואופייניים». Interchange versus mini-interchange in particular can be overridden by project cost and staging.

**Implementation notes:** this is a direct 2-D lookup `junctionType = Table31[classA][classB]` after mapping OSM `highway=*` to the five Netivei classes. It is the first gate — if it returns an interchange or grade separation, no at-grade junction geometry should be generated at all.

### 1.2 Junction angle

| Rule | Value | Source |
|---|---|---|
| **Preferred angle** | **90°** | כרך 2 §3.2.1א («הזווית המומלצת ביותר עבור צמתים היא 90°») |
| **Acceptable range** | **70°–110°** | §3.2.1א, §3.4; also §2.3.1 (T / "קמץ") and §2.3.2 (4-arm) |
| Below 70° | Treated as a complex junction needing special solutions | §3.2.1א |
| Y / "ע" / "צ" forms | Adjacent arms **> 110° or < 70°** | §2.3.1, Fig. 2.8 |
| Junction angle measurement | Clockwise, with traffic, from the **major** road to the intersecting road | §2.2.1(c), Fig. 2.2 |

Skew penalties listed in §3.2.1א: larger conflict area, longer clearing time, degraded sight lines, weaker priority clarity.

Remedy when a side road meets a curved main road at a skew: realign the side road (תוואי ישן → תוואי חדש) to meet at approximately 90° (תרשים 3.4).

> **UNCERTAIN — a second angle floor exists.** כרך 2 §4.3.5ב states the junction angle should not be **< 75°**, for sight-triangle reasons. That is stricter than the 70° floor in §3.2.1. Both were extracted from the same volume. Safest interpretation: 70° is the geometric limit, 75° is the sight-check limit, so a 70–75° junction needs an explicit sight-triangle verification. Not confirmed against the source text.

### 1.3 Vertical placement of the junction (§3.2.2)

| Location | Rule |
|---|---|
| **Sag (concave) vertical curve** | **Preferred** — junction visible from a distance; check drainage against §8.4.2 radii |
| **Crest (convex) vertical curve** | **Avoid on the main axis**: «יש להימנע ממיקום צומת בפסגת עקום אנכי קמור — בציר הראשי» |
| Crest unavoidable | Move the junction off the crest (תרשים 3.5); radii per Chapter 8 |
| Crest and main-axis radii unachievable | **Right turns only**, and the right-turn departure must begin **before** the vertical curve |

### 1.4 Staggered junctions (§3.7)

A stagger replaces a 4-arm cross with two opposite T-junctions on the main road. It only qualifies as a stagger if there is a **functional link** — traffic actually crosses from one side road to the other via the main segment. Two unrelated T-junctions that happen to be near each other are two separate junctions, not a stagger (§3.7.1).

| Rule | Value | Section |
|---|---|---|
| **Maximum stagger span** (beyond this they are two separate junctions) | **250 m** | §3.7.1 — «כאשר המרחק בין ההסתעפויות עולה על 250 מ' … נחשבות … הסתעפויות נפרדות» |
| **Minimum spacing — unsignalised right stagger, undivided main** | **60–80 m** | §3.7.5ג — «ניתן להסתפק במרחק של 60–80 מטר» |
| **Minimum spacing — same, dual-carriageway main** | **100–120 m** | §3.7.5ג — «יש להאריך … ל-100–120 מ'» |
| **Minimum spacing — left stagger** | No explicit minimum stated | §3.7.5 — instead, confirm the deceleration length for the left turn off the main road (Table 7.1, §7.6–7.8) |

Stagger types (Fig. 2.11):
- **דירוג ימני (right stagger)** — main through traffic passes the right-turn arm first, then the left-turn arm.
- **דירוג שמאלי (left stagger)** — main through traffic passes the left-turn arm first.

**Preference:** right stagger is generally more efficient for crossing capacity (Table 3.3); switch to left stagger when the left-turn storage between the two T's in a right stagger cannot be accommodated.

**Table 3.2 — conflict-point reduction (§3.7.2):**

| Junction type | Crossing | Merging | Diverging | **Total** |
|---|---|---|---|---|
| הצטלבות (4-arm cross) | 16 | 8 | 8 | **32** |
| צומת מדורג (either stagger) | 6 | 6 | 6 | **18** |

Straight-through crossing conflicts drop from **4 to 2**.

**Table 3.3 — stagger form comparison (§3.7.4):**

| Feature | Right stagger | Left stagger |
|---|---|---|
| Left-turn storage on main | Requires widening **between** the T's | Widening **outside** the shared segment |
| Effect of short spacing | Hurts left storage / causes weaving | May disturb the green wave |
| LOS for left turn off main | Higher delay (crossing volume added) | Lower capacity (opposing traffic) |
| Delay caused by secondary road | **Lower** | — |
| Crossing capacity from secondary | **Higher** | — |

**Implementation notes:** when a generator converts an OSM 4-way node into a stagger, the offset distance must land in [60, 250] m for an undivided main road and [100, 250] m for a dual main road. Below the lower bound the layout is non-compliant; above 250 m it should be emitted as two independent T-junctions with no shared geometry.

### 1.5 Four-arm junctions and split junctions

**§3.5:** an interurban 4-arm cross that is **not signalised is not acceptable** — «צומת בין-עירוני בצורת הצטלבות לא מרומזרת אינו רצוי». The stated alternatives are a roundabout, a stagger, or movement restriction (§3.1.3). A signalised cross must always be **fully channelized with built islands** (§3.3.4, Table 2.1).

**Split intersections (§3.8)** replace one cross with two or four signalised sub-junctions, with the main carriageways separated by tens of metres. Square split: cycle time **50–70 s** versus roughly **150 s** for the equivalent single cross; typical left-turn storage **60–70 m** (§3.8.4).

Formula **[3.1]** — minimum storage length of the middle segment (§3.8.5):

| Symbol | Meaning | Units |
|---|---|---|
| `ST_min` | Minimum storage length | m |
| `VOL` | Traffic volume accumulating in the storage lane | veh/h/lane |
| `C_max` | Maximum cycle time | s |
| `K_p` | Poisson coefficient from **תרשים 3.22** | dimensionless, 0.00–1.05 |
| `I` | Average vehicle length | **6.0 m** |

> **UNCERTAIN — formula [3.1] was extracted two incompatible ways.** The Chapter 3 reader recorded `ST_min = (P × VOL × C_max) / (3600 × K_p × I)`; the Chapter 7 reader recorded `ST_min = (3600 × K_p × I) / (VOL × C_max)`. These are close to reciprocals of one another and **neither is dimensionally correct** — the first yields veh/(m·h·s) and the second yields m²·s/veh. Dimensional analysis of the stated variables gives `ST_min = (VOL · C_max · I) / (3600 · K_p)`, which produces metres and matches the chart behaviour described in §3.8.5 (lower failure percentage → lower `K_p` → longer design storage). **Use the reconstructed form only after checking internal page 3-42.** The RTL PDF extraction is known to scramble subscripts on that page.

**תרשים 3.22 axes:** X = average storage `ST` per lane, 0–160 m in 10 m steps; Y = `K_p`, 0.00–1.05 in 0.05 steps; curves for failure percentages 5, 7.5, 10, 15, 20, 25 %.

Split-junction driver orientation (§3.8.6): advance signing at **200–300 m** (up to **500 m** for sign 601); directional signs 604/613 at **350/250 m** and **150 m**, repeated between sub-junctions; physical median separation of **≥ 2 lanes** or a **3 m** pedestrian crossing.

### 1.6 Minimum spacing between junctions (כרך 2 Table 2.2)

| Road type | Condition | Minimum spacing |
|---|---|---|
| Arterial | Between successive junctions | **400 m** |
| ראשית / אזורית | Full detour at 80–90 km/h design speed | **500 m** |
| ראשית / אזורית | Partial detour | **300 m** |
| מקומית with public transport access | With transit access | **150 m** |

---

## 2. Lane continuity and lane addition / drop

Source: **כרך 2 פרק 5** (internal pages 5-1 … 5-19). The core rule is quoted in [§0](#0-the-single-most-important-finding) above.

### 2.1 Definitions (§5.1.1)

**נתיב המשכי (continuous / through lane):**
> הנתיבים ההמשכיים בצומת הם הנתיבים הבסיסיים המיועדים לנסיעה ישר, והם בעלי המשך רציף בתוך הצומת.

**נתיבי-עזר (auxiliary lanes):**
> נתיבי-העזר הם הנתיבים המתווספים לכביש באזור הצומת, מעבר למספר הבסיסי של הנתיבים ההמשכיים העוברים באותו כביש, ומיועדים לשרת את התנועות הפונות.

Auxiliary lanes are **additional**, never substitutes for through lanes. A bus-priority lane (נת"צ) counts as an additional continuous lane at the junction but is **not** counted in the base lane number (§5.2.1).

### 2.2 Lane-count rules by junction type

| Rule | Section | Content |
|---|---|---|
| Through count inside junction ≥ upstream count, on **each** arm | §5.2.1 | The core rule |
| No narrowing anywhere in the junction area | §5.2.1, §5.2.4 | Keep the normal full cross-section |
| At an **unsignalised** T, do **not add** through lanes beyond the guideline count | §5.2.1 | «בהסתעפות לא מרומזרת, השיקול הבטיחותי מכתיב למתכנן לא להוסיף נתיבים המשכיים מעבר לאלה המומלצים» |
| Unsignalised T, **main road**: through count **equals** base upstream count | §5.2.2 | Dedicated turn lanes only when warranted per §5.3 |
| Unsignalised T, **minor road**: **maximum one lane** at the stop line | §5.2.2 | «בציר המשני, אין לתכנן יותר מנתיב אחד במקביל בקו העצירה» — so that queued vehicles do not block each other's sight field |
| Unsignalised T, minor road with warranted turn lanes | §5.2.2 | May have **2** lanes: right turn with a triangular island, plus one other |
| Lane add/drop side | §5.2.4 | Always the **rightmost** lane |
| Lane add/drop location | §5.2.4 | **Not adjacent** to other conflict points — turn-lane openings, acceleration lanes, or bus stops |
| Approach widening | §5.2.4 | Follows the same rightmost-lane logic; position also depends on required through-movement storage from the signal plan |

### 2.3 Signalised lane caps (§5.2.3)

| Road type | Max through (straight) lanes | Max left-turn lanes | Right turn |
|---|---|---|---|
| **Dual carriageway**, base 2 lanes/direction | **3** | **3** | Auxiliary lane |
| **Single carriageway**, base 1 lane/direction | **2** | **1** dedicated | Auxiliary lane |

Verbatim, dual carriageway:
> אין להרחיב את הגישה לצומת מרומזר **מעבר לשלושה נתיבים לנסיעה ישר** ו**מעבר לשלושה נתיבים לפנייה שמאלה**

Verbatim, single carriageway:
> אין להגדיל את מספר הנתיבים המיועדים לתנועה הממשיכה ישר בצומת המרומזר **מעבר לשניים**, וכן ניתן להוסיף **נתיב ייעודי אחד לפנייה שמאלה** (אך לא מעבר לכך), **ונתיב-עזר לפנייה ימינה**

Rationale given in §5.2.3: longer signal clearance intervals, longer pedestrian crossings, difficulty merging back to the normal cross-section, and driver disorientation. The section instructs the designer to prefer the minimum number of lanes that works.

**Implementation notes:** clamp generated approach lane counts to these caps before laying out geometry. A generator that naively sums OSM turn-lane tags will routinely exceed them.

### 2.4 Table 5.1 — post-junction continuous-lane length and taper (כרך 2 §5.2.4)

Caption: *אורך מזערי לנתיב ההמשכי במוצאי צומת מרומזר עד לתחילת סגירתו בלוכסן (מ')*

This is the length `l` for which the extra through lane must stay at **full width** after the junction exit, before its closing taper may begin.

| Design speed (km/h) | `l` after **full** acceleration lane (2 s) | `l` after **shortened** acceleration lane (3 s) | `l` with **no** acceleration lane (4 s) | Taper gradient `d` |
|---|---|---|---|---|
| **80** | **45** | **65** | **90** | **1:40** |
| **90** | **50** | **75** | **100** | **1:45** |
| **100** | **55** | **85** | **110** | **1:50** |

**Footnotes:**
1. For design speeds **60 and 70 km/h**, use the **80 km/h** row.
2. *Full* acceleration lane: ≥ **2 seconds** of travel from the end of the acceleration-lane taper to the start of the next change (Ch. 6 §6.4.2).
3. *Shortened* acceleration lane: ≥ **3 seconds** from the end of the acceleration-lane taper (Ch. 6 §6.4.3).
4. No adjacent acceleration lane (T-junction on the continuous-road side, or no triangular island plus acceleration lane): ≥ **4 seconds** before the closing taper begins.

Underlying relation, reconstructed from the footnotes:

```
l  =  V × t / 3.6            [V in km/h, t = 2, 3 or 4 s, l in m]
d  =  1 : V_d                [1 m of lateral shift per V_d metres of length]
```

Check: `80 × 2 / 3.6 = 44.4 → 45`; `80 × 4 / 3.6 = 88.9 → 90`. Consistent. Taper gradients 1:40, 1:45, 1:50 are exactly `1 : V_d/2`… no — they are `1 : (V_d × 0.5)`, i.e. 40 at 80 km/h. Note this is **the same numeric rate as the Chapter 7 centreline shift `1:(Vd/2)`** described in [§4.2](#42-how-the-left-turn-lane-is-formed-736).

**Implementation notes:** after emitting a junction, each arm that carries a dropped through lane needs a straight full-width run of `l` metres followed by a linear taper of `d × laneWidth` metres. At 80 km/h with a 3.6 m lane and a full acceleration lane that is 45 m straight plus 144 m of taper — nearly 190 m of post-junction geometry. Arms shorter than that cannot legally drop a lane and should keep the lane through to the next node.

### 2.5 Turn-lane and island warrants (Tables 5.2–5.5)

These are qualitative warrant matrices, not numeric tables.

**Table 5.2 — triangular island + dedicated right-turn lane, unsignalised:**

| Arm | From primary/regional **dual** | From primary/regional **single** | From local/access |
|---|---|---|---|
| **Main (עיקרי)** | Triangular island + dedicated lane | Same | Per operational need + pedestrian/transit |
| **Minor (משני)** | N/A | N/A | Triangular island + lane per operational need |

**Table 5.3 — triangular island + dedicated right-turn lane, signalised:**

| Arm | From primary/regional **dual** | From primary/regional **single** | From local/access |
|---|---|---|---|
| **Main** | Always island + dedicated lane | Always | — |
| **Minor** | Island; full or shortened dedicated lane per operations | Per operational need | — |

Key rule (§5.3.1): a **signalised main arm always gets a dedicated right-turn lane with a triangular island**.

**Table 5.4 — dedicated left-turn lane, unsignalised:**

| Arm | Primary/regional dual | Primary/regional single | Local/access |
|---|---|---|---|
| **Main** | **Yes** | **Yes** | **Yes** |
| **Minor** | N/A | N/A | Per operational need |

**Table 5.5 — dedicated left-turn lane, signalised:**

| Arm | Primary/regional dual | Primary/regional single | Local/access |
|---|---|---|---|
| **Main** | **Yes** | **Yes** | — |
| **Minor** | Per signal plan + operations | Per signal plan + operations | Per signal plan + operations |

A dual-carriageway minor arm should get at least **1** dedicated left-turn lane for signal flexibility.

### 2.6 Acceleration and merge lane warrants (§5.4)

**Acceleration lane after a right turn (§5.4.1), unsignalised T:**

| Condition | Requirement |
|---|---|
| Free right turn into a **dual-carriageway** main road | **Full** acceleration lane (only where no pedestrian crossing exists on the main road, §3.4.3) |
| Right turn into a partially or fully divided main road | **Full** acceleration lane |
| Heavy vehicles **> 10 %** on the main road | **Full** acceleration lane |
| Low main-road design speed; main dualled for safety not volume; minor-arm peak right turn **< 120 veh/h** and main-road LOS **A–C** | **Shortened** acceleration lane permitted |

**Signalised (§5.4.1ב):**

| Condition | Requirement |
|---|---|
| Free right turn into a dual-carriageway main road | **Full** acceleration lane |
| Peak free-right-turn **> 500 veh/h**, or turn volume exceeds the opposing through volume in the signal shadow | **Full** acceleration lane |
| Signalised right turns (stop on red) | **No** post-turn acceleration lane, but design for fallback to signs 301/302 on signal failure |

**Merge lane after a left turn (§5.4.2)** — unsignalised T only:

| Main road class | Required? |
|---|---|
| Primary / regional **dual carriageway** | **Yes** |
| **Single-carriageway** main | **Yes** — supports the two-stage cross-then-merge manoeuvre |
| Without built islands on the main road | **No** merge lane |
| Signalised junctions | **Not required** — the signal controls the merge |

Chapter 5 gives **no** numeric lengths for either. Acceleration lengths are in Ch. 6 §6.4.2/§6.4.3 and Table 6.5; merge lengths are in Ch. 7 §7.3.7.

### 2.7 Islands and kerbs (§5.5, §5.6)

**Island classification by function (§5.5.2):** channelizing/directional (triangular — Ch. 6 §6.5), division/splitter (longitudinal "טיפה" and median — Ch. 7), refuge (Ch. 10).

**By construction:** **built (בנוי)** islands are required wherever there is a pedestrian refuge, sign posts, rigid hazards, or unusual geometry. **Painted (צבוע)** channelization is permitted only in low-tier forced cases, and never where a main-road driver would be surprised after a long unchannelized stretch.

**Table 5.6 — kerb type placement:**

| Location | Kerb type |
|---|---|
| Median (מפרדה) | Sloped (משופעת), except at crossing sides |
| Triangular islands | Sloped, except at crossing sides |
| Longitudinal "טיפה" islands | **Vertical (ניצבת)** |
| At crossing sides | **Vertical** |
| Along footpaths | Vertical |
| At crosswalk level | **Flush (במפלס הכביש)** — 0 cm step |

| Parameter | Value | Section |
|---|---|---|
| Standard kerb height | **15 cm** above road surface | §5.6 |
| Crosswalk kerb | **0 cm** (flush) | §5.6, accessibility regulations |
| Roundabout apron crown | **7 cm** | §5.6, detail in §11.11 |
| Kerb standard | Israeli standard **ת"י 19**, types in תרשים 5.4 | §5.6 |

Minimum island **dimensions** are not in Chapter 5 — triangular islands are in Ch. 6 §6.5.2, longitudinal islands in Ch. 7 §7.4.2.

---

## 3. Right-turn design (כרך 2 פרק 6)

All lengths in metres, speeds in km/h.

### 3.1 Table 6.1 — corner radii for a right turn WITHOUT an island (§6.2, p. 6-3)

Caption: *אפשרויות לתכן שפות המיסעה לפנייה ימינה **ללא אי-תנועה***. Applies to low-tier interurban roads only, at **15–25 km/h** turn design speed, and only where §5.3.1 does **not** warrant a dedicated right-turn lane.

| Turn angle | Vehicle | Simple **R** | Single R + taper | Taper ratio | Taper offset | Symmetric 3-centre R₁-R₂-R₃ | Sym. offset | Asymmetric 3-centre R₁-R₂-R₃ | Asym. offset | Asymmetric alternative |
|---|---|---|---|---|---|---|---|---|---|---|
| **75°** | Bus / car | **20.0** | **13.5** | 1:10 | 0.6 | 40-14-40 | 0.6 | 50-15-70 | 0.6–3.0 | 1:15, off 0.9 → 45-15-45, off 1.8 |
| **75°** | Articulated | **20.0** | — | — | — | — | — | 50-15-70 | 0.6–3.0 | 45-15-45, 1:15, 0.9 |
| **90°** | Bus / car | **15.0** | **12.0** | 1:10 | 0.6 | 40-12-40 | 0.6 | 40-12-60 | 0.6–3.0 | 1:15, 1.2 → 55-12-55 |
| **90°** | Articulated | **18.0** | — | — | — | — | — | 40-12-60 | 0.6–3.0 | 55-12-55, 1:15, 1.2 |
| **105°** | Bus / car | **13.0** | **11.0** | 1:10 | 0.9 | 30-10-30 | 1.0 | 50-12-65 | 0.6–3.0 | 1:15, 1.2 → 55-14-55, 2.4 |
| **105°** | Articulated | **17.0** | — | — | — | — | — | 50-12-65 | 0.6–3.0 | 55-14-55, 1:15, 1.2 |

**Footnotes:** the angle is the central angle between the outer radii of the turn arc; other angles require a check against vehicle tracking diagrams. Angles in the 70°–110° band outside those tabulated are not illustrated and must be verified by tracking. Vehicle definitions are in §2.6.

**Rule for acute turns (§6.2):** «אם זווית הפנייה קטנה מ-90° … רדיוסים גדולים יותר … מומלץ פחות שימוש בעקומים מורכבים» — for angles below 90°, prefer a **larger simple radius** over a compound curve.

**Figure 6.1** gives the four 90° options concretely: (א) simple **R = 15 m** on the bus path; (ב) **R = 12 m** with a **1:10** taper of **12.6 m** each side; (ג) symmetric **40-12-40** with **12.6 m** offset; (ד) asymmetric **40-12-60** with **12.6 m** entry and **15 m** exit for an articulated truck. All four use a **0.5 m** offset from the straight edge to the curve start.

**Vehicle clearance (§6.1):** minimum **0.2 m** wheel-to-edge, and never **< 0.5 m** to the pavement edge at the start or end of the turn.

Four geometric options are permitted (§6.1 א–ד): simple circular arc; single arc on a **1:10** taper; symmetric three-centre compound curve; asymmetric three-centre compound curve.

> **UNCERTAIN — one sentence in §6.2 was extracted inverted.** The transcript records «avoid intersection arm angles 70°–110° where possible», which is the opposite of §3.2.1's rule that 70°–110° is the *acceptable* band. This is almost certainly an RTL negation flip and should read "avoid angles outside 70°–110°". Do not implement the extracted form.

### 3.2 Right turn WITH an island — radii and widths (§6.3.1)

These are the values for a channelized turning roadway and are **different from Table 6.1**, which covers unchannelized corners.

| Parameter | Value |
|---|---|
| Minimum turn radius, 90°–105° | **20 m** |
| Minimum turn radius, 75° | **27 m** |
| Preferred radius at 90° where geometry allows | **25 m** |
| Turning roadway width | **5.5 m** or **6.0 m** |
| Painted shoulder (marking 815) | **0.5 m** |
| Lane width at the inner-arc tangent point | **4.5 m** |
| Entry taper into the turn radius (deceleration side) | **1:30 – 1:60** |
| Exit taper from the turn radius (acceleration side) | **1:40 – 1:60** |
| Private-car turn design speed | **≤ 25 km/h** |

Reduced turning-roadway width below 5.5/6.0 m is permitted only where a WB-15 semi is not expected (§6.3.1).

### 3.3 Table 6.2 — deceleration length (§6.3.2, p. 6-9)

Caption: *האורך (L) הדרוש להאטה בנתיב לפנייה ימינה בצמתים*. Valid for grades **< 2.5 %**.

| Approach design speed V (km/h) | To **stop** (m) | To **25 km/h** (m) |
|---|---|---|
| **50** * | 36 | 24 |
| **60** | 50 | 38 |
| **70** | 69 | 57 |
| **80** | 95 | 82 |
| **90** | 114 | 102 |
| **100** | 139 | 127 |

\* 50 km/h is relevant for urban-street connections and ramp terminals.
"To stop" means to the crosswalk or the signal stop line.

**Design assumptions behind the table (§6.3.2א):**

| Parameter | Value |
|---|---|
| Actual entry speed | **85 %** of approach design speed |
| Turn design speed | **25 km/h** (private car, the dominant volume) |
| Deceleration rate | **2.0 m/s²** |
| Reaction time | **3.0 s** |
| Lane-change length | Already **included** in the tabulated deceleration length |
| Driver eye height | Car **1.05 m** versus truck **2.4 m**, so a lower approach speed is expected for trucks |
| Absolute minimum lane length for the lane change at 50 km/h | **35 m** |
| Austroads (Part 4A, 2017) lateral shift rate | **1.5 m/s** |

### 3.4 Table 6.3 — grade correction factors (§6.3.2)

Multiply the Table 6.2 value by:

| Grade along the deceleration length | Uphill (עלייה) | Downhill (ירידה) |
|---|---|---|
| **2.5 % – 4.4 %** | **0.90** | **1.20** |
| **4.5 % – 6.0 %** | **0.80** | **1.35** |

Use 4 % for grades ≤ 4.5 % and 5 % for grades above 4.5 %.

```
L_decel_corrected = L_Table6.2(V, target) × f_Table6.3(grade)
L_total           = L_decel_corrected + L_storage     [storage only if signalised / crosswalk stop]
```

Right-turn storage length is calculated with the **left-turn** method of Ch. 7 §7.3.5, adapted (§6.3.2ב). Deceleration is measured from the point where the lane reaches **3.0 m** width to the start of the turn radius (§6.3.2ג).

**Implementation notes:** the 3.0 m measurement anchor matters. A generator that measures from the taper start will produce lanes roughly 6/7 of a taper length too short.

### 3.5 Parallel versus diagonal deceleration lanes (§6.3.3–§6.3.5)

| Type | Use when |
|---|---|
| **Parallel** (נתיב מקביל, §6.3.4) | **Main axis, signalised** — prevents right-turn queues blocking through traffic. Also the preferred form on the **secondary axis at any junction**. |
| **Diagonal** (נתיב אלכסוני, §6.3.5) | **Main axis, non-signalised** (T-branch) — follows the natural driver path. Switch to parallel if the right-turn volume is high. |

**Parallel lane opening tapers (§6.3.4):**

| Axis | Condition | Minimum taper |
|---|---|---|
| Main (primary) | V ≤ 70 km/h | **1:10** |
| Main (primary) | V ≥ 80 km/h | **1:15** |
| Secondary | General | **1:10** |
| Secondary | Dual carriageway, signalised | Same as main, per design speed |

`L_taper = ratio × W`, with W typically **3.60 m**. Layout: approach lane width W → opening taper → parallel segment at constant width → widen to **4.5 m** at the turn-radius tangent point. Deceleration is measured from the **3.0 m** width point.

Figure 6.4א dimensions: main axis signalised uses **1:15** and widths 3.00 → 3.50 parallel → **4.50** at the radius; secondary unsignalised uses **1:10**, 3.00 at taper end → **4.50** at the radius, with a lowered kerb at the corner.

### 3.6 Table 6.4 — diagonal-lane taper and reverse-curve radius (§6.3.5, p. 6-12)

Caption: *היסט הלוכסן בכניסה לפנייה ימינה בנתיב אלכסוני מהציר הראשי (העיקרי), ורדיוס עקומים מנוגדי כיוון*

| Main-axis V_d (km/h) | Taper ratio | Reverse-curve radius R (m) |
|---|---|---|
| **≤ 60** | **1:30** | — |
| **70** | **1:40** | — |
| **80** | **1:40** | **250** |
| **90** | **1:50** | **350** |
| **100** | **1:60** | **450** |

For **V_d ≥ 80 km/h** a straight taper alone gives insufficient deceleration and visibility, so **symmetric reverse curves** must be added (Figure 6.5, laid out in three ℓ/3 segments with taper slopes 1:40 and 1:50 on the middle segment).

Figure 6.4ב: taper ≥ **1:30**, width **3.00 m** at the taper end, **4.50 m** at the radius start, deceleration length measured between those two points.

### 3.7 Shortened storage-only right-turn lane (§6.3.6)

Permitted where a right turn has a triangular island but a full dedicated approach lane is not warranted (Tables 5.2, 5.3).

| Parameter | Value |
|---|---|
| **Minimum storage length** | **20 m** (bus plus private car, or one articulated vehicle) |
| Verification | AUTOTURN or equivalent for the design heavy vehicle at **15–20 km/h** |
| Preferred geometry | Three-centre compound curve rather than simple radius plus tapers, to minimise asphalt (Figure 6.1) |

### 3.8 Table 6.5 — full acceleration length (§6.4.2, p. 6-16)

Caption: *האורך הדרוש להאצה (L) בנתיב האצה מלא לאחר פנייה ימינה בצומת (במישור)*

| Main-road design speed (km/h) | L from 25 km/h (m) |
|---|---|
| **70** * | **100** |
| **80** | **140** |
| **90** | **170** |
| **100** | **210** |

\* Single carriageway in **mountainous** terrain.

| Rule | Value |
|---|---|
| Reduced requirement, dual carriageway with topographic constraints | **170 m** at 90 km/h; **140 m** at 80 km/h |
| Uphill grade **> 4 %** | Multiply by **1.5** (100 km/h → ≈ **320 m**) |
| Downhill | **Do not shorten** |
| Minimum to qualify as a "full" acceleration lane | **≥ 140 m** |
| Maximum longitudinal grade | **6 %** (Chapter 8) |

**Design assumptions (§6.4.2):** entry from the free right turn at **25 km/h**; mean acceleration **1.2 m/s²**; merge speed **85 %** of the main-road design speed; closing taper begins where the lane width is **3.0 m**.

**Parallel geometry (§6.4.2ב):** start at the turn exit at **4.5 m** width; parallel section **3.6 m**; closing taper **1:20 – 1:35**; L is measured from the turn end (4.5 m) to the 3.0 m width point. Figure 6.7 adds: parallel width 3.50 m, island offset 2.50 m.

**Shortened acceleration lane (§6.4.3)** — diagonal, not parallel:

| Parameter | Value |
|---|---|
| Total length | **90 – 105 m** (including a bus stop if required, Ch. 10) |
| Minimum recommended | **≥ 90 m** |
| Taper ratio | **1:60 – 1:70** |
| Width | **4.5 m** at the turn-radius end → **3.0 m** at the merge |
| Entry connection taper | **1:40 – 1:60** (Figure 6.3א) |
| Grade adjustment | **None** — the shortened taper is a fixed ratio, unlike the full lane |

Figure 6.8 (signalised): provisional **150–170 m** where the design speed exceeds 80 km/h; taper transition **35 m**; full-width segment **18 m**; lane width **3.60 m**.

**Implementation notes:** `Table 6.5` is the value that feeds `Table 5.1`'s "full acceleration lane" column. A generator that emits an acceleration lane shorter than 140 m has, by definition, emitted a *shortened* lane, and must then use the 3-second column of Table 5.1 rather than the 2-second column.

### 3.9 Triangular island minimum dimensions (§6.5.2)

| Parameter | Value |
|---|---|
| **Minimum area** | **10 m²** (absolute minimum **9 m²** in existing constrained junctions) |
| **Minimum side length** | **≥ 4.0 m**, and ≥ crossing width + **0.5 m** each side if a pedestrian/bike crossing (marking 812) passes over it |
| **Large island** | Side **≥ 30 m** — for high-speed right turns and non-signalised junctions |
| Construction | Large islands may be landscaped; all others are **built up**; interurban junction islands are **illuminated** at night |

**Geometric detail of the channelizing island (§6.5.3):**

| Rule | Value |
|---|---|
| Nose recession from the pavement-edge projection | **≥ 0.6 m** |
| Gore area between built island and lane | Paint marking **815** |
| Small island corner radius | **≥ 0.5 m** |
| Medium island, main-axis corners | **0.6–0.9 m** and **0.6–0.75 m** |
| Medium island, secondary-axis corner | **0.3–0.5 m** |
| Sign posts | **≥ 1.5 m** from the built island edge (§5.6) |

**Figure 6.9 offsets from the travelled way (medium island):**

| Location | Offset |
|---|---|
| Vertex **C** (straight/right split, main axis) | **1.2 m** — gives the driver sight margin to the kerb |
| Vertex **B** (downstream, main axis) | **0.6 m** |
| Side **A–B** (secondary axis) | **0.9 m** |
| Right-turn lane entry and exit | **0.9 m** |
| Right-turn lane apex | **0.5 m** |
| Marking extension gap at nose A | **0.5 – 1.0 m** |

**Mandatory companion island (§6.5.1):** where a two-way single-carriageway arm enters through a triangular island, an **elongated (teardrop) island** or a centre divider is mandatory, parallel to triangle side **AB** and protruding beyond the triangle on the secondary axis. Never place a triangular island without the accompanying elongated island on the adjacent lane.

### 3.10 High-speed turning roadways (§6.3.7)

Design speed above the basic **25 km/h** island turn, with correspondingly larger islands. **Specific radii, widths, and superelevation are not tabulated in Chapter 6** — the section delegates to **כרך 3 §§5.3–5.7** (ramp and interchange principles). Not permitted where right-of-way is constrained, pedestrian/cyclist volumes are high, or public-transport priority lanes are needed.

---

## 4. Left-turn design (כרך 2 פרק 7)

### 4.1 Lane anatomy (§7.3.1, תרשים 7.4)

> הנתיב הייעודי לפנייה שמאלה מורכב משני מרכיבים גיאומטריים עיקריים … נתיב ייעודי לכניסת הרכב לפנייה שמאלה, לוכסן, נתיב האטה, ורצועת אחסנה לרכבים הממתינים לפנייה שמאלה.

| # | Hebrew | English | Length from |
|---|---|---|---|
| 1 | נתיב ייעודי לכניסת הרכב | Dedicated entry lane | — |
| 2 | לוכסן (אלכסון) | Diverge taper | §7.3.2 |
| 3 | נתיב האטה | Deceleration lane | §7.3.3 |
| 4 | רצועת אחסנה | Storage / parallel section | §7.3.4 (unsignalised) or §7.3.5 (signalised) |

**Configuration A** — main road, or main plus secondary, signalised or not: deceleration length (§7.3.3) + storage length (§7.3.4 or §7.3.5).
**Configuration B** — secondary road only, signalised local road: full taper length (§7.3.2) + storage length (§7.3.5).

Length composition (§7.3.2, internal p. 7-8):

```
L_parallel = L_decel + L_storage − L_taper      [configuration A]
L_parallel = L_taper_full + L_storage           [configuration B]
```

Storage is **not** counted along the taper. Deceleration may partly overlap storage off-peak.

### 4.2 Taper ratios and how the left-turn lane is formed (§7.3.2, §7.3.6)

**Taper ratio (§7.3.2), verbatim:**
> השיעור המזערי להיסט לוכסן הנתיב הייעודי לפנייה שמאלה יהיה **1:10** במהירויות תכן עד **70** קמ"ש, ו-**1:15** במהירויות **80** קמ"ש ומעלה.

| Case | Taper ratio |
|---|---|
| Main, or main + secondary, signalised or unsignalised, V_d ≤ 70 km/h | **1:10** |
| Same, V_d ≥ 80 km/h | **1:15** |
| Secondary only, signalised | **1:10** always (no deceleration lane assumed under signal) |
| Secondary unsignalised, taper from the through lane at the stop line | **1:10** |

```
L_taper = ratio × W          [W = parallel left-turn lane width, typically 3.6 m]
```

At W = 3.6 m: **1:10 → 36 m**; **1:15 → 54 m**.

**Four formation methods (§7.3.6):**

| Method | Hebrew | Technique | When |
|---|---|---|---|
| **א** | הרחבה על-פי חשבון | Widen the carriageway by the required additional width | One-way / collector approaches with room to widen |
| **ב** | הסטה הדרגתית של הדרך משני עברי הצומת | **Shift both edges gradually toward the median at rate `1 : (Vd/2)`** | Two-way undivided main road with no extra pavement width |
| **ג** | ציר משני, מרומזר | **1:10** taper to add the left-turn lane plus the through lane, and a further **1:10** taper for the right-turn lane | Signalised T on the secondary leg |
| **ד** | ללא הרחבה | One shared lane serves both left and right turns — no widening | Unsignalised approaches with a single shared lane |

**Centreline shift rate (§7.3.6ב), verbatim:**
> הדרגתית של הדרך משני עברי הצומת, **בשיעור של 1 : (Vd/2)**, כאשר **Vd** היא מהירות התכן של הדרך, בש"מ.

| V_d (km/h) | Rate | Length needed for a 3.6 m shift |
|---|---|---|
| 50 | 1:25 | 90 m |
| 60 | 1:30 | 108 m |
| 70 | 1:35 | 126 m |
| 80 | 1:40 | 144 m |

**Implementation notes:** method ב is the one a road generator will hit most often on rural two-lane roads, and it is the one that changes the *centreline*, not just the edge. The approach centreline must be offset laterally by half the added width over `(Vd/2) × offset` metres on both sides of the junction, symmetrically. Getting this wrong produces a visible kink at the junction approach.

Median/pedestrian add-on when a signalised left turn comes off the main road (§7.3.6): left-turn lane **3.60 m**, pedestrian waiting island **+2.50 m**, cyclists **+3.0 m**, giving roughly **6 m** of median width.

### 4.3 Table 7.1 — deceleration length (§7.3.3)

Caption: *אורך (L) נדרש להאטה עד לעצירה (מ')*. Valid for grades ≤ **3 %**; assumptions inherited from Ch. 6 §6.3.2 and Table 6.3.

> **UNCERTAIN — the two columns were extracted transposed.** As recorded, the table reads design speeds of 35 / 50 / 70 / 95 / 115 / 140 km/h against lengths 50 / 60 / 70 / 80 / 90 / 100 m. That cannot be right: **140 km/h is not an Israeli design speed** (the maximum in כרך 1 Table 2.4 is 120 km/h, and junction arms top out at 100 km/h in Table 8.1). Reading the columns the other way round makes the table land almost exactly on the "to stop" column of `Table 6.2` — 36/50/69/95/114/139 — which is precisely what §7.3.3 says it is derived from. The corrected orientation is presented below; the raw orientation is preserved for traceability.

**Corrected (recommended) orientation:**

| Design speed on main road (km/h) | Deceleration length L to stop (m) |
|---|---|
| 50 * | 35 |
| 60 | 50 |
| 70 | 70 |
| 80 | 95 |
| 90 | 115 |
| 100 | 140 |

\* The transcript footnote attaches "for roundabouts or urban residential connectors only" to the 35 value; under the corrected orientation that footnote most likely belongs to the 50 km/h **speed** row, which is consistent with Table 6.2's footnote on its own 50 km/h row.

**Raw orientation as extracted (do not use):** speed 35 → L 50; 50 → 60; 70 → 70; 95 → 80; 115 → 90; 140 → 100.

No closed-form formula appears in Chapter 7 — it is a lookup. A deceleration lane is **not** required on a secondary-only signalised approach, because traffic waits under a red signal most of the time (§7.3.2).

### 4.4 Storage length — unsignalised (§7.3.4)

Model: AASHTO 2018, via Figure 7.6.

| Parameter | Value |
|---|---|
| Gap acceptance | **85 %** of drivers accept a critical gap of **6.25 s** with **2.5 s** follow-up |
| Service target for a left turn off the main road | **LOS C** equivalent |
| **Failure probability for storage design** | **0.005 (0.5 %)** — note, not 5 % |

**Table 7.2 — number of stored passenger-car units `n`.** Rows = opposing left-turn volume (veh/h, PCU); columns = volume on the main road opposing the left-turn entry (veh/h).

| Opp \ Main | 100 | 150 | 200 | 250 | 300 | 350 | 400 |
|---|---|---|---|---|---|---|---|
| 200 | 2 | 2 | 2 | 2 | 2 | 2 | 2 |
| 250 | 2 | 2 | 3 | 3 | 3 | 3 | 3 |
| 300 | 2 | 2 | 3 | 3 | 4 | 4 | 4 |
| 350 | 2 | 2 | 3 | 4 | 4 | 5 | 5 |
| 400 | 2 | 3 | 3 | 4 | 5 | 6 | 6 |
| 450 | 2 | 3 | 3 | 4 | 5 | 6 | 7 |
| 500 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
| 550 | 2 | 3 | 4 | 5 | 6 | 8 | 9 |
| 600 | 2 | 3 | 4 | 5 | 7 | 9 | 10 |
| 650 | 3 | 3 | 4 | 6 | 8 | 10 | 12 |
| 700 | 3 | 4 | 5 | 6 | 9 | 12 | 14 |
| 750 | 3 | 4 | 5 | 7 | 10 | 14 | 17 |
| 800 | 3 | 4 | 6 | 8 | 11 | 15 | 20 |
| 850 | 3 | 4 | 6 | 9 | 12 | 17 | 23 |
| 900 | 3 | 5 | 7 | 10 | 14 | 20 | 28 |
| 950 | 3 | 5 | 8 | 12 | 17 | 24 | 33 |
| 1000 | 4 | 6 | 9 | 15 | 22 | 31 | 43 |
| 1050 | 4 | 6 | 10 | 19 | 28 | 40 | 55 |
| 1100 | 4 | 7 | 12 | 24 | 35 | 50 | 69 |
| 1150 | 5 | 8 | 15 | 30 | 44 | 63 | 87 |
| 1200 | 5 | 9 | 19 | 38 | 55 | 79 | 109 |

Where `n > 10`, verify capacity and consider an additional lane.

**Table 7.3 — storage length versus `n`** (15 % heavy vehicles; car 6 m, bus 13 m; +1 m gap between stored vehicles):

| n (PCU) | Storage length (m) |
|---|---|
| 1 | 20 |
| 2 | 25 |
| 3 | 31 |
| 4 | 37 |
| 5 | 43 |
| 6 | 56 |
| 7 | 62 |
| 8 | 68 |
| 9 | 74 |
| 10 | 80 |

**Adjustments:** for `n ≥ 5` add **7 m** for 25 % heavy-vehicle overflow, or calculate separately. For `n ≥ 10` consider an additional left-turn lane. **Minimum storage in all cases: 20 m.**

### 4.5 Storage length — signalised (§7.3.5)

Two cases (Figure 7.7): **case א**, the lane holds all left-turn demand but not the through queue behind it, so use the minimum left-turn volume parameter; **case ב**, the lane is too short and through traffic blocks the left-turn storage, so use the maximum left-turn cycle demand. Computation uses the Poisson graph of Figure 7.8, with **5 % failure probability** recommended.

**Table 7.4 — storage length versus `X`** (average left-turning PCU per signal cycle):

| X | Length (m) | X | Length (m) |
|---|---|---|---|
| 1 | 20 | 11 | 80 |
| 2 | 25 | 12 | 86 |
| 3 | 31 | 13 | 92 |
| 4 | 37 | 14 | 98 |
| 5 | 43 | 15 | 105 |
| 6 | 56 | 16 | 111 |
| 7 | 62 | 17 | 117 |
| 8 | 68 | 18 | 123 |
| 9 | 74 | 19 | 129 |
| 10 | 80 | 20 | 135 |

**Adjustments:** `X ≥ 5` → add **7 m**; `X ≥ 15` → add **14 m**; `X ≥ 10` → consider a second left-turn lane and split the storage between lanes. **Minimum storage: 20 m.**

`Table 7.4` and `Table 7.3` share identical values for X = n ≤ 10, which is expected — Table 7.4 simply extends the same vehicle-packing model to 20 units.

**Implementation notes:** without a traffic model, default to `L_storage = 20 m` (the floor) and `L_decel` from the corrected Table 7.1. At 80 km/h that gives a left-turn lane of `95 + 20 = 115 m` total, of which `115 − 54 = 61 m` is full-width parallel section after a 1:15 taper.

The Poisson coefficient **`K_p` does not appear in Chapter 7** — it belongs to Chapter 3 formula [3.1] for split-intersection link storage. Do not mix the two models.

### 4.6 Minimum left-turn radii (§7.2.1)

Not a speed-indexed table — narrative rules plus תרשים 7.1.

| Condition | R_min (m) |
|---|---|
| Default interurban / urban | **15.0** |
| Small junctions with heavy-vehicle queuing | **12.5** (use 15.0 if the teardrop island would have to be moved back) |
| Local / access roads with continuous heavy flow | **15.0** (mandatory) |
| Long articulated vehicles making an S-path at turn start | **12.5** minimum |
| Inner radii of opposing left-turn paths | **15.0** each |

Turn design speeds: **15 km/h** for a bus (WB-12), **10 km/h** for a truck (WB-15).

**Opposed simultaneous left turns (§7.2.3):**
> כאשר שתי פניות שמאלה מתנכסות בו-זמנית … יש לוודא קיום מרווח מזערי של **9.0 מ'** בין הרדיוסים הפנימיים של שתי הפניות, **(15.0 מ')** דרך רדיוסים פנימיים לפנייה.

- **9.0 m** minimum between the **inner** radii of the two opposed paths
- Inner turn radius **15.0 m** on each path
- Guide markings **809** across the turn paths (Figure 7.2)

**Parallel same-direction left turns (§7.2.4):**

| Rule | Value |
|---|---|
| Minimum spacing between inner radii of adjacent same-direction lanes | **5.0 m** |
| Minimum width of each left-turn lane at the exit | **4.5 m** |
| Three parallel left lanes, signalised | Inner two ≥ **4.5 m** at exit; the innermost may be **3.6 m + 0.5 m** paint |
| If R < 15 m | Verify **5 m** between paths and check that **two buses** can pass in the curve |

### 4.7 Entry and exit widths (§7.2.2)

**Entry, main → secondary:** full lane **3.6 m** at turn start, plus up to **+1.0 m** of optional painted shoulder.

**Exit, main → secondary:**

| Condition | Width |
|---|---|
| Single lane between kerbs | **≥ 6.0 m** |
| With marking 815 completion | **5.5 m** lane plus shoulders |

**Exit, secondary → main:**

| Condition | Width |
|---|---|
| Per turn lane at the exit | **≥ 4.5 m** each |
| Two left-turn lanes | **≥ 4.5 m** per lane |
| Single dedicated left, unsignalised, feeding a merge lane | **4.5 m** (§7.3.7) |
| Signalised, ending in a standard lane | **3.6 m** (+ optional 0.5 m painted edge) |

### 4.8 Merge lane after a left turn (§7.3.7)

Warranted only per §5.4.2. Function: a vehicle turning left from the secondary road onto the main road merges right into through traffic.

| Parameter | Value |
|---|---|
| Merge lane length | **60 m** |
| Merge lane width | **3.0 m** |
| Merge start width | **4.5 m** |
| Design speed at the secondary exit | **15 km/h** |
| Taper, undivided main | **1:40** → closure length **180 m** for a 3 m lateral shift |
| High-volume case (Fig. 7.11ג) | Extend taper to **1:60** → main closure **270 m**, allowing the storage taper section to shorten to **180 m**; after **90 m** the taper steepens to **1:30** for the final 3 m |

Median widths on a divided main road (Fig. 7.11): secondary-side median **2.50 m** minimum for the marking strip; merge-side **1.60 m** where cyclists or pedestrians are present; junction throat passage **≈ 6 m**.

### 4.9 Teardrop island and median opening (§7.4)

**Teardrop geometry (§7.4.2):**

| Parameter | Value | Ref |
|---|---|---|
| Painted nose width for the left-turn lane | **+3.60 m** beyond the required lane | §7.4.2א |
| Pedestrian refuge / signal pole width | **2.5 m** (3.0 m with cyclists) | §7.4.2ב |
| Longitudinal island body width (secondary arm on a divided main) | **3.0 m** | §7.4.2ג |
| Forward nose offset from the through-lane centreline | **≥ 1.0 m** and **≤ 2.0 m** | §7.4.2ד, Fig. 7.13 |
| Nose visibility distance before the junction | **10 m** at 50 km/h up to **50 m** at 80 km/h | §7.4.2ה |
| Nose corner radius (built) | **≥ 0.5 m** | §7.4.2ו |

**Figure 7.15 — detailed teardrop construction:**

| Step | Description | Dimension |
|---|---|---|
| 1 | Secondary road axis | — |
| 2 | Point back from the kerb along axis 1 | **10 m** |
| 3 | Teardrop axis from point 2, rotated right of the secondary axis | **6°** |
| 4 | Two helper lines parallel to axis 3, at half the maximum teardrop width | **1.5 m** each side (max width **3.0 m**) |
| 5a | Left-turn radius main → secondary, tangent to the median and to the left helper line | design radius |
| 5b | Left-turn radius secondary → main, tangent to the right helper line and the main median | design radius |
| 6 | Forward nose radius between the two arcs | **0.75 m** |
| 7 | Lines from a point back along the main kerb, tangent to the radii of step 5 | **20 m** |
| 8 | Rear nose radius between the step-7 lines | **0.75 m** |

**Median opening (§7.4.3):** default operation for a left turn from the secondary road is two-stage, waiting in the median opening.

| Rule | Value |
|---|---|
| Minimum continuous median width | **≥ 6.0 m** (כרך 1 Ch. 3) |
| Remaining median after a dedicated left-turn lane is installed | **≥ 2.50 m** (without lighting poles) |

---

## 5. Junction elevation design (כרך 2 פרק 8)

### 5.1 The elevation domain (§8.5.2)

> "תחום הרומים לצומת" — על כל זרוע המישור נמשך **לפחות עד נקודת ההשקה הרחוקה ביותר** ממפגש צירי הדרכים, של העקומות (ראו תרשימים 8.3, 8.5).

The junction elevation domain on each arm extends **at least to the farthest tangent point** of the centreline curves from the meeting of the road axes. It is defined per arm, must be verified for both roads before a geometric solution is chosen, and is **specific to crown design** — it is not the same as the "junction area" definitions in כרך 1 §2.1.3 or כרך 2 §2.1.3.

### 5.2 Three elevation design methods (§8.5.2)

| Hebrew | § | English | When used |
|---|---|---|---|
| שיטת שמירת השיפוע לרוחב הדרך העיקרית | 8.5.3 | **Preserve main-road crossfall** | Main + secondary with a clear hierarchy |
| שיטת המישור | 8.5.5 / 8.5.6 | **Plane method** | 4-arm cross or T; all arms get grade transitions |
| שיטת הנקודה הגבוהה | 8.5.8 | **High-point method** | Two roads of **similar rank** |

| Method | Crossfall behaviour |
|---|---|
| §8.5.3 preserve main crossfall | Main road unchanged; secondary road warped to match |
| §8.5.5 / §8.5.6 plane | All arms adjusted; transitions required on all arms |
| §8.5.8 high point | Symmetric, matched crossfall on both roads, in the 0.5–2.0 % band |

### 5.3 §8.5.3 — the crossfall-preservation method (primary method)

Purpose: keep the main road's crossfall unchanged through the junction and warp the secondary road to match.

**Step A — longitudinal grades on the main road, for roads not meeting at a right angle:**

```
[8.1]   g'₁ = g₁·cos α₁ + g₂·sin α₁
[8.2]   g'₃ = g₃·cos α₂ + g₂·sin α₂
```

**Step B — crossfall on the secondary road:**

```
[8.3]   g'₂ = (g₂ − g₁·sin α₁) / cos α₁
[8.4]   g'₃ = (g₃ − g₁·sin α₂) / cos α₂
```

| Symbol | Meaning |
|---|---|
| `g₁` | Fixed longitudinal grade on the main road |
| `g'₁` | Grade normal to the main-road crossfall |
| `g₂`, `g₃` | Grades on both sides of the main road in the junction plane |
| `g'₂`, `g'₃` | Crossfall grades on the **secondary** road, both sides of the main road |
| `α₁`, `α₂` | Angles between the grade vectors at the junction plane, relative to the main-road high-point azimuth |

**Perpendicular special case (α = 90°):** two opposing sloped planes meet at the main-road crown line, and `g'₁ = g₃ = g₂ = g'₂ = g₁ = g'₃` — all equal. The secondary longitudinal grade `g₂` in the junction zone equals the main crossfall `g'₁`.

**Secondary-road profile (§8.5.3ד), three options in preference order:**
1. **Preferred (Figure 8.3):** grade transitions placed **outside** the elevation domain, vertical curve between the differing longitudinal grades, longitudinal grades preserved on both sides.
2. Vertical curve near the junction, then the grade transitions — side grades are not preserved.
3. Vertical curve and transitions on the same side — shorter, but the vertical-curve shape distorts at the edges.

### 5.4 §8.5.5 / §8.5.6 — the plane method

The whole junction becomes **one plane** defined by the centrelines of both roads approaching it.

```
[8.5]   g'₁ = (g₁ − g₂·sin α) / cos α        Road A crossfall
[8.6]   g'₂ = (g₂ − g₁·sin α) / cos α        Road B crossfall
```

where `α` is the angle between the **descent-direction** vectors of the two centreline grades. At α = 90°, `g₂ = g'₁` and `g₁ = g'₂`. Grade transitions are required on **all** arms (§8.6).

**T-junction extra constraint (§8.5.6):** the algebraic difference between the crossfall on the continuing ("stem") road `g'₁` and the **2 %** crossfall on the secondary approach lane must not exceed:

| Design speed (km/h) | Max difference |
|---|---|
| 30 | **8 %** |
| 40–50 | **6 %** |

If exceeded, the junction must be redesigned.

### 5.5 §8.5.8 — the high-point method

For two roads of similar rank. Parallel crown adjustments on all arms approaching the junction, with a **crossfall adjustment range of 0.5 % – 2.0 %** over a length that allows comfortable traverse. Crossfall grades are **identical along both meeting roads** within the elevation domain. Figures 8.7 and 8.8 are schematic (source: TAC 1999, 2017) and carry no scale.

### 5.6 §8.5.7 — resultant grade

```
[8.7]   g_max = √(g₁² + g₂²)
[8.8]   g_max for a non-perpendicular T, from g₁, g₂ and the angle α between descent-direction vectors
```

`g_max` is the maximum resultant grade in the junction, in the direction perpendicular to the contour lines; `g₁` is the longitudinal grade and `g₂` the crossfall.

> ### RESOLVED — the §8.2.3 resultant-grade limit is a MINIMUM (a drainage floor)
>
> This was the single most consequential unresolved item in this document, because code written against the wrong reading is wrong on every junction. **It is now settled against the raw PDF text layer**, which is preserved at `docs/research/chapter8_full.txt`, lines 380–386 (PDF page 235, internal page 8-3):
>
> ```
> 380  שםיירעזמ םיעופי                          ← «שיפועים מזעריים» (RTL-scrambled)
> 381  השיפוע המזערי לאורך זרועות הצומת נקבע משיקולי ניקוז והוא
> 382  0.5%
> 383  . השיפוע השקול המרבי בצומת
> 384  מ תחפי אל )הבוגה יווקל בצינ(-              ← «(ניצב לקווי הגובה) לא יפחת מ-»
> 385  1%
> 386  . יש להקפיד על סילוק המים מתחומי הצומת במידת האפשר, ולמנוע
> ```
>
> Reassembled: «**שיפועים מזעריים** — השיפוע המזערי לאורך זרועות הצומת נקבע משיקולי ניקוז והוא **0.5%**. השיפוע השקול המרבי בצומת (ניצב לקווי הגובה) **לא יפחת מ-1%**. יש להקפיד על סילוק המים מתחומי הצומת…»
>
> **Reading A was wrong and should be discarded.** It was recorded as "השיפוע השקול המרבי בצומת לא יעלה על 1%", but the string «לא יעלה על» **does not occur anywhere in `chapter8_full.txt`**. The raw text says «לא יפחת מ־» — "shall not fall below". Reading A appears to have been reconstructed from the word «המרבי» rather than read.
>
> The four supporting arguments below all corroborate the raw text:
> 1. «לא יפחת מ־» unambiguously means "shall not fall below"; «לא יעלה על» means "shall not exceed". Only one of these is in the raw text, and the agent that quoted the raw text quoted the former.
> 2. §8.2.3 is titled **שיפועים מזעריים** — *minimum* grades. It is the same section that sets the 0.5 % longitudinal floor.
> 3. The stated rationale is **משיקולי ניקוז** — drainage — with the following sentence about clearing water off the junction. Drainage produces floors, not ceilings.
> 4. Internal consistency: `Table 8.1` permits arm longitudinal grades of **4–6 %**, and by [8.7] `g_max = √(g₁²+g₂²) ≥ g₁`. A 1 % ceiling would therefore be unsatisfiable on almost any compliant junction, whereas a 1 % floor is a meaningful and easily-met drainage requirement.
>
> The trap is terminology: «השיפוע השקול **המרבי**» is the *name of the quantity* — "the maximum resultant grade", meaning the grade in the steepest direction. The requirement is that this steepest-direction grade be **at least** 1 % so that the junction drains.
>
> **Implemented as:** `RoadNetStandards::MinJunctionResultantGrade()` returns `0.01` as a floor. There is deliberately no 1 % ceiling. The ceiling that *does* exist is `Table 8.1`'s 4–6 % arm grade, exposed as `MaxArmGradeAtJunction(Vd)`. Note that the current junction plate is dead level, so every junction sits below the drainage floor by construction; `RoadNetGrade.cpp` reports that as a count rather than a per-junction warning, pending a real drainage pass.
>
> **Loose end:** the Chapter 8 reader also noted that §8.5.7 references `Table 8.1` for a maximum resultant value on high-class roads. If such a ceiling exists it is a separate constraint from the §8.2.3 floor, and **its numeric value was never extracted**. Re-read internal page 8-18 before relying on the absence of any ceiling.

### 5.7 Table 8.1 — longitudinal grades on junction arms (§8.2.2)

| Design speed on arm (km/h) | Minimum length at max grade from the junction (m) | Maximum longitudinal grade (%) |
|---|---|---|
| 100 | 85 | 4 |
| 90 | 75 | 4 |
| 80 | 70 | 4 |
| 70 | 70 | 6 |
| 60 | 60 | 6 |
| 50 | 50 | 6 |
| 40 | 30–40 (T-junction) | 6 |
| 30 | *(approach speeds at T)* | 6 |

The maximum may be increased on arms with a topographic constraint.

> **UNCERTAIN — the bottom two rows are incomplete.** The 40 km/h "minimum length" cell reads "30–40 (T-junction)" and the 30 km/h cell has no value at all. Treat the 30 km/h minimum length as **not extracted**.

**Supporting rules (§8.2.2):**
- General junction grade range: **3 %–6 %**, lower than open-road maxima.
- Approach design speeds: main-axis arms and secondary arms use their design speed; local/access arms use **30 km/h** minimum, regional **40 km/h** (Ch. 11 §11.7).
- Preserve the grade unchanged for at least the Table 8.1 minimum length at maximum grade.
- If the grade exceeds **3.0 %**, re-check sight distances (Table 4.1, §4.2.2–4.2.4) and deceleration/acceleration lane lengths (Ch. 6).

**§8.2.3 minimum longitudinal grade: 0.5 %** (drainage).

**§8.2.4 — המרווח החופשי (גבריט):** listed in the table of contents but **not extractable** from the PDF text layer; the word "גבריט" does not appear anywhere in the extracted text. Page 8-3 carries only a cross-reference to Ch. 6 §6.2(ה). **Not extracted — re-read internal page 8-3 and Ch. 6 §6.2(ה) if vertical clearance is needed.**

### 5.8 Table 8.2 — minimum radii for a junction on a horizontal curve (§8.3)

| Design speed (km/h) | Min K (m³) | Min R with 2 % superelevation one side (m) | Min R with no superelevation, roadside 2 % grade (m) |
|---|---|---|---|
| 60 | 200 (at 6 % long. grade) | 550 | 1400 |
| 70 | 300 (6 %) | 770 | 1900 |
| 80 | 550 (4 %) | 1100 | 2500 |
| 90 | 700 (4 %) | 1400 | 3100 |
| 100 | 900 (4 %) | 1800 | 3800 |

Maximum superelevation at a junction follows the main-road longitudinal grade and speed per Table 8.1: **6 %** up to 70 km/h, **4 %** at 80–100 km/h. Values derive from כרך 1 Table 5.4.

§8.3 restates the preference for a 90° intersection and the instruction to avoid placing junctions on horizontal curves at all, because of visibility, turning-lane geometry, and superelevation conflicts.

### 5.9 Table 8.3 — minimum vertical-curve K values at junctions (§8.4)

Designed for driver eye height **1.05 m** and object height **0.15 m**.

| Speed (km/h) | Convex K — stopping | Convex K — passing | Concave K — stopping | Concave K — passing |
|---|---|---|---|---|
| 40 | 410 | 1815 | 410 | — |
| 50 | 760 | 2510 | 645 | — |
| 60 | 1230 | 3615 | 930 | — |
| 70 | 2035 | 5650 | 1260 | — |
| 80 | 3320 | 7250 | 1650 | — |
| 90 | 4915 | 9050 | 2085 | — |
| 100 | 7250 | 11600 | 2575 | — |

**Notes:** the convex minimum follows stopping sight (§4.2.4ב'); the speed is the approach/design speed on the arm, not necessarily the full road design speed; stopping values match the junction sight distances in Table 4.1; formulas are in כרך 1 §6.4.2, with eye heights 1.05 m in both directions (§4.3.5). On concave curves with longitudinal grades **below 0.4 %**, check sight triangles (§4.3).

The "Concave K — passing" column is empty in the extraction. **UNCERTAIN — treat concave passing K as not extracted.**

### 5.10 §8.6 — grade transitions on the junction arms

Required by §8.5.3, §8.5.5/§8.5.6, and §8.5.8. Transitions are made by **rotating the pavement about the lane axis** (כרך 1 §5.3), limited by the maximum rate of crossfall change `Δn` (%). Transitions are placed **outside** the elevation domain.

```
[8.9]    L = (W/n) · g' · 2% / Δn        Long (outer) side
[8.10]   L = (W/n) · g' · 2% / (−Δn)     Short (inner) side
```

| Symbol | Meaning |
|---|---|
| `L` | Transition length (m) |
| `W` | Lane width = **3.6 m** |
| `n` | Number of lanes rotated |
| `g'` | Crossfall at the arm edge (g'₁, g'₂, g'₃ from §8.3 / §8.5) |
| `2%` | Normal crossfall at the arm edge |
| `Δn` | Maximum crossfall change rate (%), from Table 8.5 / כרך 1 §5.3 |

`n` calculation: single-lane arm `n = 1`; multi-lane `n` = number of lanes from the axis to the **right** edge in the direction of travel, `n = R_d / W`, where `R_d` is the distance from the rotation axis to the right lane edge.

**Table 8.5 — maximum Δn (%) between axis and pavement edge at junctions:**

| Design speed (km/h) | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 100 |
|---|---|---|---|---|---|---|---|---|
| Base δ (single lane) | 0.75 | 0.70 | 0.67 | 0.64 | 0.56 | 0.50 | 0.46 | 0.43 |
| Urban narrow median (3.1 m, 1 lane) | — | — | — | — | 0.795 | 0.710 | 0.653 | 0.610 |
| Urban wide median (3.3 m, 1 lane) | — | — | — | — | 0.810 | 0.723 | 0.665 | 0.622 |

Footnote: at T-junctions the secondary arms are designed for approach speeds of **40 km/h** (from a main road) and **30 km/h** (from a local/access road).

The base row matches כרך 1 Table 5.6 exactly (0.64 / 0.56 / 0.50 / 0.46 / 0.43 at 60–100 km/h), which is a good consistency check.

> **UNCERTAIN — Table 8.4 (`R_d` and the correction factor) came out scrambled.** As extracted it reads median widths 3.1 / 3.2 / 3.3 against "R_d = 1", "weighted n = 8.8 / 4.4 / 4.4", and "correction = 2.444 / 2.611 / 2.611 / 0.691" — the column headers are clearly misassigned. The equivalent clean table is **כרך 1 Table 5.7**, which reads: 4-lane narrow median 3.2 m → R_d 8.8 m, n 2.444, δ_w 0.705; 4-lane wide 4.4 m → R_d 9.4, n 2.611, δ_w 0.691; 6-lane narrow 3.2 m → R_d 12.4, n 3.444, δ_w 0.645; 6-lane fast-120 narrow 6.80 m → R_d 14.2, n 3.944, δ_w 0.627; 6-lane wide 4.4 m → R_d 13.0, n 3.611, δ_w 0.638. **Use כרך 1 Table 5.7, not the garbled Table 8.4.**

### 5.11 §8.7 — drainage at the junction

- Place kerb inlets **before** the junction entry so external water never enters the junction; no drainage should flow toward the junction from outside.
- At grade transitions, match the median crossfall so water does not cross travel lanes (Figures 8.3, 8.5).
- At a concave vertical curve on an arm, place a kerb with an inlet in the curve zone.
- Position catch basins **≥ 1 m before** the crosswalk square.
- Minimum drainage grade: **0.5 %** longitudinal (§8.2.3).

**Implementation notes:** the plane method (§8.5.5) is the cheapest to implement — fit one plane through both centrelines and enforce the T-stem crossfall delta limit. The crossfall-preservation method (§8.5.3) is the correct default for a main-plus-secondary junction and is what a hierarchy-aware generator should use: freeze the main crossfall, solve [8.3]/[8.4] for the secondary crossfall, then place [8.9]/[8.10] transitions outside the domain boundary defined in §8.5.2.

---

## 6. Design vehicles (כרך 2 פרק 2)

### 6.1 Table 2.4 — geometric characteristics (§2.6.1, p. 2-38)

All dimensions in metres. Based on תקנות 313 and 377 plus the urban street guidelines of 10/2020.

| Code | Hebrew | English | Length | Width | Height | Wheelbase | Front OH | Rear OH | R₁ | R₂ |
|---|---|---|---|---|---|---|---|---|---|---|
| **P** | רכב פרטי ומסחרי קל | Passenger / light commercial | **5.00** | **2.00** | — | **2.80** | **1.00** | **1.20** | **6.85** | **3.65** |
| **SV** | רכב שירות | Service vehicle | **9.50** | **2.55** | — | **5.40** | **1.80** | **2.30** | **9.65** | **3.80** |
| **SU** | משאית עירונית | Single-unit truck | **9.00** | **2.55** | **3.55** | **6.00** | **1.20** | **1.80** | **12.55** | **7.70** |
| **BUS** | אוטובוס | Bus | **12.00** | **2.55** | **3.55** | **6.00** | **2.70** | **3.30** | **12.55** | **5.30** |
| **A-BUS** | אוטובוס מפרקי רגיל | Articulated bus | **17.95** | **2.55** | **3.55** | — | **2.70** | **3.40** | **12.45** | **5.30** |
| **BRT** | אוטובוס מפרקי ארוך | Long articulated BRT | **18.75** | **2.55** | **3.55** | — | **2.75** | **3.40** | **12.45** | **5.30** |
| **WB-12 SEMI** | רכב משא מורכב | Semi-trailer, 12 m | **14.10** | **2.55** | **3.55** | **8.40** | **0.90** | **1.00** | **12.85** | **5.30** |
| **WB-15 SEMI** | רכב משא מורכב ארוך | Long semi, WB-15 | **17.60** | **2.55** | **3.55** | **9.95** | **1.30** | **1.65** | **14.50** | **5.30** |
| **WB-15 FULL** | רכב משא מחובר | Full trailer, WB-15 | **18.75** | **2.55** | **3.55** | **5.55** | **1.30** | **1.20** | **12.50** | **5.30** |

> **UNCERTAIN — what R₁ and R₂ mean.** The table has two minimum-turning-radius columns and the extraction did not capture their headers. The most likely reading is outer and inner radius of the minimum turning circle, but this was not confirmed. Note that R₂ is a flat **5.30 m** for every heavy vehicle from BUS downward, which matches the §2.6.2 regulatory statement that the minimum front axle spacing is **≥ 5.30 m** — suggesting R₂ may be a regulatory constant rather than a computed radius. **Do not use R₁/R₂ for corner design.** Use the swept-path values from figures 2.14–2.22 instead.

### 6.2 Figures 2.14–2.22 — swept path / turning templates

These are the values a corner-radius calculation should actually use. The document is explicit: *template radii are swept-path design values for standard turn angles (30°–180°); the Table 2.4 radii are regulatory minimums — use the templates for lane and corner design.*

| Fig | Vehicle | Swept-path template values |
|---|---|---|
| **2.14** | **P** | L 5.00, W 2.00, WB 2.80, front 1.00, rear 1.20; outer path R ≈ **12.45**; inner ≈ **6.40**; swept width **2.75**; steer angle **30.8°** |
| **2.15** | **SV** | L 9.50, W 2.55, WB 5.40, front 1.80, rear 2.30; R ≈ **8.60**; swept **4.45**; steer **46.8°** |
| **2.16** | **SU** | L 9.00, W 2.55, WB 6.00, front 1.20, rear 1.80; R ≈ **12.00**; swept **4.20**; steer **33.8°** |
| **2.17** | **BUS** | L 12.00, W 2.55, WB 6.00, front 2.70, rear 3.30; R ≈ **11.00**; swept **4.40**; steer **37.9°** |
| **2.18** | **A-BUS** | L 17.95, W 2.55; R₁ ≈ **11.85**, R₂ ≈ **5.20**; articulation **50°**; steer **31.0°** |
| **2.19** | **BRT** | L 18.75, W 2.55; R ≈ **10.90**; swept **5.75** at 90°; articulation **50°** |
| **2.20** | **WB-12 SEMI** | L 14.10, W 2.55; tractor WB 3.80, kingpin-to-tandem 8.40; R ≈ **12.55**; swept **6.70**; articulation **70°** |
| **2.21** | **WB-15 SEMI** | L 17.60, W 2.55; R ≈ **14.05**; swept **7.90** at 180°; articulation **70°** |
| **2.22** | **WB-15 FULL** | L 18.75, W 2.55; R ≈ **11.90**; swept **5.60**; articulation **70°** |

### 6.3 Which vehicle to design for (§2.6.1)

| Context | Design vehicle |
|---|---|
| Typical interurban junction | The dominant types in the traffic mix — P, SV, SU, buses — with **10–15 %** heavy vehicles and a minimum of **3 %** articulated/combination trucks |
| Junction of access/local roads only | **Bus** alone |
| Continuous interurban corridor | The largest geometric vehicle on the **left-turn / critical path**, typically **WB-12** or **WB-15** |
| Roundabouts | Often requires **WB-15** accommodation (Ch. 11); articulated buses in metro corridors |
| Exceptional vehicles (tankers, agricultural) | **Not** in Table 2.4 — case-by-case check |

**§2.6.2 regulatory limits:** maximum rear outer track width **12.50 m**; maximum trailer / fifth-wheel length **12.0 m**; minimum front axle spacing **≥ 5.30 m**. Height exceedance is a *moderate* deviation for SU and BUS, and a *large* deviation for WB-12 and long WB-15.

**Implementation notes:** the swept width column is the one that drives turning-roadway width. WB-15 SEMI sweeps **7.90 m**, which is why §6.3.1 sets the turning roadway at 5.5–6.0 m only where WB-15 is not expected, and why roundabout circulatory widths in Table 11.5 reach 9.7 m.

### 6.4 Conflict points by junction form (§2.2.3)

| Form | Crossing | Merging | Diverging | **Total** | Figure |
|---|---|---|---|---|---|
| 4-arm cross (הצטלבות) | **16** | **8** | **8** | **32** | 2.4 |
| 3-arm (הסתעפות) | **3** | **3** | **3** | **9** | 2.5 |
| Roundabout (typical 4-arm, 1 lane) | **0** | **4** | **4** | **8** | 2.6 |

Adding a fourth arm to a three-arm junction roughly **quadruples** the total conflict count. A roundabout eliminates vehicle-to-vehicle crossing conflicts entirely.

The **design junction area** (§2.1.3ב) is defined as the envelope that includes **at least all conflict points** on the intersecting roadways — this is the hard inner bound for corner radii, islands, and turning lanes.

---

## 7. Roundabouts (כרך 2 פרק 11)

**Scope note:** Chapter 11 does not use "mini / rural" categories and has no standalone inscribed-circle-diameter lookup. It works in `R_c` (central island radius), `W_c` (circulatory width), and `D_c` (outer circulatory diameter).

### 7.1 Symbols (§11.4) and the inscribed diameter

| Symbol | Meaning |
|---|---|
| `R_c` | Central island radius, to the kerb, including the apron if present |
| `W_c` | Circulatory roadway width for one direction, **excluding** the apron |
| `D_c` | Outer circulatory ring diameter |
| `W_a` | Apron width |
| `W_en`, `W_ex` | Entry / exit width, including flare |
| `R_en`, `R_ex` | Entry / exit kerb radii |

```
D_inscribed = D_c ≈ 2 × (R_c + W_c)
```

### 7.2 Warrants (§11.3)

| Rule | Value |
|---|---|
| Roundabouts suit junctions where design speed **≤ 80 km/h** | **80 km/h** |
| Design speed **> 80 km/h** | Approach speed control required (§11.7) |
| Single-lane capacity ceiling | **25,000 veh/day**; **2,000 veh/h** peak |
| Two-lane capacity ceiling | **40,000 veh/day**; **3,200 veh/h** peak |
| Maximum circulatory lanes | **2** |
| Maximum arms | **4** (§11.5) |

**Table 11.3 — exit lane warrants:**

| Exit volume (veh/h) | Exit lanes |
|---|---|
| **≤ 750** | 1 |
| **750 – 1,050** | 1, consider 2 |
| **1,050 – 1,500** | 2 |

**Prohibitions:** a two-lane roundabout is **not suitable** with low pedestrian activity per Table 11.1, and is **forbidden** where two-lane entries or exits carry high pedestrian crossing demand. **Exit lanes must never exceed entry lanes** at the circulatory merge point (§11.3ה). More than 4 arms with adjacent entry spacing below 20 m is not permitted without enlarging `R_c` (§11.9ב).

Phasing is explicitly allowed: build single-lane now and upgrade to two-lane in 10–15 years, with land reserved.

### 7.3 Table 11.4 — initial central-island radius (p. 11-10)

| Roundabout type | Design speed (km/h) | **Initial R_c** (m) | **Minimum R_c** (m) |
|---|---|---|---|
| **חד-נתיבי** (single-lane) | **35** | **12** (or **20** *) | **8** |
| **חד-נתיבי** | **45** | **16** | **8** |
| **דו-נתיבי** (two-lane) | **35** | **26** | **12** |
| **דו-נתיבי** | **45** | **30** | **12** |

\* **20 m** initial `R_c` when splitter islands (איי-התנתקות) are used on a two-lane design.

**Increase `R_c` when:** the island is non-circular; arm angles are significantly unequal; the design vehicle is **WB-15 SEMI**; or two large design vehicles must pass side-by-side on the circulatory.
**The Table 11.4 minimums may be reduced when:** there are only **3 arms**; an apron is used; or the design vehicle is **WB-12 SEMI** only.

**Island shape constraints (§11.5):** maximum 4 arms; equal angles between adjacent arms preferred; roads meeting the island must be tangential; on a two-lane roundabout at least part of the entries **and** part of the exits **and** part of the circulatory must be 2 lanes; the design vehicle must not overhang the circulatory width.

### 7.4 Table 11.5 — circulatory width by R_c and design vehicle (p. 11-12)

All values in metres.

| **R_c** | Single: bus WB-12 | Single: WB-15 | Single: bus parallel | Two-lane: WB-12 ∥ | Two-lane: WB-15 ∥ | Two-lane: car ∥ |
|---|---|---|---|---|---|---|
| **8** | 6.0 | 6.6 | 7.5 | — | — | — |
| **10** | 5.7 | 6.1 | 6.9 | — | — | — |
| **12** | 5.3 | 5.7 | 6.4 | 8.9 | 9.3 | 10.0 |
| **14** | 5.1 | 5.4 | 6.1 | 8.7 | 9.0 | 9.7 |
| **16** | 4.9 | 5.2 | 5.8 | 8.5 | 8.8 | 9.4 |
| **18** | 4.7 | 5.0 | 5.5 | 8.3 | 8.6 | 9.1 |
| **20** | 4.5 | 4.8 | 5.3 | 8.1 | 8.4 | 8.9 |
| **22** | 4.4 | 4.7 | 5.1 | 8.0 | 8.3 | 8.8 |
| **24** | 4.3 | 4.5 | 5.0 | 7.9 | 8.1 | 8.6 |
| **26** | 4.3 | 4.3 | 4.8 | 7.9 | 7.9 | 8.4 |
| **28** | 4.3 | 4.3 | 4.8 | — | — | — |
| **30** | 4.3 | 4.3 | 4.8 | — | — | — |

> **UNCERTAIN — the column headers were partly garbled in extraction.** The six columns were recorded with mixed and inconsistent labels (B / B-long / B∥ / P∥ / P∥-long / P∥). The grouping into three single-lane and three two-lane columns is confident; the exact design vehicle attached to each column within a group is **inferred from the monotonic ordering** (bus < WB-12 < WB-15 in required width) rather than read directly. Verify against internal page 11-12 before using a specific column.

Worked examples for sanity checking: single lane, `R_c` 16, `W_c` 5.8 → `D ≈ 43.6 m`. Two-lane, `R_c` 26, `W_c` 8.4 → `D ≈ 68.8 m`.

**Clearance rules (§11.5):**

```
Single-lane:   W_c ≥ W_design_vehicle + 0.3 m + 0.6 m side margins
Two-lane:      W_c ≥ max(right-lane large vehicle, left-lane car) + 0.9 m combined margins
```

### 7.5 Table 11.6 — maximum deflection radius (§11.6, p. 11-14)

Vehicles must be forced to follow three radii on the straight-through path: **R₁ (entry, right), R₂ (circulatory, left), R₃ (exit, right)**. The **deflection radius** is `R_def = min(R₁, R₂, R₃)`.

| Type | Design speed (km/h) | Max R_def at **−2 %** crossfall (m) | Max R_def at **+2 %** crossfall (m) |
|---|---|---|---|
| Interurban **single-lane** | **35** | **38** | **45** |
| Interurban **two-lane** | **45** | **68** | **95** |

**Measurement construction (Figures 11.7–11.8):** offset the vehicle path **1.0 m** from the entry edge for a raised splitter island, or **1.5 m** for a painted-only splitter, then strike three arc centres from the entry edge, the island kerb, and the exit edge. Worked example (Figure 11.9): `R_c` 16 m, `W_c` 5.2 m, entry width 4.5 m, `R₂` = 26 m.

**Crossfall interaction (§11.6):** circulatory normal crossfall is **±2 %** with superelevation reversal allowed; a crowned section may widen to **±6 %**, but exceeding ±2 % requires smaller deflection radii. Entry and exit curve side slopes are designed at ±2 % and never below 2 % for drainage.

### 7.6 Approach speed control (§11.7)

Required where design speed exceeds 80 km/h, and used on dual carriageways up to 90 km/h.

| Case | Numbers from the worked examples |
|---|---|
| Single-lane, 90 km/h approach → 65 km/h transitional (Fig. 11.10) | R_round **12 m**; R₁ **460 m**; transition length **58 m**; first arc **28.5 m**; R₂ **290 m**; second arc **46.5 m** |
| Two-lane, 90 km/h (Fig. 11.11) | R_round **20 m**; arc **77 m**; R **290 m**; arc **46.5 m** |
| Dual carriageway, 90 km/h (Fig. 11.12) | R_round **20 m**; R₁ **460 m**; L_trans **58 m**; arcs **28.5 m** |
| Dual carriageway with wide median, 90 km/h (Fig. 11.13) | R_round **24 m**; arc **77 m**; R **290 m**; arc **83 m** |
| Max speed drop across a reverse curve | **≤ 30 km/h** |
| Reverse-curve grade | **2.0 % – 2.5 %** |

### 7.7 Entry and exit design (§11.8)

| Parameter | Value |
|---|---|
| **Entry kerb radius R_en** | **12 – 20 m** |
| **Exit kerb radius R_ex** | **12 – 20 m** |
| **Flare / transition length** | **15 – 100 m** |
| **Entry angle φ** | **25° ≤ φ ≤ 65°** («יש לתכנן זווית כניסה בתחום בין 25º לבין 65º») |
| Side clearance, single-lane | **0.6 m** total |
| Side clearance, two-lane | **1.0 m** total (three gaps × 0.3 m) |
| Exit may exceed entry width by | up to **20 %** |

**Table 11.7 — single-lane entry/exit kerb radius (p. 11-22),** `R_en = R_ex` in metres:

| **R_c** | BUS | SEMI WB-12 | SEMI WB-15 |
|---|---|---|---|
| 12 | 5.50 | 6.40 | 6.70 |
| 14 | 5.30 | 6.00 | 6.20 |
| 16 | 5.10 | 5.60 | 5.80 |
| 18 | 4.90 | 5.40 | 5.60 |
| 20 | 4.70 | 5.20 | 5.40 |
| 25 | 4.50 | 5.10 | 5.30 |
| 30 | 4.40 | 5.00 | 5.00 |
| 35 | 4.40 | 5.00 | 5.00 |
| 40 | 4.40 | 5.00 | 5.00 |

**Table 11.8 — two-lane entry/exit kerb radius (p. 11-23):**

| **R_c** | Bus | SEMI WB-12 | SEMI WB-15 |
|---|---|---|---|
| 12 | 8.60 | 9.00 | 9.70 |
| 14 | 8.30 | 8.70 | 9.40 |
| 16 | 8.10 | 8.40 | 9.10 |
| 18 | 7.90 | 8.20 | 8.80 |
| 20 | 7.80 | 8.00 | 8.70 |
| 25 | 7.50 | 8.00 | 8.50 |
| 30 | 7.40 | 7.50 | 7.90 |
| 35 | 7.20 | 7.50 | 7.80 |
| 40 | 7.20 | 7.50 | 7.70 |

> **UNCERTAIN — Tables 11.7/11.8 contradict the §11.8א range.** §11.8א states entry and exit kerb radii of **12–20 m**, but Tables 11.7 and 11.8 give **4.4–9.7 m** for what was recorded as the same quantity. These cannot both describe `R_en`. The most plausible reconciliation is that the tables give a *different* quantity — perhaps the minimum kerb radius at the nose or the required clearance offset — that was mislabelled during extraction. **Do not use Tables 11.7/11.8 as `R_en` without checking internal pages 11-22/11-23.** Use the §11.8א range of 12–20 m as the working value for entry and exit kerb radii.

### 7.8 Arm separation (§11.9)

| Rule | Value |
|---|---|
| Minimum spacing between adjacent entries | **≥ 20 m** |
| More than 4 arms and spacing would fall below 20 m | Increase `R_c` |
| 3 arms | Arm angles should sum to 360° with equal splits preferred |
| 4 arms | Two pairs of equal adjacent angles |

### 7.9 Sight distances (§11.10)

Computed per Ch. 4 §4.2.2 with crossfall ±2 %. Approach speed assumptions: **50 km/h** spread for interurban single-lane, **60 km/h** for two-lane. Average approach speed for `d`: single-lane **45 km/h**, two-lane **55 km/h**.

**Table 11.9 — stopping sight distance versus travel-path radius (p. 11-28):**

| Travel radius (m) | Design speed (km/h) | Stopping sight (m) |
|---|---|---|
| 10 | 21 | 16 |
| 15 | 23 | 18 |
| 20 | 26 | 21 |
| 25 | 28 | 23 |
| 30 | 30 | 26 |
| 35 | 33 | 28 |
| 40 | 34 | 30 |
| 50 | 38 | 33 |
| 60 | 40 | 35 |
| 70 | 43 | 38 |
| 80 | 45 | 40 |
| 90 | 46 | 42 |
| 100 | 48 | 43 |

**Table 11.10 — yield sight distances (p. 11-32).** Radius is the travel-path radius from Figures 11.7–11.8.

| Radius (m) | Speed (km/h) | **d** stopping (m) | **d₁** yield to left (m) | **d₂** yield to circulatory (m) |
|---|---|---|---|---|
| 10 | 21 | 29 | 15 | 23 |
| 15 | 23 | 32 | 20 | 26 |
| 20 | 26 | 36 | 25 | 28 |
| 25 | 28 | 40 | 30 | 30 |
| 30 | 30 | 42 | 35 | 33 |
| 35 | 33 | 46 | 40 | 34 |
| 40 | 34 | 47 | 50 | 38 |
| 50 | 38 | 53 | 60 | 40 |
| 60 | 40 | 56 | 70 | 43 |
| 70 | 43 | 60 | 80 | 45 |
| 80 | 45 | 62 | 90 | 46 |
| 90 | 46 | 65 | 100 | 48 |
| 100 | 48 | 67 | — | — |

Fixed offsets: the approaching driver is checked **15 m** before the entry line; eye height **1.05 m**, obstacle **0.15 m**; at an underpass or grade-separated conflict, `d` reduces to **15 m** for the left approach. **No barriers** may be placed on the central island.

### 7.10 Truck apron (§11.11)

| Parameter | Value |
|---|---|
| **Height** above the circulatory pavement | **7 cm** exactly — «7 ס"מ לא יותר ולא פחות» |
| **Crossfall** | **3 %** toward the circulatory roadway, for drainage |
| **Width W_a** | **1.0 – 1.5 m** typical; may exceed **2 m** for a WB-15 SEMI design; size to the smallest expected truck plus **0.3 m** encroachment |
| Material | Mountable, deliberately uncomfortable; **not** a running lane |

### 7.11 Vertical alignment (§11.12)

| Parameter | Value |
|---|---|
| Preferred longitudinal grade on arms | **≤ 2 %** |
| Maximum adverse grade on an arm before the circulatory | **2 %** down toward the circulatory |
| Circulatory crossfall, normal | **+2 % to −2 %** |
| Crowned circulatory section ("אזור מלך") | up to **±6 %**, requires smaller deflection radii |
| Single-lane approach at 50 km/h | Max grade **6 %**, minimum transition **50 m** |
| Two-lane approach at 60 km/h | Max grade **6 %**, minimum transition **60 m** |
| Large central islands (`R_c` above roughly 22 m) | Crowned crossfall strategy recommended |

### 7.12 Right-turn bypass (§11.13)

| Parameter | Value |
|---|---|
| **Bypass width** | **2.0 m** |
| With a pedestrian crossing on the island between entry and exit | **3.0 m** |
| Design reference | Same principles as Ch. 6 §6.3 (right-turn islands) |
| Crossings | Prefer **two** crosswalks — entry side and exit side — not one combined |

### 7.13 Generator quick-pick defaults

| Parameter | Single-lane | Two-lane |
|---|---|---|
| `R_c` starting value | **16 m** at 45 km/h | **26 m** at 35 km/h, **30 m** at 45 km/h |
| `W_c` | **5.8 m** (R_c 16, bus) | **8.4–9.7 m** |
| `D_inscribed` | **≈ 44 m** | **≈ 51–69 m** |
| `R_en`, `R_ex` | **12–20 m** (§11.8א) | **12–20 m** |
| Flare length | **15–100 m** | same |
| Entry angle φ | **25°–65°** | same |
| Arm spacing | **≥ 20 m** | same |
| `R_def` max | **45 m** (+2 %) / **38 m** (−2 %) | **95 m** / **68 m** |
| Crossfall | **±2 %** | same |
| Apron | **1.0–1.5 m** wide, **7 cm** high, **3 %** crossfall | same |

---

## 8. Cross-section values from כרך 1 that the junction work depends on

Source: **כרך 1 פרק 3 — הדרך לרוחב החתך** (HATACH.pdf, 04/2018, 64 pages) and **כרך 1 פרק 2 — מדיניות תכן בסיסית** (BASIS.pdf, 18 pages).

### 8.1 Lane widths — כרך 1 §3.2.2, Table 3.1

| Road class | Design speed (km/h) | Lane width (m) |
|---|---|---|
| דרך מהירה בין-עירונית (interurban expressway) | 100–120 | **3.7 or 3.6** ¹ |
| פרברית / מעויירת מהירה (suburban expressway) | 90–110 | **3.6** |
| דו-מסלולית ראשית / אזורית (dual primary/regional) | 80–100 | **3.6** ² |
| חד-מסלולית ראשית (single primary) | 60–80 | **3.6** |
| חד-מסלולית אזורית (single regional) | 80 | **3.6** |
| | 70 | **3.5** |
| | 60 | **3.3** |
| חד-מסלולית מקומית וגישה (single local/access) | 60–80 | **3.5 / 3.3 / 3.0** ³ |

¹ 3.7 m only where the permitted speed corresponds to a 120 km/h design.
² 3.6 m also possible at 110 km/h design if the road is fully upgraded to a dual carriageway.
³ 3.0 m only for dead-end roads (דרך ללא מוצא).

Supporting rules from §3.2.2: minimum **3.60 m** for all roads except local and regional where the design speed is below 80 km/h; dual regional gets **3.30 m** at 60 km/h and **3.50 m** at 70 km/h.

**Implementation notes:** **3.60 m is the default lane width** used throughout כרך 2 — it is the `W` in `L_taper = ratio × W` (§7.3.2), in `[8.9]`/`[8.10]`, and in the 3.6 m parallel acceleration-lane section (§6.4.2). A generator should treat 3.6 m as the baseline and only deviate for regional and local classes.

### 8.2 Shoulder widths — כרך 1 §3.3.2, Table 3.2

`W` = the active/operational width required for a safety barrier, which depends on barrier type and performance level. `*` = 3.5 m may be replaced by 3.0 m plus emergency lay-bys (§3.10).

| Road class | V_d (km/h) | Both sides (single carriageway) | Right (dual carriageway) | Left median, 2 lanes | 3 lanes | 4 lanes |
|---|---|---|---|---|---|---|
| **בין-עירונית מהירה** | 120 | — | **3.5\*** | W + 1.2 | W + 1.2 | 3.0 + W |
| **בין-עירונית מהירה** | 100–110 | — | **3.5\*** | W + 1.2 | W + 1.2 | 3.0 + W |
| **פרברית / מעויירת מהירה** | 90–110 | — | **3.0** | W + 1.2 | W + 1.2 | 3.0 + W |
| **דו-מסלולית ראשית** | 80–100 | — | **3.5\*** | W + 1.2 | W + 1.2 | 3.0 + W |
| **דו-מסלולית אזורית** | 80–100 | — | **3.0** | W + 1.2 | W + 1.2 | 3.0 + W |
| **חד-מסלולית ראשית** | 60–80 | **3.0 + W** | — | — | — | — |
| **חד-מסלולית אזורית** | 80 | **3.0 + W** | — | — | — | — |
| | 60–70 | **2.0 / 2.5 + W** | — | — | — | — |
| **חד-מסלולית מקומית / גישה** | 60–80 | **2.0 + W** | — | — | — | — |

Discrete rules from §3.3.2:

| Condition | Value |
|---|---|
| Interurban or main road, ≥ 2 lanes/direction, right shoulder | **3.5 m** (or 3.0 m plus emergency lay-bys) |
| Dual regional or suburban, design ≤ 80 km/h, right shoulder | **3.0 m** |
| Single carriageway regional/local, low volume, right shoulder | may reduce to **2.0** or **2.5 m** |
| Inner (median) shoulder, general | **1.20 m** per carriageway |
| Inner shoulder, ≥ 3 lanes/direction at 120 km/h permitted / 130 design | **3.0 m** |
| Inner shoulder, ≥ 4 lanes/direction | **3.0 m** |
| **Do not** design a median shoulder in the range | **1.2–3.0 m** |

All shoulders are **paved (סלולים)**, uniform width on both sides. Unpaved granular material appears only on the foreslope beyond the paved shoulder.

### 8.3 Median widths — כרך 1 §3.4

| Configuration | Minimum median width |
|---|---|
| Divided, **2–3 lanes/direction**, except a high-speed road | **2.4 m + barrier width** |
| Divided, **≥ 4 lanes/direction**, permitted 120 km/h | **6.0 m + barrier width** |
| Adding a 4th lane both directions at 120 km/h permitted | **10.8–11.0 m** total |
| With side lighting: 80 cm barrier reserve + W + VI | **3.2 m** typical dual carriageway |
| Combined barriers plus pole: 60 cm each barrier + 80 cm pole base | **4.40 m** |
| Narrow median for left-turn storage at intersections | **1.2 m** shoulder width (§3.4.1ח) |

Safety barriers are mandatory on both sides of the median on interurban divided roads (§3.4.1א). Undivided roads have no median.

**1+1 median treatments (§3.9.4.2):** rigid separator = double-sided concrete or approved steel barrier with marking 803 at the ends; soft separator (רצועת חיץ) = **0.60 m** between marking axes, roughly **0.75 m** total between travel lanes, rumble strips 815, cat-eye reflectors every **6 m**.

**Cross-reference:** כרך 2 §7.4.3 requires **≥ 6.0 m** continuous median width and **≥ 2.50 m** remaining after a dedicated left-turn lane is installed. That is stricter than the 2.4 m + barrier of כרך 1 §3.4.2 for 2–3 lane roads, so **use the כרך 2 value at junctions**.

### 8.4 Cross slopes — כרך 1 §3.5

| Element | Cross slope |
|---|---|
| Asphalt carriageway | **2.0–2.5 %** |
| Paved shoulder | **2–2.5 %**, may increase to **4 %** for drainage on a mild longitudinal grade |
| Treated foreslope | **3–4 %** |
| Untreated granular foreslope | **4 %** |
| Median with barrier, no drainage system | Same as the adjacent carriageway half, draining outward |
| Wide median **> 9.0 m**, or with drainage | **5 %** toward the median axis, then an underground drain |

The **2 %** figure that appears throughout the Chapter 8 elevation formulas `[8.9]`/`[8.10]` is the low end of this range — the "normal crossfall at the arm edge".

### 8.5 Design speed by road class — כרך 1 §2.2, Tables 2.3–2.5

**Table 2.4 — design speed ranges (km/h):**

| Road type | Max design speed | Recommended design range |
|---|---|---|
| **דרך מהירה** (freeway) | **120** | **120–100** |
| **פרברית / מעויירת מהירה** + dual fully grade-separated | **110** | **110–90** |
| **דרך דו-מסלולית אחרת** (other dual carriageway) | **100** | **100–80** |
| **דרך חד-מסלולית** (single carriageway) | **80** | **80–60** |

**Table 2.3 — target speed and recommended maximum permitted speed (km/h):**

| Road type | Max target (MMM) | Recommended target range |
|---|---|---|
| דרך מהירה | **110** | 110–90 |
| פרברית / מעויירת מהירה + dual grade-separated | **100** | 100–80 |
| דרך דו-מסלולית אחרת | **90** | 90–70 |
| דרך חד-מסלולית | **70** | 70–50 |

**Range interpretation:** upper = flat terrain with no sensitivity reduction; middle = hilly, or one sensitivity grade down; lower = mountainous, or two grades down.

**Speed relationships (§2.2.3–2.2.4):**

```
Design speed  =  Target speed + ~10 km/h        (10 % safety margin; +10 in practice for all interurban classes)
MMM           ≈  Target speed, and MMM is NEVER greater than design speed
Max total design-speed reduction on one road    ≤ 20 km/h
Minimum segment length for a reduction          ≥ 2 minutes of travel at design speed
Adjacent-segment design-speed delta             ≤ 10 km/h (exceptionally 20)
```

**Table 2.5 — design speed and LOS by functional class:**

| Attribute | דרך מהירה | מעויירת/פרברית מהירה | דרך ראשית | דרך אזורית | מקומית וגישה |
|---|---|---|---|---|---|
| Design speed (km/h) | **100–120** | **90–110** | Dual **80–110** *, Single **60–80** | Dual **80–100**, Single **60–80** | **60–80** |
| Design LOS | **C** (low sensitivity) / **D** | **D** | **D** | **D** normal / **E** high sensitivity | **E** |
| Carriageways | ≥ 2 | ≥ 2 | Usually 2, sometimes 1 | Usually 1, sometimes 2 | 1 |
| Access control | Full | Full | Full or partial | Partial or limited | Limited |
| At-grade junctions permitted | **No** | **No** direct | **Yes** | **Yes** | **Yes** |

\* Primary dual fully grade-separated only.

**Table 2.2 — maximum permitted speeds by vehicle and geometry (km/h):**

| Vehicle | Freeway | Suburban express, dual paved | Other dual | Single carriageway |
|---|---|---|---|---|
| Passenger car ≤ 3.5 t, motorcycle | **110** | **100** | **90** | **80** |
| Bus (non-minibus) | **100** | **100** | **90** | **80** |
| Commercial > 12 t | **80** | **80** | **80** | **80** |

**Implementation notes:** the speed tables are keyed by **carriageway type plus access control**, not by functional class alone. A `highway=primary` OSM way can land in three different speed rows depending on whether it is dual or single and whether it is grade-separated. A generator needs at least three independent axes — functional class, carriageway type, access control — and must not derive design speed from `highway=*` alone.

### 8.6 Maximum longitudinal grade — כרך 1 §6.3.1, Table 6.2

| Road class | V_d (km/h) | מישורי (flat) | גבעי (hilly) | הררי (mountainous) |
|---|---|---|---|---|
| בין-עירונית מהירה | 100 | 4 | 5 | 6 |
| מהירה מעויירת וראשית ממוחלפת | 90 | 5 | 6 | 7 |
| ראשית ואזורית דו-מסלולית | 80 | 6 | 7 | 8 |
| ראשית ואזורית חד-מסלולית | 60 | 7 | 8 | 9 |
| מקומית וגישה | 60 | 8 | 9 | 10 |

Adjustments: **+1 %** maximum for environmental sensitivity criteria; **−1 %** on the downhill lane where carriageways have separate alignments (except expressways); **+2 %** maximum for a דלת-תנועה low-traffic road.

**Minimum longitudinal grade (§6.3.2):** **0.4 %** normally; **0.2 %** in cut; grades between 0.2 % and 0.4 % require a longitudinal drainage system. In cut, if a segment has ≤ 1 % longitudinal grade, increase the crossfall to **2.0–2.5 %**.

Note that כרך 2 Table 8.1 caps junction-arm grades tighter than this — **4 %** at 80–100 km/h against כרך 1's 4–8 % — which is the expected relationship: junction arms are held flatter than open road.

### 8.7 Minimum horizontal curve radius — כרך 1 §5.2.1, Table 5.1

| V_d (km/h) | e_max | f | **R_min (m)** |
|---|---|---|---|
| 60 | 0.10 | 0.16 | **110** |
| 70 | 0.10 | 0.13 | **170** |
| 80 | 0.10 | 0.13 | **220** |
| 90 | 0.08 | 0.11 | **340** |
| 100 | 0.08 | 0.10 | **440** |
| 110 | 0.08 | 0.09 | **565** |
| 120 | 0.08 | 0.09 | **670** |

```
R_min = V_d² / [127 · (e_max + f)]
```

`e_max` = **0.10** for single-carriageway roads at 60–80 km/h; **0.08** for divided roads at 90–120 km/h.

Related: **Table 5.3** gives the radius above which a uniform 2 % superelevation suffices (535 / 770 / 1050 / 1390 / 1790 / 2270 / 2760 m for 60–120 km/h), and **Table 5.4** gives `N_R`, the radius above which the normal 2–2.5 % crown is kept with no superelevation transition (1400 / 1900 / 2500 / 3100 / 3800 / 4700 / 5500 m).

These are open-road values. At a junction on a horizontal curve, use **כרך 2 Table 8.2** instead — it is far more restrictive (550 m at 60 km/h versus 110 m).

### 8.8 Compound curves — כרך 1 §5.7.1

Directly relevant because Chapter 6 uses three-centre compound curves for right-turn corners.

| Parameter | Limit |
|---|---|
| Maximum radius ratio between successive arcs | **1 : 1.5** |
| Recommended ratio | **1 : 1.25** |
| Speed difference between arcs | **≤ 10 km/h** |
| Design speed | Set by the smallest radius |
| Permitted use | **Junctions and interchanges only** — not general interurban alignment |

Sanity check against Table 6.1: the symmetric 40-12-40 compound curve has a ratio of 1:3.33, well outside the 1:1.5 open-road limit. That is expected and permitted, because §5.7.1 explicitly carves out junctions from the ratio rule.

### 8.9 Auxiliary-lane tapers on the open road — כרך 1 §3.9

Not junction geometry, but the closest thing כרך 1 has to a lane add/drop rule, and useful as a fallback where כרך 2 is silent.

```
Merge taper   (lane drop)      L_M = 0.6 · V · W
Diverge taper (lane addition)  L_D = 0.4 · V · W
```

At V = 80 km/h and W = 3.6 m: `L_M` = **172.8 m**, `L_D` = **115.2 m**.

Climbing-lane tapers are given as ratios instead (§3.9.3.5א): **1:50** minimum on the exit/merge, **1:25** on the entry/diverge, with **1:40** permitted at entry where curvature aids continuity.

> **Do not apply these to junctions.** The junction lane-drop taper is `1:V_d` from כרך 2 Table 5.1, which at 80 km/h and 3.6 m gives **144 m** — not the 172.8 m that `L_M` would give. The two rules serve different situations.

Emergency lay-by (§3.10.3): widen the shoulder from **3.0 m to 4.5 m** over **30 m** at a **1:20** taper, spaced **2.0 km** apart.

---

## Appendix A — Sight distance and sight triangles (כרך 2 פרק 4)

Not one of the eight requested topics, but extracted alongside Chapter 8 and needed by any junction that is generated with visibility checks.

### A.1 Table 4.1 — stopping sight distance at junctions (metres)

| V (km/h) | t_a (s) | a (m/s²) | Flat ≤ 3 % | Uphill 4–5 % | Uphill 6–8 % | Downhill 4–5 % | Downhill 6–8 % |
|---|---|---|---|---|---|---|---|
| 30 | 4.19 | 0.43 | 35 | 35 | 35 | 35 | 35 |
| 40 | 4.19 | 0.43 | 40 | 40 | 40 | 35 | 35 |
| 50 | 4.19 | 0.43 | 55 | 50 | 50 | 40 | — |
| 60 | 4.19 | 0.43 | 70 | 65 | 65 | 45 | — |
| 70 | 3.96 | 0.40 | 90 | 85 | 85 | 80 | — |
| 80 | 3.76 | 0.38 | 115 | 105 | (105) | — | — |
| 90 | 3.57 | 0.36 | 140 | 130 | (130) | — | — |
| 100 | 3.41 | 0.35 | 170 | 160 | (155) | — | — |

Parenthesised values are grades not recommended at those speeds (Ch. 8). 30 and 40 km/h are approach speeds (§4.2.4ד').

### A.2 Table 4.2 — intersection decision sight distance

| Design speed (km/h) | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 100 |
|---|---|---|---|---|---|---|---|---|
| Manoeuvre speed V_M (km/h) | 28 | 35 | 42 | 51 | 63 | 70 | 77 | 85 |
| **D_S^I (m)** | **70** | **85** | **100** | **120** | **150** | **170** | **190** | **215** |

### A.3 Sight triangle legs (§4.3.3, §4.3.4)

```
[4.3]   L₁ = V_d(major) · t_g / 3.6
```

`L₁` is the main-road leg, `L₂` the secondary-road leg, and `t_g` the critical gap.

**L₂ values (§4.3.4):**

| Control on the secondary road | L₂ |
|---|---|
| Stop sign 302 / stop line | **2.0 m** (vehicle 0.5 m back from the line + 1.5 m to the driver's eye) |
| Yield 301 / decelerating approach | **12.0 m** (the 302 assumptions plus a 10 m decision zone) — private car only |

**Critical gaps t_g (Diagram 4.3), seconds:**

| Manoeuvre | Private car | Bus / coach |
|---|---|---|
| Right turn (triangle A) | 5.5 | 6.5 |
| Left turn / cross (triangle B) | 6.0 | 7.5 |
| Case 1' / 2' combined | 6.5 | 8.5 |
| Case 2, stop sign 302 | 7.5 | 9.5 |
| Left from secondary (case 3) | 6.5 | 8.0 |
| Cross, near lane (A) | 6.5 | 8.5 |
| Cross, far lane (A1) | 7.0 | — |
| Cross (B) | 8.0 | — |

Additions: **+0.5 s** per extra lane crossed for a private car, **+1.0 s** per extra lane for a truck or bus.

**Table 4.5 — L₁ (m) from t_g and main-road speed:**

| V_main ↓ / t_g → | 5.5 | 6.0 | 6.5 | 7.0 | 7.5 | 8.0 | 8.5 | 9.0 | 9.5 | 10.0 | 10.5 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **30** | 45 | 50 | 55 | 60 | 65 | 65 | 70 | 75 | 80 | 85 | 90 |
| **40** | 60 | 65 | 70 | 80 | 85 | 90 | 95 | 100 | 105 | 110 | 115 |
| **50** | 75 | 85 | 90 | 95 | 105 | 110 | 120 | 125 | 130 | 140 | 145 |
| **60** | 90 | 100 | 110 | 115 | 125 | 135 | 140 | 150 | 160 | 165 | 175 |
| **70** | 105 | 115 | 125 | 135 | 145 | 155 | 165 | 175 | 185 | 195 | 205 |
| **80** | 120 | 135 | 145 | 155 | 165 | 180 | 190 | 200 | 210 | 220 | 235 |
| **90** | 140 | 150 | 165 | 175 | 190 | 200 | 215 | 225 | 240 | 250 | 265 |
| **100** | 155 | 165 | 180 | 195 | 210 | 220 | 235 | 250 | 265 | 280 | 290 |

### A.4 Objects in the sight triangle (§4.3.7)

Nothing may exceed **0.80 m** in height inside the sight triangle: trees, fences, concrete barriers, retaining and acoustic walls are all capped at 0.80 m at the top. Bus shelters, awnings, and large billboards are **prohibited outright**. Traffic signs and posts essential for guidance are permitted.

Eye and object heights (§4.3.5): private car eye **1.05 m**, object **1.05 m**; truck check at eye **2.40 m**; bus verification at eye **1.80 m**. On a split main road, check the triangles on each carriageway separately.

**Implementation notes:** the 0.80 m ceiling is a hard constraint on any procedural vegetation or street-furniture scatter placed near a junction. The sight triangle is `L₁ × L₂` with `L₂` = 2 m or 12 m and `L₁` from Table 4.5 — at 80 km/h with a 6.5 s gap that is a 145 m × 12 m wedge per approach.

---

## Appendix B — Index of UNCERTAIN items and gaps

Sixteen items where the transcripts were ambiguous, self-contradictory, or incomplete — **item 1 has since been resolved** against the preserved raw extraction in `docs/research/`, leaving fifteen open. Ordered roughly by how much damage a wrong assumption would do.

| # | Item | Location | Nature of the problem |
|---|---|---|---|
| 1 | ~~**§8.2.3 resultant grade: ≤ 1 % or ≥ 1 %**~~ **RESOLVED** | [§5.6](#56-857--resultant-grade) | Settled against the raw text layer (`docs/research/chapter8_full.txt` lines 380–386): «לא יפחת מ-1%» — a **drainage floor**. «לא יעלה על» occurs nowhere in the file, so the ≤ reading was never in the source. Implemented as a floor. |
| 2 | **Table 7.1 columns transposed** | [§4.3](#43-table-71--deceleration-length-733) | As extracted, the table implies a 140 km/h design speed, which does not exist. Reading the columns the other way lands exactly on Table 6.2. Corrected orientation given. |
| 3 | **Tables 11.7 / 11.8 contradict §11.8א** | [§7.7](#77-entry-and-exit-design-118) | §11.8א says entry/exit kerb radii are 12–20 m; the tables give 4.4–9.7 m for the same stated quantity. Use the §11.8א range. |
| 4 | **Formula [3.1] extracted two incompatible ways** | [§1.5](#15-four-arm-junctions-and-split-junctions) | The two recorded forms are near-reciprocals and **neither is dimensionally correct**. A dimensionally-consistent reconstruction is offered but unverified. |
| 5 | **§3.3.2 "may narrow through lanes" at signalised junctions** | [§0](#0-the-single-most-important-finding) | Directly contradicts the verbatim §5.2.1 prohibition. Treat §5.2.1 as authoritative. |
| 6 | **Table 8.4 (`R_d` and correction factor) is scrambled** | [§5.10](#510-86--grade-transitions-on-the-junction-arms) | Column headers misassigned. Use the equivalent clean **כרך 1 Table 5.7** instead. |
| 7 | **Table 11.5 column labels garbled** | [§7.4](#74-table-115--circulatory-width-by-r_c-and-design-vehicle-p-11-12) | Grouping into single/two-lane is confident; the design vehicle per column within each group is inferred from ordering, not read. |
| 8 | **Table 2.4 (כרך 2) R₁/R₂ meaning** | [§6.1](#61-table-24--geometric-characteristics-261-p-2-38) | Column headers not captured. R₂ is a constant 5.30 m across all heavy vehicles, suggesting it may be a regulatory value rather than a radius. Use the figure 2.14–2.22 swept paths instead. |
| 9 | **§6.2 angle sentence inverted** | [§3.1](#31-table-61--corner-radii-for-a-right-turn-without-an-island-62-p-6-3) | Recorded as "avoid 70°–110°", almost certainly an RTL negation flip of "avoid outside 70°–110°". |
| 10 | **Junction angle floor: 70° or 75°?** | [§1.2](#12-junction-angle) | §3.2.1 says 70°; §4.3.5ב says not below 75° for sight. Probably a geometric limit versus a sight limit, but unconfirmed. |
| 11 | **Table 8.1 bottom rows incomplete** | [§5.7](#57-table-81--longitudinal-grades-on-junction-arms-822) | The 40 km/h minimum-length cell reads "30–40 (T-junction)"; the 30 km/h cell is empty. |
| 12 | **Table 8.3 concave-passing K column empty** | [§5.9](#59-table-83--minimum-vertical-curve-k-values-at-junctions-84) | Not extracted. |
| 13 | **§8.5.7 possible separate ceiling on resultant grade** | [§5.6](#56-857--resultant-grade) | One transcript claims §8.5.7 references Table 8.1 for a maximum on high-class roads. If it exists it is separate from the §8.2.3 floor, and **its value was never extracted**. |
| 14 | **§8.2.4 המרווח החופשי (גבריט)** | [§5.7](#57-table-81--longitudinal-grades-on-junction-arms-822) | **Not extracted** — the word does not appear in the PDF text layer at all. Re-read internal page 8-3 and Ch. 6 §6.2(ה). |
| 15 | **Table 2.1 (כרך 2) control × channelization matrix** | [§6.4](#64-conflict-points-by-junction-form-223) | Extracted in a degraded form with several rows that do not parse cleanly. The junction-type abbreviations (מ.מ. / מ.ח. / מ.ב.מ.) are reliable; the row-by-row matrix is not fully trustworthy. |
| 16 | **`X_cr` separated-merge overlap coefficients (כרך 1 §3.9.2.2ז)** | [§8.9](#89-auxiliary-lane-tapers-on-the-open-road--כרך-1-39) | Formula `X_cr = c_t · d · V^0.833` recorded, but **`c_t` and `d` were never tabulated** in the extracted text. Only the "≥ 3 s travel-time separation" rule is usable. |

### Topics with no data at all

None of the eight requested topics is a wholesale gap. Every one has real numbers from the source. The gaps are the individual cells and coefficients listed above.

Two adjacent topics were confirmed *absent* from the junction volume and are pointers rather than gaps:

- **High-speed turning roadway radii, widths, and superelevation** — כרך 2 §6.3.7 explicitly delegates to **כרך 3 §§5.3–5.7**. Not extracted — read כרך 3 if free-flow right-turn ramps are needed.
- **Interchange design** — delegated to **כרך 3 (מחלפים)** via §3.1.3–3.1.5. Not extracted.

---

## Appendix C — Consolidated numeric quick-reference

Everything a generator needs most often, in one block. Every value is cited above.

```text
# ---- Lane continuity (כרך 2 §5.2) ----
through_lanes_in_junction >= upstream_through_lanes        # §5.2.1, hard rule
lane_width_in_junction    >= upstream_lane_width           # §5.2.1, hard rule
added_or_dropped_lane      = RIGHTMOST                     # §5.2.4
signalised_cap_dual        = {through: 3, left: 3}         # §5.2.3
signalised_cap_single      = {through: 2, left: 1}         # §5.2.3
unsig_minor_lanes_at_stop  = 1  (2 if turn-separated)      # §5.2.2

# ---- Post-junction lane drop (כרך 2 Table 5.1) ----
l_full_accel   = {80: 45, 90: 50, 100: 55}                 # 2 s
l_short_accel  = {80: 65, 90: 75, 100: 85}                 # 3 s
l_no_accel     = {80: 90, 90:100, 100:110}                 # 4 s
taper_d        = {80: 40, 90: 45, 100: 50}                 # 1:d
# V_d 60 and 70 use the 80 row.

# ---- Junction angle (כרך 2 §3.2.1) ----
angle_preferred = 90
angle_range     = [70, 110]

# ---- Stagger (כרך 2 §3.7) ----
stagger_min_undivided = [60, 80]
stagger_min_dual      = [100, 120]
stagger_max_span      = 250

# ---- Right turn, no island (Table 6.1, 90 deg, bus) ----
R_simple        = 15.0
R_compound_sym  = [40, 12, 40]
R_compound_asym = [40, 12, 60]
edge_offset     = 0.5

# ---- Right turn, with island (§6.3.1) ----
R_min           = {75: 27, 90: 20, 105: 20}   # prefer 25 at 90 deg
turn_lane_width = 5.5 | 6.0                   # + 0.5 painted shoulder
width_at_radius = 4.5
decel_anchor_w  = 3.0

# ---- Deceleration, right turn (Table 6.2, to 25 km/h) ----
L_decel_25 = {50: 24, 60: 38, 70: 57, 80: 82, 90: 102, 100: 127}
L_decel_stop = {50: 36, 60: 50, 70: 69, 80: 95, 90: 114, 100: 139}
grade_factor = {up_2.5_4.4: 0.90, down_2.5_4.4: 1.20,
                up_4.5_6.0: 0.80, down_4.5_6.0: 1.35}      # Table 6.3

# ---- Right-turn lane tapers ----
taper_parallel_main = {"<=70": 10, ">=80": 15}              # §6.3.4
taper_diagonal      = {60: 30, 70: 40, 80: 40, 90: 50, 100: 60}   # Table 6.4
reverse_curve_R     = {80: 250, 90: 350, 100: 450}          # Table 6.4
taper_into_radius   = [30, 60]                              # 1:30 - 1:60
taper_out_of_radius = [40, 60]

# ---- Acceleration (Table 6.5) ----
L_accel_full = {70: 100, 80: 140, 90: 170, 100: 210}
L_accel_uphill_gt_4pct = L_accel_full * 1.5
L_accel_short = [90, 105]                                   # taper 1:60 - 1:70
accel_close_taper = [20, 35]                                # 1:20 - 1:35

# ---- Triangular island (§6.5.2, §6.5.3) ----
island_area_min   = 10.0      # m2; 9.0 absolute in constrained retrofits
island_side_min   = 4.0
island_nose_setbk = 0.6
island_corner_R   = {small: 0.5, med_main: [0.6, 0.9], med_sec: [0.3, 0.5]}
island_side_large = 30.0

# ---- Left turn (כרך 2 פרק 7) ----
taper_ratio_left  = {"<=70": 10, ">=80": 15}                # §7.3.2
L_taper           = taper_ratio * W                          # W = 3.6 -> 36 or 54
centreline_shift  = 1 / (V_d / 2)                            # §7.3.6ב
L_decel_left      = {50: 35, 60: 50, 70: 70, 80: 95, 90: 115, 100: 140}  # Table 7.1, CORRECTED
L_storage_min     = 20                                       # §7.3.4, §7.3.5
L_storage_by_n    = {1:20, 2:25, 3:31, 4:37, 5:43, 6:56, 7:62, 8:68, 9:74, 10:80}
R_min_left        = 15.0     # 12.5 in small junctions with heavy queues
opposed_inner_sep = 9.0      # §7.2.3
parallel_inner_sep= 5.0      # §7.2.4
exit_lane_width   = 4.5
merge_lane        = {length: 60, width: 3.0, taper: 40}      # §7.3.7

# ---- Junction elevation (כרך 2 פרק 8) ----
max_grade_on_arm  = {100: 4, 90: 4, 80: 4, 70: 6, 60: 6, 50: 6, 40: 6, 30: 6}
min_length_at_max = {100: 85, 90: 75, 80: 70, 70: 70, 60: 60, 50: 50}
min_long_grade    = 0.005                                    # 0.5 %, §8.2.3
resultant_grade   = ">= 0.01"                                # drainage FLOOR, §8.2.3 (confirmed)
normal_crossfall  = 0.02
delta_n_max       = {30:0.75, 40:0.70, 50:0.67, 60:0.64, 70:0.56,
                     80:0.50, 90:0.46, 100:0.43}             # Table 8.5
K_convex_stop     = {40:410, 50:760, 60:1230, 70:2035, 80:3320, 90:4915, 100:7250}
K_concave_stop    = {40:410, 50:645, 60:930, 70:1260, 80:1650, 90:2085, 100:2575}
R_min_on_curve_2pct = {60:550, 70:770, 80:1100, 90:1400, 100:1800}   # Table 8.2

# ---- Roundabout (כרך 2 פרק 11) ----
R_c_initial   = {("single",35): 12, ("single",45): 16,
                 ("two",35): 26,   ("two",45): 30}           # Table 11.4
R_c_min       = {"single": 8, "two": 12}
W_c           = "Table 11.5[R_c][vehicle]"
D_inscribed   = 2 * (R_c + W_c)
R_en = R_ex   = [12, 20]                                     # §11.8א
flare_length  = [15, 100]
entry_angle   = [25, 65]
arm_spacing   = 20                                           # minimum
R_def_max     = {("single","-2%"): 38, ("single","+2%"): 45,
                 ("two","-2%"): 68,    ("two","+2%"): 95}    # Table 11.6
apron         = {width: [1.0, 1.5], height_cm: 7, crossfall: 0.03}
crossfall_circ= 0.02                                          # up to 0.06 if crowned
max_design_speed_for_roundabout = 80

# ---- Cross-section (כרך 1) ----
lane_width_default = 3.6
lane_width = {"expressway_120": 3.7, "expressway": 3.6, "dual_main": 3.6,
              "single_main": 3.6, "single_regional_80": 3.6,
              "single_regional_70": 3.5, "single_regional_60": 3.3,
              "local": 3.5, "local_deadend": 3.0}            # כרך 1 Table 3.1
shoulder_right = {"interurban_fast": 3.5, "suburban_fast": 3.0,
                  "dual_main": 3.5, "dual_regional": 3.0,
                  "single_main": 3.0, "local": 2.0}          # כרך 1 Table 3.2
shoulder_median_inner = 1.2       # 3.0 if >= 3 lanes/dir at 130 design
median_min = {"2_3_lanes": 2.4, "4plus_lanes_120": 6.0}      # + barrier width
median_min_at_junction = 6.0      # כרך 2 §7.4.3 -- stricter, use this
median_after_left_lane = 2.5
crossfall_carriageway = [0.020, 0.025]
design_speed = {"freeway": [100,120], "suburban_express": [90,110],
                "primary_dual": [80,110], "primary_single": [60,80],
                "regional_dual": [80,100], "regional_single": [60,80],
                "local": [60,80]}                            # כרך 1 Table 2.5
R_min_horizontal = {60:110, 70:170, 80:220, 90:340, 100:440, 110:565, 120:670}
```

---

*Compiled from agent transcripts of the source PDFs. Every numeric value above carries its section or table citation; anything the extraction left ambiguous is marked UNCERTAIN and indexed in Appendix B. Before implementing an UNCERTAIN value, re-read the internal page cited next to it.*
