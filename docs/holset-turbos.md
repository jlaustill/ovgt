# Holset H1-Frame Turbo Reference (HX35 / HX40 / HY35 / H1C)

Research notes for turbo selection on the test truck. Captures what interchanges
across the Holset H1 frame, the unique factory configurations, aftermarket parts,
and the specific build currently being assembled.

**Status:** research + parts-gathering. Started 2026-07-24.

> **Relevance to the north star:** every turbo in this document is **fixed
> geometry** — no vanes. None of it feeds the CE *control* mission
> ([`north-star.md`](north-star.md)); CE stays read-only/monitoring on any of
> these. The [HE351VE](#adjacent-the-he-frame) remains the only path in the Holset
> family back to a controllable nozzle.

---

## The core interchange fact

One **"H1 frame" bearing housing** is shared by **H1C, WH1C, HX35, HX35W, HY35W,
H1E, WH1E, HX40, HX40W** — same 11 mm journal, 7 mm shaft thread, same oil
feed/drain. That single fact is why this whole mix-and-match hobby exists. Rebuild
kits ship as one SKU covering HX35 / HX35W / HY35 / HX40 / HE351.

Directly confirmed in hand: bearing housing **3530521** (82 mm OD, on the 3592766
HX35W core) is catalogued under both **HX35** and **HX40** assemblies.

⚠️ But see the [HY35W exception](#️-2026-07-24-the-cores-on-hand-are-hy35w-not-hx35)
— the shared bearing housing does **not** imply a shared turbine rotating assembly.

Naming: a leading/trailing **`W` means wastegated** (HX35**W**, **W**H1C).

---

## Unique factory configurations

| Model | Compressor (ind/exd, blades) | Turbine (ind/exd) | Turbine housing | Comp cover | Application |
|---|---|---|---|---|---|
| **H1C** | 54/~82, 8 bl | 70/60, 12 bl straight | 14 or 16 cm², undivided, **no WG** | bolted | '89–93 Dodge 5.9, 6BT industrial |
| **WH1C** | 56/82, 8 bl | 70/60, 12 bl straight | 12 cm², divided, int. WG | **V-band** | '94–95 Dodge 5.9 |
| **HX35 / HX35W (8-blade)** | 56/82, 8 bl | 70/60, 12 bl straight | 12 cm², divided, int. WG, 3" V-band out | bolted | '95–98 12v |
| **HX35 / HX35W (7-blade)** | 56/76.5, 7 bl | 70/60, 12 bl straight | 12 cm², divided, int. WG | bolted | '98.5–02 24v manual |
| **HX35 industrial** | 56/82 or 56/76.5 | 70/60 | 14 or 16 cm², **no WG**, V-band or 4-bolt out | bolted | 6BT industrial, gensets |
| **HY35W** | 54/76.5, 7 bl | 65.5/58, 12 bl **curved** | 9 cm², **undivided**, int. WG | bolted | '00–02 24v automatic |
| **H1E** | ~58, 8 bl | 76/64 | 16–18 cm² typical | bolted | industrial / medium duty |
| **WH1E** | 58, 8 bl | 76/64 | 4-bolt CHRA→housing pattern | **V-band** | industrial; ~60 lb/min |
| **HX40 (8-blade)** | 58/83 or 60/83, 8 bl | 76/64, 12 bl | 14 / 16 / 18 cm² | bolted | 6BT/6CT, swaps |
| **HX40 "Super 40" (7-blade)** | 60/86, 7 bl | 76/64, 12 bl | 14 / 16 / 18 cm² | bolted | the popular swap spec |
| **HX40 (6-blade)** | 56/86 or 60/86, 6 bl | 76/64 | 14 / 16 / 18 cm² | bolted | European / industrial |
| **HX40W** | ~60/86 | 76/64 | 16 or 19 cm², often non-gated, 6-bolt or 4" out | bolted | ISC 8.3, Class 3–8 |

**Still disputed — the 7-blade HX35 compressor inducer, 54 vs 56 mm.** Mopar1973Man
says **56**/76.5; vendor listings for PN 3592766 (a '99–02 7-blade HX35W) say
**54**/77. An in-hand attempt on 2026-07-24 could not resolve it: tip-to-tip
calipering on a 7-blade wheel is geometrically unreliable and the corrected range
straddles both answers.

Likely explanation for the dispute itself: **uncorrected or half-corrected
tip-to-tip readings.** A true 56 mm 7-blade measures 53.2 mm across the tips — close
enough to "54" that a casual measurement publishes the wrong number. Treat *every*
compressor figure in these forum tables as potentially a couple of mm low.

### cm² is not A/R

Holset rates turbine housings by **throat area in cm²**, not A/R — there is no clean
conversion (A/R also depends on the radius the throat sits at). Vendor-quoted rough
equivalences, useful only for cross-shopping:

| cm² | ≈ A/R |
|---|---|
| 9 | .48–.55 |
| 12 | .60–.63 |
| 14 | .70 |
| 16 | .83 |
| 18 | .96 |

---

## What actually swaps

| Part | Rule |
|---|---|
| **Cartridge / CHRA** | Swaps across the H1 frame. Oil lines, mounting, drain all common. ⚠️ **Exception: the HY35W.** Its turbine rotating assembly is not shared — the 70/60 wheel & shaft (3519336) that fits HX35/HX35W/H1C/WH1C is explicitly *not* sold for the HY35W. See [the 2026-07-24 finding](#️-2026-07-24-the-cores-on-hand-are-hy35w-not-hx35). |
| **Compressor cover** | Two mounts: **bolted** vs **V-band** (V-band on WH1C, WH1E). Cover contour is wheel-specific. A bigger wheel needs a matched cover **and** machining: bearing housing bore → ~88 mm, backplate from ~84.5 mm out to ~87 mm. |
| **Turbine housing** | Must match the **turbine wheel** it wraps. Two wheel families: **70/60 straight** (H1C/HX35) and **76/64** (H1E/HX40). An HX40 housing physically bolts over an HX35 wheel but leaves a huge tip gap — fits, doesn't work. **Asymmetry worth remembering: a housing can be bored *out* on a lathe to accept a bigger wheel, but never filled *in* to accept a smaller one.** (Done in practice — the DPS Turbonator housing was lathe-bored to clear the S363's 80 mm wheel.) So mismatches in the too-small-wheel direction are dead ends; too-big-wheel mismatches are often machinable. |
| **Turbine wheel** | 12-bl straight 70/60 (HX35) · 12-bl **curved** 65.5/58 (HY35/HE341/HE351) · 12-bl 76/64 (HX40). The curved wheel pairs with the 9 cm undivided housings. |
| **CHRA→housing pattern** | HX35/HX40 use the standard clamp/bolt pattern; WH1E uses a **4-bolt** pattern that drops into HX40-style (incl. Bullseye) housings. |
| **Turbine inlet flange** | HX35 12 cm twin-scroll is a **T3 pattern** (M8, 3.375" × 1.75" centers). ⚠️ Dodge-sourced castings often have **2 through-holes + 2 threaded holes** — the threaded pair must be drilled out to slip over manifold studs. |
| **Turbine outlet** | **HX35, HX40, and HY35 share the same 5-bolt outlet flange** — which is why one DPS adapter covers all three. Some housings add a removable 3" V-band flange on top of that 5-bolt face; the DPS adapter **replaces that 5-bolt flange and reuses its gasket**. Medium-duty HX40W goes 6-bolt / 4". |
| **T3/T4 adapters** | T3→T4 and Holset-flange adapter plates exist (AZ CNC and others). |

---

## Aftermarket options

| Category | Available |
|---|---|
| **Billet compressor wheels** | 60 mm billet + matched cover (HX35/40, Fenley) · BAE 62/86 and 66/84 · DAP 7+7 billet HX35 (repl. 3599649) · HX40 11+0 67×89×95 billet · Wicked Wheel / WW2 (reblade of stock) |
| **Turbine wheels** | 10-blade 67/76 upgrade (H1C/H1E/HX35/HX40) · Super40 10-bl 64/76 · Mamba D5 64/76 5+5 · AVP wheel+shaft for HX35/HX35W/HY35/H1C |
| **Turbine housings** | Bullseye (BEP) T3 .55 / .70, stainless .70 A/R w/ 3" V-band · T3 .82 open-scroll non-gated for the 70/60 wheel · Turbo Lab HX35 & HX40 housings · PDP 14 cm non-gated (H1C/HX35, '89–02) · OEM 16 cm V-band (4038909H) · Super HX40 T3 twin-entry 14/16/18 cm² |
| **Complete upgraded turbos** | Fleece **Cheetah** HX35 63 mm (~650 hp, 40–45 psi) · Industrial Injection PhatShaft · CPP drop-ins · Turbo Lab 67 mm HE351CW kit (~750 hp) · Speeding Parts Super HX40 Competition (65/75 turbine, 14/16/18 cm) |

### How many combinations, really

Nominal math is ~200+ (11 compressor wheels × 7 housings × 3 turbine wheels), but
constraints collapse it:

- Turbine wheel **binds** you to a housing family (70/60 vs 76/64) → 2 branches.
- Compressor wheel **binds** you to a cover + machining spec → ~4 practical steps
  per branch (56 / 58–60 / 62–63 / 66–67 mm).
- Housing size is the only genuinely free axis: ~4 useful sizes per branch.

**≈30 sane builds, ~10 that people actually run.** Hard ceiling: the HX35
compressor **chokes past ~58 mm** — bigger wheels cavitate rather than flow, which
is exactly why Holset stepped to the HX40 frame instead of fitting a bigger wheel.
Past ~65 lb/min, an S300/S400 frame beats stacking parts on an HX35.

---

## Frame-size intuition

Turbines on hand, for scale:

| Wheel | Inducer | Relative flow area (D²) | Relative rotational inertia (~D⁵) |
|---|---|---|---|
| HY35W | 65.5 mm | 1.00× | 1.00× |
| DPS S362 | 78 mm | 1.42× | ~2.4× |
| DPS S363 | 80 mm | 1.49× | ~2.7× |

DPS default their S362 to a **78 mm** turbine and the S363 to **80 mm** — larger than
the generic S300 turbine sizes usually quoted.

The point: flow area scales with D², but **rotational inertia scales with roughly
D⁵** (mass ∝ D³, radius of gyration ∝ D). A wheel that looks moderately bigger is
dramatically harder to accelerate. That gap *is* the post-upshift hole that killed
the big-compound plan — and it's the entire pitch for variable geometry, which
presents a small effective throat to a big wheel to get the small turbine's response
with the big turbine's flow ceiling.

**Shop capability note:** lathe access is available and housings have been bored
in-house, so machining-based options are genuinely on the table, not theoretical.

## Adjacent: the HE frame

**HE341CW, HE351CW, HE351VE** are a later, separate casting family — *not* members
of the H1 interchange pool.

| Model | Compressor | Turbine | Housing | Application |
|---|---|---|---|---|
| HE341CW | 56/76.5, 7 bl | 65.5/58 curved | 9 cm², undivided, WG | '03–04 24v |
| HE351CW | 60/84.5, 7 bl | 65.5/58 curved | 9 cm², undivided, WG | '04.5–07 5.9 |
| HE351VE | — | — | **variable nozzle** | 6.7 OEM VGT |

HE351VE actuator is J1939 CAN (PGN 0xFFC6 from SA 0, ID 0x18FFC600, 3 bytes) — drops
into OVGT's existing FlexCAN/J1939 stack. See the HE351VE notes in project memory.

---

## Current build: 12 cm wastegated housing on an existing HX35

**Goal:** 70/60 turbine in a **divided T3, 12 cm², internally wastegated** housing
whose **actuator mounts to the turbine housing only** (no compressor-cover bracket),
so the DPS exhaust adapter makes it a direct bolt-in to the current setup.

### Part number verification (2026-07-24)

| PN | Verdict |
|---|---|
| **3532214** | ✅ **CONFIRMED.** Multiple independent vendors list it identically: *"Holset 12cm Wastegated HX35 Turbine Housing. Fits 1988–1998 Dodge Cummins 5.9L 12V with HX35, H1C, or WH1C. **Long housing.** Also fits 1998–2002 if you remove the V-band flange (5 bolts) and bolt the elbow directly. **Actuator is contained on the turbine side and does not need to bolt to compressor housing.**"* Bored for the 70/60 wheel, divided, T3 inlet. ~**$599** at Diesel Auto Power. |
| **3591217T** | ⚠️ **UNVERIFIED.** Absent from both major public Holset PN references (Boost Lab, J&H Diesel). Only web trace is an eBay listing titled *"**Unmarked** Cummins Holset 3591217 Turbo Turbine Exhaust"* — unmarked = no Holset number cast in. The **"T" suffix is not a Holset convention** (genuine service parts use **"H"**: 3537817**H**, 4038909**H**). Almost certainly an **aftermarket reproduction** of the 3532214; the "interchange/superseded" fields are the *seller's* claim, not a Holset supersession record. ~**$120** on eBay. |

**Decision:** buy the **3591217T** first. At 5× the price difference the gamble is
worth it, and repro 12 cm housings generally work fine. Fall back to a genuine
3532214 only if the casting is wrong.

### ⚠️ 2026-07-24: the cores on hand are HY35W, not HX35

Inspection of the cores turned up **HY35W**, not HX35. This breaks the plan above,
because the two are *not* in the same turbine-wheel family:

| | HY35W (what's on hand) | HX35 (what the housing needs) |
|---|---|---|
| Turbine wheel | **65.5/58, 12 bl curved** | **70/60, 12 bl straight** |
| Stock housing | 9 cm², **undivided** | 12 cm², divided |
| Compressor | 54/76.5, 7 bl | 56/82 or 56/76.5 |

**The 12 cm housing is bored for the 70/60 wheel.** Dropping an HY35W cartridge in
leaves an oversized tip gap — the same failure mode as an HX40 housing over an HX35
wheel: it bolts up, it doesn't work.

**And you cannot rebuild past it.** AVP's 70/60 turbine wheel & shaft (OEM
**3519336**, 173.50 mm OAL, 11 mm journal) lists fitment for HX35 / HX35W / H1C /
WH1C and states explicitly: *"We don't list the HY35W for this item because it will
not fit the HY35W turbo."* The commonly-repeated explanation is a differing bearing
housing / shaft length, but that reason is forum inference — the sourced fact is the
fitment exclusion itself.

So the H1-frame "everything interchanges" rule has a real boundary: **the CHRA
family is shared, but the HY35W's turbine rotating assembly is not.**

#### Options from here

| Option | Cost | Notes |
|---|---|---|
| **A. Source a real HX35 / H1C / WH1C core** | used core | Plan proceeds untouched; the $120 housing stays the right buy. Cheapest path to the stated goal. |
| **B. Keep HY35W, find a bigger divided housing for the 58/65.5 wheel** | ? | Thin market — the curved-wheel family (HY35/HE341/HE351) is almost exclusively 9 cm undivided. |
| **C. Rebuild HY35W to a larger turbine wheel + matching housing** | highest | The "67 mm HE351CW-style" upgrade path; aftermarket HX40-style housings exist for HY35/HE341/HE351 *when paired with a 76/67 wheel*. Most money, most machining. |

**Also worth weighing:** the HY35W is a **54 mm compressor on a 9 cm housing** — the
smallest configuration in the H1 frame, sized for a stock '00–02 automatic. Against
a ~400 hp / ~45–50 lb/min target it is likely undersized regardless of what housing
goes on it, which argues for option A on merit and not just on cost.

### Test plan: stock-spec baseline, then iterate

**Configuration under test:** **54 mm** compressor (assumed — see the
[open measurement](#-2026-07-24-second-core-is-an-hx35w--build-is-on)) · 70/60
turbine · **divided 12 cm²** housing (PN 3592766 core + 3591217T housing).
Deliberately the stock HX35 spec — the single best-documented point in the whole
catalog, so deviations are interpretable rather than mysterious.

If 54 mm holds, the outgoing HY35W carries the *same* compressor, making this a
**hot-side-only change** — turbine 58→60 mm exducer, housing 9 cm undivided → 12 cm
divided, no compressor confound. Clean attribution, and a better experiment than the
originally-intended 56 mm would have given. If it turns out to be 56 mm, the swap
changes both ends and attribution gets muddier.

**Method:** install → log → adjust wastegate spring → decide bigger or smaller. The
spring is the only tuning axis that doesn't require pulling the turbo, so it gets
exercised first.

**Sizing sanity:** at 54 mm the compressor is worth roughly **45–50 lb/min** against
a ~45–50 lb/min target for 400 hp — operating *at* the ceiling, not inside it. At
56 mm it'd be ~50–55 lb/min, i.e. genuine margin. Either way: if CE falls off at
peak flow, the compressor is the constraint and the answer is a billet 60–62 mm
wheel, not a different turbine.

#### Decision criteria — pre-register these before collecting data

The wastegate is the cleanest turbine-sizing signal because it isolates the turbine
from the compressor:

| Observation | Reading |
|---|---|
| Gate stays **shut** at target boost under load | Turbine is the limit → **go bigger** |
| Gate **cracks open early / often** | Turbine headroom exists → could **go smaller** for response |

Supporting signals, all already logged by the telemetry stack:

| Signal | Too small | Too big |
|---|---|---|
| **BPR** (drive/boost, target 1.5) | climbs steeply at high load → turbine choking | stays low but boost is lazy |
| **CE** (the north-star objective) | peaks then falls off at high flow → compressor past the map's right edge | poor at low flow / surge |
| **TIT / EGT** | rises with BPR — the turbine-too-small tell, and the safety limit | — |
| **Time-to-boost** from a repeatable trigger | — | lazy |

**TODO — set the actual thresholds.** These are judgment calls that depend on how
the truck is used, and should be written down *before* the first pull:

- Peak BPR that means "too small": `____`
- CE floor at peak flow worth acting on: `____`
- TIT ceiling (safety, not sizing): `____`
- Time-to-boost that counts as "lazy": `____`

#### Baseline validity

Prior hard-won lesson on this truck: a loose/leaking turbo bleeds drive pressure and
produces underboost that looks *exactly* like a sizing problem. Do not out-tune a
mechanical leak.

**Largely addressed:** the turbo now runs **Stage 8 locking fasteners on studs**.
These lock mechanically via a retaining clip rather than by friction, so they cannot
back out — which is precisely the prior failure (a turbine bolt that walked all the
way out). Fastener rotation is off the table as a drive-pressure leak path.

Residual leak paths, if underboost ever reappears: a **crushed or relaxed gasket**,
a **warped flange**, or a **stretched/cracked stud** — none of which involve a nut
turning, so Stage 8 won't catch them. Much rarer, but they're what's left to check
before concluding "turbine too small."

Secondary: at ~4000 ft the pressure ratio for a given manifold pressure is higher
than sea-level map talk assumes, which pushes operation further up the compressor
map. Factor that in when reading CE against published maps.

### ✅ 2026-07-24: second core is an HX35W — build is on

**PN 3592766 — confirmed genuine Holset HX35W.** Listed identically by multiple
vendors (TurboTurbos, Goldfarb):

> Model designation **HX35W-E7755M/E12PY11**. Fits **1999–2002 Dodge Ram 2500/3500,
> Cummins 6BT / ISB 5.9L**. Alternate PNs: 3592767, 3774569, 3800799.

**Turbine: 70/60 mm, 12 blades, 12 cm² T3 twin-entry housing** — exactly the wheel
the 3591217T housing is bored for. The 12 cm housing plan proceeds on this core.

**Compressor: 54 mm per the vendor listings — measurement NOT yet conclusive.**
Tip-to-tip readings on 2026-07-24 came in at 52–54 mm, which after the 7-blade
odd-count correction back-solves to **54.7–56.8 mm** — spanning both candidate
specs. The caliper-on-blades method cannot settle 54 vs 56 on this wheel.

Working assumption is **54 mm**, on the strength of the vendor listings for PN
3592766 (54/77) rather than the measurement. **Pending: cover-bore measurement**
(planned 2026-07-25 morning) to make it firm.

> #### ⚠️ Measuring an odd-blade compressor wheel
>
> With an **odd** blade count no blade is diametrically opposite another, so a
> caliper reading is always **small**. Flat jaws don't touch two tips at points —
> one jaw rests on a single tip while the opposite jaw **bridges two tips**:
>
> ```
> M = R · (1 + cos(180°/N))          actual D = M · 2/(1 + cos(180°/N))
> ```
>
> Sanity check: for a 3-flute end mill this gives `M = 0.75 D`, which matches the
> well-known machinist rule that a 3-flute measures 75 % of its diameter across the
> flutes. Rotating the wheel does not help — every orientation is identical.
>
> | Blades | M / D | Correction |
> |---|---|---|
> | 3 | 0.7500 | × 1.3333 |
> | 5 | 0.9045 | × 1.1056 |
> | **7** | **0.9505** | **× 1.0521** |
> | 9 | 0.9698 | × 1.0311 |
> | 11 | 0.9797 | × 1.0207 |
>
> Even-blade wheels (6, 8, 12) have opposed tips and measure true.
>
> **This method cannot resolve 54 vs 56 mm here.** A 52–54 mm tip-to-tip spread
> back-solves to **54.7–56.8 mm** — straddling both candidates. Uncorrected or
> half-corrected tip readings are a plausible source of the 54-vs-56 disagreement
> across the published tables.
>
> **Use the cover bore instead.** Caliper the **compressor cover's inducer bore** —
> a true circle, no blade geometry — and subtract ~0.5–0.8 mm total tip clearance.
> *(Earlier revision of this doc used a chord model giving × 1.0257; that assumed
> point contact on two tips and understates the correction. Superseded.)*

**Consequence — the two cores share essentially the same compressor.** HY35W is
54/76.5 7-blade; this HX35W is 54/77 7-blade. For '99–02, HY35W and HX35W differ
almost entirely on the **turbine** side (58 vs 60 mm exducer, 9 cm undivided vs
12 cm divided). That makes the swap a **clean hot-side-only experiment** — anything
the logs show is attributable to turbine and housing, not confounded by a compressor
change. Better experimental hygiene than the originally-intended 56 mm would have
given.

**Flow margin is tight.** 54 mm is worth roughly **45–50 lb/min** against a ~45–50
lb/min target for 400 hp — i.e. operating *at* the compressor's ceiling, not near
it. Expect CE to fall off at peak flow; if it does, the **compressor** is the
binding constraint, not the turbine, and the upgrade path is a **billet 60–62 mm
wheel + machined cover** (see aftermarket table above; requires opening the bearing
housing bore to ~88 mm and the backplate to ~87 mm — in-house lathe work is
available).

**Center section: `3530521`** (the last digit is faint on the casting — initially
misread as 6-digit `353052`). This is the **bearing housing**, 82 mm OD, and it is
listed under *both* HX35 and HX40 assemblies: HX35W gensets (3597508/3530521),
HX40M (4035800/3530521), 6CT HX40W, and 1993 6BT. Nice corroboration of the shared-
frame thesis this document rests on — the same casting number appears under HX35W
and HX40W assemblies.

Everything on this core is internally consistent and genuine: HX35W assembly
**3592766** built on the common **3530521** center section.

### Identifying a used core

**Holset service part numbers are 7 digits** (3532214, 4035199, 3519336). Anything
else on the castings is not a part number.

**The data tag is the authority.** Riveted to the compressor housing or, on many
models, the center bearing housing — on HX35/HX40 look *near the compressor outlet
or on the flat machined pad of the bearing housing*. It carries **model + Cummins/
Holset part number + serial**. Older H1C/H1E may be **stamped into the casting**
instead of tagged. VE models (HE351VE) put the plate on the actuator side.

Model format: first letter(s) = frame family (HX / HE / H1 / H2); the number =
relative compressor size; suffix **`W` or `CW` = conventional wastegated**,
**`VE` = variable geometry**.

**Casting numbers are secondary** — they can confirm frame family and sometimes the
housing variant when the tag is long gone, but they are not part numbers.

Observed on the HY35W core (2026-07-24), neither identifiable:

| Marking | Assessment |
|---|---|
| `8654-PP` | 4 digits + letter suffix — signature of a **die-cast mold/pattern number**. Foundry identifier, no model information. |
| `2028600067` | 10 digits, nowhere near Holset's 7-digit format. Likely a supplier lot/traceability number. |

**Resolved — the markings don't matter here.** The HY35W ID is settled by two
independent lines of evidence: known provenance (the specific truck it came off, an
'01 automatic) and physical measurements that match HY35W spec exactly. Casting
numbers are only the fallback for an unknown-history core with no tag. Closed
question; don't reopen it on the strength of an unidentified foundry mark.

**For this build, model ID is not actually the thing that matters.** Two
measurements decide everything:

1. **Turbine wheel exducer** — 58 mm (HY35W family, unusable with the 12 cm
   housing) vs 60 mm (HX35 family, correct).
2. **Turbine housing** — undivided single-entry 9 cm (HY35W) vs divided 12 cm
   (HX35). Visible at a glance, no tools.

A tag is nice confirmation; those two facts are the decision.

### Measurement checklist

Before/after the housing arrives — nobody wants to do this in 37 °C:

- [x] ~~**Count compressor blades**~~ — done 2026-07-24: first core is **HY35W**,
      corroborated by provenance (came off an '01 automatic — exactly the HY35W
      application). See above.
- [x] ~~**Second core**~~ — arrived 2026-07-24, **confirmed HX35W**, PN
      **3592766**. See below.
- [~] **Caliper the 3592766 compressor inducer** — 2026-07-24 tip-to-tip readings
      52–54 mm are **inconclusive** (correct to 54.7–56.8 mm, straddling both specs).
- [ ] **Measure the compressor cover inducer bore** (planned 2026-07-25 AM) — a true
      circle, sidesteps the odd-blade problem. Subtract ~0.5–0.8 mm tip clearance.
      This is what actually settles 54 vs 56 mm.
- [ ] **Caliper the compressor inducer** to confirm the HY35W ID (54 mm expected).
      The other giveaway is the turbine housing: HY35W is **undivided single-entry
      9 cm**, HX35 is **divided 12 cm**
- [ ] **Measure intermediate hot pipe** — is it **4"** or **4-3/8" (4.4")**? Decides
      which DPS adapter to order
- [ ] **Turbine inlet flange holes** on the received housing — 4 through-holes, or
      2 through + 2 threaded needing drill-out?
- [ ] Verify the received casting actually measures 12 cm² / accepts the 70/60 wheel
- [ ] **Photograph the wastegate actuator boss** the moment it lands, before
      spending any time on it — on an unmarked repro, the flapper/actuator
      provision is the detail most likely to be subtly wrong, and it's the exact
      feature the housing is being bought for

**Not concerns:**

- *Outlet mounting* — settled: the DPS adapter **replaces the 5-bolt flange
  directly and reuses the same gasket.** It does not clamp to the V-band.
- *"Long housing" geometry* — irrelevant here. Everything on the truck is currently
  positioned relative to an **S300 DPS Turbonator**, so minor fitment adjustment is
  required no matter which housing goes on.

### DPS exhaust adapter

Diesel Power Source *"HX35 | HX40 | HY35 Turbo Exhaust Adapter, 4 in & 4-3/8 in"* —
converts the 3" outlet to 4" or 4-3/8". The **4 vs 4-3/8 is the outlet pipe size, not
the bolt pattern**; one adapter family covers HX35/HX40/HY35 because they share the
5-bolt outlet flange.

**Mounting (confirmed):** the adapter **replaces the stock 5-bolt flange directly
and reuses the same gasket** — it does *not* clamp onto the existing V-band. So the
only open question on the adapter is which size to order, which falls out of the hot
pipe measurement.

---

## Sources

Forum- and vendor-sourced; numbers vary between sources and none of it is a Holset
catalog. Verify by measuring.

- [Mopar1973Man — Holset turbo specs](https://mopar1973man.com/cummins/articles.html/general-cummins/84_engine/89_air-exhaust/holset-turbo-specs-r156/)
- [Turbo Lab — HX40 / Super 40 specs](http://turbolabofamerica.com/holset-hx40-super-40-turbo-specs/) · [HX40 turbine housings](https://turbolabofamerica.com/holset-hx40-exhaust-housing-turbine-housing/)
- [DSMtuners — Holset wheel sizes & trim](https://www.dsmtuners.com/threads/holset-wheel-sizes-and-trim-info.428146/) · [HX35 housing options](https://www.dsmtuners.com/threads/hx35-turbine-housing-options.454462/) · [HX35 V-band outlet](https://www.dsmtuners.com/threads/hx35-turbine-housing-v-band.432719/)
- [d-series — Holset specs HY/HX/H1C/H1E](https://www.d-series.org/threads/holset-turbo-specs-hy-hx-h1c-wh1c-h1e-wh1e.120686/)
- [The Truck Stop — HX40W / non-gated / Super40](https://www.thetruckstop.us/forum/threads/holset-hx40w-non-gated-hx40-super-hx40.43524/)
- [Diesel Auto Power — 3532214 listing](https://www.dieselautopower.com/12cm-wastegated-hx35-turbine-housing-3532214) · [16 cm V-band HX35 4038909H](https://www.dieselautopower.com/hx35-4038909h)
- [DieselTuff — 12 cm wastegated H1C/HX35](https://www.dieseltuff.com/product/holset-12cm-wastegated-housing-for-h1c-or-hx35/)
- [Diesel Power Source — HX35/HX40/HY35 exhaust adapter](https://www.dieselpowersource.com/hx35-hx40-hy35-turbo-exhaust-adapter)
- [Boost Lab — Holset PN reference](https://www.theboostlab.com/holset-part-numbers/) · [J&H Diesel — Holset PN reference](https://jhdiesel.com/holset-part-number-reference/)
- [Pure Diesel Power — 14 cm non-gated housing](https://puredieselpower.com/dodge-products/turbos-and-accessories/turbo-components/89-02-h1c-hx35-dodge-cummins-14cm-non-gated-housing.html) · [Fleece Cheetah HX35](https://puredieselpower.com/dodge-products/2nd-gen-24v-98.5-02/94-02-dodge-cummins-turbos/dodge-cummins-fleece-63mm-hx35-cheetah-turbo.html)
- [Fenley — 60 mm billet wheel + cover](https://fenley-turbo.myshopify.com/products/hx35-40-60mm-billet-wheel-and-cover) · [Bullseye T3 .70 housing](https://www.extremepsi.com/store/Bullseye-Power-HX35-Turbine-Housing-Ver-2.0-Stainless-Steel-T-3-.70-A-R-3.0-V-Band.html) · [Speeding Parts Super HX40 Competition](https://www.speedingparts.com/p/turbo-accessories/turbo/holset-turbocharger/holset-super-hx40-competition.html)
