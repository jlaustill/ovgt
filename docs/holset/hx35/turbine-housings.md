# HX35 / H1C Turbine Housings — Part Number Reference

**Scope:** every genuine Holset part number catalogued as a *turbine housing* for the
**H1C / WH1C / HX35 / HX35W / HX35M** family — i.e. the **70/60 mm straight-blade
turbine wheel**. Compiled 2026-09-12.

This is the wheel-family boundary that matters: per `docs/holset-turbos.md:84`, a
turbine housing must match the wheel it wraps, and the 70/60 straight family
(H1C/HX35) is distinct from the 76/64 family (H1E/HX40) and from the 65.5/58 curved
family (HY35/HE341/HE351). A housing can be bored **out** for a bigger wheel but
never filled **in** for a smaller one.

## How this list was produced

Three tiers, because no single source does both jobs:

1. **Authenticity + model attribution** — the [J&H Holset PN reference](https://jhdiesel.com/holset-part-number-reference/),
   a 4,117-row Cummins/Holset cross-reference. It proves a number is a real Holset
   part and which model it is assigned to. Reproduce with:

   ```bash
   curl -sL -A "Mozilla/5.0" https://jhdiesel.com/holset-part-number-reference/ -o jh.html
   # filter: Generic Description == "Turbine Housing" AND Description matches HX35|H1C|WH1C
   ```

   Fetch it **raw** — summarizers report the table as specless and miss it entirely.

2. **Specifications** — vendor listings (Diesel Auto Power, Thoroughbred, Denco,
   DieselTuff). The catalog carries *no* cm², divided/open, gated/non-gated or
   outlet data. Bar for acceptance: two independent vendors agreeing.

   ⚠️ **Boost Lab is not an independent second source.** It serves a **~379-row
   truncation** of this same dataset (J&H: 4,080 rows), and its HX35/H1C housings are a
   strict subset — the first 7 of the 28 below, adding nothing. Treating the two as
   corroborating sources produced a wrong conclusion once already (see the 3591217T
   correction in `docs/holset-turbos.md:169`). Count rows before trusting a reference.

   ⚠️ **The rendered page truncates; the HTML does not.** A browser copy-paste, or any
   summarizer, sees roughly the first 380 rows and stops — short of where the catalog
   reaches HX35 numbers at all. Three separate tools under-reported this table ("no specs",
   "~1,200 rows", 379 rows). Only `curl` + parse returned all 4,080. The full extract is
   archived in-repo at [`../jhdiesel-full-catalog.tsv`](../jhdiesel-full-catalog.tsv)
   (4,080 rows, TSV) — use that rather than re-fetching; the site rate-limits and now
   returns 403 to automated requests.

3. **Authenticity check** — genuine Holset service parts carry an **`H`** suffix
   (`3521927H`, `3537817H`). A `T` suffix is not a Holset convention and indicates an
   aftermarket reproduction (see `3591217T` in `docs/holset-turbos.md:169`).

**Most cells below are `?`.** Tier 1 returned all 28 numbers; tier 2 has so far returned
data for four (`3521927` 16 cm, `3524123` 12 cm, `3537021` 14 cm, plus partial elsewhere). Per-casting specs for OEM industrial H1C housings are essentially
absent from the public web. `?` means *not yet sourced*, never *not applicable* —
do not fill these in from inference.

## Part numbers

Inducer/exducer describe the **wheel the housing is bored for**, not the housing
itself. `70/60` is the family definition rather than a per-PN measurement, so it is
marked `70*/60*` wherever it is inherited from the catalog's model attribution and
has not been independently confirmed for that casting.

| PN | Model | Inducer | Exducer | Volume | Divided | Wastegate | Inlet | Outlet | Verified | raw_notes |
|---|---|---|---|---|---|---|---|---|---|---|
| 3519409 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3520730 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE.H1C`; one search hit describes it as a *water-cooled* H1C turbine housing — unconfirmed, and water-cooling is normally a bearing-housing trait, so treat as suspect |
| 3521927 | H1C | 70* | 60* | **16 cm²** | ? | **non-gated** | ? | V-band (or cast elbow) | **vendor-confirmed** | `HOUSING,TURBINE H1C`. Sold as **3521927H** — H suffix passes the genuineness rule. "Fits 1988–1998 Dodge Cummins 5.9L 12V with HX35, H1C, or WH1C. Also fits 1998–2002 if the current housing has a V-band flange or a different cast elbow is used." ~$186 ($185.68 at Diesel Auto Power). Thoroughbred adds: "Does not work with 2000–2002 Automatic Trucks" |
| 3522743 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3522744 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3522746 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3522747 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3523048 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3523242 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3523243 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3524123 | H1C | 70* | 60* | **12 cm²** | **divided** (twin entry) | **non-gated** | T3 | ⚠️ **unresolved** — see below | **vendor-confirmed** | `HOUSING,TURBINE H1C`. "Holset 12cm Non-Wastegated Turbine Housing. Fits 1988–1998 Dodge Cummins 5.9L 12V With HX35, H1C, or WH1C. **Short housing.**" ~$253 at Diesel Auto Power. Gillett sells the same casting as **GDS TH-01** — *"Genuine, Holset (Cummins Turbo Technologies) 12cm2 short outlet, Non-Wastegated Turbine housing for H1C, WH1C, HX35 & HX35W"*, **twin entry**, $305; adds *"direct fit 1988–1993; fits 1994–2002 with modifications (requiring exhaust repositioning)."* |
| 3524425 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3525130 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3525691 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3532676 | WH1C | 70* | 60* | ? | ? | likely WG | ? | ? | catalog only | `HOUSING,TURBINE.WH1C.ASSY`. Leading/trailing **W = wastegated** (`docs/holset-turbos.md:30`), so gated is a naming inference, not a sourced fact. `.ASSY` = housing + hardware, not a bare casting |
| 3536860 | HX35W | 70* | 60* | ? | ? | likely WG | ? | ? | catalog only | `HOUSING,TURBINE.HX35W.PER ASSY` |
| 3537021 | HX35 | 70* | 60* | **14 cm²** | ? | **non-gated** | ? | ? | **vendor-confirmed** | `HOUSING,TURBINE.HX35`. Diesel Auto Power: "Holset 14cm Non-Wastegated Turbine Housing", $306.90, listed **Special Order**. The 14 cm is the scarce size (`docs/holset-turbos.md` notes most HX35W were 12 cm) |
| 3537491 | HX35W | 70* | 60* | ? | ? | likely WG | ? | ? | catalog only | `HOUSING,TURBINE.HX35W.PER ASSY` |
| 3539323 | HX35W | 70* | 60* | ? | ? | likely WG | ? | ? | catalog only | `HOUSING,TURBINE.HX35W.ASSY` |
| 3539724 | HX35 | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE.HX35` |
| 3591155 | HX35W | **?** | **?** | ? | ? | likely WG | ? | ? | catalog only ⚠️ | `HOUSING,TURBINE.HX35W.ASSY`. ⚠️ **Wheel family in doubt.** An eBay listing titles it *"Genuine Holset HX35W HX40w 67 / 76 mm Cummins Turbo Housing 3591155 - 3532214"* — 67/76 is the **HX40** wheel, not 70/60. Either sloppy seller text or this casting spans both. Do not assume 70/60. The same listing pairs it with 3532214, which hints at an **assembly PN ↔ bare casting PN** relationship (unverified) |
| 3593889 | HX35M | **?** | **?** | ? | ? | ? | ? | ? | catalog only ⚠️ | `HOUSING,TURBINE.HX35M.ASSY`. ⚠️ **Variant suffix unverified.** `M` (marine?) is an application variant like `G` (gas/CNG), and the catalog holds no HX35M shaft-and-wheel to confirm it runs the 70/60 diesel wheel. Do not inherit the family default here |
| 3790153 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE.H1C` |
| 4036501 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE.H1C` |
| 4036616 | HX35 | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE.HX35` |
| 4043900 | HX35W | 70* | 60* | ? | ? | likely WG | ? | ? | catalog only | `HOUSING,TURBINE.HX35W` |
| 4045764 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE.H1C` |
| 4048561 | HX35W | 70* | 60* | ? | ? | likely WG | ? | ? | catalog only | `HOUSING,TURBINE.HX35W.PERMANENT ASS` (description truncated in source) |

## What this experiment is actually testing (2026-09-12)

**The objective is turbine-side sizing, not compressor-side.** The question is whether the
**70/60 wheel is too small or correctly sized for HP duty** in the compound, to be answered
from logged **spool-up, EGT and backpressure** — then the next step gets chosen on data.

The 54 mm compressor decision in [`compressor-housings.md`](compressor-housings.md) is
therefore not a compromise, it is a **held constant**. Keeping the compressor fixed while
the turbine is under test is the right design.

### ⚠️ The confound: three different faults produce the same BPR signature

`docs/holset-turbos.md:264` already states that a **too-stiff spring** causes *"BPR
climbing."* So does an undersized turbine wheel, and so does an undersized housing. All
three push drive pressure the same direction, and **high BPR alone cannot tell them apart:**

| Cause | Signature |
|---|---|
| Turbine wheel too small | BPR climbs at high flow, EGT up, spool fast |
| Housing A/R too small | BPR climbs at high flow, EGT up, spool fast |
| Spring preload too stiff | BPR climbs — *per the existing table at `:264`* |

### ⚠️ And there is no wastegate position feedback

The build deletes the pneumatic canister for a pure spring gate (`docs/holset-turbos.md:267`),
so **nothing reports gate position.** The decisive discriminator — *"is the gate already
wide open and BPR still high?"* (turbine genuinely too small) versus *"is the gate barely
cracking?"* (spring too stiff) — is not currently observable.

**Cheapest way to separate them: a spring-preload sweep.** Run the same load case across
preloads and watch BPR.
- BPR falls as preload is reduced → **the spring was the binding constraint**, not the wheel.
- BPR stays high across the whole preload range → **the turbine side is genuinely the
  restriction.** Only then is "too small" established.

Requires no new hardware. A gate-position sensor on the arm would settle it directly and is
worth considering if the sweep proves ambiguous.

### The housing is the cheap second data point — test it before condemning the wheel

Testing "the 70/60" actually means testing **70/60 in a specific A/R**. The wheel and the
housing are separate variables and this reference now documents genuine options at three
sizes on the same wheel:

| Step | PN | Size | Gate |
|---|---|---|---|
| 1 | `3532214` / `3591217` | **12 cm²** | wastegated, turbine-side actuator |
| 2 | `3537021` | **14 cm²** | non-gated (~$307) |
| 3 | `3521927` | **16 cm²** | non-gated (~$186) |

If 12 cm² proves too restrictive, **the next step is 14 cm² on the same wheel** — far cheaper
than a different turbo, and it isolates A/R from wheel size. Only if a larger A/R still
can't hold BPR is the *wheel* the problem. ⚠️ Note steps 2 and 3 are **non-gated**, which
`docs/holset-turbos.md:235` says removes the only boost control on the truck — viable as a
back-to-back test, not as a configuration to leave installed.

### Pass/fail criterion already exists

**BPR = 1.5** is the existing PI setpoint ([[project_bpr_boost_control]],
`docs/holset-turbos.md`). If the 70/60 in 12 cm² cannot hold BPR at or under target at the
flow the truck actually runs — *with the gate confirmed open* — it is too small.

### Sensor gap: TOT

Turbine **outlet** temperature is on the sensor roadmap but not yet fitted. Drive pressure
plus pre-turbine EGT gives "too small / not too small"; adding TOT gives expansion ratio and
turbine efficiency, turning a judgement call into a number. For a turbine-sizing experiment
specifically, **TOT is the single sensor that would most improve the answer.**

## Meets the current requirement (2026-09-12)

**Requirement:** 12 cm² · 70/60 wheel · divided/dual-scroll · and either **no wastegate**
or a wastegate whose actuator is **self-contained on the turbine housing** — because a
spring gate on the housing currently fitted fouls the compressor cover.

### ✅ `3532214` — the match

12 cm², divided T3, bored 70/60, wastegated, and the vendor text speaks directly to the
clearance problem:

> "**Actuator is contained on the turbine side and does not need to bolt to compressor
> housing.**" · "**Long housing.**"

~$599.50 at Diesel Auto Power; also stocked by DieselTuff. This is the first-gen /
WH1C-era self-contained gate, from before Holset moved the canister onto a
compressor-cover bracket — which is why the interference is a *generational* mismatch,
not a faulty part. `3591217T` remains the ~$120 claimed reproduction of this casting.

⚠️ **`3532214` is not in the table below** — it is absent from the Holset cross-reference
(see Caveats).

### ⚠️ `3524123` — non-gated alternative, two blockers

12 cm², twin-entry divided, **short housing**, ~$253–305, genuine Holset casting. Cheaper
and clears the compressor cover trivially (no gate at all). But:

**1. It deletes the only boost control on the truck.** `docs/holset-turbos.md:235`:
*"Primary tuning lever: the wastegate spring — and it is the ONLY boost control on the
truck."* The S472 LP stage is ungated (`:237`), and the HP gate also supplies **overspeed
protection** (`:304`). Going non-gated is only viable alongside an external gate.

**2. The outlet is UNRESOLVED — do not treat this as settled either way.**

Raised 2026-09-12 on the observation that the outlet looked like it might not be the
5-bolt flange. **No verified image of this casting has been found**, and product photos
are not acceptable evidence. What the sources actually support is a *different* claim
than the one first recorded:

- The **long/short distinction is about outlet POSITION, not bolt pattern.** Short outlet
  is a direct drop-in for **1988–1993** trucks; on **1994–1998** it reportedly requires
  **moving the exhaust forward ~1.25"**. Long outlet is the better choice on 1994–2002.
  That is a clearance/geometry problem, not necessarily a flange-pattern problem.
- Forum sources further claim the H1C and HX35 outlets **share a 3-1/8" V-band clamp**,
  and that a 3" V-clamp flange fits both 1989–1998 H1C and HX35 outlets — which would
  make the outlet *compatible*, just displaced.
- Against that, other forum text says the early H1C outlet is *"a hose connection"* while
  the HX35 uses a V-band. These claims are not mutually consistent, and all of them are
  forum-sourced.

So the flange pattern is **genuinely unknown for `3524123`**, and the earlier phrasing
("appears not to be the 5-bolt") is withdrawn as overstated — it was an impression, not a
finding.

**Why it still has to be settled before buying:** `docs/holset-turbos.md:88` records that
the DPS exhaust adapter *"replaces that 5-bolt flange and reuses its gasket."* If the
outlet differs in pattern, the adapter path breaks outright; if it differs only by ~1.25"
in position, the adapter may still bolt up but the downpipe geometry moves. Those are very
different outcomes and only one of them is survivable without fabrication.

**How to settle it without trusting a photo:**
1. **Ask Gillett Diesel** — they sell this exact casting as their own GDS TH-01 and can
   state the outlet pattern and the 1.25" offset directly. Diesel Auto Power likewise.
2. **Cummins QuickServe / parts.cummins.com** — `3524123` is a genuine Cummins/Holset
   number, so the OEM catalog should carry an exploded view rather than marketing art.
3. **Ask for a photo of the actual item in stock**, tape measure across the flange, not
   the catalog image.

### 🆕 `3591217` — cast-in number found 2026-09-12, verdict upgraded

An eBay listing titled *"12CM 96-98 Ram 5.9 T3 HX35w WH1C twin scroll turbine exhaust
housing 60mm"* shows **`3591217` and `12 L54` cast into the housing**, described as:

> "12cm twin scroll turbine housing for 60mm HX35 wheel. This fits the 96–98 5.9 style,
> **with the wastegate actuator rod perpendicular to the inlet flange.** Refer to photos
> for wastegate arm location when closed. Weight as shown in photos: **13 lbs 2 oz**"

**This overturns the basis of the UNVERIFIED verdict.** That verdict rested on an earlier
listing described as *"**Unmarked** Cummins Holset 3591217"* — unmarked meaning no Holset
number cast in. A casting with the number **molded into it** is the opposite evidence, and
per Holset identification practice *"casting numbers are molded into the compressor and
turbine housings and the center bearing housing"*. So `3591217` reads as a **genuine
Holset casting number**.

**A sand-cast finish is not a red flag.** OEM cast-iron turbine housings are sand cast
(SiMo ductile iron) — rough surface texture and raised numbers that are awkward to read are
exactly what a genuine Holset housing looks like. A crisp, machined-looking number would be
*more* suspicious, not less.

**`12 L54`:** the `12` is very plausibly the **cm² size cast into the housing**, which
agrees with the listing's own 12 cm twin-scroll claim; `L54` reads as a foundry/date code.
Inference, not confirmed.

**Where this leaves the three original arguments against it:**

| Argument | Status |
|---|---|
| Absent from both public PN references | ❌ Withdrawn — `3532214` is absent too |
| Unmarked, no number cast in | ❌ Contradicted — this casting carries `3591217` |
| `T` suffix is not a Holset convention | ⚠️ Still stands, but it describes the **seller's** designation, not the casting. The cast-in number is plain `3591217` |

**Against the current requirement this listing matches on every stated spec:** 12 cm²,
twin scroll (divided), 60 mm wheel, T3. The open variable is the one the seller
helpfully calls out — **wastegate arm geometry**. "Actuator rod perpendicular to the inlet
flange" is precisely what decides compressor-cover clearance, so compare the seller's
arm-when-closed photo against the housing currently fitted before buying.

**Useful forensics to request:** the **13 lb 2 oz** weight is a good discriminator — ask
for it on any competing example, since a reproduction is unlikely to match an OEM casting's
mass closely. Also worth confirming: divided (twin-entry) inlet, and that the bore is cut
for the 60 mm exducer rather than the 64 mm HX40 wheel.

## Excluded / unresolved candidates

Numbers that surfaced and did **not** earn a row, recorded so the reasoning is not redone:

| PN | Claim | Why excluded |
|---|---|---|
| `3794679` | eBay: *"HOUSING TURBINE 3794679 Replacement Fits Cummins Onan HX35G"* | **Wrong or unproven wheel family.** `HX35G` is the **gas/CNG** variant (the catalog literally lists `TURBOCHARGER,HX35GAS`) — Orion Bus, Solaris, Westport Tata, ISLG-280 CNG, Onan gensets; water-cooled, billet 7-blade compressor. No HX35G shaft-and-wheel exists in the catalog, so nothing links it to the 70/60 straight diesel wheel. Also *"Replacement Fits"* is aftermarket-replacement phrasing. **Not** excluded for being absent from the catalog — see below |
| `3780476` | TurboTurbos: HX35G turbine housing (Solaris Cummins) | Same gas/CNG family question; absent from catalog |

**Catalog absence was not the reason for either.** Five vendor-confirmed genuine HX35G
parts — `3780476`, `5357728`, `5357934`, `3599491`, `4042333` — are all missing from the
cross-reference, and it carries **no HX35G turbine housing at all** despite holding an
HX35G compressor housing (`3591012`), bearing housing (`3592561`), cores, actuators and
10+ HX35G turbochargers. `3794679` also sits three numbers from `3794682`
(`KIT,TURBOCHARGER`, independently vendor-cited as an HX35G reference) and eleven from
`3794668` (`TURBOCHARGER ID21 HX35W`), so the number is very plausibly real.

## Caveats

**The catalog is not exhaustive.** `3532214` — the housing `docs/holset-turbos.md:168`
marks ✅ CONFIRMED on multi-vendor agreement, 12 cm² divided wastegated — **does not
appear in the J&H catalog at all**, and so is not a row above. Neither do `3592766`
(the HX35W core on hand) or `3530521` (its bearing housing). Absence from the catalog
is therefore *not* evidence against a part number.

**`4038909H` is a complete turbocharger, not a housing.** The catalog lists `4038909`
as `TURBOCHARGER,ID21,HX35`, and [Diesel Auto Power](https://www.dieselautopower.com/hx35-4038909h)
confirms it is a full assembly — 52/82 compressor, 70/60 rotor, *with* a 16 cm² non-WG
V-band housing. It is currently mis-filed under "Turbine housings" in
`docs/holset-turbos.md:99`. The bare 16 cm² housing is `3521927`.

**`3519336` attribution conflicts.** `docs/holset-turbos.md:191` cites AVP listing it as
the 70/60 turbine wheel & shaft for HX35/HX35W/H1C/WH1C; the J&H catalog calls it
`WHEEL,TURBOCHARGER H2`. Unresolved — flagged here because the same lookup surfaced it.

**Sources blocked at time of writing:** DSMtuners (403), 4btswaps (402 paywall),
Speeding Parts (403), turboford (403), TurboMaster (empty catalog). These are the most
likely places to close the `?` cells; retry with a different fetch path.

## Sources

- [J&H Diesel — Holset PN reference](https://jhdiesel.com/holset-part-number-reference/) — tier 1, the 28 numbers
- [Diesel Auto Power — 3521927](https://www.dieselautopower.com/holset-16cm-non-wastegated-turbine-housing-3521927) · [Thoroughbred — 3521927H](https://www.thoroughbreddiesel.com/3521927h/) · [Denco — 3521927](https://www.dencodiesel.com/products/3521927-turbine-housing-hx35) · [US Diesel Parts — 3521927H](https://usdieselparts.com/dodge-16cm-performance-turbo-housing-3521927h/)
- [Diesel Auto Power — 4038909H](https://www.dieselautopower.com/hx35-4038909h) — the turbo-not-a-housing correction
- [Diesel Auto Power — 3532214](https://www.dieselautopower.com/12cm-wastegated-hx35-turbine-housing-3532214) · [DieselTuff — 12 cm wastegated H1C/HX35](https://www.dieseltuff.com/product/holset-12cm-wastegated-housing-for-h1c-or-hx35/)
- [Diesel Auto Power — housings, 1989–93 12V](https://www.dieselautopower.com/dodge-ram-cummins/1989-1993-5-9l-12-valve-dodge-cummins/1989-1993cummins-turbochargers/housings) — the single most productive spec source: 12/14/16 cm non-WG + 12 cm WG with prices
- [Gillett Diesel — GDS TH-01 12 cm non-WG short outlet](https://gillettdiesel.com/products/gds-holset-12cm-non-wastegated-turbine-housing-short-outlet-th-01) · [Diesel Auto Power — 3524123](https://www.dieselautopower.com/holset-12cm-non-wastegated-turbine-housing-3524123)
- Local archive: [`../jhdiesel-full-catalog.tsv`](../jhdiesel-full-catalog.tsv) — full 4,080-row Holset cross-reference, extracted 2026-09-12
- Sibling docs: [`compressor-housings.md`](compressor-housings.md) · [`chra-cores.md`](chra-cores.md)
- Parent doc: `docs/holset-turbos.md`
