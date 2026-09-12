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

**Most cells below are `?`.** Tier 1 returned all 28 numbers; tier 2 returned data for
only two of them. Per-casting specs for OEM industrial H1C housings are essentially
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
| 3521927 | H1C | 70* | 60* | **16 cm²** | ? | **non-gated** | ? | V-band (or cast elbow) | **vendor-confirmed** | `HOUSING,TURBINE H1C`. Sold as **3521927H** — H suffix passes the genuineness rule. "Fits 1988–1998 Dodge Cummins 5.9L 12V with HX35, H1C, or WH1C. Also fits 1998–2002 if the current housing has a V-band flange or a different cast elbow is used." ~$186. Thoroughbred adds: "Does not work with 2000–2002 Automatic Trucks" |
| 3522743 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3522744 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3522746 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3522747 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3523048 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3523242 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3523243 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3524123 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3524425 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3525130 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3525691 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE H1C` |
| 3532676 | WH1C | 70* | 60* | ? | ? | likely WG | ? | ? | catalog only | `HOUSING,TURBINE.WH1C.ASSY`. Leading/trailing **W = wastegated** (`docs/holset-turbos.md:30`), so gated is a naming inference, not a sourced fact. `.ASSY` = housing + hardware, not a bare casting |
| 3536860 | HX35W | 70* | 60* | ? | ? | likely WG | ? | ? | catalog only | `HOUSING,TURBINE.HX35W.PER ASSY` |
| 3537021 | HX35 | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE.HX35` |
| 3537491 | HX35W | 70* | 60* | ? | ? | likely WG | ? | ? | catalog only | `HOUSING,TURBINE.HX35W.PER ASSY` |
| 3539323 | HX35W | 70* | 60* | ? | ? | likely WG | ? | ? | catalog only | `HOUSING,TURBINE.HX35W.ASSY` |
| 3539724 | HX35 | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE.HX35` |
| 3591155 | HX35W | **?** | **?** | ? | ? | likely WG | ? | ? | catalog only ⚠️ | `HOUSING,TURBINE.HX35W.ASSY`. ⚠️ **Wheel family in doubt.** An eBay listing titles it *"Genuine Holset HX35W HX40w 67 / 76 mm Cummins Turbo Housing 3591155 - 3532214"* — 67/76 is the **HX40** wheel, not 70/60. Either sloppy seller text or this casting spans both. Do not assume 70/60. The same listing pairs it with 3532214, which hints at an **assembly PN ↔ bare casting PN** relationship (unverified) |
| 3593889 | HX35M | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE.HX35M.ASSY` |
| 3790153 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE.H1C` |
| 4036501 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE.H1C` |
| 4036616 | HX35 | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE.HX35` |
| 4043900 | HX35W | 70* | 60* | ? | ? | likely WG | ? | ? | catalog only | `HOUSING,TURBINE.HX35W` |
| 4045764 | H1C | 70* | 60* | ? | ? | ? | ? | ? | catalog only | `HOUSING,TURBINE.H1C` |
| 4048561 | HX35W | 70* | 60* | ? | ? | likely WG | ? | ? | catalog only | `HOUSING,TURBINE.HX35W.PERMANENT ASS` (description truncated in source) |

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
- Local archive: [`../jhdiesel-full-catalog.tsv`](../jhdiesel-full-catalog.tsv) — full 4,080-row Holset cross-reference, extracted 2026-09-12
- Parent doc: `docs/holset-turbos.md`
