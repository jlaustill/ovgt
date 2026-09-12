# HX35 / H1C Cores (CHRA) — Part Number Reference

**Scope:** every genuine Holset part number catalogued as a *core assembly* (CHRA — the
cartridge: bearing housing + shaft & wheel + compressor wheel) for the
**H1C / WH1C / HX35 / HX35W / HX35G** family. Compiled 2026-09-12.

Method and sourcing tiers are identical to [`turbine-housings.md`](turbine-housings.md).
Source data: [`../jhdiesel-full-catalog.tsv`](../jhdiesel-full-catalog.tsv). The catalog
files all of these under the generic description **`Core`** — there is no separate "CHRA"
or "cartridge" category, so `Core` is the term to filter on.

## What actually interchanges

`docs/holset-turbos.md:82` is the governing note: **the cartridge swaps across the H1 frame**
— oil lines, mounting and drain are common — **with one documented exception, the HY35W**,
whose turbine rotating assembly is not shared. The 70/60 wheel & shaft (`3519336`) that fits
HX35 / HX35W / H1C / WH1C is explicitly *not* sold for the HY35W.

Two cautions carried over from the turbine-housing work:

- ⚠️ **`HX35G` cores are the gas/CNG variant** (the catalog lists `TURBOCHARGER,HX35GAS`):
  Orion Bus, Solaris, Westport Tata, ISLG-280 CNG, Onan gensets. **Water-cooled**, billet
  7-blade compressor. No HX35G shaft-and-wheel exists anywhere in the catalog, so nothing
  establishes that these run the 70/60 straight diesel wheel. Flagged in the table; do not
  assume diesel-family interchange.
- ⚠️ **Blade generation matters for what the core carries.** A core's compressor wheel is
  either 8-blade 56×83 ('95–98) or 7-blade 54×78 ('99–02), and those are **not**
  interchangeable between each other's covers. See
  [`compressor-housings.md`](compressor-housings.md) — this is also what decides whether the
  wastegate canister lands on the turbine housing or the compressor cover.

**The core on hand is `3592766`** — confirmed genuine HX35W, 54 mm 7-blade compressor,
70/60 12-blade turbine, built on the common `3530521` bearing housing
(`docs/holset-turbos.md:416`). Note it does **not** appear in the catalog, and neither does
`3530521` — another reminder that absence proves nothing.

## Part numbers

`?` means *not yet sourced*, never *not applicable*. The catalog gives model attribution
only — no wheel sizes, no bearing type, no water-cooling flag.

| PN | Model | Compressor wheel | Turbine wheel | Water-cooled | Verified | raw_notes |
|---|---|---|---|---|---|---|
| 3521950 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY.H1C` |
| 3523317 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3523320 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3523324 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3523754 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3524054 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3524607 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3537811 | HX35G | ? | ? | likely **yes** | catalog only ⚠️ | `CORE ASSEMBLY HX35G`. ⚠️ **Gas/CNG variant** — water-cooled, billet 7-blade; 70/60 diesel wheel NOT established |
| 3537815 | HX35W | ? | ? | ? | catalog only | `CORE ASSEMBLY HX35W` |
| 3537827 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY H1C` |
| 3537828 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY H1C` |
| 3538794 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY H1C` |
| 3545304 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3545695 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3545697 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3545699 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3545713 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3575062 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3575065 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3580176 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3580229 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3580240 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3580738 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 3595027 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 3596922 | WH1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,WH1C` |
| 3768480 | HX35G | ? | ? | likely **yes** | catalog only ⚠️ | `CORE ASSEMBLY.HX35G`. ⚠️ **Gas/CNG variant** — water-cooled, billet 7-blade; 70/60 diesel wheel NOT established |
| 3768483 | HX35G | ? | ? | likely **yes** | catalog only ⚠️ | `CORE ASSEMBLY.HX35G`. ⚠️ **Gas/CNG variant** — water-cooled, billet 7-blade; 70/60 diesel wheel NOT established |
| 4027086 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4027098 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4027099 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4027207 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4027208 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4027209 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4027212 | HX35W | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35W` |
| 4027214 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY.HX35` |
| 4027250 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4027331 | HX35 | ? | ? | ? | catalog only | `CONJUNTO ROTATIVO HX35`. Spanish-language catalog entry (*conjunto rotativo* = rotating assembly); same thing as `CORE ASSEMBLY,HX35` |
| 4027379 | HX35W | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35W` |
| 4027399 | H1C | ? | ? | ? | catalog only | `CORE ASSEMBLY,H1C` |
| 4027410 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4027446 | HX35W | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35W` |
| 4027536 | HX35W | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35W` |
| 4027571 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4027581 | HX35G | ? | ? | likely **yes** | catalog only ⚠️ | `CORE ASSEMBLY,HX35G`. ⚠️ **Gas/CNG variant** — water-cooled, billet 7-blade; 70/60 diesel wheel NOT established |
| 4027747 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4027752 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY.HX35` |
| 4027825 | HX35W | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35W` |
| 4027839 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4027843 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4027864 | HX35W | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35W` |
| 4027883 | HX35W | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35W` |
| 4027946 | HX35W | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35W` |
| 4030804 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4030871 | HX35W | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35W` |
| 4030873 | HX35W | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35W` |
| 4030891 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4031349 | HX35G | ? | ? | likely **yes** | catalog only ⚠️ | `CORE ASSEMBLY,HX35G`. ⚠️ **Gas/CNG variant** — water-cooled, billet 7-blade; 70/60 diesel wheel NOT established |
| 4031388 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4031397 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY.HX35` |
| 4031399 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |
| 4032035 | HX35W | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35W` |
| 4032102 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY.HX35` |
| 4032103 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY.HX35` |
| 4032389 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY.HX35` |
| 4032737 | HX35W | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35W` |
| 4034166 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY.HX35` |
| 4042187 | HX35 | ? | ? | ? | catalog only | `CORE ASSEMBLY,HX35` |

## Caveats

**67 numbers, almost no differentiation.** Thirty-odd of these read `CORE ASSEMBLY,HX35` or
`CORE ASSEMBLY,HX35W` with nothing in the catalog to separate them — different wheel trims,
bearing specs and applications collapse to the same description string. Picking a core by
part number alone is not possible from this data; the data tag on the unit is the authority
(`docs/holset-turbos.md:498` — riveted near the compressor outlet or on the bearing
housing's flat machined pad).

**Catalog absence proves nothing**, as established at length in
[`turbine-housings.md`](turbine-housings.md): the known-genuine `3592766` core and its
`3530521` bearing housing are both missing from this same cross-reference.

## Sources

- Archived cross-reference: [`../jhdiesel-full-catalog.tsv`](../jhdiesel-full-catalog.tsv)
- Sibling docs: [`turbine-housings.md`](turbine-housings.md) · [`compressor-housings.md`](compressor-housings.md) · parent `docs/holset-turbos.md`
