# HX35 / H1C Compressor Housings — Part Number Reference

**Scope:** every genuine Holset part number catalogued as a *compressor housing* for the
**H1C / WH1C / HX35 / HX35W / HX35G** family. Compiled 2026-09-12.

Method, sourcing tiers and the `H`-suffix authenticity rule are identical to
[`turbine-housings.md`](turbine-housings.md) — read that first. Source data is the
archived cross-reference at [`../jhdiesel-full-catalog.tsv`](../jhdiesel-full-catalog.tsv).

## ⭐ The wastegate-tab rule (answers the current requirement)

**The wheel generation determines where the wastegate canister mounts** — and therefore
whether the compressor cover carries actuator tabs at all:

| Generation | Compressor wheel | Application | Wastegate canister mounts on | Cover has tabs? |
|---|---|---|---|---|
| **8-blade** | **56 / 82–83 mm** | '95–98 12v (manual) | **the TURBINE housing** | ✅ **NO TABS** |
| **7-blade** | **54 / 77–78 mm** | '99–02 24v (manual) | **the COMPRESSOR housing** | ❌ has tabs |

> *"You can normally tell them apart because the wastegate canister is mounted on the
> turbine housing, whereas the 7-blade's wastegate canister mounts on the compressor
> housing."* · *"When selecting a donor turbo for a turbine housing, make sure the
> actuator does not attach to the compressor cover."*

**So the tab-free cover you want is the 8-blade / 56 mm one** — the '95–98 12v generation.
This also explains the original problem: the core on hand, **`3592766`, is the 7-blade
54 mm** (`docs/holset-turbos.md:416`), i.e. the generation whose gate is *designed* to bolt
to the compressor cover. The spring gate fouling the cover is a generational mismatch, not
a defect — the same root cause as the turbine-housing clearance issue.

⚠️ **The wheel and cover must change together.** 7-blade is 54×78, 8-blade is 56×83;
they are **not interchangeable between each other's housings**. A 56 mm cover requires the
56 mm 8-blade wheel. The on-hand core's 54 mm wheel will not run in it.

⚠️ **Flow figures conflict — do not plan on them yet.** One forum source claims the
8-blade flows **52 lb/min** and the 7-blade **60 lb/min**, i.e. the *larger* wheel flowing
*less*. That is backwards on its face and contradicts `docs/holset-turbos.md:40–41`
(8-blade 56/82, 7-blade 54/76.5). Against the ~45–50 lb/min target this matters, so treat
both numbers as unverified until a compressor map is found.

**Aftermarket alternative:** matched housing + wheel sets exist and sidestep the whole
question — Gillett `CH-10` (60 mm 7-blade upgrade housing **and** wheel), and the Fenley
60 mm billet + matched cover already noted at `docs/holset-turbos.md:97`. Buying the pair
guarantees the contour matches the wheel.

## Part numbers

`?` means *not yet sourced*, never *not applicable*. Per-PN compressor-housing specs are as
absent from the public web as the turbine-housing ones — the catalog gives model attribution
only, and vendors list these as bare "GENUINE HOLSET COMPRESSOR HOUSING" with no wheel size
or tab information.

| PN | Model | Inducer | Exducer | WG tabs | Inlet | Outlet | Verified | raw_notes |
|---|---|---|---|---|---|---|---|---|
| 3532465 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35` |
| 3532467 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35` |
| 3536495 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35` |
| 3536502 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35` |
| 3537726 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35` |
| 3537732 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35` |
| 3538769 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR.HX35` |
| 3590646 | HX35W | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR.HX35W.ASSY` |
| 3591005 | HX35W | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35W,ASSY` |
| 3591006 | HX35W | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR.HX35W.ASSY` |
| 3591008 | HX35W | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR.HX35W.ASSY` |
| 3591012 | HX35G | ? | ? | ? | ? | ? | catalog only ⚠️ | `HOUSING,COMPRESSOR.HX35G.ASSY`. ⚠️ **Gas/CNG variant** — `G` is the natural-gas application (catalog lists `TURBOCHARGER,HX35GAS`); water-cooled, billet 7-blade. Wheel family not established for the diesel build |
| 3591014 | HX35W | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR.HX35W.ASSY` |
| 3592355 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR.HX35` |
| 3595195 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35` |
| 3595896 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35` |
| 3596776 | HX35W | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35W,ASSY` |
| 3597306 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35` |
| 3598643 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35,ASSY` |
| 3599319 | HX35W | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35W,ASSY` |
| 3599984 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35` |
| 4036101 | HX35W | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35W,ASSY` |
| 4036492 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35` |
| 4036745 | HX35W | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35W,ASSY` |
| 4037109 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35` |
| 4037836 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR.HX35` |
| 4038469 | HX35W | ? | ? | ? | ? | ? | **vendor-listed** | `HOUSING,COMPRESSOR,HX35W,ASSY`. Denco: *"BRAND NEW GENUINE HOLSET COMPRESSOR HOUSING"*, HX35, 5.0 kg, A$356.36. No wheel size or tab detail given |
| 4040185 | HX35W | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR,HX35W,ASSY` |
| 4044030 | HX35 | ? | ? | ? | ? | ? | catalog only | `HOUSING,COMPRESSOR.HX35` |

## Caveats

**The catalog is not exhaustive** — same as the turbine-housing list. `3591005`/`3591006`/
`3591008`/`3591014` are all `HOUSING,COMPRESSOR.HX35W.ASSY` with no way to tell them apart
from the catalog alone, and the bare-casting vs `.ASSY` distinction (housing alone vs
housing + fitted hardware) is unresolved here exactly as it is there.

**Tabs are the thing to verify, and no vendor states it.** Not one listing found mentions
actuator bracket bosses either way. Until a vendor or a measured photo confirms it, the
only reliable signal is the **generation** (8-blade '95–98 = no tabs) rather than the part
number. Ask for a photo of the cover's outer face.

## Sources

- Archived cross-reference: [`../jhdiesel-full-catalog.tsv`](../jhdiesel-full-catalog.tsv)
- [Mopar1973Man — Holset turbo specs](https://mopar1973man.com/cummins/articles.html/general-cummins/84_engine/89_air-exhaust/holset-turbo-specs-r156/) — blade count / canister location rule
- [Denco — 4038469 compressor housing](https://www.dencodiesel.com/products/4038469-holset-compressor-housing-hx35)
- [Gillett — CH-10 60 mm 7-blade upgrade housing + wheel](https://www.gillettdiesel.com/shop/Holset-HX-Upgrade-Compressor-Housing--and--Wheel-60mm--7-blade-CH-10)
- Sibling docs: [`turbine-housings.md`](turbine-housings.md) · [`chra-cores.md`](chra-cores.md) · parent `docs/holset-turbos.md`
