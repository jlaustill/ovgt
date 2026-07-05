# 2026-06-19 Work Plan — C-Next agents-guide review → then MongoDB telemetry

> Context snapshot so nothing is lost across compaction. Two parts, **strict order**:
> **A must fully complete before B.**

## Sequence / status
1. **[ACTIVE] Part A — C-Next agents-guide test review.** Must finish first.
2. **[DEFERRED] Part B — MongoDB telemetry** (design agreed; spec/build parked until A done).

---

## Part A — C-Next agents-guide test review (DO FIRST)

**Goal:** make the C-Next *agents guide* accurately and completely reflect real c-next
behavior, as proven by the actual test suite.

**Paths:**
- **Tests (review these):** `/home/linux/code/c-next` — branch `main`, remote `jlaustill/c-next`.
  **973 `*.test.cnx`** files across 8 `tests/` dirs. Regenerate ordered list:
  `find /home/linux/code/c-next -name '*.test.cnx' | sort`
- **Agents guide (update + PR this):** `/home/linux/code/ogauge/docs/cnext-ai-reference.md`
  (959 lines; symlinked to `~/.claude/cnext-ai-reference.md`). Remote `jlaustill/ogauge`.
  **DECISION (2026-06-19): RELOCATE the guide into c-next** → `/home/linux/code/c-next/docs/cnext-ai-reference.md`, update the `~/.claude/cnext-ai-reference.md` symlink, and **PR to `jlaustill/c-next`** on branch `docs/agents-guide-relocate-and-audit`. The old `ogauge` copy is removed in a separate small ogauge change at the end.
- Note: `c-next/docs/learn-cnext-in-y-minutes.md` is the *full syntax* doc, **not** the agents guide.

**Procedure (per user, strict — do NOT deviate):**
- **Sequential ONLY. No parallelism, no subagent fan-out.** One test at a time, in a loop.
- Process tests in `sort` order so progress is resumable.
- For each test:
  1. Read the test (and its generated `.c`/`.h` if helpful) to learn the exact syntax/behavior it proves.
  2. Compare against the agents guide.
  3. If the guide is **wrong** → fix it. If it's **missing** something → add it. Record the change.
- After the whole loop: **one PR** to `c-next` (the relocated guide + all corrections), plus a small separate `ogauge` change removing the old copy.

**Loss-proofing while grinding:**
- Work on an `ogauge` branch (e.g. `docs/cnext-agents-guide-test-audit`); commit locally as changes accumulate (push + PR only at the end).
- Keep the **cursor** and **findings log** below up to date every few tests.

**Progress tracker (resumable):**
- Total tests: **973**
- **Sweep order: feature-ordered** — walk the guide's sections top to bottom, reviewing each feature's test dir(s) against that section. Guide section order: Operators → Types → Variable Declarations → Overflow → Arrays → Strings → Structs → Enums → Bitmaps → Bit Manipulation → Casting → Functions → Pass-by-Ref → Control Flow → Scopes → Includes → Preprocessor → Constants → Callbacks → Atomic → Critical → Register → MISRA → Part 2 (C/C++ interop) → Part 3 (Common AI Mistakes) → Part 4 (Transpilation).
- **Relocation: DONE** — guide at `c-next/docs/cnext-ai-reference.md`, symlink updated, baseline committed on `docs/agents-guide-relocate-and-audit`. Editing happens on this c-next copy.
- **Cursor — last completed:** §Register (bitmap-typed fields + access-violation) + §MISRA confirmed → **PART 1 COMPLETE.** **Next: Part 2 C/C++ Interop** → `c-interop`(20), `null-check`(25), `external-types`(7), `external-const-array`, `interop/*`, `real-libraries/*`(cjson, freertos), `template-types`(5), `global-type`(5), `header-gen`/`header-generation`, `cpp-interop`, `cpp-mode`, + **C++ object construction** (Issue #375) + `cpp-constructors`(5). **BEHAVIOR AUDIT COMPLETE** (Parts 1–4) → **PR OPEN: jlaustill/c-next#1009** (22 commits, all CI green); signed-compound-shift bug filed: **jlaustill/c-next#1008**. **NOW: exhaustive per-file pass** (user-requested) — read EVERY remaining test file; push any new findings to PR #1009. Then remove old guide copy from ogauge, then MongoDB spec.

**Exhaustive-pass progress:**
- Fully read: arithmetic, comparison, ternary, primitives, literals, floats, sizeof, logical, scope-resolution.
- **PRIORITY unread (likely findings — §Control Flow was skipped in the sweep):** switch(30), control-flow(17), for-loops(15), do-while(7), conditions(10); basics(18), overflow-modifiers(16), initialization(11), postfix-chains(10), assignment(4), expression-contexts(4).
- Bulk-confirm unread: const(48) + per-type-clone remainders across bitwise/compound-assign/casting/float-cast/array*/string*/struct*/enum*/scope(44)/bitmap/bit-indexing/register/c-interop/null-check/atomic/explicit-length/etc.
- Long tail: analysis(17), warnings(11), regression(12), bugs/issue-*(~15), issue-296/497/502/516/522/525(~25), header-gen/generation, template-types(5), global-type(5), real-libraries(3), comments(6), misc singletons.
- Regenerate list: `find tests -name '*.test.cnx' | sort`. Total ~960.

**⚠️ METHODOLOGY CORRECTION (user, 2026-06-19):** the SPEC (Accepted/Implemented ADRs in `c-next/docs/decisions/` + `learn-cnext-in-y-minutes.md`) is canonical — a test exercising non-spec behavior is a **BUG**, not a feature to document. Re-validated at-risk edits:
- pass-by-ref edit ✓ spec (ADR-006: everything-a-reference; semantics always by-ref; small/read-only may be by-value optimization). KEPT.
- compound-assign 10-op list ✓ spec (ADR-013 line 511 enumerates all 10). KEPT.
- switch rules ✓ spec (ADR-025: ≥2 clauses/16.6, no-bool/16.7, no-dup, default-last/16.5, enum exhaustiveness + `default(n)` counted default) → **ADDED** to §Control Flow.
- **float ÷0 "valid" claim was WRONG → reverted** (commit c956df20). 
- **break/continue working = BUG** (y-minutes: "No break/continue"); guide stays correct (Mistake #10 + §Functions rule).
- **Bugs filed:** #1008 (signed compound shift), #1010 (float ÷0), #1011 (break/continue), #1012 (compound-assign bypasses E0381 use-before-init), #1014 (multi-dim C-style trailing brackets `u8[4] matrix[8]`), #1015 (trailing-bracket size inference `u8 data[]`), #1016 (scoped-type trailing brackets `Sc.S items[4]`), #1017 (string-type trailing brackets `string<N> c[K]`), #1018 (broken test-execution tests / CI gap).
- **Stale test (NOT a language bug):** `tests/references/array-pass-by-ref.test.cnx:30` is `test-execution` but FAILS to compile — uses the (correctly) rejected C-style param `u32 dst[4]` (should be `u32[4] dst`). Implies a CI gap (a broken test-execution test passing CI). Flagged to user; offered to fix the one line. (Other C-style-param hits are fine: `main(u8 args[][])` argv is special and compiles; `sizeof/array-param-error` + `analysis/sizeof-array-param` correctly expect E0601.)
- **2nd broken test + E0381 mapping:** `tests/basics/primitive-types.test.cnx:48` (test-execution) reads an uninitialized local (`u32 uninitialized; if (uninitialized != 0)`) → fails with **E0381**. Verified E0381 fires on EVERY read-before-explicit-assign (if-condition, return, initializer); only compound-assign skips it (#1012). Storage is still zero-init (`uint32_t x = 0;`). So ≥2 test-execution tests fail to compile locally → likely CI gap. **OPEN QUESTION (asked user 2026-06-19):** is E0381-on-read intended (storage zeroed, but reading before explicit assign is a deliberate error → these tests are the bug to fix), or is E0381 itself the bug (per "no uninitialized variables exist", reading a zeroed local should yield 0 → file it)? **RESOLVED (user 2026-06-19):** E0381-on-read is **intended** (storage zeroed `= 0`, but reading-before-explicit-assign is a deliberate safety error). The broken tests were **filed as #1018** (user chose file-not-fix — do NOT modify the tests). **3 broken test-execution tests total:** `references/array-pass-by-ref.test.cnx` (C-style param), `basics/primitive-types.test.cnx:48` (E0381 read-before-init), `initialization/switch-branch-init.test.cnx` (colon switch syntax `case X: {`) — 3rd added as a comment on #1018. Guide's "all variables zero-initialized" describes STORAGE; E0381 is the separate use-before-assign safety net. (My original guide revert stands — correct.)
- **Session-2 guide additions (committed → PR #1009):** strings global-scope op restriction + single-byte `s[i]` semantics; slice broadened to all int arrays + const-offset/zero-length rules; enum-member-as-array-dim (no cast); §Scopes visibility defaults (vars private / funcs public); nullable `E0901` (ignoring a NULL-returning result); nested-template `> >` spacing; C++ class-with-ctor field-init is function-only (#517). Families CONFIRMED accurate (no guide change needed): bitmaps, bit-indexing, registers (incl. w1c/w1s), atomic, critical, ISR, volatile, callbacks (nominal typing), pass-by-ref/value, forward-decls (no recursion), C interop (structs/enums/unions/typedefs/extern/macros), template types, global.Type.
- **C-style trailing-bracket check has gaps** (root cause shared by #1014/#1015): the "C-style array declaration is not allowed" check fires for 1D *sized* (`u8 b[256]` ✓ rejected) but MISSES multi-dim 2nd+ dim (`u8[4] m[8]` #1014), 1D empty-bracket inference (`u8 data[]` #1015), qualified/scoped type names (`Sc.S items[4]` #1016), and parameterized `string<N> c[K]` (#1017). **INVENTORY COMPLETE** — root cause: the check only fires on bare type identifiers (primitive/bare-struct/bare-enum/bare-bitmap are all correctly rejected); any *non-bare* type expression (scoped `Sc.T`, parameterized `string<N>`, array-position `u8[N]`, or empty-bracket inference) bypasses it. When later tests use these forms it is KNOWN (#1014/#1015/#1016/#1017) — confirm behavior and move on, no new filings. **Array PARAMETERS are clean** — `u8 d[]` rejected ("C-style array parameter…use 'u8[] d'"), unbounded `u8[] d` rejected ("all dimensions must have explicit sizes") → valid param is `u8[16] d`. (Minor: the param fix-it message suggests `u8[] d`, which is itself rejected as unbounded — message-quality nit, flagged to user, not filed.) Canonical decl forms (y-minutes/ADR-035/036): `u8[N] x`, `u8[] x <- […]` (inference), `u8[N][M] x` (multi-dim).
- **Rule for the rest of the exhaustive pass:** when a test contradicts the guide, check the ADR/y-minutes first — the test may be the bug. Verify suspected bugs with `cnext` before filing.

**Exhaustive-pass findings (spec-validated):**
- **§Variable Declarations (ADR-015) — REVERTED (user correction 2026-06-19):** the guide's "All variables are zero-initialized. No uninitialized variables exist." is **CORRECT** — verified codegen: local `u32 x;` → `uint32_t x = 0;`. My earlier edit (claiming locals aren't auto-zeroed) was WRONG and is **reverted** (commit 439eb136, pushed to PR #1009). The `E0381` use-before-read error is a *separate* safety net layered on top of zero-init, and it is **inconsistent**: `u32 y; z <- y + 1;` errors, but `u32 sum; sum +<- 5;` compiles. Flagged to the user, who **confirmed it IS a bug** → filed as **jlaustill/c-next#1012** (compound-assign reads its LHS, so it must error `E0381` like a plain read; storage is still zero-init). **Not** a doc change. **Lesson (3rd correction): the user's stated INTENT overrides ADR wording AND test/transpiler behavior** — ADR-015's "locals error on use before init" line did not override the user's "everything is zero-initialized" intent.
- §Control Flow remainder (do-while/for/if/conditions): all confirm (E0701 non-bool condition; E0702 func-call-in-condition even when `!!`-nested). switch rules added; break/continue = bug #1011.
- postfix-chains, type-widening, clamp-wide-operand (Issue #94), struct-zero-init: confirm the guide, no change.
- **§Arrays/§Bit-Manipulation (analysis):** added — indices must be **unsigned ints** (signed→E0850, float→E0851). Spec-enforced (error codes). FIXED.
- **Part 2 (template-types, Issue #291):** added **§C++ Template Types** — `Type<Args>` instantiation passed through (e.g. `FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>`). FIXED.
- analysis dir otherwise confirms (division/modulo/float-modulo, sizeof-array-param/side-effects, undefined-function, uninitialized-variable, self-recursion). MINOR-skipped: `///` doc-comments (ADR-043), `://` URI comment exception, reserved-param-name `funcName_` prefix (E0227) — note if a comments section is ever added.
- **STILL TO READ:** const(48), overflow-modifiers remainder, basics(18), initialization remainder, expression-contexts, assignment, warnings(11), global-type(5), header-gen/generation, real-libraries(3), enum-comprehensive(10), nested-structs, regression(12), bugs/issue-*(~15), issue-296/497/502/516/522/525(~25), + per-type clone remainders.

**Part 2 in progress:** reviewed c-interop + cpp-constructors + external-types/cpp-init; added **C++ object construction** section + **C globals** (extern vars + macro constants) note. STILL TO REVIEW in Part 2: `null-check`(25, the nullable `c_` interop), `interop/anonymous-structs`(8), `external-types` details, `real-libraries` (cjson/freertos), remaining `c-interop` (unions/typedefs/nested — mostly confirm guide). Then **Part 3** (verify each of the 17 mistakes + new findings) and **Part 4** (Transpilation Reference + Known Issues table).
- **DEFERRED EDITS (apply when those sections come up):** (a) §Bit-Manipulation: float bit-indexing + bit-index compound-assign unsupported; (b) §Scopes: scoped structs (`scope X { public struct Y {} }`, `this.Y`, private=internal); (c) Part 2 interop: **C++ object construction** `CppClass obj(constArg1,…);` (args must be const vars, not literals; works in scope; Issue #375) + `cpp-constructors` (copy/default construction). → `arrays`, `multi-dim-arrays`(21), `array-initializers`(12), `array-declaration-syntax`, `array-param-assignment`(6), `array-struct-member`(5), `string-array-init`(5), `static-allocation`(8). Then §Strings (`string`, `string-assignment`, `slice-assignment`, `explicit-length`), §Structs, §Enums, §Bitmaps, §Bit-Manipulation (+2 deferred notes), §Functions, §Scopes, §Includes, §Preprocessor, §Constants, §Callbacks, §Atomic, §Critical, §Register, §MISRA → Part 2 interop, Part 3 mistakes, Part 4 transpilation. Deferred edits: float bit-indexing + bit-index compound-assign → §Bit Manipulation.
- Findings / guide changes so far:
  - **Operators (arithmetic, ADR-051):** guide said `/ % (same as C)`; omitted divide-by-zero. Added "Division and modulo guard against divide-by-zero" note — compile-time-zero rejection (`E0800`) + `safe_div`/`safe_mod`. _(committed)_
  - **Operators (bitwise, MISRA 10.1):** guide said `& | ^ ~ << >> (same as C)` — wrong for shifts. Signed shift (`i8/i16/i32/i64`) is `E0805`; `& | ^ ~` fine on signed. Split BITWISE vs SHIFT + "Shift operators require unsigned operands" note. _(committed 1f03266d)_
  - **Operators (comparison):** clean (all relational ops match "same as C"); but the block never listed `< > <= >=` → added a RELATIONAL line.
  - **Operators (compound-assign):** COMPOUND list was missing `/<-`, `%<-`, `^<-`, `>><-` (all 10 exist). Added them. _(edited; commit pending — bundled with RELATIONAL)_
  - **Bit Manipulation (DEFERRED to that section):** `x[a,b] +<- v` (compound-assign on a bit-index) is unsupported ("Compound assignment operators not supported for bit field access"). Add a note when auditing Bit Manipulation.
  - ⚠️ **LIKELY C-NEXT BUG (flagged to user 2026-06-19, awaiting decision):** signed compound shift-assign bypasses the `E0805` signed-shift check. `i8 r <- a << 2` → E0805, but `i8 a; a <<<- 2` / `a >><- 2` compiles to raw `(int8_t)(a << 2)` / `(int8_t)(a >> 2)` incl. UB `(int8_t)(-1 << 2)`. Repro: `tests/bitwise/i8-shift-ops` (errors) vs `tests/compound-assign/i8-compound-assign` (passes). Resolution pending: fix upstream (reject signed `<<<-`/`>><-`) vs intended (then document the exception). Shift note left stating the intended rule meanwhile.
  - **Control Flow (ternary):** added "condition must be PARENTHESIZED" (guide only showed parenthesized examples; `a > b ? a : b` is a parse error). Nested/non-boolean/bare-bool rules already correct. _(committed)_
  - **§Types (primitives):** (1) scalar values expose `.bit_length` (`u32`→32, `f64`→64, `bool`→8) — guide only showed it on arrays; added note. (2) clamp/wrap apply to signed too (saturate at min & max) — added note. clamp-default + wrap confirmed. _(committed 6d79023e)_
  - **§Types (literals) — NEW SECTION added:** guide had zero literal docs. Added `## Literals`: number bases (`0x`/`0X`, `0b`/`0B`, case-insensitive), integer type suffixes (`42u8`/`0xABCDu16`/`100I32`; u8/u16/u32/i8/i16/i32, case-insensitive), char literals (`'A'`=u8 65; init/compare/array/switch/arith) + escape sequences. _(committed b4d98cee)_
  - **§Types (floats):** (1) float `%` forbidden (`E0804`) — guide said `%` "same as C"; flagged `%` integer-only on ARITHMETIC line + division note. (2) float `/0.0` is VALID (inf/NaN) — E0800 is integer-only; added a Floats bullet. (3) added float literals + `f32`/`f64` suffixes to Literals. (4) **DEFERRED to §Bit Manipulation:** floats support bit-indexing (`f32val[24,8]` IEEE-754 byte read/write via union punning, MISRA 21.15). _(committed 9d06a298)_
  - **§Type Casting (casting + float-cast):** widening/narrowing/sign rules confirmed (+ bit-index workarounds for both). Added: (1) literal range-checking errors (`u8<-256`, `u8<-(-1)`); (2) enum→int explicit cast `(u32)Enum.X` (any width); (3) float→int **clamps** to range (not just truncates) — refined the FLOAT TRUNCATION line. cpp-cast-mode/reinterpret = codegen detail, not doc-worthy. _(committed ff3efa55)_
  - **sizeof — NEW SECTION added (ADR-023):** `sizeof(type/var/struct/member/array)` → byte size (compile-time); usable in expressions + as array dimensions; errors `E0601` (array param), `E0602` (side effects). Guide had no sizeof docs. _(committed 17d35d99)_
  - **§Arrays:** added compile-time bounds-checking (ADR-036) + size-inference (`u8 data[] <- […]`, ADR-035). Multi-dim guide form `u8[N][M] name` confirmed valid (split form `u8[N] name[M]` → same C). String arrays allow C-style `string<N> arr[K]` → note in §Strings. _(committed)_
  - **AUDIT APPROACH (from §Arrays on):** read every behavior-unique/error/feature test in full; **spot-sample per-type clone families** (`i8`/`i16`/`u64`/… variants that are mechanically identical and never yield doc changes). Disclosed to user; switch to exhaustive if asked.
  - **§Strings:** added (1) non-const must be sized / const auto-sizes; (2) literal/concat overflow = error; (3) string `=`/`!=` compare + **compound ops forbidden** (`s +<- x`); (4) string-literal escapes; (5) **slice assignment** `buf[constOff, constLen] <- value` = little-endian byte copy into u8[]/string (compile-time const offset+len, bounds-checked) — bytes on arrays vs bits on scalars. Directly enables the MongoDB C-Next JSON builder. _(committed abdc5c50)_
  - **§Structs:** added "don't repeat the struct type in an initializer" (`Point p <- Point{…}` ✗, ADR-014). Named-init / nested / zero-init / member-access / length-on-members confirmed. Scoped structs + C++ construction deferred (see DEFERRED EDITS above). _(committed 36f7e28e)_
  - **§Enums:** added (1) strong typing — no int↔enum assign/compare (use `(u32)Enum.X`), no negative values; (2) qualify members (`State.IDLE`); unqualified only where target type is known (assign/arg), errors in comparisons/non-enum switch. Scoped enums deferred → §Scopes. _(committed eb8c575e)_
  - **§Bitmaps + §Bit-Manipulation:** added (1) bitmap fields by name, not bracket-index (`f[3]` error); (2) float bit-indexing (`f32[24,8]` IEEE-754 bytes) [was deferred]; (3) compound-assign on bit-index target unsupported (`x[0,4] +<- v`) [was deferred]; (4) bytes-on-array vs bits-on-scalar reminder. bitmap-typed register fields → defer to §Register. _(committed 4d7c2e70)_
  - **§Functions / §Pass-by-Reference:** refined "ALL params pass-by-reference" — modified params → mutable pointer (caller sees changes, e.g. swap); unmodified → auto-const, by value (scalars)/const (structs). Define-before-use (no forward decl/recursion) confirmed. _(committed 1cc717d9)_
  - **§Scopes:** name-resolution (local→scope→global, this./global., bare scope calls, cross-scope, private) all confirmed. Added **Scoped Types** subsection (struct/enum/register inside a scope; `this.T`/`Scope.T`; private=internal) [was deferred] + "scopes cannot be nested" (ADR-016). _(committed 561179a3)_
  - **§Includes/§Preprocessor/§Callbacks/§Atomic/§Critical:** confirmed (`#define` value → E0502, `.h` without `.cnx` alt OK, callback function-as-type, atomic+clamp/wrap). **Added §ISR (ADR-040)** (built-in `void(void)` type for interrupt handlers — param/field/invoke). **Added `volatile` keyword (ADR-108)** (optimization barrier, distinct from atomic; `atomic volatile` = error). _(committed d10f7844)_
  - **§Register:** added bitmap-typed register fields (`CTRL: BitmapType rw @ off` → `Reg.CTRL.Field`) [was deferred] + access-mode violation is a compile error. §MISRA section confirmed accurate (rule table + 13.5/12.2 patterns). **→ PART 1 COMPLETE.** _(committed e86d2087)_
  - **Part 2 interop (in progress):** added **Constructing C++ Objects** section (`Cls obj(constArgs)` Issue #375 + `obj <- {field:val}` aggregate, Issue #924; arrays default-construct; literal arg = parse error) and a **C globals** note (extern vars/arrays/const + `#define` macro constants usable by C name). C structs/enums/unions/typedefs interop confirmed. Nullable: added **E0908** (a `c_` var must be null-checked before use). **Updated the obsolete "Anonymous Struct Flags Workaround"** → inline nested anon-struct init now works (Issue #882) — a doc that documented a since-fixed limitation. _(committed 83e55971)_
  - **Part 3 (Common AI Mistakes):** all 17 entries verified accurate (operators, array init, pointers, malloc, c_ prefix, bare bool, func-in-condition, narrowing, ++/--, break/continue, type aliases, bitmap size, int-for-C-enum, void*, same-base-name, string-concat-macros, sign-change-widening). No changes.
  - **Part 4 (Transpilation Reference):** refined the `void f(u32 x)` row (modified→pointer / read-only→by-value) + added a `volatile u32 x` row. Known-Issues table left as-is (signed-compound-shift bug NOT added — pending user decision). _(edited; commit pending)_

---

## Part B — MongoDB telemetry (DEFERRED until Part A is done)

**Goal:** stream Teensy telemetry to MongoDB **and** a live terminal view, with tuning
built into the tool. Kills the "paste serial text and grep it" bottleneck; enables
matched-operating-point analysis (manual vs auto), settled-CE-only views, BPR-error and
τ trends, and offline replay of the `~` gate.

**Agreed decisions (from the 2026-06-19 brainstorm):**
- **Scope:** both — live view **and** capture — from the start.
- **Live tuning:** integrated into the tool (keypresses → serial commands `bpr/kp/ki/auto/<pos>`).
- **On-wire format:** newline-delimited JSON (**NDJSON**), one object per sample.
- **Firmware emitter:** a **reusable C-Next `Json` builder scope** — fixed `string[N]` buffer +
  cursor, bounds-checked `addInt/addFloat/addBool/addStr` + `begin()/end()`; `N` proven from a
  worst-case dummy string; **no padding** (clean numeric values → real numbers in Mongo);
  adding a field = one `addField` call. Deterministic, memory-safe, MISRA-clean; could graduate
  into a reusable c-next json module. (C-Next string slice syntax exists today — confirmed by
  user — but the builder appends rather than overwrites, so it's not needed.)
- **Sequencing:** C-Next builder **first** — no C++ `snprintf` stepping stone.
- **Rate:** fixed **~10 Hz** (every 100 ms), one object/tick. (Gate runs at 10 Hz; 1 Hz can't
  reconstruct it. ~250 B × 10 Hz ≈ 25% of 115200 baud.)
- **Payload:** the debug-line fields **+ the gate's raw inputs** (COT, COP, CIP, boost, COT &
  boost slopes, `settledTimer`, settled flag) **+ `mode`** (manual/auto/brake) **+ `t_ms`**
  (firmware millis, for exact dt). This is what enables **offline replay/tuning of the `~` gate**.
- **Host:** a TS **CLI/TUI** using **OpenTUI** + `serialport` + `mongodb` driver. Read NDJSON →
  `JSON.parse` → live display → insert; keypresses send tuning commands. **No web server, no sockets.**
- **Mongo collections:**
  - `ovgt.telemetry` — one doc/sample (parsed JSON + host `ts`, `sessionId`).
  - `ovgt.settle_events` — τ measurements as their own docs.
  - `ovgt.sessions` — host creates one on connect (`sessionId`, start/end, `--label`, git SHA).
    Firmware stays session-agnostic; host owns stamping. (Mongo is already in the stack; mongodb
    MCP lets Claude query it directly to assist analysis.)

**Recommended-but-unconfirmed (decide when we revisit B):**
- Emit JSON **alongside** the existing human pretty-line during build-out, then retire the
  pretty-line once the TUI works (JSON-only). Avoids a format flag; keeps `logToTee.sh` usable meanwhile.
- Session label mechanism: `--label` CLI flag vs a TUI keypress.
- Exact field names/list.

**Related:** the `~` settled-flag fix (boost-flat too strict at 10 Hz; 0/854 cleared on the
2026-06-19 drive). The new 10 Hz logs are explicitly meant to help tune that fix. See
`memory/project_cot_probe_tau_baseline.md`.

**Next step when B resumes:** finalize the spec at
`docs/plans/2026-06-19-mongodb-telemetry-design.md`, user review, then writing-plans.
