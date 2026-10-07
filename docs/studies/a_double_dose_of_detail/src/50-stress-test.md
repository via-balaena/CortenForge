# Stress test, round 1

Run 2026-10-07 on the book at `fea25180` (code identical to `main` @ `3520544e`). The five reports and the consolidated triage are in `$SCRATCH/rigid_stress/` and are not in the repo. This chapter records what was searched for, what was found, and where each finding was sent; it does not say the book is now correct.

## Method

**Criteria.** A finding is something that would make an implementer, following the spec as written, do the wrong thing, get stuck, or ship something the owner decided against. Each is scored blocker, should-fix or nit, and needs a referent (book `file:line`, a research section, a repo `file:line` at HEAD, or a command and its output).

- **K1 Decision fidelity** — every chapter agrees with 01-decisions, the C1–C11 resolutions and the (c)-answers.
- **K2 Implementable in order** — each commit names its change and must-fail; no forward dependency or cycle; each commit green alone.
- **K3 Referents** — a commit's claims are what the cited research and the code at HEAD say.
- **K4 Coverage** — every scope row, census cluster, C-resolution and answer lands in exactly one commit.
- **K5 Gaps** — places the spec admits, or does not admit, it leaves unspecified.
- **K6 The census gate** — P01's ratchet can fail, runs in CI without MuJoCo, and works across both PRs.
- **K7 Parity rule** — each deviation fits one of the four kinds and reaches the divergences table.
- **K8 Breaking and downstream** — each breaking change names its call sites, checked with `git grep` at HEAD.
- **K9 Hazards** — anything the spec asks an implementer to run that could hang or allocate more than a few GB.
- **K10 Public text** — no local paths, user names, private file names or product scan names.

**Reviewers.** Five cold reviewers, read-only on the repo, no build and no `cargo`; each recorded HEAD `fea25180` and an empty `git status --short` before and after. R1 took Rigid-physics (P01–P53); R2 Rigid-loading (L01–L50); R3 coverage, worked from the sources outward; R4 nine named gaps (K5), two groups of them delegated to forks of R4 under the same rules; R5 the whole plan.

**A list of suspected defects** (26 entries) was written by the planner before the run. It was not given to the reviewers, but it sat in the directory that held their criteria. R3 and R5 read it by accident and said so, so their findings are not independent of it. R4 said it did not read it; R1 and R2 did not say.

## Counts

| reviewer | focus | blocker | should-fix | nit | read the list |
|---|---|---|---|---|---|
| R1 | Rigid-physics | 8 | 17 | 12 | did not say |
| R2 | Rigid-loading | 6 | 12 | 9 | did not say |
| R3 | coverage | 4 | 29 | 12 | yes |
| R4 | gaps | 6 | 28 | 13 | no |
| R5 | whole plan | 4 | 13 | 0 | yes |

The counts are each reviewer's own, before duplicates across reviewers were merged into T1–T23. This was the first pass, so 0 findings came from an earlier pass's fixes.

## The findings, and where each went

"Fixed in" names the chapter a fix was assigned to in this round. Whether each fix landed as written is the job of *Fix-diff pass* below.

| T | finding | reviewers | fixed in |
|---|---|---|---|
| T1 | C1 (plain arithmetic) is contradicted: P48 writes `f64::mul_add`, L03 prescribes fused multiply-adds, and P50's `6369216c` agree claim and L03's census counts were measured on the fused build; the bitwise goldens have no unfused generator | R1 B1, R2 B4 S9, R3 B1 S5, R4 X-1, R5 B1 | fused text cut in ch. 20 and 22; the oracle is a `-ffp-contract=off` build of MuJoCo 3.5.0 (40-verification, *The unfused oracle*; measured below) |
| T2 | C7 (MuJoCo's flex meaning) has no carrier: L10 still implements the old plan, and `node=`, `<elasticity damping>`, an empty `element` and element- vs vertex-based contacts have no commit | R2 B1, R3 B2 S17, R4 C7-1–C7-6, R5 B4 | fixed in ch. 22 (L10 from R4's draft; L31 and L32 rebased on it) |
| T3 | L47 and L49 form a cycle, and L47 refuses a MuJoCo-loadable doc until L49 | R2 B3 N5, R3 B3, R5 B3 | fixed in ch. 22 (L49 before L47) |
| T4 | L35 excludes mesh re-centring, against Q63; f32 embedded vertices (Q121) have no commit | R2 B5, R3 B4, R5 S6 | fixed in ch. 22 (L35, breaking, readers named) |
| T5 | divergence rows arrive in P53/L50, after the commits whose `divergence=` notes need them; the table has no ID column | R1 S8, R2 S1, R3 S27, R5 B2 S4 | fixed in ch. 20 and 22 (each commit adds the rows its verdicts name; P01 adds the ID column) |
| T6 | P05 names no type, test or downstream; sim-ml-chassis's action layout would swap halves silently | R1 B4, R4 P05-1–P05-8, R5 S5 | decision D1 (01-decisions, fourth round); fixed in ch. 20 (P05 from R4's draft plus the D1 rename, cross-crate) |
| T7 | P14 has no change and no must-fail; `derivatives/stress-test` does not flip; P53 keeps pre-P14 rows | R1 B5 S2, R4 P14-1–P14-7 | fixed in ch. 20 (P14 from R4's draft; P53 rows cut) |
| T8 | C9: P42 says the stored normal is not renormalised | R1 B2, R3 S2, R4 C9-1–C9-5 | fixed in ch. 20 (P42; P52 depends on P42) |
| T9 | C8: P10 keeps `# Panics` for the joint-layout check | R1 B3, R2 S12, R3 S1, R4 F8 | fixed in ch. 20 (P10) |
| T10 | physics tests that need later loading fixes: P39, P52, P33, P18 | R1 B6 B7 S11 S12, R2 S11, R5 S6 | fixed in ch. 20 (those tests set the fields in code) |
| T11 | answered questions with no carrier (Q17, Q22–Q24, Q28–Q30, Q44, Q46, Q66, Q69, Q73, Q108, Q118, Q119, Q127, Q129, U12) and the class-(a) U-findings | R1 B8 S16, R2 S10, R3 S6–S24, R4 F13 | fixed in ch. 20 and 22; the U table in 13-open-questions names each carrier (U11 has none) |
| T12 | L05 is not green alone: sim-urdf writes `<inertial pos>` only when non-zero | R2 B2 | fixed in ch. 22 (L05, cross-crate) |
| T13 | must-fails whose parent side hangs or grows are bounded by `timeout` only | R1 S14, R2 B6 S7 S8 N1 N2 N7, R5 S11 | rule added to 40-verification; entries fixed in ch. 22 |
| T14 | L46 has no change or must-fail; P23 is not green alone and leaves init-sleep failures at reset unspecified | R2 S5, R4 F1–F7 | fixed in ch. 22 (L46 from R4's draft) and ch. 20 (P23) |
| T15 | the gate has no rule for a transient regression, no pinned bless platform, no enforced append-only, no per-commit drift check | R1 S9 S10, R5 S1–S3 | fixed in ch. 20 (P01) |
| T16 | census clusters are closed only in part, and the residual docs have no owner | R3 S26 | owner table in 40-verification *The census residuals* |
| T17 | the tooling exists only in scratch; the per-commit loop and its costs were never piloted; runtime MJCF was never captured | R5 S8–S10 | 40-verification *Before implementation* |
| T18 | text that still says a superseded thing, and § references to the deleted single-file spec | R3 K1 sweep, R1 S1, R2 S3 | fixed in ch. 00, 01, 10–13, 40, 41, 20 and 22 |
| T19 | two titles carry two scopes, which the commit-msg hook refuses (P40, L36) | R1 S5 | fixed in ch. 20 and 22 |
| T20 | the same loading rule is placed in two commits, some with conflicting field types | R2 S2 | fixed in ch. 22 |
| T21 | L47 has no "accepted with no effect" verdict | R2 S6 | fixed in ch. 22 |
| T22 | PR boundary: L31's core rule can land in Rigid-physics; L44 may split; `cargo test -p sim-gpu` missing from the physics checks | R5 S5 | fixed in ch. 20 and 22 |
| T23 | smaller items: `newton_solved` readers, dead code under clippy, P02's "×1" claim, "L41" meaning ledger-L41, the licensed gate for P03, `compute_history_addresses` owner, `d22fcd34`'s commit, P53's removed rows, one `.mjb` version bump, nits | R1 S3 S4 S6 S7 S13 S15 S17, R2 N3, others | fixed in ch. 20 and 22 |

## The suspected-defects list, scored

The 26 entries were scored against the reports by reading them.

- **21 found by at least one of R1, R2, R4**, none of which disclosed reading the list: p1–p11 (with p5b), p14, p15, p19–p25.
- **3 found only by R3 or R5**, who had read it: p13, p17, p18.
- **1 found only in part**, by R5: p12.
- **1 wrong:** p16 said P24's GPU shader change needs a GPU machine to run; CI runs `sim-gpu` on lavapipe (`.github/workflows/quality-gate.yml:600-606`).

**What the list missed** — classes no entry named, found by the reviewers:

- a downstream that compiles and changes meaning silently (sim-ml-chassis's action layout);
- physics tests whose fixtures need a loading fix from the later PR;
- a loading commit that is not green alone because a converter in another crate omits an attribute (sim-urdf);
- must-fail inputs that hang or grow memory at the parent;
- divergence rows landing after the gate needs them;
- commit titles the hook's scope regex refuses;
- an init-sleep refusal assumed to fire at load that does not (L46), and a test it turns red (P23, `sleeping.rs:2612`);
- one rule placed in two commits;
- a missing third verdict in the schema pass;
- two non-public file names in a research section (A6);
- a flex Jacobian that assumes three slide dofs per vertex;
- MuJoCo's x86_64 wheel as an unfused oracle;
- census residuals with no owner;
- a claim that the harness can run once per commit before the commit that makes it deterministic (P02).

**A class an earlier check cleared.** The planner had earlier judged A6's two non-public file names harmless. R1 (nit 12) and R5 (S13) flagged them, and they are cut. That is evidence about the method: a read by the book's author had cleared a K10 item that two cold reviewers then found.

## What this search could and could not see

- **Nothing was built or run.** Every "flips" and "fails at the parent" in the reports is from reading unless it cites a research measurement. Per-commit greenness in this order is unmeasured, and so are per-commit census counts: each research section measured on its own base, and A20 §2.10 measured end states only.
- **Each reviewer's stated blind spots**, from its report:
  - R1: whether anything compiles; dependents `git grep` cannot name; MuJoCo citations not re-opened; A3, A5 beyond §3.3, A9, A11, A13 §3, A15 and A16 §0–§1 not opened.
  - R2: orderings that live only in the research; a line reference shows a pointer, not the behaviour; paraphrases outside its grep words; behavioural consumers and runtime MJCF.
  - R3: research items that never reached 13-open-questions or 10-scope (it swept the hand-off lists of A7, A11, A16, A19, A20 and A21 only); answer-to-commit mapping by topic reading; ledger rows outside L26–L51.
  - R4: C9's census effect; whether any CI job has a GPU.
  - R5: research outside A20 Part 2 and A6 §4 read by excerpt only; no line-by-line referent checks; the cross-platform libm risk.
- **The decisions and answers were treated as fixed.** No reviewer judged whether they are right.

## The oracle measurement (T1)

The question: does an x86_64 MuJoCo 3.5.0 build, which contains no fused multiply-adds, give the same bits as a MuJoCo 3.5.0 C build with contraction off? If yes, the x86_64 wheel could be the oracle. Scripts and outputs: `$SCRATCH/oracle_x86/` (`RESULT.md`).

| measured | result |
|---|---|
| fused instructions (`vfmadd`/`vfmsub`/`vfnmadd`/`vfnmsub`) | 0 in the Linux manylinux x86_64 `.so` (clang 20.1.8), 0 in the macOS x86_64 wheel (`macosx_10_16_x86_64`), 0 in the x86_64 slice of the arm64 wheel (the binary A18 §3.4 counted). The counter was made to fail: `box.c` built for x86_64 with `-mfma` gives 185 |
| A21's C build, rebuilt | equals A21's output files byte for byte: 4,400 poses and 3,000 frames, with and without FMA |
| arm64 wheel vs the C build with FMA | equal on every case: 4,400 poses, 3,000 frames, 20,211 inertia tensors |
| arm64 wheel vs the C build without FMA | differs on 1,153 of 4,400 poses, 1,664 of 3,000 frames, 19,960 of 20,211 tensors; max 4.13e-12 |
| the C build without FMA vs A10's plain Rust `mjuu_eig3` port | equal on 20,211 of 20,211 (A10 had stated it by reading) |
| an x86_64 build vs the C build without FMA | **not measured**: 0 cases. This machine has no Rosetta (`arch -x86_64` fails), and the x86_64 Python bindings refuse to load under Rosetta (`mujoco/__init__.py:40-50`) |

**Decision (engineering):** the oracle is the committed `-ffp-contract=off` build (40-verification, *The unfused oracle*): it is the one measured to equal a plain port, and it runs on the machine that blesses. Not seen: an x86_64 run of any kind; differences other than fusion (libm, other code generation); the AVX paths a whole step takes on x86_64.

## Fix-diff pass
