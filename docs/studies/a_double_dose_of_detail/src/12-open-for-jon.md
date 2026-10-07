# Open for Jon

**All five items are answered** (2026-10-06; recorded in 01-decisions). Each item keeps its question and gives its answer.


1. **Visual-only and capacity-hint elements** (`camera light material texture visual skin`, `<statistic>` except `meaninertia`, `<size memory njmax nconmax nkey nstack nuserdata>`). Read literally, "MuJoCo-valid things we don't implement are errors" refuses them: 0 in-tree docs flip, but **47 of the 53 submodule models that load today stop loading** (226 of 253 use one). **Answer:** a third verdict, **accepted with no effect**, each element listed in the divergences table.
2. **Where MuJoCo itself fails.** (a) MuJoCo 3.5.0 refuses its own 2-body cables (internal exclude naming, "body 'B_1' not found", 8 corpus docs once `curve` is fixed); (b) lengthrange computation does not converge (4); (c) qhull fails on a flat mesh (1 + 2 CI tests). **Answer:** load and list, provided a test shows our result right (the lenient kind). (a) loads; (b) and (c) have no such test and are refused (Q122, Q103 in 13-open-questions).
3. **P-L34 delay/history.** Implement MuJoCo 3.5.0's history (samples inserted in `mj_advance`, actuation and sensors read delayed values) or refuse `delay > 0` / `nsample > 0` as a stated limitation. **Answer:** implement in Rigid.
4. **Known divergences:** sleep re-forwards on the sleep step and sleep timing (step 69 vs MuJoCo 76, A2 Q3); dim-3 tet re-orientation and boundary flaps (A6 §1.10); STL vertices not deduplicated (3× MuJoCo on `fourier_n1`); principal-axis order of a full inertia (A5 §4.4); hull face order. **Answer:** fix in Rigid; the hull is matched and tested as sets, in our own order, unless Jon wants the qhull port (01-decisions).
5. **P-L32 multi-joint dynamics** — in Rigid or its own PR. Cause isolated and fix measured (21-multi-joint-bias). **Answer:** in Rigid (second round), P24.

Also for Jon's review, though the rule settles them: the 14 earlier decisions are in 11-settled as recommended; sensor derivatives change semantics (01-decisions).
