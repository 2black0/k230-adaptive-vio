# EqVIO SC-AFB adaptive/resource/robustness/realtime evidence — 2026-10-06

User authorized this package on the connected K230. Scope: mono cam0 half-resolution,
timestamp-paced camera AND IMU replay on the actual board. This is not live sensor integration.

## Frozen measurement contract

- Single Linux-visible core. Preserve usual board services; record hardware/config/binary hashes.
- Arrival schedule follows dataset timestamps against steady_clock, anchored to the first retained measurement.
- Process all measurements in timestamp order, with no drops or clock reset after a late frame.
- Arrival-to-output means scheduled camera arrival to completion of estimator state serialization.
  It includes waiting behind earlier work, image-reader work, estimator/controller and state logging.
  It excludes live camera exposure/driver, external consumer/transport and durable file flushing.
- Existing SimpleDataServer buffers one decoded image. getImage also decodes the next image;
  its work remains charged to the current frame. Initial dataset loading precedes the replay epoch.
- Pending camera count is inferred from scheduled arrivals at dispatch, not a physical capture queue.
  Include p50/p95/p99/max latency, 50 ms miss fraction, dispatch lateness, backlog maximum/trend,
  expected/processed frame counts, CPU/RSS, ATE/RPE and controller traces.
- 50 ms is the exploratory deadline for 20 Hz input; verify actual timestamp intervals per dataset.
  Report measured misses explicitly. Do not equate p99 or mean FPS with a hard realtime guarantee.
- Do not call ATE <5 m a tracking success criterion: it is the inherited numerical-divergence screen.
  Report continuous error and each sequence's accuracy regression separately.

## Frozen arms and run order

All arms: seed 0/1/2 where repeated, featureDistOverride 80, identical config/input/output logging.
SC-AFB: caps 5/20, bootstrap floor 8, hold 5, auto-floor window50 percentile40,
auto-hold window50 gain40, both enabled. No adaptive covariance. This recipe is shared
across datasets; it differs from the legacy TUM-VI adaptive-covariance caps3/7 recipe.

115 full-sequence runs:

1. Pilot normal: MH02, V203, TUM room3 × baseline/static15/SC-AFB × 3 seeds =27.
2. Same pilot under one competing CPU process, 30–90 s after replay begins =27.
3. Same sequences/seeds: base policy, floor only, hold only =27; full SC-AFB from pilot reused.
4. Baseline/SC-AFB on remaining 14 sequences at seed0 =28; pilot seed0 reused for all17 summary.
5. MH01 from original start, baseline/SC-AFB ×3 seeds =6. Main MH01 retains disclosed 20.55 s offset.

Static15 approximates archived applied budgets (MH02 mean15.16, V203 mean17.01).
It is a fixed common control, not a guaranteed compute-matched control; report measured cost differences.
No accuracy tuning or fixed-cap selection from new outcomes. Rotate pilot arm order by seed.

## Analysis and interpretation

- First analyze archived offline latency and reconstruct controller traces using the existing checked helper.
- Compare normal and contention runs on the same temporal segments; before/loaded/after differ in scene.
- Three seeds expose estimator variation as well as board timing variation. Report paired values,
  ranges and sequence coverage; three seeds alone do not establish population-level confidence.
- Robustness here concerns accepted-observation scarcity, difficult benchmark motion, initialization
  and CPU contention. No illumination/corruption/general outdoor robustness claim.
- Budget decisions use accepted observations, not measured available CPU. Resource efficiency and
  tolerance to contention can be tested; do not claim direct CPU-aware feedback absent that mechanism.
- Additional fixed-cap sweep is warranted only if static15 cost differs enough to leave the causal
  comparison inconclusive. Do not claim adaptivity superior merely because baseline is slower.
- Preserve failures and regressions. Initialization failure does not invalidate the disclosed trimmed-start
  experiment, but limits the initialization robustness claim.
- Run completion is not claim confirmation. Broad realtime/robustness claims remain conditional on results.
