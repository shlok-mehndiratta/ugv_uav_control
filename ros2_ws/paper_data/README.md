# `paper_data/` — curated data behind the paper's figures and table

Each subfolder holds the exact runs used to produce one figure or table in the paper. Every run is a
per-step CSV; where present, a `_summary.txt` gives the aggregate metrics for that run.

| Subfolder | Backs | Contents |
|---|---|---|
| `fig3a_e0/` | **Fig. 3** — resolution model (`R ∝ z²`) | static-altitude ArUco position-error runs (`e0_z*.csv`) |
| `fig3_envelope/` | **Fig. 4** — estimation vs. navigation cost across altitude | altitude sweep on the Low-/Mid-/High-κ tracks (CSV + summary) |
| `fig4_adaptive/` | **Fig. 5** — adaptive vs. fixed altitude | Var-κ track: adaptive and fixed at {2.5, 4.0, 7.0} m, 3 repetitions each (`f4_*.csv`) |
| `table2_codesign/` | **Table 2** — event-triggered co-design | continuous (C1) vs. event-triggered (C2), 3 repetitions (`gcod_*`; CSV + summary) |

These are the promoted, validated runs only; fresh experiments produced by the launch files land in
`../results/` (scratch, not tracked). All runs are headless (`GZ_HEADLESS=1`).
