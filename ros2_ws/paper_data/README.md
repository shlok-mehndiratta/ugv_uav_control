# `paper_data/` — the exact data behind every figure and table in the paper

This folder holds **only** the curated runs used to produce the paper's figures/table, so the
paper is reproducible without hunting through scratch output. Each figure regenerates from the
data here via a script in `../scripts/` (writing into `../../paper/media/`). New/experimental
runs go to `../results/` (scratch); only validated runs are promoted here.

| Subfolder | Backs | Script (`../scripts/`) | Contents |
|---|---|---|---|
| `fig1_testbed/` | **Fig. 1** (single camera, dual role) | `make_testbed_figure.py` | `testbed_frame.png` (one real captured downward-camera frame) |
| `fig3_envelope/` | **Fig. 3** (coverage–resolution "two jaws") | `plot_envelope.py` | `env_{track1,track4,track_slalom}_z{2,2.5,3,4,6}_*` (CSV + summary); gentle=track1, slalom=track_slalom, sharp=track4 |
| `fig4_altitude/` | **Fig. 4** (adaptive vs fixed altitude) | `plot_altitude_codesign.py` | `cga_{adaptive,fixedlow,fixedhi}_20260706*` (R=3 runs each; CSV + summary) |
| `table2_codesign/` | **Table II** (event-triggered co-design) | `fill_paper_numbers.py` | `gcod_{C1,C2}_r{1,2,3}_20260715*` (CSV + summary); C1 = continuous, C2 = event-triggered |
| `reproducibility/` | headless-fork claim (Sec. V) | — | `repro_hl_*` (headless runs; ≈0.03 m agreement) |

## Regenerate the figures
```bash
cd ros2_ws                      # scripts use repo-relative paths; cwd does not matter
python3 scripts/make_testbed_figure.py       # -> paper/media/Fig_Testbed.png   (Fig. 1)
python3 scripts/plot_envelope.py             # -> paper/media/Fig_Envelope.png  (Fig. 3)
python3 scripts/plot_altitude_codesign.py    # -> paper/media/Fig_AltPolicy.png (Fig. 4)
python3 scripts/fill_paper_numbers.py        # prints Table II (C1 vs C2)
```
Fig. 1 and Fig. 4 regenerate **byte-identically** to the shipped figures. Table II reproduces
exactly (solves 797±16→32±1, aerial CPU 3123±56→253±6 ms, steady-state trΣ≈0.0095, ArUco ≈99.8/99.9 %).

## Note on Fig. 3
The canonical Fig. 3 shipped in the paper is `paper/media/Fig_Envelope.png` (committed). Its
original one-off plotting script was not preserved; `plot_envelope.py` regenerates it from the data
here — the right panel (mean trΣ) reproduces exactly, and the left panel (steering effort std(ω))
reproduces the same curves/trend up to the exact per-run std window (`--window`, default full-run).
