# ACODS submission — folder structure & status

IFAC-affiliated conference; template is **ifacconf**. This folder is the paper's build root.

## Files

| File | Role | Status |
|---|---|---|
| `ACODS.tex` | master document | draft (SITL-forward); being reframed theory-first |
| `references.bib` | **all references** (BibTeX) | ✅ set up; 4 legacy keys marked `%VERIFY` |
| `ifacconf.cls`, `ifacconf.bst` | IFAC class + bib style | provided |
| `media/` | figures (generated from `paper_data/`) | folder ready; images pending sim runs |
| `build.sh` | compile script | ✅ |
| `THEORY_DRAFT.tex` | §Theoretical Analysis (Prop A/E) | ✅ done, compiles |
| `RELATED_WORK_DRAFT.tex` | §Related Work (rewritten) | ✅ done, compiles |
| `SYSTEM_MODELS_DRAFT.tex` | §System Architecture & Models | ✅ done, compiles |
| `THEORY_DRAFT_standalone.tex` | compile harness for the theory | helper |
| `LIT_SURVEY.md`, `PAPER_PLAN.md`, `SIM_AND_PLOTS.md` | planning docs | reference |

## How references work in ifacconf (the standard way)

`ifacconf.cls` **auto-loads `natbib`** and sets `\bibliographystyle{ifacconf}` in
**author–year** mode. So:

- Put all entries in `references.bib` (BibTeX).
- In `ACODS.tex`, cite with **`\citet{key}`** ("Author (2020)") and **`\citep{key}`**
  ("(Author, 2020)"). Plain `\cite{}` also works (natbib maps it), but `\citet/\citep`
  are the IFAC convention.
- **Do NOT** `\usepackage{cite}` — it conflicts with natbib. (Already removed.)
- `\bibliography{references}` sits before `\end{document}`. (Already wired.)
- Build runs BibTeX: `pdflatex → bibtex → pdflatex → pdflatex` (see `build.sh`).

> Note: this dev box has a **minimal TeX install** (missing `algorithm`, `pst-*`, …),
> so the full `ACODS.tex` builds on a `texlive-full` machine or Overleaf. The section
> drafts compile here standalone via the `article` harness.

## Assembly plan (integrate the finished drafts into `ACODS.tex`)

1. Replace the current `\section{RELATED WORK}` body with `RELATED_WORK_DRAFT.tex`.
2. Replace `\section{System Architecture and Models}` body with `SYSTEM_MODELS_DRAFT.tex`.
3. Insert `THEORY_DRAFT.tex` as a new `\section{Theoretical Analysis}` **after** the
   System section and **before** the controller section.
4. Switch `\cite{}` → `\citet/\citep` as appropriate (author–year reads better inline).

## Post-run checklist (after the sim matrix lands — content only)

- [ ] Generate figures into `media/` (plot scripts read `paper_data/`).
- [ ] Fill the `\RES...` result macros (top of `ACODS.tex`) from the new runs:
      envelope two-jaw numbers, adaptive-vs-fixed, co-design Table II.
- [ ] Rename tracks throughout: Gentle/Slalom/Sharp → **Low-κ / Mid-κ / High-κ / Var-κ**.
- [ ] Write **Abstract + Introduction** (deferred — framing-sensitive; lead-identity
      decided once plots confirm the two jaws).
- [ ] Write the **Simulation Study** results prose against final numbers.
- [ ] Replace the 4 `%VERIFY` bib entries with the confirmed citations.
- [ ] Final `./build.sh` on a full-TeX machine; check page count ≤ 6 and no undefined cites.
