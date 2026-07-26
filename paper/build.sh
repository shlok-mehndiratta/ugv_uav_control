#!/usr/bin/env bash
# Build the ACODS paper (IFAC ifacconf: natbib author-year + ifacconf.bst).
# Usage: ./build.sh [mainfile-without-.tex]   (default: ACODS)
# Requires a full TeX Live (this repo's dev box has a minimal install missing
# algorithm/pst-*; build on a machine with texlive-full or Overleaf).
set -e
cd "$(dirname "$0")"
MAIN="${1:-ACODS}"
echo ">> pdflatex pass 1"; pdflatex -interaction=nonstopmode "$MAIN.tex" >/dev/null
echo ">> bibtex";          bibtex "$MAIN" >/dev/null || echo "   (bibtex warnings — check undefined keys)"
echo ">> pdflatex pass 2"; pdflatex -interaction=nonstopmode "$MAIN.tex" >/dev/null
echo ">> pdflatex pass 3"; pdflatex -interaction=nonstopmode "$MAIN.tex" >/dev/null
echo "Built $MAIN.pdf ($(pdfinfo "$MAIN.pdf" 2>/dev/null | awk '/Pages/{print $2}') pages)"
echo "Check for issues:"
grep -h "Warning: Citation" "$MAIN.log" | sort -u || true
