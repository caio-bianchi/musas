# Build the MUSAS documentation.
#   make notebook  -> re-run the snippets and rebuild getting_started.ipynb
#   make pdf       -> compile musas_documentation.pdf (needs LuaLaTeX + minted)
#   make all       -> both, in this order
#   make clean     -> remove LaTeX auxiliary files and generated datasets
PYTHON ?= python

all: notebook pdf

notebook:
	$(PYTHON) tools/build_notebook.py

pdf:
	latexmk -lualatex -shell-escape -interaction=nonstopmode musas_documentation.tex

clean:
	latexmk -c musas_documentation.tex
	rm -rf _minted* outputs

.PHONY: all notebook pdf clean
