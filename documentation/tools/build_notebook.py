"""Build and execute getting_started.ipynb from the code snippets.

The PDF documentation includes the same files from ``snippets/``, so the code
shown in the PDF and in the notebook is always identical. Run from the
``documentation`` directory with the Python environment where MUSAS is
installed:

    python tools/build_notebook.py
"""

from __future__ import annotations

import base64
import contextlib
import io
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
SNIPPETS = ROOT / "snippets"
NOTEBOOK = ROOT / "getting_started.ipynb"
OUTPUT_TEXT = SNIPPETS / "output"   # printed output, included by the PDF

# (markdown text, snippet file or None)
CELLS = [
    ("# MUSAS — Getting Started\n\n"
     "Companion notebook of the MUSAS documentation (`musas_documentation.pdf`). "
     "Every code cell is identical to a listing of the PDF, in the same order.\n\n"
     "Run this notebook from the `documentation` folder. If MUSAS is not "
     "installed yet, install it directly from GitHub:\n\n"
     "```\n%pip install \"git+https://github.com/caio-bianchi/musas.git\"\n```",
     None),
    ("## 1. Imports and output folders", "01_setup.py"),
    ("## 2. Airspace\n\nA permanent obstacle: a vertical cylinder of radius 15 m "
     "centred at (0, 50).", "02_airspace.py"),
    ("## 3. Flight plans\n\nTwo UASs that would reach the origin at the same time. "
     "UAV B also crosses the obstacle.", "03_flight_plans.py"),
    ("## 4. Strategic deconfliction algorithms\n\nConflicts are predicted with a "
     "30 m separation sphere. Airspace conflicts are removed by a lateral detour "
     "and traffic conflicts by delaying the departure.", "04_algorithms.py"),
    ("## 5. Simulation and UASs", "05_simulation.py"),
    ("## 6. Run", "06_run.py"),
    ("## 7. Metrics", "07_metrics.py"),
    ("## 8. Figure", "08_plot.py"),
    ("## 9. Tactical layer and a dynamic geofence\n\nA new scenario without "
     "strategic deconfliction: a geofence appears ahead of the UAS during "
     "the flight.", "09_tactical.py"),
    ("## 10. Tactical results", "10_tactical_plot.py"),
    ("## 11. Plugging in your own algorithm\n\nAny class that implements the "
     "contract can replace a built-in algorithm.", "11_custom_algorithm.py"),
    ("## 12. Reading the dataset", "12_dataset.py"),
    ("## 13. Animation\n\nThe GIF is written to `outputs/quickstart.gif`.",
     "13_animation.py"),
]


def figure_outputs() -> list[dict]:
    outputs = []
    for number in plt.get_fignums():
        buffer = io.BytesIO()
        plt.figure(number).savefig(buffer, format="png", dpi=90,
                                   bbox_inches="tight")
        outputs.append({
            "output_type": "display_data",
            "metadata": {},
            "data": {
                "image/png": base64.b64encode(buffer.getvalue()).decode(),
                "text/plain": ["<Figure>"],
            },
        })
    plt.close("all")
    return outputs


def main() -> None:
    namespace: dict = {"__name__": "__main__"}
    cells = []
    count = 0
    import os
    os.chdir(ROOT)
    OUTPUT_TEXT.mkdir(exist_ok=True)
    for markdown, snippet in CELLS:
        cells.append({"cell_type": "markdown", "id": f"md-{len(cells):02d}",
                      "metadata": {},
                      "source": markdown.splitlines(keepends=True)})
        if snippet is None:
            continue
        source = (SNIPPETS / snippet).read_text()
        count += 1
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            exec(compile(source, snippet, "exec"), namespace)
        outputs = []
        text_file = OUTPUT_TEXT / snippet.replace(".py", ".txt")
        if stdout.getvalue():
            text_file.write_text(stdout.getvalue())
        elif text_file.exists():
            text_file.unlink()
        if stdout.getvalue():
            outputs.append({"output_type": "stream", "name": "stdout",
                            "text": stdout.getvalue().splitlines(keepends=True)})
        outputs += figure_outputs()
        cells.append({
            "cell_type": "code", "id": f"code-{count:02d}",
            "execution_count": count, "metadata": {},
            "outputs": outputs,
            "source": source.rstrip("\n").splitlines(keepends=True),
        })
        print("executed", snippet, file=sys.stderr)
    notebook = {
        "cells": cells,
        "metadata": {
            "kernelspec": {"display_name": "Python 3", "language": "python",
                           "name": "python3"},
            "language_info": {"name": "python"},
        },
        "nbformat": 4,
        "nbformat_minor": 5,
    }
    NOTEBOOK.write_text(json.dumps(notebook, indent=1, ensure_ascii=False) + "\n")
    print("Wrote", NOTEBOOK, file=sys.stderr)


if __name__ == "__main__":
    main()
