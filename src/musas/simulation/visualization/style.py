"""Publication-oriented visual style shared by plots and animations."""

from __future__ import annotations

import matplotlib as mpl


# Okabe--Ito inspired colors. The darker yellow improves contrast on white.
UAV_PALETTE = (
    "#0072B2",
    "#D55E00",
    "#009E73",
    "#CC79A7",
    "#E69F00",
    "#56B4E9",
    "#6F4E7C",
    "#333333",
)

INK = "#202124"
MUTED_INK = "#4B5563"
GRID = "#D1D5DB"
SPINE = "#6B7280"

AIRSPACE_STYLES = {
    "geofence": {
        "color": "#B6403A",
        "alpha": 0.13,
        "hatch": "///",
        "label": "Geofence",
    },
    "obstacle": {
        "color": "#4B5563",
        "alpha": 0.18,
        "hatch": "xx",
        "label": "Obstacle",
    },
    "wind_zone": {
        "color": "#3B82A0",
        "alpha": 0.08,
        "hatch": "..",
        "label": "Wind zone",
    },
}

ACADEMIC_RCPARAMS = {
    "font.family": "serif",
    "font.serif": (
        "STIX Two Text",
        "STIXGeneral",
        "DejaVu Serif",
        "Times New Roman",
    ),
    "mathtext.fontset": "stix",
    "font.size": 10.0,
    "axes.titlesize": 11.0,
    "axes.labelsize": 10.0,
    "axes.labelcolor": INK,
    "axes.edgecolor": SPINE,
    "axes.linewidth": 0.75,
    "xtick.labelsize": 8.5,
    "ytick.labelsize": 8.5,
    "xtick.color": MUTED_INK,
    "ytick.color": MUTED_INK,
    "xtick.direction": "in",
    "ytick.direction": "in",
    "xtick.major.size": 4.0,
    "ytick.major.size": 4.0,
    "xtick.minor.size": 2.0,
    "ytick.minor.size": 2.0,
    "legend.fontsize": 8.0,
    "legend.title_fontsize": 8.5,
    "legend.frameon": True,
    "legend.fancybox": False,
    "legend.framealpha": 0.96,
    "legend.edgecolor": "#9CA3AF",
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "savefig.facecolor": "white",
    "savefig.edgecolor": "white",
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.04,
}


def academic_style_context():
    """Return a local Matplotlib context for publication-ready rendering."""
    return mpl.rc_context(ACADEMIC_RCPARAMS)
