from pathlib import Path

from musas.airspace import Airspace, Cylinder3D, Obstacle
from musas.algorithms import (
    DetourPathPlanner,
    FirstOrderDynamics,
    MinimumSeparationConflictPrediction,
    PerfectSensor,
    RecursiveTimeShiftDeconfliction,
    WaypointGuidance,
)
from musas.domain import FlightPlan, Waypoint
from musas.simulation import (
    Simulation,
    SimulationDataset,
    VisualizationConfig,
    calculate_metrics,
    plot_dataset,
)

OUTPUTS = Path("outputs")      # SQLite datasets written by the runs
FIGURES = Path("figures")      # figures used in the PDF documentation
OUTPUTS.mkdir(exist_ok=True)
FIGURES.mkdir(exist_ok=True)
