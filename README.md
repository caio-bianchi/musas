# MUSAS — Modular UTM Simulation and Assessment System

MUSAS is a Python framework for simulating UAS Traffic Management (UTM) and
assessing **strategic** and **tactical deconfliction** strategies. Built on the
SimPy discrete-event engine, it keeps every UTM function as an independent,
replaceable algorithm: you describe the traffic, the airspace, and the
algorithms; MUSAS runs the operations, exchanges the messages, records the run
in a SQLite dataset, and computes metrics, figures, and animations from it.

- **Replaceable algorithms** — tracking, conflict prediction, strategic and
  tactical deconfliction, guidance, dynamics, and sensors are plugged in
  through explicit contracts.
- **Non-nominal conditions** — wind zones, obstacles, geofences activated
  during the run, and network and processing delays.
- **Persistent datasets** — plans, trajectories, reports, tracks, airspace
  changes, and module events are stored for every run.
- **Assessment** — safety, operational, and efficiency metrics,
  publication-ready plots, and animations.

## Installation

Requires Python 3.11 or newer.

```bash
pip install "git+https://github.com/caio-bianchi/musas.git"
```

To reproduce the experiments with the exact library versions used in the
thesis, clone the repository and install `requirements.txt` from its root:

```bash
git clone https://github.com/caio-bianchi/musas.git
cd musas
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## Quick example

Two UASs would cross at the origin; the strategic layer delays the second one.

```python
from musas.airspace import Airspace
from musas.algorithms import (
    FirstOrderDynamics,
    MinimumSeparationConflictPrediction,
    RecursiveTimeShiftDeconfliction,
    WaypointGuidance,
)
from musas.domain import FlightPlan, Waypoint
from musas.simulation import Simulation, SimulationDataset, calculate_metrics

predictor = MinimumSeparationConflictPrediction(minimum_separation_distance=30.0)
simulation = Simulation(
    airspace=Airspace(),
    strategic_conflict_prediction_algorithm=predictor,
    strategic_deconfliction_algorithm=RecursiveTimeShiftDeconfliction(predictor),
    dataset_path="quickstart.sqlite",
    overwrite_dataset=True,
)

# Waypoint(x, y, z, t): metres and seconds
plans = {
    "UAV A": FlightPlan([Waypoint(-100, 0, 100, 2), Waypoint(100, 0, 100, 22)]),
    "UAV B": FlightPlan([Waypoint(0, -100, 100, 2), Waypoint(0, 100, 100, 22)]),
}
for name, plan in plans.items():
    simulation.add_uav(name, plan, FirstOrderDynamics(), WaypointGuidance(max_speed=15.0))

result = simulation.run(until=60.0)
simulation.close()

with SimulationDataset("quickstart.sqlite") as dataset:
    metrics = calculate_metrics(dataset, minimum_separation_distance=30.0,
                                collision_distance=1.0)
print(metrics["loss_of_separation_events"], metrics["minimum_pairwise_separation"])
```

## Documentation

| | |
|---|---|
| [`documentation/musas_documentation.pdf`](documentation/musas_documentation.pdf) | User guide and API reference: installation, quick start, simulation guide, every public class and function, writing your own algorithms |
| [`documentation/getting_started.ipynb`](documentation/getting_started.ipynb) | Executed notebook with every example of the quick start |

## Repository layout

| Path | Content |
|---|---|
| `src/musas/` | The package: `simulation`, `airspace`, `domain`, `algorithms` (contracts and built-ins), `uav`, `utm`, `core` |
| `experiments/` | Notebooks of the thesis experiments: seven simple validation scenarios (1–7) and paired Monte Carlo load tests (8–11) |
| `documentation/` | LaTeX source and PDF of the documentation, the getting-started notebook, and its code snippets |
| `requirements.txt` | Environment used to produce the reported results |

The built-in algorithms are simplified reference implementations meant to
exercise the framework and serve as baselines; they are not intended as
definitive UTM solutions.

## Context

MUSAS was developed as the undergraduate thesis *A Modular Simulation
Framework for Assessing Deconfliction Strategies in the UTM Landscape*, by
Caio Bianchi, advised by Prof. Juliana de Melo Bezerra, at the Instituto
Tecnológico de Aeronáutica (ITA).
