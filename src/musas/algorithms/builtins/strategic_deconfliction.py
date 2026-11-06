from __future__ import annotations

from musas.airspace.airspace import Airspace
from musas.algorithms.builtins._trajectory import shifted_flight_plan
from musas.algorithms.builtins.conflict_prediction import (
    MinimumSeparationConflictPrediction,
)
from musas.algorithms.builtins.conflicts import StrategicAirspaceConflict
from musas.algorithms.builtins.path_planning import DetourPathPlanner
from musas.algorithms.interfaces.conflict_prediction import (
    ConflictPredictionAlgorithm,
)
from musas.algorithms.interfaces.strategic_deconfliction import (
    StrategicDeconflictionAlgorithm,
)
from musas.domain.flight import Conflict, FlightPlan, FlightPlanUpdate
from musas.domain.message_types import StrategicDecision


class RecursiveTimeShiftDeconfliction(StrategicDeconflictionAlgorithm):
    """Resolve airspace conflicts spatially and traffic conflicts temporally.

    Known obstacles and geofences are handled first by ``path_planner``, which
    prioritizes lateral detours that preserve altitude and uses a vertical
    maneuver only as a fallback. The resulting airspace-free route is then
    shifted forward in time until it is separated from every previously
    approved flight plan. In accordance with first-come, first-served
    priority, existing plans are never modified.
    """

    def __init__(
        self,
        conflict_prediction_algorithm: (
            ConflictPredictionAlgorithm | None
        ) = None,
        path_planner: DetourPathPlanner | None = None,
        time_shift_step: float = 5.0,
        maximum_time_shifts: int = 20,
    ) -> None:
        if time_shift_step <= 0:
            raise ValueError("time_shift_step must be greater than zero")
        if maximum_time_shifts < 0:
            raise ValueError("maximum_time_shifts cannot be negative")

        self.conflict_prediction_algorithm = (
            conflict_prediction_algorithm
            if conflict_prediction_algorithm is not None
            else MinimumSeparationConflictPrediction()
        )
        self.path_planner = (
            path_planner
            if path_planner is not None
            else DetourPathPlanner(self.conflict_prediction_algorithm)
        )
        self.time_shift_step = float(time_shift_step)
        self.maximum_time_shifts = int(maximum_time_shifts)

    def deconflict(
        self,
        proposed_flight_plan: FlightPlan,
        conflicts: list[Conflict],
        inactive_flight_plans: dict[str, FlightPlan],
        active_flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> tuple[
        StrategicDecision,
        FlightPlan | None,
        list[FlightPlanUpdate],
    ]:
        if not conflicts:
            return StrategicDecision.APPROVE, None, []

        airspace_free_plan = self.path_planner.plan(
            proposed_flight_plan,
            airspace,
        )
        if airspace_free_plan is None:
            return StrategicDecision.REJECT, None, []

        approved_flight_plans = {
            **inactive_flight_plans,
            **active_flight_plans,
        }
        resolved_plan = self._apply_recursive_time_shift(
            airspace_free_plan,
            approved_flight_plans,
            airspace,
            shift_count=0,
        )
        if resolved_plan is None:
            return StrategicDecision.REJECT, None, []

        return StrategicDecision.COUNTER_PROPOSAL, resolved_plan, []

    def _apply_recursive_time_shift(
        self,
        candidate: FlightPlan,
        approved_flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
        shift_count: int,
    ) -> FlightPlan | None:
        conflicts = self.conflict_prediction_algorithm.predict(
            proposed_flight_plan=candidate,
            approved_flight_plans=approved_flight_plans,
            airspace=airspace,
        )
        if not conflicts:
            return candidate

        if any(
            isinstance(conflict, StrategicAirspaceConflict)
            for conflict in conflicts
        ):
            replanned_candidate = self.path_planner.plan(candidate, airspace)
            if replanned_candidate is None:
                return None
            return self._apply_recursive_time_shift(
                replanned_candidate,
                approved_flight_plans,
                airspace,
                shift_count=shift_count,
            )
        if shift_count >= self.maximum_time_shifts:
            return None

        return self._apply_recursive_time_shift(
            shifted_flight_plan(candidate, self.time_shift_step),
            approved_flight_plans,
            airspace,
            shift_count=shift_count + 1,
        )
