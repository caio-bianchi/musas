from __future__ import annotations

from musas.airspace.airspace import Airspace
from musas.algorithms.builtins._trajectory import shifted_flight_plan
from musas.algorithms.builtins.conflict_prediction import (
    MinimumSeparationConflictPrediction,
)
from musas.algorithms.builtins.conflicts import (
    StrategicAirspaceConflict,
    StrategicTrafficConflict,
)
from musas.algorithms.builtins.lateral_detour_deconfliction import (
    LateralDetourDeconfliction,
)
from musas.algorithms.interfaces.conflict_prediction import (
    ConflictPredictionAlgorithm,
)
from musas.algorithms.interfaces.strategic_deconfliction import (
    StrategicDeconflictionAlgorithm,
)
from musas.domain.flight import Conflict, FlightPlan, FlightPlanUpdate
from musas.domain.message_types import StrategicDecision


class RecursiveTimeShiftDeconfliction(StrategicDeconflictionAlgorithm):
    """Resolve traffic temporally and known airspace conflicts laterally.

    Known obstacle and geofence conflicts are first delegated to a
    ``LateralDetourDeconfliction``. Each remaining traffic-conflict iteration
    shifts every waypoint by ``time_shift_step``. The first fully
    conflict-free candidate is returned as a counterproposal.
    """

    def __init__(
        self,
        conflict_prediction_algorithm: ConflictPredictionAlgorithm | None = (
            None
        ),
        time_shift_step: float = 5.0,
        maximum_iterations: int = 20,
        airspace_deconfliction_algorithm: (
            LateralDetourDeconfliction | None
        ) = None,
    ) -> None:
        if time_shift_step <= 0:
            raise ValueError("time_shift_step must be greater than zero")
        if maximum_iterations <= 0:
            raise ValueError("maximum_iterations must be greater than zero")
        if (
            conflict_prediction_algorithm is not None
            and not isinstance(
                conflict_prediction_algorithm,
                ConflictPredictionAlgorithm,
            )
        ):
            raise TypeError(
                "conflict_prediction_algorithm must inherit from "
                "ConflictPredictionAlgorithm"
            )
        self.conflict_prediction_algorithm = (
            conflict_prediction_algorithm
            if conflict_prediction_algorithm is not None
            else MinimumSeparationConflictPrediction()
        )
        self.time_shift_step = float(time_shift_step)
        self.maximum_iterations = int(maximum_iterations)
        if (
            airspace_deconfliction_algorithm is not None
            and not isinstance(
                airspace_deconfliction_algorithm,
                LateralDetourDeconfliction,
            )
        ):
            raise TypeError(
                "airspace_deconfliction_algorithm must be a "
                "LateralDetourDeconfliction"
            )
        self.airspace_deconfliction_algorithm = (
            airspace_deconfliction_algorithm
            if airspace_deconfliction_algorithm is not None
            else LateralDetourDeconfliction(
                self.conflict_prediction_algorithm
            )
        )

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

        approved_flight_plans = {
            **inactive_flight_plans,
            **active_flight_plans,
        }
        candidate = proposed_flight_plan
        if any(
            isinstance(conflict, StrategicAirspaceConflict)
            for conflict in conflicts
        ):
            candidate = (
                self.airspace_deconfliction_algorithm
                .find_airspace_free_plan(
                    proposed_flight_plan,
                    approved_flight_plans,
                    airspace,
                )
            )
            if candidate is None:
                return StrategicDecision.REJECT, None, []

        remaining_conflicts = self.conflict_prediction_algorithm.predict(
            proposed_flight_plan=candidate,
            approved_flight_plans=approved_flight_plans,
            airspace=airspace,
        )
        if not remaining_conflicts:
            return StrategicDecision.COUNTER_PROPOSAL, candidate, []
        if any(
            not isinstance(conflict, StrategicTrafficConflict)
            for conflict in remaining_conflicts
        ):
            return StrategicDecision.REJECT, None, []

        for iteration in range(1, self.maximum_iterations + 1):
            shifted_candidate = shifted_flight_plan(
                candidate,
                iteration * self.time_shift_step,
            )
            candidate_conflicts = (
                self.conflict_prediction_algorithm.predict(
                    proposed_flight_plan=shifted_candidate,
                    approved_flight_plans=approved_flight_plans,
                    airspace=airspace,
                )
            )
            if not candidate_conflicts:
                return (
                    StrategicDecision.COUNTER_PROPOSAL,
                    shifted_candidate,
                    [],
                )

        return StrategicDecision.REJECT, None, []
