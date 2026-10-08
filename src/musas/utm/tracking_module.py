from __future__ import annotations

from collections import deque
from collections.abc import Callable

import simpy

from musas.algorithms.builtins.constant_time_processing import (
    ConstantTimeProcessing,
)
from musas.algorithms.interfaces.tracking import TrackingAlgorithm
from musas.algorithms.interfaces.processing_time import (
    ProcessingTimeModel,
)
from musas.core.message import Message
from musas.core.network import CommunicationNetwork
from musas.domain.message_types import MessageType
from musas.domain.tracking import TrackPrediction, TrackingReport
from musas.utm.flight_plan_manager import FlightPlanManager
from musas.utm.tactical_deconfliction_module import TacticalDeconflictionModule


class TrackingModule:
    """Maintain report histories and coordinate trajectory prediction."""

    def __init__(
        self,
        name: str,
        env: simpy.Environment,
        network: CommunicationNetwork,
        flight_plan_manager: FlightPlanManager,
        algorithm: TrackingAlgorithm,
        history_window: int,
        prediction_horizon: int,
        prediction_step: float,
        tactical_deconfliction_module: TacticalDeconflictionModule | None = None,
        processing_time: ProcessingTimeModel = ConstantTimeProcessing(0),
        # functions that are called by simulation when report is observed or prediction is computed
        # if they are None, tracking keeps working, but these data are not sent to Simulation.
        report_observer: Callable[[TrackingReport], None] | None = None,
        prediction_observer: Callable[[TrackPrediction], None] | None = None,
    ) -> None:
        if history_window <= 0:
            raise ValueError("history_window must be greater than zero")
        if prediction_horizon <= 0:
            raise ValueError("prediction_horizon must be greater than zero")
        if prediction_step <= 0:
            raise ValueError("prediction_step must be greater than zero")
        self.name = name
        self.env = env
        self.network = network
        self.flight_plan_manager = flight_plan_manager
        self.algorithm = algorithm
        self.history_window = int(history_window)
        self.prediction_horizon = int(prediction_horizon)
        self.prediction_step = float(prediction_step)
        self.processing_time = processing_time
        self.tactical_deconfliction_module = tactical_deconfliction_module
        self.report_observer = report_observer
        self.prediction_observer = prediction_observer

        self.inbox = simpy.Store(env)
        self._report_histories: dict[str, deque[TrackingReport]] = {}
        self._predictions: dict[str, TrackPrediction] = {}
        self._completed_uavs: set[str] = set()

        network.register(self)
        flight_plan_manager.add_completion_observer(self.complete_uav)

    def run(self):
        while True:
            message = yield self.inbox.get()

            if message.message_type == MessageType.PERIODIC_REPORT:
                yield self.env.process(self.process_report(message))

    def _processing_context(
        self,
        message: Message,
        operation: str,
    ) -> dict[str, object]:
        history = self._report_histories.get(message.source, ())
        return {
            "module": self.name,
            "operation": operation,
            "message": message,
            "uav_name": message.source,
            "queue_length": len(self.inbox.items),
            "history_size": len(history),
            "track_count": len(self._predictions),
            "prediction_horizon": self.prediction_horizon,
            "prediction_step": self.prediction_step,
        }

    def process_report(
        self,
        message: Message,
    ):
        report = TrackingReport(
            uav_name=message.source,
            position=list(message.payload["position"]),
            velocity=list(message.payload["velocity"]),
            timestamp=float(message.issue_timestamp),
            received_at=float(self.env.now),
        )
        if report.uav_name in self._completed_uavs:
            return None
        self._store_report(report)
        if self.report_observer is not None:
            self.report_observer(report)

        flight_plan = self.flight_plan_manager.get(report.uav_name)
        if flight_plan is None:
            self._predictions.pop(report.uav_name, None)
            return None

        delay = self.processing_time.calculate(
            **self._processing_context(message, "tracking_prediction")
        )
        if delay < 0:
            raise ValueError("processing time cannot be negative")
        if delay:
            yield self.env.timeout(delay)

        prediction = self.algorithm.predict(
            uav_name=report.uav_name,
            report_history=self.get_report_history(report.uav_name),
            flight_plan=flight_plan,
            prediction_start=float(self.env.now),
            prediction_horizon=self.prediction_horizon,
            prediction_step=self.prediction_step,
        )
        self._predictions[report.uav_name] = prediction
        if self.prediction_observer is not None:
            self.prediction_observer(prediction)
        if self.tactical_deconfliction_module is not None:
            self._publish_prediction(prediction)

        return prediction

    def _publish_prediction(
        self,
        prediction: TrackPrediction,
    ) -> None:
        message = Message(
            source=self.name,
            destination=self.tactical_deconfliction_module.name,
            message_type=MessageType.TRACK_UPDATE,
            payload={
                "track_prediction": prediction,
            },
            issue_timestamp=self.env.now,
        )

        self.network.send(message)

    def _store_report(self, report: TrackingReport) -> None:
        history = self._report_histories.get(report.uav_name, ())
        ordered_reports = sorted(
            (*history, report),
            key=lambda stored_report: stored_report.timestamp,
        )
        self._report_histories[report.uav_name] = deque(
            ordered_reports[-self.history_window :],
            maxlen=self.history_window,
        )

    def get_report_history(self, uav_name: str) -> tuple[TrackingReport, ...]:
        return tuple(self._report_histories.get(uav_name, ()))

    def get_prediction(self, uav_name: str) -> TrackPrediction | None:
        return self._predictions.get(uav_name)

    def get_all_report_histories(self) -> dict[str, tuple[TrackingReport, ...]]:
        return {
            uav_name: tuple(history)
            for uav_name, history in self._report_histories.items()
        }

    def get_all_predictions(self) -> dict[str, TrackPrediction]:
        return self._predictions.copy()

    def remove_uav(self, uav_name: str) -> TrackPrediction | None:
        self._report_histories.pop(uav_name, None)
        removed = self._predictions.pop(uav_name, None)
        if self.tactical_deconfliction_module is not None:
            self.tactical_deconfliction_module.remove_uav(uav_name)
        return removed

    def complete_uav(self, uav_name: str) -> TrackPrediction | None:
        """Permanently stop tracking a completed UAV."""
        self._completed_uavs.add(uav_name)
        self._report_histories.pop(uav_name, None)
        removed = self._predictions.pop(uav_name, None)
        if self.tactical_deconfliction_module is not None:
            self.tactical_deconfliction_module.complete_uav(uav_name)
        return removed
