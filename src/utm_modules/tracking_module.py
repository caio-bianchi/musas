from __future__ import annotations

from collections import deque

import simpy

from src.algorithms.interfaces.tracking import TrackingAlgorithm
from src.domain.message_features import MessageType
from src.domain.tracking import TrackPrediction, TrackingReport
from src.utm_modules.flight_plan_manager import FlightPlanManager
from src.utm_modules.tactical_deconfliction_module import TacticalDeconflictionModule
from src.core.message import Message

class TrackingModule:
    """Maintain report histories and coordinate trajectory prediction."""

    def __init__(
        self,
        name,
        env,
        network,
        flight_plan_manager: FlightPlanManager,
        algorithm: TrackingAlgorithm,
        history_window: int,
        prediction_horizon: int,
        prediction_step: float,
        tactical_deconfliction_module: TacticalDeconflictionModule = None,
        processing_time: float = 0.0,
    ):
        if history_window <= 0:
            raise ValueError("history_window must be greater than zero")
        if prediction_horizon <= 0:
            raise ValueError("prediction_horizon must be greater than zero")
        if prediction_step <= 0:
            raise ValueError("prediction_step must be greater than zero")
        if processing_time < 0:
            raise ValueError("processing_time cannot be negative")

        self.name = name
        self.env = env
        self.network = network
        self.flight_plan_manager = flight_plan_manager
        self.algorithm = algorithm
        self.history_window = int(history_window)
        self.prediction_horizon = int(prediction_horizon)
        self.prediction_step = float(prediction_step)
        self.processing_time = float(processing_time)
        self.tactical_deconfliction_module = tactical_deconfliction_module

        self.inbox = simpy.Store(env)
        self._report_histories: dict[str, deque[TrackingReport]] = {}
        self._predictions: dict[str, TrackPrediction] = {}

        network.register(self)

    def run(self):
        while True:
            message = yield self.inbox.get()

            if self.processing_time:
                yield self.env.timeout(self.processing_time)

            if message.message_type == MessageType.PERIODIC_REPORT:
                self.process_report(message)

    def process_report(self, message) -> TrackPrediction | None:
        report = TrackingReport(
            uav_name=message.source,
            position=list(message.payload["position"]),
            velocity=list(message.payload["velocity"]),
            timestamp=float(message.issue_timestamp),
            received_at=float(self.env.now),
        )
        self._store_report(report)

        flight_plan = self.flight_plan_manager.get(report.uav_name)
        if flight_plan is None:
            self._predictions.pop(report.uav_name, None)
            return None

        prediction = self.algorithm.predict(
            uav_name=report.uav_name,
            report_history=self.get_report_history(report.uav_name),
            flight_plan=flight_plan,
            prediction_start=float(self.env.now),
            prediction_horizon=self.prediction_horizon,
            prediction_step=self.prediction_step,
        )
        self._predictions[report.uav_name] = prediction
        if self.tactical_deconfliction_module is not None:
            self._publish_prediction(prediction)
        
        return prediction

    def _publish_prediction(self, prediction):
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
        return self._predictions.pop(uav_name, None)
