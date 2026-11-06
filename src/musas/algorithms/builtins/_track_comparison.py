"""Helpers for comparing two predicted tracks state by state.

Tactical conflict prediction and tactical deconfliction both compare tracks in
the same way: every state of one track is paired with the state of the other
track that is closest in time, and the pair is used only if the time
difference does not exceed a tolerance.

Two optimizations are shared here:

* **Nearest-in-time lookup with binary search.** A track's states are ordered
  by time, so the closest state is found with :func:`bisect.bisect_left` in
  ``O(log n)`` instead of scanning every state.
* **Bounding-box pre-filter.** Every predicted position of a track lies inside
  the track's axis-aligned bounding box. Hence, the distance between the
  bounding boxes of two tracks is a lower bound for every separation between
  them. When that lower bound already exceeds the distance of interest, the
  state-by-state comparison can be skipped without changing any result.
"""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
from math import dist, sqrt

from musas.domain.tracking import PredictedState, TrackPrediction


@dataclass(frozen=True)
class IndexedTrack:
    """States of one track prepared for repeated comparisons.

    ``states`` holds the estimated state followed by the predicted states,
    sorted by time. ``minimum_corner`` and ``maximum_corner`` delimit the
    bounding box that contains every state position.
    """

    states: tuple[PredictedState, ...]
    timestamps: tuple[float, ...]
    minimum_corner: tuple[float, ...]
    maximum_corner: tuple[float, ...]

    @classmethod
    def from_track(cls, track: TrackPrediction) -> IndexedTrack:
        return cls.from_states(
            (track.estimated_state, *track.predicted_states)
        )

    @classmethod
    def from_states(
        cls,
        states: tuple[PredictedState, ...],
    ) -> IndexedTrack:
        # A stable sort keeps the original order of states that share a
        # timestamp, so ties are resolved exactly as a linear scan would.
        ordered = tuple(sorted(states, key=lambda state: state.timestamp))
        positions = [state.position for state in ordered]
        return cls(
            states=ordered,
            timestamps=tuple(float(state.timestamp) for state in ordered),
            minimum_corner=tuple(
                min(float(position[axis]) for position in positions)
                for axis in range(3)
            ),
            maximum_corner=tuple(
                max(float(position[axis]) for position in positions)
                for axis in range(3)
            ),
        )

    def nearest_state(self, timestamp: float) -> PredictedState:
        """Return the state closest in time to ``timestamp``.

        When two states are equally close, the earlier one is returned,
        matching ``min(states, key=lambda s: abs(s.timestamp - timestamp))``.
        """
        index = bisect_left(self.timestamps, timestamp)
        if index == 0:
            return self.states[0]
        if index == len(self.states):
            return self.states[-1]
        before = self.states[index - 1]
        after = self.states[index]
        if abs(before.timestamp - timestamp) <= abs(after.timestamp - timestamp):
            return before
        return after


def bounding_box_gap(first: IndexedTrack, second: IndexedTrack) -> float:
    """Return a lower bound for every separation between two tracks.

    The value is the Euclidean distance between the two bounding boxes, and is
    zero when the boxes overlap.
    """
    squared_gap = 0.0
    for first_min, first_max, second_min, second_max in zip(
        first.minimum_corner,
        first.maximum_corner,
        second.minimum_corner,
        second.maximum_corner,
    ):
        axis_gap = max(0.0, second_min - first_max, first_min - second_max)
        squared_gap += axis_gap * axis_gap
    return sqrt(squared_gap)


def aligned_separations(
    first: IndexedTrack,
    second: IndexedTrack,
    maximum_time_difference: float,
) -> list[float]:
    """Return the separation at each state of ``first``, in time order.

    Each state of ``first`` is paired with the closest-in-time state of
    ``second``. States without a partner within ``maximum_time_difference``
    are omitted.
    """
    separations: list[float] = []
    for state in first.states:
        other = second.nearest_state(state.timestamp)
        if abs(other.timestamp - state.timestamp) <= maximum_time_difference:
            separations.append(dist(state.position, other.position))
    return separations
