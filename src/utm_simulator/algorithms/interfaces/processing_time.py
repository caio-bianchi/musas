from abc import ABC, abstractmethod


class ProcessingTimeModel(ABC):
    """Contract for context-dependent simulated processing delays."""

    @abstractmethod
    def calculate(self, **context: object) -> float:
        """Return the processing delay for the supplied context."""
        ...
