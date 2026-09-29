from pathlib import Path
from typing import Any

import simpy


class SimulationLogger:
    """Write simple, chronological simulation events to a text file."""

    def __init__(self,env: simpy.Environment,file_path: str | Path) -> None:
        self.env = env
        self.file_path = Path(file_path)
        self.file_path.parent.mkdir(parents=True, exist_ok=True)
        self.file_path.write_text("", encoding="utf-8")

    def log(self,component: str,event: str,**details: Any) -> None:
        detail_text = ", ".join(
            f"{name}={value}"
            for name, value in details.items()
        )
        line = (
            f"[t={float(self.env.now):8.3f}] "
            f"[{component}] {event}"
        )
        if detail_text:
            line += f" | {detail_text}"

        with self.file_path.open("a", encoding="utf-8") as log_file:
            log_file.write(line + "\n")
