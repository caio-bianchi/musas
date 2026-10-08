from dataclasses import asdict, is_dataclass
from enum import Enum
import json
from pathlib import Path
from typing import Any
from musas.airspace import Geometry

def json_dumps(value: Any) -> str:
    """
    Converts any python value in JSON string.
    """
    return json.dumps(
        value,
        default=_json_default,
        ensure_ascii=False,
        sort_keys=True,
    )

def _json_default(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return asdict(value)
    if hasattr(value, "__dict__"):
        return {
            key: item
            for key, item in vars(value).items()
            if not key.startswith("_")
        }
    return repr(value)

def geometry_data(geometry: Geometry) -> tuple[str, dict[str, Any]]:
        """
        Return the fully qualified type name and attributes of a geometry.

        All concrete Geometry implementations are expected to be dataclasses.
        """
        geometry_type = type(geometry)
        type_name = (
            f"{geometry_type.__module__}."
            f"{geometry_type.__qualname__}"
        )

        return type_name, asdict(geometry)

