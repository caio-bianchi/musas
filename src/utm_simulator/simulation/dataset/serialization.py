from dataclasses import asdict, is_dataclass
from enum import Enum
import json
from pathlib import Path
from typing import Any


def json_dumps(value: Any) -> str:
    return json.dumps(
        value,
        default=_json_default,
        ensure_ascii=False,
        sort_keys=True,
    )


def geometry_data(geometry: Any) -> tuple[str, dict[str, Any]]:
    geometry_type = type(geometry)
    type_name = f"{geometry_type.__module__}.{geometry_type.__qualname__}"
    if is_dataclass(geometry):
        return type_name, asdict(geometry)

    try:
        values = {
            key: value
            for key, value in vars(geometry).items()
            if not key.startswith("_")
        }
    except TypeError:
        values = {"representation": repr(geometry)}
    return type_name, values


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
