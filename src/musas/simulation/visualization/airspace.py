import math

from matplotlib.patches import Circle, Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

from musas.simulation.dataset import AirspaceObjectRecord
from musas.simulation.visualization.config import (
    View,
    VisualizationConfig,
    projection_indices,
)
from musas.simulation.visualization.data import (
    VisualizationData,
    active_object_ids,
    object_is_active,
)


def draw_airspace(
    axis,
    data: VisualizationData,
    config: VisualizationConfig,
    timestamp: float,
) -> None:
    active_geofences = active_object_ids(data, "geofence", timestamp)
    active_wind_zones = active_object_ids(data, "wind_zone", timestamp)

    for item in data.airspace_objects:
        if item.object_type == "geofence":
            if (
                config.show_geofences
                and item.object_id in active_geofences
                and object_is_active(item, timestamp)
            ):
                _draw_region(axis, config.view, item, "crimson", 0.20)
        elif item.object_type == "obstacle":
            if config.show_obstacles:
                _draw_region(axis, config.view, item, "dimgray", 0.28)
        elif (
            item.object_type == "wind_zone"
            and config.show_wind_zones
            and item.object_id in active_wind_zones
            and object_is_active(item, timestamp)
        ):
            _draw_region(axis, config.view, item, "deepskyblue", 0.16)
            _draw_wind_vector(axis, config.view, item)


def geometry_extremes(item: AirspaceObjectRecord) -> list[tuple]:
    geometry = item.geometry
    geometry_type = item.geometry_type.rsplit(".", 1)[-1]
    if geometry_type == "Cylinder3D":
        radius = geometry["radius"]
        return [
            (
                geometry["center_x"] - radius,
                geometry["center_y"] - radius,
                geometry["minimum_z"],
            ),
            (
                geometry["center_x"] + radius,
                geometry["center_y"] + radius,
                geometry["maximum_z"],
            ),
        ]
    if geometry_type == "Box3D":
        return [
            (
                geometry["minimum_x"],
                geometry["minimum_y"],
                geometry["minimum_z"],
            ),
            (
                geometry["maximum_x"],
                geometry["maximum_y"],
                geometry["maximum_z"],
            ),
        ]
    return []


def _draw_region(axis, view, item, color, alpha) -> None:
    geometry = item.geometry
    geometry_type = item.geometry_type.rsplit(".", 1)[-1]
    if geometry_type == "Cylinder3D":
        _draw_cylinder(axis, view, geometry, color, alpha)
    elif geometry_type == "Box3D":
        _draw_box(axis, view, geometry, color, alpha)


def _draw_cylinder(axis, view, geometry, color, alpha) -> None:
    center_x = geometry["center_x"]
    center_y = geometry["center_y"]
    radius = geometry["radius"]
    minimum_z = geometry["minimum_z"]
    maximum_z = geometry["maximum_z"]
    if view == "3d":
        angles = np.linspace(0.0, 2.0 * np.pi, 30)
        heights = np.array([minimum_z, maximum_z])
        angles, heights = np.meshgrid(angles, heights)
        x = center_x + radius * np.cos(angles)
        y = center_y + radius * np.sin(angles)
        axis.plot_surface(
            x,
            y,
            heights,
            color=color,
            alpha=alpha,
            linewidth=0,
        )
        for height in (minimum_z, maximum_z):
            axis.plot(
                center_x + radius * np.cos(angles[0]),
                center_y + radius * np.sin(angles[0]),
                height,
                color=color,
                alpha=min(1.0, alpha + 0.25),
            )
    elif view == "xy":
        axis.add_patch(
            Circle((center_x, center_y), radius, color=color, alpha=alpha)
        )
    else:
        horizontal_center = center_x if view == "xz" else center_y
        axis.add_patch(Rectangle(
            (horizontal_center - radius, minimum_z),
            2.0 * radius,
            maximum_z - minimum_z,
            color=color,
            alpha=alpha,
        ))


def _draw_box(axis, view, geometry, color, alpha) -> None:
    x0, x1 = geometry["minimum_x"], geometry["maximum_x"]
    y0, y1 = geometry["minimum_y"], geometry["maximum_y"]
    z0, z1 = geometry["minimum_z"], geometry["maximum_z"]
    if view == "3d":
        vertices = [
            (x0, y0, z0),
            (x1, y0, z0),
            (x1, y1, z0),
            (x0, y1, z0),
            (x0, y0, z1),
            (x1, y0, z1),
            (x1, y1, z1),
            (x0, y1, z1),
        ]
        faces = [
            [vertices[index] for index in face]
            for face in (
                (0, 1, 2, 3),
                (4, 5, 6, 7),
                (0, 1, 5, 4),
                (2, 3, 7, 6),
                (1, 2, 6, 5),
                (0, 3, 7, 4),
            )
        ]
        axis.add_collection3d(Poly3DCollection(
            faces,
            facecolors=color,
            edgecolors=color,
            alpha=alpha,
        ))
    else:
        bounds = {
            "xy": (x0, x1, y0, y1),
            "xz": (x0, x1, z0, z1),
            "yz": (y0, y1, z0, z1),
        }[view]
        axis.add_patch(Rectangle(
            (bounds[0], bounds[2]),
            bounds[1] - bounds[0],
            bounds[3] - bounds[2],
            color=color,
            alpha=alpha,
        ))


def _draw_wind_vector(
    axis,
    view: View,
    item: AirspaceObjectRecord,
) -> None:
    velocity = item.properties.get("velocity")
    if not velocity or math.isclose(np.linalg.norm(velocity), 0.0):
        return
    center = _geometry_center(item)
    if center is None:
        return
    if view == "3d":
        axis.quiver(*center, *velocity, color="dodgerblue", length=3.0)
    else:
        first, second = projection_indices(view)
        axis.quiver(
            center[first],
            center[second],
            velocity[first],
            velocity[second],
            color="dodgerblue",
            angles="xy",
            scale_units="xy",
            scale=0.25,
        )


def _geometry_center(item: AirspaceObjectRecord):
    geometry = item.geometry
    geometry_type = item.geometry_type.rsplit(".", 1)[-1]
    if geometry_type == "Cylinder3D":
        return (
            geometry["center_x"],
            geometry["center_y"],
            (geometry["minimum_z"] + geometry["maximum_z"]) / 2.0,
        )
    if geometry_type == "Box3D":
        return (
            (geometry["minimum_x"] + geometry["maximum_x"]) / 2.0,
            (geometry["minimum_y"] + geometry["maximum_y"]) / 2.0,
            (geometry["minimum_z"] + geometry["maximum_z"]) / 2.0,
        )
    return None
