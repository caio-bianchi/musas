SCHEMA_VERSION = 1


SCHEMA_SQL = """
    CREATE TABLE metadata (
        key TEXT PRIMARY KEY,
        value_json TEXT NOT NULL
    );

    CREATE TABLE uavs (
        name TEXT PRIMARY KEY,
        final_status TEXT NOT NULL,
        dynamics_type TEXT NOT NULL,
        guidance_type TEXT NOT NULL,
        sensor_type TEXT NOT NULL
    );

    CREATE TABLE flight_plan_waypoints (
        uav_name TEXT NOT NULL,
        plan_kind TEXT NOT NULL,
        plan_version INTEGER NOT NULL,
        waypoint_index INTEGER NOT NULL,
        x REAL NOT NULL,
        y REAL NOT NULL,
        z REAL NOT NULL,
        timestamp REAL NOT NULL,
        PRIMARY KEY (
            uav_name, plan_kind, plan_version, waypoint_index
        )
    );

    CREATE TABLE trajectory (
        uav_name TEXT NOT NULL,
        timestamp REAL NOT NULL,
        x REAL NOT NULL,
        y REAL NOT NULL,
        z REAL NOT NULL,
        vx REAL NOT NULL,
        vy REAL NOT NULL,
        vz REAL NOT NULL,
        status TEXT NOT NULL,
        PRIMARY KEY (uav_name, timestamp)
    );

    CREATE INDEX trajectory_time_index
        ON trajectory(timestamp);

    CREATE TABLE airspace_objects (
        object_type TEXT NOT NULL,
        object_id TEXT NOT NULL,
        name TEXT,
        geometry_type TEXT NOT NULL,
        geometry_json TEXT NOT NULL,
        active_from REAL,
        active_until REAL,
        properties_json TEXT NOT NULL,
        PRIMARY KEY (object_type, object_id)
    );

    CREATE TABLE airspace_snapshots (
        timestamp REAL PRIMARY KEY,
        geofence_ids_json TEXT NOT NULL,
        wind_zone_ids_json TEXT NOT NULL
    );

    CREATE TABLE tracking_reports (
        uav_name TEXT NOT NULL,
        timestamp REAL NOT NULL,
        received_at REAL NOT NULL,
        x REAL NOT NULL,
        y REAL NOT NULL,
        z REAL NOT NULL,
        vx REAL NOT NULL,
        vy REAL NOT NULL,
        vz REAL NOT NULL,
        PRIMARY KEY (uav_name, timestamp, received_at)
    );

    CREATE TABLE tracking_predictions (
        uav_name TEXT NOT NULL,
        generated_at REAL NOT NULL,
        state_kind TEXT NOT NULL,
        horizon_index INTEGER NOT NULL,
        predicted_timestamp REAL NOT NULL,
        x REAL NOT NULL,
        y REAL NOT NULL,
        z REAL NOT NULL,
        vx REAL NOT NULL,
        vy REAL NOT NULL,
        vz REAL NOT NULL,
        PRIMARY KEY (
            uav_name, generated_at, state_kind, horizon_index
        )
    );

    CREATE TABLE events (
        event_id INTEGER PRIMARY KEY AUTOINCREMENT,
        timestamp REAL NOT NULL,
        component TEXT NOT NULL,
        event TEXT NOT NULL,
        details_json TEXT NOT NULL
    );

    CREATE INDEX events_type_index ON events(event);
    CREATE INDEX events_time_index ON events(timestamp);
"""
