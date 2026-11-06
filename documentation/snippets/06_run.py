result = simulation.run(until=60.0)
simulation.close()      # flush and close the SQLite dataset

for name, uav in result.uavs.items():
    plan = result.latest_flight_plans[name]
    print(f"{name}: {uav.status.name}, departs at "
          f"t = {plan.waypoint_list[0].t:.1f} s, "
          f"{len(plan.waypoint_list)} waypoints")
