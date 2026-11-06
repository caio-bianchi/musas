strategic_predictor = MinimumSeparationConflictPrediction(
    minimum_separation_distance=30.0,
)
strategic_deconfliction = RecursiveTimeShiftDeconfliction(
    conflict_prediction_algorithm=strategic_predictor,
    path_planner=DetourPathPlanner(lateral_clearance=15.0),
    time_shift_step=3.0,
)
