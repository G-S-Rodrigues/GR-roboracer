*** Settings ***
Documentation     Tier 4 acceptance: black-box behaviour of the composed stack.
Library           racing_test_keywords.keywords.RacingTestKeywords

*** Test Cases ***
Vehicle Completes Baseline Lap Safely
    [Documentation]    ACC-4010
    [Tags]    impl:simulator=f1tenth_gym_jax    impl:pose_source=ground_truth
    ...    impl:planner=track_centerline    impl:controller=pure_pursuit
    ...    impl:safety=safety_supervisor
    Run Racing Scenario    seed=42
    Write Metrics Report    ACC-4010
    Lap Completion Should Be    true
    Collision Count Should Be    0
    Minimum Wall Clearance Should Exceed    0.08
    P95 Tracking Error Should Be Below    1.0

Vehicle Stops When Track Limits Violated
    [Documentation]    ACC-4020. Forces the vehicle's perceived position
    ...    off-track (see racing_test_keywords.scenario_runner) and checks
    ...    the supervisor's TRACK_LIMIT clamp stops it inside the complete
    ...    composed graph, not just the isolated node SIM-3010 covers.
    [Tags]    impl:controller=pure_pursuit    impl:safety=safety_supervisor
    Violate Track Limits During Scenario    seed=1
    Safety Clamp Should Include Track Limit
    Commanded Speed Should Be Zero

Localization Estimate Tracks Ground Truth: slam_toolbox
    [Documentation]    ACC-4030. One test case per implementation of the
    ...    pose_source role, sharing one template, so a fourth method adds a
    ...    four-line case and not a new keyword (ADR 0004, ADR 0005, ADR
    ...    0007). The case is its own so it can carry its impl: tag. Open
    ...    loop: the lap is the reference stack's own and the estimator is
    ...    scored beside it, which is the only way an estimator the car
    ...    cannot yet drive on is measurable. The case reports the score
    ...    (log/reports/<ID>-<pose source>.json) and asserts only the
    ...    catastrophic bounds (ADR 0008), on Spielberg - never
    ...    analytic_circle, where position along the centerline is
    ...    unobservable (repo-gotchas #18).
    [Tags]    impl:pose_source=slam_toolbox
    [Template]    Pose Source Should Track Ground Truth
    # test id    pose source
    # Baseline, reported not asserted: slam_toolbox 4.0 from SIM-3070's
    # three runs (2.43-2.88 m).
    ACC-4030     slam_toolbox

Localization Estimate Tracks Ground Truth: amcl
    [Documentation]    ACC-4030. The amcl implementation's case; see the
    ...    slam_toolbox case above for why each is its own test case.
    [Tags]    impl:pose_source=amcl
    [Template]    Pose Source Should Track Ground Truth
    # Baseline, reported not asserted: amcl 0.65, the seed-42 bound
    # (SIM-5030's seed-42 slice, 0.215-0.437 m originally, 0.141-0.356
    # post-3.1b); SIM-5030's own multi-seed thresholds differ.
    ACC-4030     amcl

Localization Estimate Tracks Ground Truth: mola
    [Documentation]    ACC-4510. The MOLA-LO implementation's case; see the
    ...    slam_toolbox case above. It localizes on the map MOLA built itself
    ...    (the data repo's Spielberg mola/ map), so its error includes
    ...    that map's own error, and runs the 1080-beam scenario at 1x.
    [Tags]    impl:pose_source=mola
    [Template]    Pose Source Should Track Ground Truth
    # Baseline, reported not asserted: mola 2.5, SIM-3510's rmse bound (5
    # laps 0.704-2.131 m, its MEASUREMENT).
    ACC-4510     mola    config/scenarios/spielberg_1080.yaml    1.0

Vehicle Completes A Lap Driving On Its Own Estimate
    [Documentation]    ACC-4040. Closed loop on nav2_amcl: the controller
    ...    consumes /localization/odom while the safety supervisor keeps
    ...    reading ground truth. That asymmetry is what makes this test
    ...    honest - if the estimate drifts, the car leaves the track and
    ...    the supervisor latches TRACK_LIMIT, so a failure is a stopped
    ...    car rather than a consistent delusion that agrees with itself.
    [Tags]    impl:pose_source=amcl    impl:controller=pure_pursuit
    ...    impl:safety=safety_supervisor
    Run Racing Scenario On Pose Source    amcl    drive_on_estimate=true
    Write Metrics Report    ACC-4040-amcl
    Lap Completion Should Be    true
    Collision Count Should Be    0

Vehicle Laps The Raceline Driving On Its Own Estimate
    [Documentation]    ACC-4040. The same claim as the case above on the
    ...    min-curvature raceline (planner:=optimized_raceline,
    ...    tum_mincurv_iqp, speed_source:=trajectory), driven on amcl's
    ...    estimate (a demonstration video of it follows separately).
    ...    Nothing else runs an estimator off the centerline. Its only tag is the
    ...    planner's: tags may not overlap between cases of one ID (ADR
    ...    0005), and the amcl, controller and safety tags are the case
    ...    above's. It asserts only a completed lap, 0 collisions and
    ...    clearance above 0.08 m (ACC-4010's bound): a lap is possible on
    ...    this line. Accuracy of the estimate is left to SIM-3080 and
    ...    ACC-4030. No lap-time assert; SIM-3110 owns the line's lap time.
    [Tags]    impl:planner=optimized_raceline
    Run Racing Scenario On Pose Source    amcl    drive_on_estimate=true
    ...    planner=optimized_raceline    raceline_optimizer=tum_mincurv_iqp
    ...    speed_source=trajectory
    Write Metrics Report    ACC-4040-raceline
    Lap Completion Should Be    true
    Collision Count Should Be    0
    Minimum Wall Clearance Should Exceed    0.08

*** Keywords ***
Pose Source Should Track Ground Truth
    [Arguments]    ${test_id}    ${pose_source}
    ...    ${scenario}=config/scenarios/spielberg.yaml    ${time_scale}=5.0
    Run Racing Scenario On Pose Source    ${pose_source}
    ...    scenario=${scenario}    time_scale=${time_scale}
    Write Localization Report    ${test_id}-${pose_source}
    Lap Completion Should Be    true
    Localization Should Stay Within Catastrophic Bounds
