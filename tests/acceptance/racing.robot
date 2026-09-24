*** Settings ***
Documentation     Tier 4 acceptance: black-box behaviour of the composed stack.
Library           racing_test_keywords.keywords.RacingTestKeywords

*** Test Cases ***
Vehicle Completes Baseline Lap Safely
    [Documentation]    ACC-4010
    Run Racing Scenario    seed=42
    Lap Completion Should Be    true
    Collision Count Should Be    0
    Minimum Wall Clearance Should Exceed    0.08
    P95 Tracking Error Should Be Below    1.0

Vehicle Stops When Track Limits Violated
    [Documentation]    ACC-4020. Forces the vehicle's perceived position
    ...    off-track (see racing_test_keywords.scenario_runner) and checks
    ...    the supervisor's TRACK_LIMIT clamp stops it inside the complete
    ...    composed graph, not just the isolated node SIM-3010 covers.
    Violate Track Limits During Scenario    seed=1
    Safety Clamp Should Include Track Limit
    Commanded Speed Should Be Zero

Localization Estimate Tracks Ground Truth
    [Documentation]    ACC-4030. One row per implementation of the
    ...    pose_source role, so a fourth method adds a row and not a test
    ...    case (ADR 0004, ADR 0005). Open loop: the lap is the reference
    ...    stack's own and the estimator is scored beside it, which is the
    ...    only way an estimator the car cannot yet drive on is measurable.
    ...    Each bound is the measurement named beside it, on Spielberg -
    ...    never analytic_circle, where position along the centerline is
    ...    unobservable (repo-gotchas #18).
    [Template]    Pose Source Should Track Ground Truth
    # pose source    ate_rmse below    availability above
    # Each bound is its own tier-3 case's, so a row and its tier-3 case
    # cannot drift apart: slam_toolbox 4.0 from SIM-3070's three runs
    # (2.43-2.88 m), amcl 0.65 from SIM-5030's ten (0.215-0.437 m).
    slam_toolbox     4.0               0.99
    amcl             0.65              0.99

Vehicle Completes A Lap Driving On Its Own Estimate
    [Documentation]    ACC-4040. Closed loop on nav2_amcl: the controller
    ...    consumes /localization/odom while the safety supervisor keeps
    ...    reading ground truth. That asymmetry is what makes this test
    ...    honest - if the estimate drifts, the car leaves the track and
    ...    the supervisor latches TRACK_LIMIT, so a failure is a stopped
    ...    car rather than a consistent delusion that agrees with itself.
    Run Racing Scenario On Pose Source    amcl    drive_on_estimate=true
    Lap Completion Should Be    true
    Collision Count Should Be    0

*** Keywords ***
Pose Source Should Track Ground Truth
    [Arguments]    ${pose_source}    ${error_bound}    ${availability_bound}
    Run Racing Scenario On Pose Source    ${pose_source}
    Lap Completion Should Be    true
    Localization Availability Should Exceed    ${availability_bound}
    Localization Error Should Be Below    ${error_bound}
