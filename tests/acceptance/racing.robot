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
