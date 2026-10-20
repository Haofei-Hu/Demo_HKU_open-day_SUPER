# Final reference completion after a missing terminal sample

## Recorded failure

The physical mission `20261020_150021_384581` took off to the configured 1 m
floor-height target and started its first 4.5 m outbound leg. The final accepted
command was already at the committed endpoint, with speed approximately
0.000001 m/s and acceleration 0.000256 m/s². Its timestamp was 5.934 ms before
the end of the polynomial, so the old completion rule still classified it as
tracking. The newest-command queue policy can supersede the terminal command
when the planner's restart arrives before the next supervisor cycle; a dropped
terminal command exposes the same dependency. The recorded failure alone does
not distinguish those two delivery cases.

SUPER then finished that trajectory. Its measured position was outside its
goal-arrival tolerance, so it generated another `PlanFromRest` towards the same
goal. The new initial position matched the logged raw localization position.
The complete supervisor rejected a 0.3996127309 m reference step over 50.323 ms
and stopped. No outbound arrival or return was recorded.

The report's peak FC-feedback/reference differences were 0.7697 m forward,
0.4011 m lateral and 0.3043 m vertical. These are measured localization/reference
differences, not independent proof of physical displacement. Their cause and
the physical vehicle's tracking performance are not corrected or validated by
the terminal-reference software change.

## Correction

Once the committed polynomial ends, the supervisor may hold its last verified
final reference without requiring another terminal command. This requires the
existing endpoint and stationary-derivative checks, confirmed final-goal
feedback, and a conservative bound on the remaining polynomial displacement
within the existing 15 mm matching tolerance. It retains the original received
stamp and publishes new hold commands with zero velocity feedforward.

Reference completion and aircraft arrival remain separate. A vehicle still
offset from the target receives the verified hold position; the next leg is
issued only after current FC position and velocity satisfy the arrival check.
An intermediate stop or unfinished/moving curve does not qualify. Continuous
checks still reject a discontinuous continuation that is actually to be tracked.
Fresh localization, source/configuration identities, venue bounds, and pilot
takeover checks remain active.

This is a Python supervisor update. A system already built with all ten venue
markers, including `VENUE_BACKUP_WAYPOINTS_V1`, does not need a C++ rebuild.

## Verification

The regression fixture preserves both polynomial coefficient arrays and both
command timestamps from the physical report. With terminal completion disabled,
the existing processing reproduces the recorded 0.3996127309 m rejection,
both with an unavailable terminal command and with a queued terminal command
superseded by the newest-command policy.
With the correction, it retains the prior endpoint and original timestamp.
The measured-position test verifies that a 40 cm vehicle offset cannot advance
the mission; confirmed arrival is still required before the return goal.

Additional tests cover the planned end not yet reached, missing final-goal
feedback, intermediate endpoints, moving terminal states, an unobserved loop
remaining in a polynomial, and existing stationary-handoff and jump rejection.
The full regression suite contains 89 passing checks. Cloud integration used
real SUPER and a frozen occupied map with ideal virtual feedback, deliberately
dropping commands near final endpoints on all ten legs. It completed five round
trips, all 20 stages, the final six-second hold and virtual landing/disarming.
See the [machine-readable verification record](final_reference_completion.json).

The cloud test does not use aircraft actuators or PX4 dynamics. It validates
the reference-completion transition and mission sequencing, not physical
tracking accuracy or complete physical mission success.
