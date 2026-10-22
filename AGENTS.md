# Development

- Keep application version 1.0.0 and upstream pins intact.
- Use C++17, Google formatting, RAII and explicit errors for the host bridge.
- Keep scene IDs, spawn poses, target assets and route parameters in scenario profiles. Share transport, video, clock and PX4 lifecycle code across scenarios; do not fork the bridge for each scene.
- Keep scenario-specific target selection and acceptance criteria out of shared flight-session safety checks. Adding a profile does not mean its flight behavior has been validated.
- Keep simulation fixtures separate from follower inputs; never drive the aircraft from a scripted camera pose.
- For car-following traffic demos, keep one explicit escort behind the selected car throughout the run. Validate its rear position and vehicle separation; other traffic may shuffle around the pair.
- SITL tests target only system ID 2 under `/simulation` and require recent disarmed M7 telemetry. M7 HITL flight testing is explicitly authorized only through its separate opt-in harness, which must verify system ID 1, SYS_HITL=1, active HIL state and simulated-only actuator output before arming. Never reuse the SITL harness to command M7 or run physical motor drivers.
- Preserve normal arming checks, timestamp checks, selected-identity behavior and the latched hold on target loss.
- Report skipped tests and failed sustained-following tests accurately. Do not lower acceptance criteria to obtain a pass.
- Keep local paths, addresses, credentials, SDKs, assets, binaries and recordings out of Git.
- Do not call AirSim GetImages while the continuous camera bridge is subscribed: the one-shot probe interrupts camera publication. Review recorded frames instead.
