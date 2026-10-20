# Development

- Keep application version 1.0.0 and upstream pins intact.
- Use C++17, Google formatting, RAII and explicit errors for the host bridge.
- Keep simulation fixtures separate from follower inputs; never drive the aircraft from a scripted camera pose.
- Flight tests target only SITL system ID 2 under `/simulation`. Require recent disarmed M7 telemetry; never arm or command the real M7.
- Preserve normal arming checks, timestamp checks, selected-identity behavior and the latched hold on target loss.
- Report skipped tests and failed sustained-following tests accurately. Do not lower acceptance criteria to obtain a pass.
- Keep local paths, addresses, credentials, SDKs, assets, binaries and recordings out of Git.
