# External components

- [ProjectAirSim v1.0.1](https://github.com/iamaisim/ProjectAirSim/tree/v1.0.1): Microsoft/IAMAI C++ client and robot templates, MIT. The installer obtains these from the pinned upstream checkout and preserves its license files.
- [PX4](https://github.com/Swamy-BV/PX4-Autopilot): separately built firmware-matched SITL and Micro XRCE-DDS client libraries. Upstream BSD-3-Clause and dependency notices remain in that checkout.
- GStreamer, x264, LLVM and other system packages are installed separately and retain their respective licenses.
- The Neighborhood Unreal environment is downloaded from the ProjectAirSim release, verified and kept outside version control. Refer to the release's notices for its assets and runtime.

No upstream simulator binary, scene asset, SDK source or firmware is vendored in this repository.
