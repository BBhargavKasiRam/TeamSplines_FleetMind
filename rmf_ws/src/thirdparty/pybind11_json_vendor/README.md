> [!WARNING]
> [A packaging problem upstream in Ubuntu Resolute](https://openrobotics.zulipchat.com/#narrow/channel/526042-Infrastructure-General/topic/Conflicting.20version.20of.20nlohmann-json.20in.20Resolute/with/593414087) has made it necessary to un-archive this package.

# A vendor package for `pybind11_json`

This package helps bring the C++ [pybind11_json](https://github.com/pybind/pybind11_json) library into a workspace when it is not already available in the underlay. `pybind11_json` provides easily conversion between `nlohmann::json` and `py::object`.
