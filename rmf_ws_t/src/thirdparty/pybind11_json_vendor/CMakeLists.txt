cmake_minimum_required(VERSION 3.10)
project(pybind11_json_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
find_package(pybind11_vendor REQUIRED)

find_package(pybind11_json QUIET)

ament_vendor(pybind11_json_vendor
  SATISFIED ${pybind11_json_FOUND}
  VCS_URL https://github.com/pybind/pybind11_json.git
  VCS_VERSION 0.2.15
)

ament_package()
