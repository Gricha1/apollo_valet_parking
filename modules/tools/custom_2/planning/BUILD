load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "collision",
    srcs = ["collision.py"],
    deps = [
    ],
)

py_library(
    name = "dwa_steering",
    srcs = ["dwa_steering.py"],
    deps = [
    ],
)

py_library(
    name = "generateMap",
    srcs = ["generateMap.py"],
    deps = [
        "//modules/tools/custom_2/planning:collision",
        "//modules/tools/custom_2/planning:utilsPlanning",
        "//modules/tools/custom_2/planning:RRTRLDYNOBS",
    ],
)


py_library(
    name = "posq",
    srcs = ["posq.py"],
    deps = [
        "//modules/tools/custom_2/EnvLib:utils",
    ],
)

py_library(
    name = "reedShepp",
    srcs = ["reedShepp.py"],
    deps = [
    ],
)

py_library(
    name = "RRT",
    srcs = ["RRT.py"],
    deps = [
    ],
)

py_library(
    name = "RRTRLDYNOBS",
    srcs = ["RRTRLDYNOBS.py"],
    deps = [
        "//modules/tools/custom_2/planning:validate",
        "//modules/tools/custom_2/planning:posq",
        "//modules/tools/custom_2/planning:collision",
        "//modules/tools/custom_2/planning:utilsPlanning",
    ],
)

py_library(
    name = "utilsPlanning",
    srcs = ["utilsPlanning.py"],
    deps = [
        "//modules/tools/custom_2/EnvLib:utils",
    ],
)

py_library(
    name = "validate",
    srcs = ["validate.py"],
    deps = [
        "//modules/tools/custom_2/planning:utilsPlanning",
        "//modules/tools/custom_2/planning:dwa_steering",
    ],
)

py_library(
    name = "validateDynObs",
    srcs = ["validateDynObs.py"],
    deps = [
    ],
)