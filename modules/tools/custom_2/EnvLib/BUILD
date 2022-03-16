load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "ObstGeomEnv",
    srcs = ["ObstGeomEnv.py"],
    deps = [
        "//modules/tools/custom_2/planning:generateMap",
        "//modules/tools/custom_2/EnvLib:line",
        "//modules/tools/custom_2/EnvLib:Vec2d",
        "//modules/tools/custom_2/EnvLib:utils",
        "//modules/tools/custom_2/planning:utilsPlanning",
    ],
)


py_library(
    name = "line",
    srcs = ["line.py"],
    deps = [
        "//modules/tools/custom_2/EnvLib:Vec2d",
    ],
)

py_library(
    name = "Vec2d",
    srcs = ["Vec2d.py"],
    deps = [
    ],
)

py_library(
    name = "utils",
    srcs = ["utils.py"],
    deps = [
        "//modules/tools/custom_2/EnvLib:line",
    ],
)