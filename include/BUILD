hdrs = glob([
    "unitree/common/*.hpp",
    "unitree/common/**/*.hpp",
    "unitree/idl/*.hpp",
    "unitree/idl/**/*.hpp",
    "unitree/robot/*.hpp",
    "unitree/robot/**/*.hpp",
])

cc_import(
    name = "static_import_libunitree",
    hdrs = hdrs,
    # includes = ["unitree"],
    alwayslink = True,
    static_library = select({
        "@platforms//cpu:x86_64": "//lib:amd64_lib",
        "@platforms//cpu:arm64": "//lib:arm64_lib",
        "//conditions:default": "@platforms//:incompatible",
    }),

)

cc_library(
    name = "lib_unitree",
    hdrs = hdrs,
    # includes = ["unitree"],
    deps = [
        ":static_import_libunitree",
        "//thirdparty:lib_ddsc",
        "//thirdparty:lib_ddscxx",
    ],
    copts = ["-fpic"],
    linkopts = ["-pthread"],
    alwayslink = True,
    visibility = ["//visibility:public"],
)