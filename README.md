# Bazel build for Unitree SDK 2
Bazel build for the Unitree SDK 2 library that allows importing via `bazel_dep`.

## Usage
Add the following to your `MODULE.bazel` file:

```python
bazel_dep(name = "unitree-bazel")
git_override(
    module_name = "unitree-bazel",
    remote = "https://github.com/jeh15/unitree-bazel.git",
    commit = "b4083cfd4635f989f1a625b06ff48596fd8ea0b3",
)
```

Make sure to update the `commit` to the latest commit.

This module exports the Unitree SDK 2 library as the following target:

```python
"@unitree-bazel//:unitree_sdk2"
```
