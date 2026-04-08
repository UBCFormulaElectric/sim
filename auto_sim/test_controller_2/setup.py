# Available at setup time due to pyproject.toml
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

__version__ = "0.0.1"

# get all sources in the build/_deps/alglib-src/alglib/src directory
import os

alglib_srcs = []
for root, dirs, files in os.walk("build/_deps/alglib-src/src"):
    for file in files:
        if file.endswith(".cpp"):
            alglib_srcs.append(os.path.join(root, file))
print(alglib_srcs)

ext_modules = [
    Pybind11Extension(
        "Controller",
        ["pybind.cpp", "sim.cpp", "controller.cpp", "path.cpp", "perception.cpp"] + alglib_srcs,
        include_dirs=[".", "build/_deps/cdt-src/CDT/include", "build/_deps/alglib-src/src"],
        language="c++",
    ),
]

setup(
    name="Controller",
    version=__version__,
    author="Edwin Zheng",
    author_email="ezheng09@student.ubc.ca",
    url="https://github.com/UBCFormulaElectric/sim",
    description="A simple controller for testing",
    long_description="",
    ext_modules=ext_modules,
    extras_require={"test": "pytest"},
    # Currently, build_ext only provides an optional "highest supported C++
    # level" feature, but in the future it may provide more features.
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.9",
)
