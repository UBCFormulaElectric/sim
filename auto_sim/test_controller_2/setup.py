from pathlib import Path
import sys

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

__version__ = "0.0.1"

source_dir = Path(__file__).parent
deps_dir = source_dir / "build" / "_deps"

alglib_srcs = list((deps_dir / "alglib-src" / "src").glob("*.cpp"))
cpp_standard_flag = "/std:c++latest" if sys.platform == "win32" else "-std=c++23"

ext_modules = [
    Pybind11Extension(
        "Controller",
        [str(path) for path in source_dir.glob("*.cpp")] + [str(path) for path in alglib_srcs],
        include_dirs=[
            str(source_dir),
            str(deps_dir / "cdt-src" / "CDT" / "include"),
            str(deps_dir / "eigen-src"),
            str(deps_dir / "auto_diff-src"),
            str(deps_dir / "auto_diff-src" / "autodiff" / "forward"),
            str(deps_dir / "auto_diff-src" / "autodiff" / "forward" / "utils"),
            str(deps_dir / "alglib-src" / "src"),
            str(deps_dir / "simde-src"),
            str(deps_dir / "proxsuite-src" / "include"),
            str(deps_dir / "proxsuite-build" / "include"),
        ],
        define_macros=[
            ("EIGEN_STACK_ALLOCATION_LIMIT", "0"),
            ("EIGEN_MAX_STATIC_ALIGN_BYTES", "0"),
        ],
        extra_compile_args=[cpp_standard_flag],
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
