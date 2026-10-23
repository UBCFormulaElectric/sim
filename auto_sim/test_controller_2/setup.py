from pathlib import Path
import shutil
import sys
import sysconfig

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

__version__ = "0.0.1"

source_dir = Path(__file__).parent
build_dir = source_dir / "build"
# CMake names the module after the target plus this interpreter's ABI suffix,
# e.g. controller.cp313-win_amd64.pyd or controller.cpython-313-darwin.so.
artifact_name = "controller" + sysconfig.get_config_var("EXT_SUFFIX")
# Single-config generators (Ninja/Make) output to build/, multi-config
# generators (Visual Studio/Xcode) output to build/<Config>/.
search_dirs = [build_dir] + [
    build_dir / config for config in ("Release", "RelWithDebInfo", "Debug", "MinSizeRel")
]


def pick_artifact(candidates: list[Path]) -> Path:
    print("CANDIDATES", candidates)
    """Choose which build to install when several configs have been built."""
    find_artifact_name = next((c for c in candidates if c.name == artifact_name), None)
    if find_artifact_name is not None:
        return find_artifact_name
    raise Exception(f"Could not find the expected artifact {artifact_name} among the candidates: {[str(c) for c in candidates]}")


def find_artifact() -> Path:
    candidates = [d / artifact_name for d in search_dirs if (d / artifact_name).is_file()]
    if candidates:
        return pick_artifact(candidates)

    others = sorted(
        p.relative_to(source_dir)
        for d in search_dirs
        for pattern in ("controller*.so", "controller*.pyd")
        for p in d.glob(pattern)
    )
    hint = f" Found builds for other interpreters: {[str(p) for p in others]}." if others else ""
    raise FileNotFoundError(
        f"Expected prebuilt controller extension {artifact_name} in {build_dir}.{hint} "
        "Reconfigure CMake from this interpreter's pipenv, or with "
        f"-DUSE_PIPENV_PYTHON=OFF -DPython_EXECUTABLE={sys.executable}, and rebuild."
    )


class PrebuiltBuildExt(build_ext):
    """Install the CMake-built extension without compiling it again."""

    def build_extension(self, ext):
        artifact = find_artifact()
        destination = Path(self.get_ext_fullpath(ext.name))
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(artifact, destination)


setup(
    name="Controller",
    version=__version__,
    author="Edwin Zheng",
    author_email="ezheng09@student.ubc.ca",
    url="https://github.com/UBCFormulaElectric/sim",
    description="A simple controller for testing",
    long_description="",
    ext_modules=[Extension("Controller", sources=[])],
    extras_require={"test": "pytest"},
    cmdclass={"build_ext": PrebuiltBuildExt},
    zip_safe=False,
    python_requires=">=3.9",
)
