from pathlib import Path
import shutil

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

__version__ = "0.0.1"

source_dir = Path(__file__).parent
artifact = source_dir / "build" / "Debug" / "controller.cp313-win_amd64.pyd"


class PrebuiltBuildExt(build_ext):
    """Install the CMake-built extension without compiling it again."""

    def build_extension(self, extension):
        if not artifact.is_file():
            raise FileNotFoundError(
                f"Expected prebuilt controller extension at {artifact}"
            )

        destination = Path(self.get_ext_fullpath(extension.name))
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
