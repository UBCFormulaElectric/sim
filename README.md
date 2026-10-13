# sim

Simulink vehicle and controls simulation. The torque vectoring controller runs the firmware C++ code from
[Consolidated-Firmware](https://github.com/UBCFormulaElectric/Consolidated-Firmware) through C Function blocks.

## Requirements

- MATLAB R2025b with Simulink, Vehicle Dynamics Blockset, Simulink 3D Animation, DSP System Toolbox and
  Simscape Electrical
- Visual Studio 2022 with the "Desktop development with C++" workload
- CMake, Git, and the Python setup from the Consolidated-Firmware README

## Setup

1. Clone this repo and Consolidated-Firmware side by side:

   ```
   Formula-Electric/
   ├── sim/
   └── Consolidated-Firmware/
   ```

2. Build the torque vectoring library (from `Consolidated-Firmware/`):

   ```
   cmake -S . -B build_fw_test_msvc -G "Visual Studio 17 2022" -A x64 -DPLATFORM=firmware -DTARGET=test -DUSE_COMMIT_INFO=MINIMAL
   cmake --build build_fw_test_msvc --target hexray_VC_torque_vectoring --config Debug
   ```

   The build folder must be named `build_fw_test_msvc`.

3. In MATLAB, run once per computer (from `sim/`):

   ```matlab
   setup_firmware_path
   ```

   This stores your Consolidated-Firmware path and selects Visual C++ 2022 as the compiler. If the firmware repo is not
   next to `sim/`, it asks you for the folder.

4. Open `driver_sim` (driver in the loop) or `controls_testing` (scripted inputs) and run it. Keep `sim/` as the MATLAB
   current folder. The first run is slow because the custom code compiles.

## Day to day

| Change | Then |
|---|---|
| Torque vectoring C++ | Rebuild the library (step 2) and run the sim. If it still uses old code, delete `slprj/`. |
| A vehicle constant | Edit `vehicle_params.m`. Section 1 mirrors the firmware `constants.hpp`; keep both in sync. |
| `tv_debug.h` | Nothing. The model imports the bus types on every run. |