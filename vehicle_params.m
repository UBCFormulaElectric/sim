function veh = vehicle_params()
% VEHICLE_PARAMS  Single source of truth for the vehicle constants used by the Simulink models.
%
%   veh = vehicle_params();
%
% The models read every vehicle constant from this struct (block parameters use veh.<NAME>), so a value is changed
% here and nowhere else. The top-level models (driver_sim, controls_testing) call this function in their PreLoadFcn
% and InitFcn, which puts `veh` in the base workspace before every run.
%
% Two sources of truth for now:
%   * Section 1 mirrors the firmware constants (Consolidated-Firmware/firmware/hexray/VC/src/app/torque_vectoring/
%     shared_datatypes/constants.hpp and firmware/shared/srcpp/util/util_units.hpp). The field names are the firmware
%     names. When a firmware value changes, change it here too.
%   * Section 2 holds the parameters that only the simulation uses (plant, powertrain, brakes).
%   * Section 3 holds values derived from the two sections above. Do not edit them directly.

%% 1. Values mirrored from the firmware (constants.hpp / util_units.hpp)
% Physical
veh.GRAVITY = 9.81;              % m/s^2

% Aero
veh.FRONTAL_AREA_M2   = 0.94;    % m^2, from aero team
veh.AIR_DENSITY_KGPM3 = 1.2205;  % kg/m^3
veh.LIFT_COEFF        = 1.7;     % from aero team
veh.DRAG_COEFF        = 0.92;
veh.COP_REAR          = 0.68;    % fraction of the aero load on the rear axle
veh.COP_RIGHT         = 0.5;     % fraction of the aero load on the right side (not modelled in the plant)

% Dimensions
veh.WHEELBASE_m          = 1.550;  % m
veh.TRACK_WIDTH_m        = 1.100;  % m
veh.DIST_FRONT_AXLE_CG_m = 0.837;  % m, front axle to CG (parameter 'a')
veh.DIST_HEIGHT_CG_m     = 0.30;   % m, CG height

% Mass and inertia
veh.CAR_MASS_CG_NO_DRIVER_KG    = 223.7;  % kg
veh.DRIVER_MASS_KG              = 70.0;   % kg
veh.CAR_YAW_MOMENT_INERTIA_KGM2 = 400.0;  % kg*m^2

% Powertrain
veh.GEAR_RATIO                 = 14.3;  % motor revs per wheel rev (util_units.hpp)
veh.RULES_BASED_POWER_LIMIT_KW = 80.0;  % kW, FSAE limit

% Steering
veh.STEER_WHEEL_RANGE_rad = 1.48632;  % rad, steering wheel angle at full lock

% Suspension (firmware sign convention: negative = top of the wheel leans in)
veh.STATIC_CAMBER_FRONT_deg  = -1.0;   % deg
veh.FRONT_BUMP_CAMBER_deg_mm = 0.02;   % deg/mm
veh.STATIC_CAMBER_REAR_deg   = -0.75;  % deg
veh.REAR_BUMP_CAMBER_deg_mm  = 0.06;   % deg/mm

%% 2. Simulation-only parameters
% Wheel radius used to turn tire force into wheel torque in the force-feedback path. The firmware uses its own
% WHEEL_RADIUS_M (18 in / 2 = 0.2286 m); the two are intentionally different for now.
veh.SIM_WHEEL_RADIUS_M = 0.23089;  % m, unloaded radius from the tire file

% Air pressure; the plant computes density as Pabs / (R * T), so the temperature is derived (section 3)
veh.AMBIENT_PRESSURE_PA = 101325;  % Pa
veh.AIR_GAS_CONSTANT    = 287.058; % J/(kg*K), value used inside the Vehicle Body block

% Suspension roll (plant camber model)
veh.ROLL_GRADIENT_F_deg_g = 1.05;  % deg/g
veh.ROLL_GRADIENT_R_deg_g = 0.81;  % deg/g

% Steering (plant Kinematic Steering block)
veh.STEERING_RATIO = 10;  % steering wheel angle / road wheel angle

% Powertrain
veh.PACK_VOLTAGE_V      = 450;  % V, DC bus voltage fed to the motor model
veh.MOTOR_TEMP_C        = 80;   % degC, motor temperature fed to the motor model
veh.MOTOR_I_MAX_A       = 105;  % A, motor peak current
veh.INVERTER_V_MARGIN_V = 30;   % V

% Brakes (the brake disc and pad data live in the tire block with the full_norm.tir parameters and are set there)
veh.MAX_BRAKE_PRESSURE_PA = [600000, 600000, 300000, 300000];  % Pa, [FL FR RL RR]

%% 3. Derived values (do not edit)
veh.CAR_MASS_AT_CG_KG   = veh.CAR_MASS_CG_NO_DRIVER_KG + veh.DRIVER_MASS_KG;
veh.DIST_REAR_AXLE_CG_m = veh.WHEELBASE_m - veh.DIST_FRONT_AXLE_CG_m;
% Air temperature that gives AIR_DENSITY_KGPM3 in the plant (rho = Pabs / (R * T))
veh.AIR_TEMP_K = veh.AMBIENT_PRESSURE_PA / (veh.AIR_GAS_CONSTANT * veh.AIR_DENSITY_KGPM3);
% Pitch moment coefficient that gives the firmware aero balance in the plant. The Vehicle Body block puts a
% fraction b/L + Cpm/Cl of the downforce on the front axle, and the firmware puts (1 - COP_REAR) there.
veh.PITCH_MOMENT_COEFF = veh.LIFT_COEFF * ((1 - veh.COP_REAR) - veh.DIST_REAR_AXLE_CG_m / veh.WHEELBASE_m);
end
