function apply_sim_setup(save_models)
% APPLY_SIM_SETUP  One-time migration of the Simulink models:
%   1. block parameters read the vehicle constants from veh (vehicle_params.m),
%   2. the C Function blocks find the torque vectoring header and library through $FE_FIRMWARE_ROOT$
%      (set once per computer with setup_firmware_path),
%   3. the top-level models load both in their PreLoadFcn and InitFcn.
%
%   apply_sim_setup()       % change the models and save them
%   apply_sim_setup(false)  % change the models in memory only (to test)
%
% Run it from the sim folder with the models closed or saved. It is safe to run again; it only sets parameters.

if nargin < 1
    save_models = true;
end

% Paths inside the firmware repo; identical for every user.
TV_HEADER_DIR_IN_REPO = 'firmware\hexray\VC\src\app\torque_vectoring';
TV_HEADER             = 'torque_vectoring_matlab.h';
TV_LIBRARY_IN_REPO    = 'build_fw_test_msvc\firmware\hexray\VC\Debug\hexray_VC_torque_vectoring.lib';

assignin('base', 'FE_FIRMWARE_ROOT', firmware_root());
assignin('base', 'veh', vehicle_params());

subsystem_files = {'vehicle_sim', 'tire_model', 'PowertrainMatlab', 'controls'};
top_models      = {'controls_testing', 'driver_sim'};
for name = [subsystem_files, top_models]
    load_system(name{1});
end
for name = subsystem_files
    set_param(name{1}, 'Lock', 'off');
end

%% vehicle_sim
body = 'vehicle_sim/Vehicle Body 3DOF Dual Track';
set_param(body, ...
    'm',    'veh.CAR_MASS_AT_CG_KG', ...
    'a',    'veh.DIST_FRONT_AXLE_CG_m', ...
    'b',    'veh.DIST_REAR_AXLE_CG_m', ...
    'h',    'veh.DIST_HEIGHT_CG_m', ...
    'w',    '[veh.TRACK_WIDTH_m, veh.TRACK_WIDTH_m]', ...
    'Izz',  'veh.CAR_YAW_MOMENT_INERTIA_KGM2', ...
    'Af',   'veh.FRONTAL_AREA_M2', ...
    'Cd',   'veh.DRAG_COEFF', ...
    'Cl',   'veh.LIFT_COEFF', ...
    'Cpm',  'veh.PITCH_MOMENT_COEFF', ...
    'Pabs', 'veh.AMBIENT_PRESSURE_PA', ...
    'Tair', 'veh.AIR_TEMP_K', ...
    'g',    'veh.GRAVITY');

steering = 'vehicle_sim/Kinematic Steering';
set_param(steering, ...
    'TrckWdth',  'veh.TRACK_WIDTH_m', ...
    'WhlBase',   'veh.WHEELBASE_m', ...
    'StrgRng',   'veh.STEER_WHEEL_RANGE_rad', ...
    'StrgRatio', 'veh.STEERING_RATIO');

set_param('vehicle_sim/MAX_BRAKE_PRESSURE', 'Value', 'veh.MAX_BRAKE_PRESSURE_PA');
set_param('vehicle_sim/Constant',  'Value', 'veh.MOTOR_TEMP_C * ones(1, 4)');  % motor temperatures
set_param('vehicle_sim/Constant1', 'Value', 'veh.PACK_VOLTAGE_V');            % DC bus voltage

%% tire_model
% Only the camber MATLAB Function (suspension values). The tire block (HEIKO) and the tire pressure are not touched:
% the tire block's parameters come from full_norm.tir.
use_veh_in_camber_function();

%% PowertrainMatlab
set_param('PowertrainMatlab/Gearbox/Gear Ratio', 'Gain', 'veh.GEAR_RATIO');
set_param('PowertrainMatlab/Inverse_Gearbox',    'Gain', 'veh.GEAR_RATIO');
set_param('PowertrainMatlab/I_max',              'Value', 'veh.MOTOR_I_MAX_A');
set_param('PowertrainMatlab/V_margin',           'Value', 'veh.INVERTER_V_MARGIN_V');

%% controls
set_param('controls/speed controller/Gain1', 'Gain', 'veh.GEAR_RATIO');  % wheel speed -> motor speed

%% controls_testing
set_param('controls_testing/force feedback calculator/Constant9', 'Value', 'veh.SIM_WHEEL_RADIUS_M');
set_param('controls_testing/force feedback calculator/Gain',      'Gain',  '-1 / veh.GEAR_RATIO');
set_param('controls_testing/Constant3', 'Value', 'veh.RULES_BASED_POWER_LIMIT_KW * 1000');  % W

%% C Function blocks: torque vectoring header and library relative to $FE_FIRMWARE_ROOT$
% The header field is pasted into #include "..." as is, so it holds only the file name; the folder goes in the search
% directory, which (like the library field) expands $FE_FIRMWARE_ROOT$ from the base workspace.
c_function_owners = {'controls', 'driver_sim'};  % controls_testing uses the blocks of the controls subsystem file
for owner = c_function_owners
    blocks = find_system(owner{1}, 'MatchFilter', @Simulink.match.allVariants, 'LookUnderMasks', 'all', ...
                         'LookInsideSubsystemReference', 'off', 'BlockType', 'CFunction');
    for k = 1:numel(blocks)
        for prefix = {'Sim', ''}  % simulation target and code generation target
            set_param(blocks{k}, ...
                [prefix{1} 'CustomHeaderFile'],      TV_HEADER, ...
                [prefix{1} 'CustomSearchDirectory'], ['$FE_FIRMWARE_ROOT$\' TV_HEADER_DIR_IN_REPO], ...
                [prefix{1} 'CustomLibraries'],       ['$FE_FIRMWARE_ROOT$\' TV_LIBRARY_IN_REPO]);
        end
    end
end

%% Callbacks: load the firmware root and veh before the models are edited or simulated, and re-import the debug
%% bus types from the firmware header
load_constants = sprintf('FE_FIRMWARE_ROOT = firmware_root();\nveh = vehicle_params();');
% No 'Names' list: every type in the header is imported, so a new struct in tv_debug.h needs no model change.
% 'Overwrite' replaces bus objects left in the base workspace by an older header (the cause of past bus mismatches).
import_types   = sprintf(['Simulink.importExternalCTypes(fullfile(FE_FIRMWARE_ROOT, ''%s'', ''%s''), ', ...
    '''Overwrite'', ''on'');'], TV_HEADER_DIR_IN_REPO, TV_HEADER);
for name = top_models
    set_param(name{1}, 'PreLoadFcn', load_constants);
    set_param(name{1}, 'InitFcn', sprintf('%s\n%s', load_constants, import_types));
end

%% Save
if save_models
    % Inner subsystem files first: a file cannot be saved while a subsystem file it references has unsaved changes.
    save_order = {'tire_model', 'PowertrainMatlab', 'controls', 'vehicle_sim', 'controls_testing', 'driver_sim'};
    for name = save_order
        save_system(name{1});
    end
    disp('apply_sim_setup: models saved.');
else
    disp('apply_sim_setup: models changed in memory only (not saved).');
end
end

function use_veh_in_camber_function()
% The camber MATLAB Function reads veh as a (non-tunable) parameter instead of its hard-coded constants.
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', 'tire_model/MATLAB Function');
s     = chart.Script;
s = replace_line(s, 'TRACK_WIDTH     =', '    TRACK_WIDTH     = veh.TRACK_WIDTH_m * 1000;                    % mm');
s = replace_line(s, 'ROLL_GRADIENT_F =', '    ROLL_GRADIENT_F = deg2rad(veh.ROLL_GRADIENT_F_deg_g);          % rad/g');
s = replace_line(s, 'BUMP_CAMBER_F   =', '    BUMP_CAMBER_F   = deg2rad(veh.FRONT_BUMP_CAMBER_deg_mm);       % rad/mm');
s = replace_line(s, 'STATIC_F_L      =', '    STATIC_F_L      = -deg2rad(veh.STATIC_CAMBER_FRONT_deg);       % rad - left wheel, top leans right = positive');
s = replace_line(s, 'STATIC_F_R      =', '    STATIC_F_R      =  deg2rad(veh.STATIC_CAMBER_FRONT_deg);       % rad - right wheel, top leans left = negative');
s = replace_line(s, 'ROLL_GRADIENT_R =', '    ROLL_GRADIENT_R = deg2rad(veh.ROLL_GRADIENT_R_deg_g);          % rad/g');
s = replace_line(s, 'BUMP_CAMBER_R   =', '    BUMP_CAMBER_R   = deg2rad(veh.REAR_BUMP_CAMBER_deg_mm);        % rad/mm');
s = replace_line(s, 'STATIC_R_L      =', '    STATIC_R_L      = -deg2rad(veh.STATIC_CAMBER_REAR_deg);        % rad - left wheel');
s = replace_line(s, 'STATIC_R_R      =', '    STATIC_R_R      =  deg2rad(veh.STATIC_CAMBER_REAR_deg);        % rad - right wheel');
s = strrep(s, 'function gamma = CamberAngles(Ay)', 'function gamma = CamberAngles(Ay, veh)');
chart.Script = s;
% The new signature argument appears as an input; make it a parameter read from the workspace.
d         = chart.find('-isa', 'Stateflow.Data', 'Name', 'veh');
d.Scope   = 'Parameter';
d.Tunable = false;
end

function s = replace_line(s, starts_with, new_line)
lines = splitlines(s);
hit   = find(startsWith(strtrim(lines), strtrim(starts_with)), 1);
if isempty(hit)
    error('apply_sim_setup: line starting with "%s" not found in the camber function.', starts_with);
end
lines{hit} = new_line;
s          = strjoin(lines, newline);
end
