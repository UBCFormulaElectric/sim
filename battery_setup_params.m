% BATTERY PACK PARAMETERS (140 series 5 parallel)
% Molicel P30B Cell Data

% 1. CAPACITY
pack_capacity_Ah = 15.0; % 3.0Ah cell * 5 Parallel

% 2. TEMPERATURE VECTOR
% Corresponds to [0 C, 23 C, 60 C]
T_vec = [273.15, 296.15, 333.15]; 

% 3. OCV CURVE (7x3 Matrix)
soc_breakpoints = [0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0];

% Base Cell Voltage (23C)
cell_v_nominal = [2.50, 3.15, 3.40, 3.65, 3.90, 4.08, 4.20]'; 

% Scale to 140S Pack
pack_v_nominal = cell_v_nominal * 140;

% Create 7x3 Matrix (Repeat the nominal curve for all 3 temps)
% We assume OCV (Chemical energy) doesn't change much with temp
pack_ocv_table = repmat(pack_v_nominal, 1, 3);

% 4. INTERNAL RESISTANCE (7x3 Matrix)
% Base Cell Resistance (23C) derived from Impedance Graph (P30B Specific)
% Values: [30, 24.5, 24.8, 23.5, 23.2, 25, 28] mOhm
cell_r_nominal = [0.030, 0.0245, 0.0248, 0.0235, 0.0232, 0.025, 0.028]';

% Scaled to 140S 5P Pack (x28)
pack_r_nominal_vec = cell_r_nominal * 28;

% Temperature Scaling Factors [Cold(0C), Nominal(23C), Hot(60C)]
% Cold = 2x resistance, Hot = 0.8x resistance
temp_scaling = [2.0, 1.0, 0.8]; 
test_temp_scaling = [2.0, 1.05, 0.8];

% Create 7x3 Matrix
% Multiplies the Column (SOC) by the Row (Temp Scaling)
pack_r0_table = pack_r_nominal_vec * temp_scaling;
test_pack_r0_table = pack_r_nominal_vec * test_temp_scaling;

% 5. EKF SPECIFIC PARAMETERS
% The EKF uses a constant average. 
% New Average of pack_r_nominal_vec is approx 0.716 Ohms
R_int_ekf = 0.716; 
dt = 0.01;