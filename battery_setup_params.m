% BATTERY PACK PARAMETERS (140 series 5 parallel)

% CAPACITY
% Single cell = 3.0Ah. Pack = 5 parallel * 3.0 = 15.0 Ah
pack_capacity_Ah = 15.0; 

% OCV CURVE (scaled for 140 Series cells)
soc_breakpoints = [0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0];
cell_ocv = [2.5, 3.0, 3.35, 3.60, 3.90, 4.05, 4.20];
pack_ocv_curve = cell_ocv * 140; % Scaled for 140 series cells

% INTERNAL RESISTANCE (Approximation per datasheet)
% Calculating average R0 for simplicity in the EKF
% R_pack = R_cell * (N_series / N_parallel) = R_cell * 28
% Avg cell resistance approx 0.02 Ohm -> Pack approx 0.56 Ohm
pack_resistance_Ohms = 0.56; 

% SIMULATION TIME STEP
dt = 0.01; % Run the filter at 100Hz (0.01s)