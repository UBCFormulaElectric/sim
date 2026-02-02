% Load files
fileFor = @(t) sprintf('A2370DD_T%dC.mat',t);
data80 = load(fileFor(80));
data100 = load(fileFor(100));
data120 = load(fileFor(120));

% quicky bilinear interpolation
function m = bilinear_interpolation(mat_t1, mat_t2, S_idx, fracT)
    % S_idx is the decimal index of speed. The integer part represents the
    % lower bound speed and the fraction part represents the amount to
    % interpolate
    % fracT represents the weighting to give between mat_t1 and mat_t2
    S_idx_lb = fix(S_idx);
    fracS = S_idx - S_idx_lb;
    S_idx_ub = S_idx_lb + 1; % this should be safe as is fracS = 0, then it should short circuit the multiplication calculation

    % we choose to interpolate on speed first because it reduces the
    % dimension of the array
    m_at_Sop_1 = (1-fracS)*mat_t1(S_idx_lb, :) + fracS*mat_t1(S_idx_ub, :);
    m_at_Sop_2 = (1-fracS)*mat_t2(S_idx_lb, :) + fracS*mat_t2(S_idx_ub, :);
    % then interpolate on motor temperature
    m = (1-fracT)*m_at_Sop_1 + fracT*m_at_Sop_2;
end

function val = trilinear_query(mat_t1, mat_t2, S_idx, I_idx, fracT)
    m = bilinear_interpolation(mat_t1, mat_t2, S_idx, fracT);
    I_idx_lb = fix(I_idx);
    fracI = I_idx - I_idx_lb;
    I_idx_ub = I_idx_lb + 1;
    val = (1 - fracI)*m(I_idx_lb) + fracI*m(I_idx_ub);
end

function res = motor_model_func(V_dc, T_dmd, Temp, S_op)
% Inputs:
%   V_dc  - DC bus / peak-phase voltage limit (V)
%   T_dmd - requested electromagnetic torque (Nm)
%   Temp  - requested motor temperature (degC) (use 80-120 ideally)
%   S_op  - operating speed (rpm) at which to find operating point
%
% Output (struct res)
%   res.I_op       - approximated phase current (A) corresponding to column
%   res.T_emg_op   - Electromagnetic torque value found (Nm)
%   res.V_op       - phase-peak voltage found (same units as Voltage_Phase_Peak)
%   res.T_Shaft    - shaft torque (usable torque) (Nm)
%   res.I_phase    - stator phase current (ARMS)
%   res.Mech_loss  - mechanical loss (W)
%   res.Pf         - power factor
%
% Notes:
%   - Implementation chooses two temp files T1/T2 surrounding Temp, 
%     interpolates across speed to evaluate T and V at S_op, sweeps currents 
%     low->high to find exact or interpolated torque match, then interpolates
%     other maps between T1/T2 to produce final outputs.m,

    global data80
    global data100
    global data120
    rows = 201;
    cols = 21;
    
    if Temp <= 80
        t1 = data80;
        t2 = data80;
        fracT = 0;
    elseif 80 < Temp && Temp <= 100
        t1 = data80;
        t2 = data100;
        fracT = (Temp - 80)/(100-80);
    elseif 100 <= Temp && Temp < 120
        t1 = data100;
        t2 = data120;
        fracT = (Temp-100)/(120-100);
    elseif 120 <= Temp
        t1 = data120;
        t2 = data120;
        fracT = 1;
    end

    % round S_op to nearest increment of 20000/rows
    speed_inc = 20000/(rows - 1);
    S_idx = S_op / speed_inc;
    current_inc = 25/(cols - 1);
    
    % find current limit from voltage
    V = bilinear_interpolation(t1.Voltage_Phase_Peak, t2.Voltage_Phase_Peak, S_idx, fracT);
    V_find = find(diff(V > V_dc) == 1, 1); % find lmao
    if isempty(V_find)
        % figure out if you are too low or too high
        if T_dmd < V(1)
            V_cur_lim_idx = 0;
        else
            V_cur_lim_idx = cols * current_inc;
        end
    else
        % note that tq_find+1 as tq_find in [1,n-1] (as difference array
        % removes an element at the end)
        V_cur_lim_idx = V_find + (V_dc - V(V_find))/(V(V_find + 1) - V(V_find));
    end

    % find current required for torque
    Tq = bilinear_interpolation(t1.Electromagnetic_Torque, t2.Electromagnetic_Torque, S_idx, fracT);
    Tq_find = find(diff(Tq > T_dmd) == 1, 1); % find lmao
    if isempty(Tq_find)
        % figure out if you are too low or too high
        if T_dmd < Tq(1)
            Tq_cur_lim_idx= 0;
        else
            Tq_cur_lim_idx = 25;
        end
    else
        % note that tq_find+1 as tq_find in [1,n-1] (as difference array
        % removes an element at the end)
        Tq_cur_lim_idx = Tq_find + (T_dmd - Tq(Tq_find))/(Tq(Tq_find + 1) - Tq(Tq_find));
    end
    
    % find the current draw
    I_idx = min(Tq_cur_lim_idx, V_cur_lim_idx);

    res.I_op = (I_idx-1) * current_inc; % stupid ahh 1 indexing
    res.T_emg_op = trilinear_query(t1.Electromagnetic_Torque, t2.Electromagnetic_Torque, S_idx, I_idx, fracT);
    res.V_op = trilinear_query(t1.Voltage_Phase_Peak, t2.Voltage_Phase_Peak, S_idx, I_idx, fracT);
    res.T_Shaft = trilinear_query(t1.Shaft_Torque, t2.Shaft_Torque, S_idx, I_idx, fracT);
    res.I_phase = trilinear_query(t1.Stator_Current_Phase_RMS, t2.Stator_Current_Phase_RMS, S_idx, I_idx, fracT);
    res.Pf = trilinear_query(t1.Power_Factor, t2.Power_Factor, S_idx, I_idx, fracT);
end