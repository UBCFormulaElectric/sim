% Load files
fileFor = @(t) sprintf('A2370DD_T%dC.mat',t);
data80 = load(fileFor(80));
data100 = load(fileFor(100));
data120 = load(fileFor(120));

function res = motor_model_func(V_dc, T_dmd, Temp, S_op)
% MOTOR_OP_POINT  Find operating point with speed & temperature interpolation
%
% Inputs:
%   V_dc  - DC bus / peak-phase voltage limit (V)
%   T_dmd - requested electromagnetic torque (Nm)
%   Temp  - requested motor temperature (degC) (use 80-120 ideally)
%   S_op  - operating speed (rpm) at which to find operating point
%
% Output (struct res) - same fields as before, plus interpolated indices/frac
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
        data1 = data80;
        data2 = data80;
        fracT = 0;
    elseif 80 < Temp && Temp <= 100
        data1 = data80;
        data2 = data100;
        fracT = (Temp - 80)/(100-80);
    elseif 100 <= Temp && Temp < 120
        data1 = data100;
        data2 = data120;
        fracT = (Temp-100)/(120-100);
    elseif 120 <= Temp
        data1 = data120;
        data2 = data120;
        fracT = 0;
    end

    % round S_op to nearest increment of 20000/rows
    speed_inc = 20000/(rows - 1);
    S_idx_lb = floor(S_op / speed_inc);
    S_idx_ub = ceil(S_op / speed_inc);
    S_lb = S_idx_lb * speed_inc;
    S_ub = S_idx_ub * speed_inc;

    current_inc = 25/(cols - 1);

    % linear interpolation in speed
    if S_ub == S_lb
        fracS = 0;
    else
        fracS = (S_op - S_lb) / (S_ub - S_lb);
    end
    
    % find current limit from voltage
    % quicky bilinear interpolation
    Vmat1 = data1.Voltage_Phase_Peak;
    Vmat2 = data2.Voltage_Phase_Peak;
    V_at_Sop_1 = (1-fracS)*Vmat1(S_idx_lb, :) + fracS*Vmat1(S_idx_ub, :);
    V_at_Sop_2 = (1-fracS)*Vmat2(S_idx_lb, :) + fracS*Vmat2(S_idx_ub, :);
    V = (1-fracT)*V_at_Sop_1 + fracT*V_at_Sop_2;
    % find lmao
    V_find = find(diff(V > V_dc) == 1, 1);
    if isempty(V_find)
        % figure out if you are too low or too high
        if T_dmd < V(1)
            V_cur_lim = 0;
        else
            V_cur_lim = cols * current_inc;
        end
    else
        % note that tq_find+1 as tq_find in [1,n-1] (as difference array
        % removes an element at the end)
        V_cur_lim = ( ...
            V_find + ...
            (V_dc - V(V_find))/(V(V_find + 1) - V(V_find)) ...
        ) * cur_inc;
    end

    % find current required for torque
    Tqmat1 = data1.Electromagnetic_Torque;
    Tqmat2 = data2.Electromagnetic_Torque;
    Tq_at_Sop_1 = (1-fracS)*Tqmat1(S_idx_lb, :) + fracS*Tqmat1(S_idx_ub, :);
    Tq_at_Sop_2 = (1-fracS)*Tqmat2(S_idx_lb, :) + fracS*Tqmat2(S_idx_ub, :);
    Tq = (1-fracT)*Tq_at_Sop_1 + fracT*Tq_at_Sop_2;
    % find lmao
    Tq_find = find(diff(Tq > T_dmd) == 1, 1);
    if isempty(Tq_find)
        % figure out if you are too low or too high
        if T_dmd < Tq(1)
            Tq_cur_lim= 0;
        else
            Tq_cur_lim = 25;
        end
    else
        % note that tq_find+1 as tq_find in [1,n-1] (as difference array
        % removes an element at the end)
        Tq_cur_lim = ( ...
            Tq_find + ...
            (T_dmd - Tq(Tq_find))/(Tq(Tq_find + 1) - Tq(Tq_find)) ...
        ) * cur_inc;
    end
    
    % find the current draw
    final_I = min(Tq_cur_lim, V_cur_lim);

    % res.I_op = final_I;
    % res.T_emg_op = final_T;
    % res.V_op = final_V;
    % res.S_op = S_op;
    
    % report interpolated temperature
    % Tmat2 = data1.Electromagnetic_Torque;
    % T1s = Tmat2(S_idx_lb, :);
    % T2s = Tmat2(S_idx_ub, :);
    % T_at_Sop(col) = (1-fracS)*T1s + fracS*T2s;
end