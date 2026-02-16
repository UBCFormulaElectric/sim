function res = motor_op_point(V_dc, T_dmd, Temp, S_op) 
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
%     other maps between T1/T2 to produce final outputs.

%% 0) tolerances
epsV = 1e-4;   % V tolerance (deltaV)
epsT = 1e-4;   % torque tolerance (deltaT)

%% 1) Choose T1/T2 (two nearest bracket temps per flowchart)
availableTemps = [80 100 120];

if Temp > 80 && Temp < 100
    T1 = 80; T2 = 100;
elseif Temp > 100 && Temp < 120
    T1 = 100; T2 = 120;
elseif Temp == 100
    T1 = 100; T2 = 100;
else
    % fallback
    [~, idxNearest] = min(abs(availableTemps - Temp));
    T1 = availableTemps(idxNearest);
    T2 = T1;
end

% Load files
fileFor = @(t) sprintf('A2370DD_T%dC.mat',t);
dat1 = load(fileFor(T1));  % lower temp
dat2 = load(fileFor(T2));  % higher temp (or same if equal)

notes = {};
notes{end+1} = sprintf('Requested Temp=%.2g -> using T1=%d, T2=%d (sweep on T2).', Temp, T1, T2);

%% 2) Build speed & current axes from Temp2 (sweep file)
Tmat2 = dat2.Electromagnetic_Torque;
Vmat2 = dat2.Voltage_Phase_Peak;

[NR, NC] = size(Tmat2);
speeds = linspace(0,20000,NR).';      % column vector (rpm)
currents = linspace(0,105,NC);       % row vector (A)

res.currents = currents;
res.speeds = speeds;

% --- Determine speed indices for S_op ---
is_exact_speed = any(abs(speeds - S_op) < 1e-12);

if S_op <= speeds(1)
    S1_idx = 1; S2_idx = 1; fracS = 0;
elseif S_op >= speeds(end)
    S1_idx = NR; S2_idx = NR; fracS = 0;
elseif is_exact_speed
    % Exact match → no interpolation
    S1_idx = find(abs(speeds - S_op) < 1e-12, 1, 'first');
    S2_idx = S1_idx;
    fracS = 0;
else
    % Normal interpolation case
    S2_idx = find(speeds >= S_op,1,'first');
    S1_idx = S2_idx - 1;
    fracS = (S_op - speeds(S1_idx)) / (speeds(S2_idx) - speeds(S1_idx));
end

notes{end+1} = sprintf('Speed interpolation between row %d (%.1f rpm) and row %d (%.1f rpm), frac=%.4g.', ...
                       S1_idx, speeds(S1_idx), S2_idx, speeds(S2_idx), fracS);

%% 3) Interpolate T and V in speed dimension
T_at_Sop = zeros(1,NC);
V_at_Sop = zeros(1,NC);

for col = 1:NC
    T1s = Tmat2(S1_idx, col);
    T2s = Tmat2(S2_idx, col);
    V1s = Vmat2(S1_idx, col);
    V2s = Vmat2(S2_idx, col);
    % linear interpolation in speed
    T_at_Sop(col) = (1-fracS)*T1s + fracS*T2s;
    V_at_Sop(col) = (1-fracS)*V1s + fracS*V2s;
end

%% 4) Sweep currents to find operating point (exact equality)
found_exact = false;
final_I = NaN;
final_T = NaN;
final_V = NaN;
final_col = NaN;
final_I_frac = 0;

fallback_exists = false;
fallback_T = -Inf;
fallback_col = NaN;
fallback_T_val = NaN;
fallback_V_val = NaN;

targetT = T_dmd + epsT;  % new equality target

% Iterate columns low -> high
for col = 1:NC
    Tv = T_at_Sop(col);
    Vv = V_at_Sop(col);
    v_ok = (Vv <= V_dc + epsV);
    t_ok = (abs(Tv - targetT) < epsT);   % equality
    if v_ok && t_ok
        found_exact = true;
        final_col = col;
        final_I = currents(col);
        final_T = Tv;
        final_V = Vv;
        notes{end+1} = sprintf('Exact equality match at column %d (I=%.3g A): T=%.4g == T_dmd+deltaT, V=%.4g <= V_dc.', ...
                               col, final_I, final_T, final_V);
        break;
    end
    % fallback
    if v_ok && ~t_ok
        if Tv > fallback_T
            fallback_T = Tv;
            fallback_col = col;
            fallback_T_val = Tv;
            fallback_V_val = Vv;
            fallback_exists = true;
        end
    end
end

% Interpolation between adjacent columns
if ~found_exact
    for col = 1:NC-1
        Vc = V_at_Sop(col);   Vcp = V_at_Sop(col+1);
        Tc = T_at_Sop(col);   Tcp = T_at_Sop(col+1);

        if ( (Tc <= targetT && Tcp >= targetT) || (Tc >= targetT && Tcp <= targetT) )
            denom = Tcp - Tc;
            if abs(denom) > epsT
                alpha = (targetT - Tc) / denom;
                if alpha >= 0 && alpha <= 1
                    V_interp = (1-alpha)*Vc + alpha*Vcp;
                    if V_interp <= V_dc + epsV
                        I_interp = (1-alpha)*currents(col) + alpha*currents(col+1);
                        found_exact = true;
                        final_col = col;
                        final_I = I_interp;
                        final_T = targetT;
                        final_V = V_interp;
                        final_I_frac = alpha;
                        notes{end+1} = sprintf('Interpolated equality match between cols %d-%d at alpha=%.4g', col, col+1, alpha);
                        break;
                    end
                end
            end
        end
    end
end

% Final fallback if no exact match
if ~found_exact
    if fallback_exists
        final_col = fallback_col;
        final_I = currents(final_col);
        final_T = fallback_T_val;
        final_V = fallback_V_val;
        notes{end+1} = sprintf('No exact torque match: using fallback at col %d (I=%.3g A) with T=%.4g, V=%.4g.', ...
                               final_col, final_I, final_T, final_V);
    else
        % No operating point
        res.I_op_idx = NaN; res.I_op = NaN; res.T_emg_op = NaN; res.V_op = NaN;
        res.S_op = S_op; res.T_Shaft = NaN; res.I_phase = NaN; res.Total_Loss = NaN; res.Pf = NaN;
        notes{end+1} = 'No operating point found: voltage constraint prevents operation.';
        res.notes = notes;
        return;
    end
end

%% 5) Fill results
res.I_op = final_I;
res.T_emg_op = final_T;
res.V_op = final_V;
res.S_op = S_op;

%% 6) Interpolate other maps by speed & temperature
interp_map = @(map2d) interp2(currents, speeds, map2d, res.I_op, res.S_op, 'linear');

try
    Shaft_T_1 = interp_map(dat1.Shaft_Torque);
    I_phase_1 = interp_map(dat1.Stator_Current_Phase_RMS);
    Total_Loss_1 = interp_map(dat1.Total_Loss);
    Pf_1 = interp_map(dat1.Power_Factor);
catch
    idx_row = S1_idx; idx_col = final_col;
    Shaft_T_1 = dat1.Shaft_Torque(idx_row, idx_col);
    I_phase_1 = dat1.Stator_Current_Phase_RMS(idx_row, idx_col);
    Total_Loss_1 = dat1.Total_Loss(idx_row, idx_col);
    Pf_1 = dat1.Power_Factor(idx_row, idx_col);
    notes{end+1} = 'Warning: interpolation on Temp1 failed, used nearest-grid values for dat1.';
end

try
    Shaft_T_2 = interp_map(dat2.Shaft_Torque);
    I_phase_2 = interp_map(dat2.Stator_Current_Phase_RMS);
    Total_Loss_2 = interp_map(dat2.Total_Loss);
    Pf_2 = interp_map(dat2.Power_Factor);
catch
    idx_row = S1_idx; idx_col = final_col;
    Shaft_T_2 = dat2.Shaft_Torque(idx_row, idx_col);
    I_phase_2 = dat2.Stator_Current_Phase_RMS(idx_row, idx_col);
    Total_Loss_2 = dat2.Total_Loss(idx_row, idx_col);
    Pf_2 = dat2.Power_Factor(idx_row, idx_col);
    notes{end+1} = 'Warning: interpolation on Temp2 failed, used nearest-grid values for dat2.';
end

% Temperature interpolation
if T1 == T2
    fracTemp = 0;
else
    fracTemp = (Temp - T1) / (T2 - T1);
end

res.T_Shaft = (1-fracTemp)*Shaft_T_1 + fracTemp*Shaft_T_2;
res.I_phase = (1-fracTemp)*I_phase_1 + fracTemp*I_phase_2;
res.Total_Loss = (1-fracTemp)*Total_Loss_1 + fracTemp*Total_Loss_2;
res.Pf = (1-fracTemp)*Pf_1 + fracTemp*Pf_2;

notes{end+1} = sprintf('Interpolated shaft torque, phase current, Total_Loss, Pf between T1=%d and T2=%d with fracTemp=%.4g.', T1, T2, fracTemp);

%% 7) Record cases where torque not available or fallback used
if ~found_exact && fallback_exists
    notes{end+1} = 'Final result is fallback: torque demand not achievable at available voltage; returning best torque under V constraint.';
elseif ~found_exact && ~fallback_exists
    notes{end+1} = 'Final result: no operating point (neither torque nor voltage satisfied).';
end

%% 8) finalize outputs
res.notes = notes;

end
