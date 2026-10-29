
% Mechanical braking calculation
% Draft for team review - not connected to Simulink yet

% Brake parameters
pedalForce = 428;        % N, force listed at tire lockup
pedalRatio = 5.1;
frontBias = 0.55;        % Assumed front force fraction
masterArea = 0.000220;   % m^2

frontArea = 0.002027;    % m^2, front caliper per wheel
rearArea = 0.001013;     % m^2, rear caliper per wheel

padMu = 0.41;
rotorRadius = 0.150;     % m
tireRadius = 0.23089;    % m, from vehicle_params.m

% Vehicle parameters
carMass = 293.7;        % kg, including driver
tireMu = 1.4;           % Estimated tire friction
g = 9.81;

% Calculate hydraulic pressure
frontPressure = pedalForce * pedalRatio * frontBias / masterArea;
rearPressure = pedalForce * pedalRatio * (1-frontBias) / masterArea;

% Calculate braking torque per wheel
% Assumes piston areas are effective clamping areas
frontTorque = frontPressure * frontArea * padMu * rotorRadius;
rearTorque = rearPressure * rearArea * padMu * rotorRadius;

% Convert wheel torque into braking force
frontForce = frontTorque / tireRadius;
rearForce = rearTorque / tireRadius;

% Total mechanical braking force from four wheels
totalBrakeForce = 2*frontForce + 2*rearForce;

% Estimate tire grip limit on level ground
% Simple total grip estimate, without weight transfer
tireGripLimit = tireMu * carMass * g;

% Compare braking force with tire grip
usableBrakeForce = min(totalBrakeForce, tireGripLimit);
deceleration = usableBrakeForce / carMass;

% Display results
fprintf('Front pressure: %.0f Pa\n', frontPressure);
fprintf('Rear pressure: %.0f Pa\n', rearPressure);
fprintf('Front braking force per wheel: %.2f N\n', frontForce);
fprintf('Rear braking force per wheel: %.2f N\n', rearForce);
fprintf('Total mechanical braking force: %.2f N\n', totalBrakeForce);
fprintf('Estimated tire grip limit: %.2f N\n', tireGripLimit);
fprintf('Estimated usable braking force: %.2f N\n', usableBrakeForce);
fprintf('Estimated deceleration: %.2f m/s^2\n', deceleration);
