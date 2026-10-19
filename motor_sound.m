function audio = motor_sound(motor_omega, motor_torque, veh)
%#codegen
% MOTOR_SOUND  One stereo audio frame (veh.MOTOR_SOUND_FRAME_S long) of the car's drivetrain sound, as heard
%              from the cockpit.
%
%   audio = motor_sound(motor_omega, motor_torque, veh)
%
%   motor_omega  : 4x1 motor shaft speeds [rad/s], order fl fr rl rr
%   motor_torque : 4x1 motor torques [Nm]
%   veh          : struct from vehicle_params()
%   audio        : Nx2 frame at 48 kHz, N = 48000 * veh.MOTOR_SOUND_FRAME_S (left, right); the left motors play on
%                  the left channel
%
% Mix measured from the pass-by recording of the car (IMG_7764), orders relative to the motor shaft frequency:
%   * motor electrical order 2*f_e (order 10 for 5 pole pairs)          - the strongest tone
%   * low-order lines at orders ~2.4 and ~3.8 (source not identified)    - about 6-8 dB below it
%   * gearbox sun/planet mesh (order ~16.6) and planet/ring mesh (~6.1)  - about 8 dB below it
%   * broadband road and wind noise that rises with speed, most of its energy below 500 Hz
% Not modelled: the 8 kHz inverter PWM tone and the 6*f_e / 12*f_e torque ripple (not in the recordings), and the
% constant ~1.24 kHz pump/fan hum of the recording (it only rings at idle and adds nothing to the sense of speed).
%
% The phase of every tone carries over between frames, and both the frequency and the loudness ramp linearly
% inside a frame, so speed and torque changes do not click. The mix is normalised so the limiter stays almost linear (heavy limiting sounds like static).

FS = 48000;   % audio sample rate [Hz]
N  = round(FS * veh.MOTOR_SOUND_FRAME_S);   % samples per frame; veh is a non-tunable parameter, so N is constant
NUM_TONES = 5;
MAX_TORQUE_NM = 21;
% Speed-dependent tones: 2 x electrical, low line A, low line B, ring mesh, sun mesh
LOW_ORDER_A = 2.4;
LOW_ORDER_B = 3.8;
TONE_GAIN = [1.0, 0.5, 0.4, 0.4, 0.4];
% Road / wind noise, fitted to the on-board recording: its RMS (in units of the strongest tone's amplitude) is
% NOISE_RMS_REF at NOISE_REF_RPM and grows with speed^NOISE_SPEED_EXP; above NOISE_MAX_RPM (the highest speed in the
% recording) it stays constant. Scale it with veh.ROAD_NOISE_SCALE.
NOISE_RMS_REF   = 1.45;
NOISE_REF_RPM   = 3300;
NOISE_SPEED_EXP = 1.8;
NOISE_MAX_RPM   = 9700;
NOISE_CUTOFF_HZ  = 300;                  % wind / road rumble: three one-pole low-pass stages at this frequency
NOISE_FILTER_RMS = 0.08586;              % RMS of unit white noise after those three stages
% Tones above ROLLOFF_HZ are attenuated (second order), as in the recording (only ~8 % of its energy is above
% 2.5 kHz); without it the tones near top speed (3-5 kHz) ring.
ROLLOFF_HZ = 2000;
% Small random variation of each tone's loudness so steady tones do not sound like pure sine waves
JITTER_DEPTH  = 0.15;                    % +-15 % (about +-1.3 dB)
JITTER_SMOOTH = 0.85;                    % per-frame smoothing of the random variation (closer to 1 = slower)

persistent phase f_prev g_prev jitter noise_prev lp1 lp2 lp3
if isempty(phase)
    phase      = zeros(4, NUM_TONES);
    f_prev     = zeros(4, NUM_TONES);
    g_prev     = zeros(4, NUM_TONES);
    jitter     = zeros(4, NUM_TONES);
    noise_prev = 0;
    lp1       = zeros(1, 2);
    lp2       = zeros(1, 2);
    lp3       = zeros(1, 2);
end

f_shaft = abs(motor_omega(:)) / (2 * pi);
f_e     = veh.MOTOR_POLE_PAIRS * f_shaft;
f_now   = veh.MOTOR_SOUND_PITCH_SCALE * ...
          [2 * f_e, LOW_ORDER_A * f_shaft, LOW_ORDER_B * f_shaft, ...
           veh.GEAR_MESH_ORDER_RING * f_shaft, veh.GEAR_MESH_ORDER_SUN * f_shaft];

load_frac = min(abs(motor_torque(:)) / MAX_TORQUE_NM, 1);           % 0..1
jitter = JITTER_SMOOTH * jitter + (1 - JITTER_SMOOTH) * randn(4, NUM_TONES) * 3;
gain = (0.25 + 0.75 * load_frac) * TONE_GAIN ...                     % 4 x NUM_TONES, louder with torque
       ./ (1 + (f_now / ROLLOFF_HZ) .^ 2) ...                        % high tones quieter
       .* (1 + JITTER_DEPTH * max(min(jitter, 1), -1));              % slow random loudness variation
gain(f_now <= 30 | f_now >= 0.45 * FS) = 0;                          % inaudible or would alias

ramp  = (1:N)' / N;
audio = zeros(N, 2);
for m = 1:4
    channel = 1 + (m == 2 || m == 4);                                % fl, rl -> left; fr, rr -> right
    for k = 1:NUM_TONES
        f  = f_prev(m, k) + (f_now(m, k) - f_prev(m, k)) * ramp;     % linear frequency ramp in the frame
        g  = g_prev(m, k) + (gain(m, k)  - g_prev(m, k)) * ramp;     % linear gain ramp: no step at the frame edge
        ph = phase(m, k) + 2 * pi * cumsum(f) / FS;
        audio(:, channel) = audio(:, channel) + g .* sin(ph);
        phase(m, k) = mod(ph(end), 2 * pi);
    end
end
f_prev = f_now;
g_prev = gain;

% Road and wind noise: white noise through three one-pole low-pass stages, a low rumble rather than a hiss,
% louder with speed
rpm = min(mean(f_shaft) * 60, NOISE_MAX_RPM);
noise_rms = veh.ROAD_NOISE_SCALE * NOISE_RMS_REF * (rpm / NOISE_REF_RPM) ^ NOISE_SPEED_EXP;
a = exp(-2 * pi * NOISE_CUTOFF_HZ / FS);
noise = randn(N, 2);
for n = 1:N
    lp1 = a * lp1 + (1 - a) * noise(n, :);
    lp2 = a * lp2 + (1 - a) * lp1;
    lp3 = a * lp3 + (1 - a) * lp2;
    noise(n, :) = lp3;
end
noise_gain = noise_prev + (noise_rms - noise_prev) * ramp;          % ramp the noise level too
audio = audio + (noise_gain / NOISE_FILTER_RMS) .* noise;
noise_prev = noise_rms;

% Normalise by the loudest possible mix (two motors per channel at full load, noise peaks at 3x RMS at top speed),
% then a gentle limiter that stays almost linear
max_noise_rms = veh.ROAD_NOISE_SCALE * NOISE_RMS_REF * (NOISE_MAX_RPM / NOISE_REF_RPM) ^ NOISE_SPEED_EXP;
full_scale = 2 * sum(TONE_GAIN) + 3 * max_noise_rms;
audio = tanh(veh.MOTOR_SOUND_VOLUME * audio / full_scale);
end
