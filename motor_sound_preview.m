function wav_file = motor_sound_preview(play)
% MOTOR_SOUND_PREVIEW  Listen to motor_sound.m without running Simulink.
%
%   motor_sound_preview()        % render a launch + acceleration + lift-off, play it and save a WAV
%   motor_sound_preview(false)   % only save the WAV
%
% Change the tone gains in motor_sound.m or the sound parameters in vehicle_params.m, then run this again.

if nargin < 1
    play = true;
end

veh = vehicle_params();
FS = 48000;
DT = veh.MOTOR_SOUND_FRAME_S;        % one motor_sound frame per call, as in driver_sim
N  = round(FS * DT);
t  = (0:DT:7)';
% A rough acceleration run: full torque to 0.5 s, power limited after 1.5 s, motor speed tops out near 19 000 rpm,
% then lift off and coast down from 5 s.
rpm = 19000 * (1 - exp(-t / 1.6));
rpm(t > 5) = rpm(find(t <= 5, 1, 'last')) * exp(-(t(t > 5) - 5) / 6);
torque = 21 * ones(size(t));
torque(t > 1.5) = 21 * 1.5 ./ t(t > 1.5);
torque(t > 5) = 0;

clear motor_sound                     % reset the persistent phases
audio = zeros(numel(t) * N, 2);
for i = 1:numel(t)
    omega = rpm(i) * 2 * pi / 60 * [1; 1.01; 0.99; 1.02];   % small differences between the wheels
    frame = motor_sound(omega, torque(i) * ones(4, 1), veh);
    audio((i - 1) * N + (1:N), :) = frame;
end

wav_file = fullfile(tempdir, 'motor_sound_preview.wav');
audiowrite(wav_file, audio, FS);
fprintf('Saved %s (%.1f s)\n', wav_file, size(audio, 1) / FS);
if play
    sound(audio, FS);
end
end
