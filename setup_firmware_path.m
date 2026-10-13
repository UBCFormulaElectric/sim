function setup_firmware_path(root)
% SETUP_FIRMWARE_PATH  Tell the Simulink models where your Consolidated-Firmware checkout is. Run once per computer.
%
%   setup_firmware_path()                               % pick the folder (or accept ../Consolidated-Firmware)
%   setup_firmware_path('D:\code\Consolidated-Firmware') % give the folder directly
%
% The path is stored in your MATLAB preferences, so it stays set across MATLAB sessions. The models read it with
% firmware_root() and put it in the base workspace as FE_FIRMWARE_ROOT. The C Function blocks then use
% $FE_FIRMWARE_ROOT$\<path inside the repo>, which is the same for every user.
%
% It also selects Microsoft Visual C++ 2022 as the MEX compiler for C and C++, the compiler the torque vectoring
% library is built with.

if nargin < 1 || isempty(root)
    sibling = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'Consolidated-Firmware');
    if is_firmware_root(sibling)
        root = sibling;
    else
        root = uigetdir(pwd, 'Select your Consolidated-Firmware folder');
        if isequal(root, 0)
            error('setup_firmware_path: no folder selected.');
        end
    end
end

root = char(root);
if ~is_firmware_root(root)
    error(['setup_firmware_path: "%s" is not a Consolidated-Firmware checkout ', ...
           '(torque_vectoring_matlab.h not found under it).'], root);
end

setpref('FormulaElectricSim', 'FirmwareRoot', root);
assignin('base', 'FE_FIRMWARE_ROOT', root);
fprintf('Firmware root set to %s\n', root);

select_msvc_compiler();
end

function select_msvc_compiler()
% The torque vectoring library is built with Visual C++ 2022, so Simulink must compile and link the C Function
% blocks with the same compiler (C for the subsystem files, C++ for the top-level models).
for lang = {'C', 'C++'}
    installed = mex.getCompilerConfigurations(lang{1}, 'Installed');
    msvc      = installed(contains({installed.Name}, 'Microsoft Visual C++') & strcmp({installed.Version}, '17.0'));
    if isempty(msvc)
        error(['setup_firmware_path: Microsoft Visual C++ 2022 was not found for %s. Install Visual Studio 2022 ', ...
               'with the "Desktop development with C++" workload, then run setup_firmware_path again.'], lang{1});
    end
    mex(['-setup:' msvc(1).MexOpt], lang{1});
end
fprintf('C and C++ compiler set to %s\n', msvc(1).Name);
end

function ok = is_firmware_root(root)
ok = isfolder(root) && isfile(fullfile(root, 'firmware', 'hexray', 'VC', 'src', 'app', 'torque_vectoring', ...
                                       'torque_vectoring_matlab.h'));
end
