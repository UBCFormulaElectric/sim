function root = firmware_root()
% FIRMWARE_ROOT  Path of this user's Consolidated-Firmware checkout, as stored by setup_firmware_path.
%
% The model callbacks run  FE_FIRMWARE_ROOT = firmware_root();  so the C Function blocks can use $FE_FIRMWARE_ROOT$.

if ~ispref('FormulaElectricSim', 'FirmwareRoot')
    error('firmware_root: the firmware path is not set. Run setup_firmware_path once.');
end
root = getpref('FormulaElectricSim', 'FirmwareRoot');
if ~isfolder(root)
    error('firmware_root: "%s" does not exist. Run setup_firmware_path again.', root);
end
end
