classdef driver_select_callback

    methods(Static)

        % Use the code browser on the left to add the callbacks.


        function driver_select(callbackContext)
            blk = gcb;
            val = get_param(blk, 'driver_select');
            set_param(blk, 'LabelModeActiveChoice', val);
        end

    end
end