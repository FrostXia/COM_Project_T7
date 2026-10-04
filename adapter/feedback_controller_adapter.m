function [omega,memory,info] = feedback_controller_adapter(qHat,ref,config,memory)
% FEEDBACK_CONTROLLER_ADAPTER Translate platform pose inputs for shared controllers.
if strcmp(config.controller,'mpc')
    [omega,memory,info]=external_controller(qHat,ref,config,memory);
    return
end
e=tracking_error_state(qHat,ref.q(:,1));
[omega,memory,info]=feedback_control_step(e,ref.w(1),config,memory);
end
