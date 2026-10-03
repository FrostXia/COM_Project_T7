function [omega,memory,info] = p7_controller_adapter(qHat,ref,config,memory)
% P7_CONTROLLER_ADAPTER Translate platform pose inputs for shared controllers.
if strcmp(config.controller,'mpc')
    [omega,memory,info]=p7_external_controller(qHat,ref,config,memory);
    return
end
e=p7utils.error_state(qHat,ref.q(:,1));
[omega,memory,info]=project7_feedback_step(e,ref.w(1),config,memory);
end
