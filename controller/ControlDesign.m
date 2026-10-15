classdef ControlDesign
    % CONTROLDESIGN Related operations grouped as static methods.
    methods (Static)

        function [K,Klqr,design] = design_feedback_gains(p,verbose,parametersOnly)
        % DESIGN_FEEDBACK_GAINS Fixed nominal SF/LQR for edot = A*e + B*u + d.
        % [K,Klqr,design] = ControlDesign.design_feedback_gains
        % Optional p overrides are for controlled parameter experiments.
        %% 1. Screenshot parameters and explicit override validation
        base=struct('v0',0.5,'vr',0.5,'wrNom',pi/4,'poles',[-1 -2 -4],'Q',eye(3),'R',1);
        if nargin<1 || isempty(p), p=base; else
            assert(isstruct(p) && isscalar(p) && all(isfield(p,fieldnames(base))), ...
                'Trajectory:Parameter','Supply all design fields from SimulationConfig.default_config.design.');
            names=fieldnames(p);
            assert(all(ismember(names,fieldnames(base))),'Trajectory:Parameter','Unknown design field.');
            for i=1:numel(names), base.(names{i})=p.(names{i}); end
            p=base;
        end
        if nargin<2, verbose=true; end
        if nargin<3, parametersOnly=false; end
        validateattributes(p.v0,{'double'},{'scalar','real','finite','positive'});
        validateattributes(p.vr,{'double'},{'scalar','real','finite','positive'});
        validateattributes(p.wrNom,{'double'},{'scalar','real','finite','nonzero'});
        validateattributes(p.poles,{'double'},{'vector','numel',3,'finite'});
        assert(all(real(p.poles)<0),'Trajectory:Poles','Continuous poles must have negative real parts.');
        assert(isequal(size(p.Q),[3 3]) && isreal(p.Q) && all(isfinite(p.Q(:))) && ...
            norm(p.Q-p.Q','fro')<1e-10 && min(eig(p.Q))>0,'Trajectory:Q','This prototype requires Q positive definite.');
        validateattributes(p.R,{'double'},{'scalar','real','finite','positive'});
        %% 2. Nonlinear error convention u = omega - omegaReference
        A=[0 p.wrNom 0;-p.wrNom 0 p.vr;0 0 0];
        B=[0;0;-1];
        d=[p.vr-p.v0;0;0];
        % This is a local affine expansion about e=0,u=0, not an equilibrium
        % when vr~=v0. Gains use (A,B); d is already present in the nonlinear
        % plant/reference motion and must not be added a second time to omega.
        Co=[B A*B A*A*B];
        assert(rank(Co)==3,'Trajectory:Controllability','Nominal model is not fully controllable.');
        design=struct('parameters',p,'A',A,'B',B,'d',d,'controllability',Co);
        K=[]; Klqr=[];
        if parametersOnly, return; end
        assert(exist('place','file')~=0 && exist('lqr','file')~=0, ...
            'Trajectory:Toolbox','Control System Toolbox is required for place and lqr.');
        %% 3. Pole placement, as in Assignment 2 Q4
        K=place(A,B,p.poles);
        Acl=A-B*K;
        design.K=K; design.Acl=Acl; design.polesSF=eig(Acl);
        target=poly(p.poles); actual=poly(Acl);
        assert(norm(actual-target)<1e-7*max(1,norm(target)), ...
            'Trajectory:Verification','Pole placement verification failed.');
        %% 4. Continuous LQR and Riccati residual
        [Klqr,Plqr,Elqr]=lqr(A,B,p.Q,p.R);
        Acl_lqr=A-B*Klqr;
        design.Klqr=Klqr; design.Plqr=Plqr;
        design.Acl_lqr=Acl_lqr; design.polesLQR=Elqr;
        design.careResidual=norm(A'*Plqr+Plqr*A-Plqr*B*(p.R\(B'*Plqr))+p.Q,'fro');
        assert(all(real(eig(Acl_lqr))<0),'Trajectory:Verification','Nominal LQR is not stable.');
        %% 5. Print matrices and poles; keep full precision in returned variables
        if verbose
            fprintf('\nTrajectory: robot v0=%.6g m/s, reference vr=%.6g m/s, nominal Wr=%.6g rad/s\n',p.v0,p.vr,p.wrNom);
            disp('A ='); disp(A); disp('B ='); disp(B); disp('d ='); disp(d);
            disp('K ='); disp(K); disp('Acl = A-B*K ='); disp(Acl);
            disp('eig(Acl) ='); disp(design.polesSF);
            disp('Q ='); disp(p.Q); disp('R ='); disp(p.R);
            disp('Klqr ='); disp(Klqr); disp('Plqr ='); disp(Plqr);
            disp('Acl_lqr = A-B*Klqr ='); disp(Acl_lqr);
            disp('eig(Acl_lqr) ='); disp(Elqr);
            fprintf('CARE residual: %.3g\n',design.careResidual);
        end
        end

        function [omega,memory,info] = feedback_control_step(e,wr,config,memory)
        % FEEDBACK_CONTROL_STEP Fixed nominal SF/LQR design with time-varying feedforward.
        if isempty(memory) || ~isfield(memory,'design')
            parameters=config.design;
            [~,~,d]=ControlDesign.design_feedback_gains(parameters,false);
            memory=struct('design',d);
        end
        switch config.controller
            case 'sf', K=memory.design.K;
            case 'lqr', K=memory.design.Klqr;
            otherwise, error('Trajectory:Controller','Expected sf or lqr.');
        end
        % Retain the same feedback law. The speed-mismatch drift is generated
        % by the separate robot/reference motion, not an extra angular command.
        omega=wr-K*e;
        info=struct('ok',isfinite(omega),'message','Fixed nominal feedback with reference feedforward.', ...
            'predictedQ',[],'constraintResidual',NaN);
        end

        function c=prepare_controller_design(path,c)
        % PREPARE_CONTROLLER_DESIGN Resolve one fixed SF/LQR design for a complete run.
        if ~ismember(c.controller,{'sf','lqr'}), return; end
        vr=SimulationConfig.reference_speed(c);
        a=TrajectoryPath.compute_nominal_turn(path,vr,c.nominalTurn);
        c.design.v0=c.v0; c.design.vr=vr; c.design.wrNom=a.radiansPerSecond; a.source='automatic';
        if ~isempty(c.nominalOverride)
            validateattributes(c.nominalOverride,{'double'},{'scalar','real','finite','nonzero'});
            c.design.wrNom=c.nominalOverride; a.source='manual';
        end
        a.selectedRadiansPerSecond=c.design.wrNom;
        c.nominalReference=a;
        end

        function e = controller_tracking_error(ref)
        % CONTROLLER_TRACKING_ERROR Use the session's common measured tracking error.
        assert(isstruct(ref) && isfield(ref,'measuredTrackingError'), ...
            'Trajectory:Measurement','Reference must include measuredTrackingError.');
        e=ref.measuredTrackingError;
        validateattributes(e,{'double'},{'size',[3 1],'real','finite'});
        end

        function [omega,memory,info] = feedback_controller_adapter(qHat,ref,config,memory)
        % FEEDBACK_CONTROLLER_ADAPTER Dispatch the current measured-error interface.
        if strcmp(config.controller,'mpc')
            [omega,memory,info]=ControlDesign.mpc_controller_adapter(qHat,ref,config,memory);
            return
        end
        e=ControlDesign.controller_tracking_error(ref);
        [omega,memory,info]=ControlDesign.feedback_control_step(e,ref.w(1),config,memory);
        end

        function [omega,memory,info]=mpc_controller_adapter(qHat,ref,config,memory)
        % MPC_CONTROLLER_ADAPTER Map the platform pose interface to nonlinear error MPC.
        %% 1. Map public platform configuration into the shared MPC configuration
        c=config;
        c.vr=SimulationConfig.reference_speed(config);
        assert(isfield(config,'algorithm') && isstruct(config.algorithm), ...
            'Trajectory:Algorithm','Supply algorithm settings from SimulationConfig.default_config.');
        defaults=SimulationConfig.default_config;
        expected=fieldnames(defaults.algorithm);
        assert(isequal(sort(fieldnames(config.algorithm)),sort(expected)), ...
            'Trajectory:Algorithm','Supply the complete current algorithm structure.');
        c.mpc=config.algorithm;
        validateattributes(qHat,{'double'},{'size',[3 1],'finite','real'});
        assert(isequal(size(ref.q),[3 c.N+1]),'Trajectory:Preview','ref.q must be 3-by-(N+1).');
        %% 2. Use the common measured error and explicit reference-speed sequence
        e0=ControlDesign.controller_tracking_error(ref);
        assert(isfield(ref,'vr'),'Trajectory:Preview','Reference must include its vr sequence.');
        vr=ref.vr;
        [omega,memory,info]=solve_nonlinear_mpc(e0,ref.w,vr,c,memory);
        %% 3. Optional predicted global pose for the platform's diagnostics
        info.predictedQ=[];
        if info.ok
            E=info.predictedError; qr=ref.q;
            th=qr(3,:)-E(3,:);
            xx=qr(1,:)-cos(th).*E(1,:)+sin(th).*E(2,:);
            yy=qr(2,:)-sin(th).*E(1,:)-cos(th).*E(2,:);
            info.predictedQ=[xx;yy;th];
        end
        end
    end
end
