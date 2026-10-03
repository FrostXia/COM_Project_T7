function m=noise_metrics(out)
% NOISE_METRICS True tracking quality, not just filtered-curve smoothness.
L=out.log; idx=L.t>=out.config.analysisStart;
m=struct('positionRMSE',NaN,'headingRMSE',NaN,'meanError',nan(3,1), ...
    'variance',nan(3,1),'measurementVariance',nan(3,1), ...
    'controlNoiseVariance',NaN,'commandEnergy',NaN,'correctionEnergy',NaN, ...
    'peakPhysicalOmega',NaN,'meanSolveSeconds',NaN,'maxSolveSeconds',NaN, ...
    'saturationFraction',NaN,'sampleCount',sum(idx), ...
    'eligibleForComparison',strcmp(out.status,'completed'));
if any(idx)
    E=L.errorTrue(:,idx);
    m.positionRMSE=sqrt(mean(sum(E(1:2,:).^2,1)));
    m.headingRMSE=sqrt(mean(E(3,:).^2));
    m.meanError=mean(E,2);
    if sum(idx)>1
        m.variance=var(E,0,2);
        m.measurementVariance=var(L.measurementNoise(:,idx),0,2);
    end
end
if ~isempty(L.omegaCommand)
    m.commandEnergy=sum(L.omegaCommand.^2)*out.config.Ts;
    m.correctionEnergy=sum((L.omegaCommand-L.omegaReference).^2)*out.config.Ts;
    m.peakPhysicalOmega=max(abs(L.omegaPhysical));
    m.meanSolveSeconds=mean(L.solveSeconds);
    m.maxSolveSeconds=max(L.solveSeconds);
    m.saturationFraction=mean(abs(L.omegaRequested-L.omegaCommand)>1e-9);
    if numel(L.controlNoise)>1, m.controlNoiseVariance=var(L.controlNoise,0,2); end
end
end
