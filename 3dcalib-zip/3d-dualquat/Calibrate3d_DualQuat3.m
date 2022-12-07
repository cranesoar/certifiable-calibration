%
% Copyright © 2012, The Massachusetts Institute of Technology. All rights reserved. 
%
% THE LICENSOR EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES CONCERNING THIS 
% SOFIWARE AND DOCUMENTATION, INCLUDING ANY WARRANTIES OF MERCHANTABILITY, 
% FITNESS FOR ANY PARTICULAR PURPOSE, NON- INFRINGEMENT AND WARRANTIES OF 
% PERFORMANCE, AND ANY WARRANTY THAT MIGHT OTHERWISE ARISE FROM COURSE OF 
% DEALING OR USAGE OF TRADE. NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH 
% RESPECT TO THE USE OF THE SOFIWARE OR DOCUMENTATION. Under no circumstances 
% shall the Licensor be liable for incidental, special, indirect, direct or 
% consequential damages, or loss of profits, interruption of business, or 
% related expenses which may arise from use of Software or Documentation, 
% including but not limited to those resulting from defects in Software 
% and/or Documentation, or loss or inaccuracy of data of any kind. 
%
% This software is licensed under the "LIMITED RESEARCH LICENSE (SOURCE
% CODE)" as described in the included LICENSE.txt
%
% Please cite the paper below if you are using this software in your work:
% Brookshire, J.; Teller, S. Extrinsic Calibration from Per-Sensor Egomotion. 
%   Robotics: Science and Systems, 2012.
%
function [estimatedCalibration, optimOutput] = Calibrate3d_DualQuat3(sensor1_expressedIn_prevSensor1, cov1, sensor2_expressedIn_prevSensor2, cov2, calib, initialGuessType, doOutput)

    if ( ~exist('doOutput', 'var') )
        doOutput = 1;
    end
    
    if ( ~exist('sensor1_expressedIn_prevSensor1', 'var') )
        addpath('../DERIVESTsuite'); %for jacobianest
        load('../3d-euler/simdata/calib2-015.mat')  %24.150, 1.89, 1.5, 1.294, 1.164, 15.3  %non PSD: 30.16 30.149 30.237 30.331 30.365 30.387
        calib = run.true.calib;
        cov1 = run.true.cov1;
        cov2 = run.true.cov2;
        sensor1_expressedIn_prevSensor1 = run.observations{88}.sensor1_expressedIn_prevSensor1;
        sensor2_expressedIn_prevSensor2 = run.observations{88}.sensor2_expressedIn_prevSensor2;
        initialGuessType = 'euler';
        doOutput = 1;
    end
    
    if size(sensor1_expressedIn_prevSensor1,2)==6
        % the parameters are in euler states, resampe them
        N = size(sensor1_expressedIn_prevSensor1,1);
        times = [1:N]'*0.1;
        [sensor1_expressedIn_prevSensor1_dq, cov1_dq, ~, meanIters1, cov1_euler, subMatRConds1] = InterpolateStatesAndCovariances_DualQuat(times, sensor1_expressedIn_prevSensor1, cov1, times);
        [sensor2_expressedIn_prevSensor2_dq, cov2_dq, ~, meanIters2, cov2_euler, subMatRConds2] = InterpolateStatesAndCovariances_DualQuat(times, sensor2_expressedIn_prevSensor2, cov2, times);
        calib_dq = EulerStateToDualQuat(calib);
        
        mask = zeros(size(cov1_dq));
        for r = 1:6:size(cov1_dq)
            mask(r:r+5,r:r+5) = 1;
        end
        cov1_dq = mask .* cov1_dq;
        cov2_dq = mask .* cov2_dq;
        
        optimOutput.meanIters1 = meanIters1;
        optimOutput.meanIters2 = meanIters2;
    else
        % the parameters are already in dual quat form
        sensor1_expressedIn_prevSensor1_dq = sensor1_expressedIn_prevSensor1;
        cov1_dq = cov1;  %cov is assumed to be a 12Nx12N matrix in the dual quat lie algebra
        sensor2_expressedIn_prevSensor2_dq = sensor2_expressedIn_prevSensor2;
        cov2_dq = cov2;
        calib_dq = calib;
    end
    
    try
        sqrt_invS = sparse(inv(chol(blkdiag(cov1_dq, cov2_dq), 'lower')));        
    catch err
        estimatedCalibration = nan(1,8);
        optimOutput.error = err.message;
        warning('failed because covariance matrices were not PSD');
        return
    end
    %sqrt_invS = sparse(inv(sqrt_posdef(blkdiag(cov1_dq, cov2_dq))));
    
    annealOutput = [];
    switch lower(initialGuessType)
        case 'zeros'
            calibGuess_dq = [1 0 0 0, 0 0 0 0];
        case 'known'
            calibGuess_dq = calib_dq;
        case 'anneal'
            if doOutput == 1
                options = saoptimset('StallIterLimit', 20, 'TolFun', 1e-3, 'PlotFcns', {@saplotbestx,@saplotbestf,@saplotx,@saplotf});
            else
                options = saoptimset('StallIterLimit', 20, 'TolFun', 1e-3);
            end
            [calibGuess_euler,~,~,annealOutput] = simulannealbnd ( @(x)DoMinimization(x, sensor1_expressedIn_prevSensor1_dq, sensor2_expressedIn_prevSensor2_dq, sqrt_invS), [0 0 0 0 0 0], [-10 -10 -10 -pi -pi/2 -pi], [10 10 10 pi pi/2 pi], options);
            calibGuess_dq = EulerStateToDualQuat(calibGuess_euler);
        case 'euler'
            addpath('..\3d-euler');
            [estimatedCalibration_euler, ~] = Calibrate3d(sensor1_expressedIn_prevSensor1, cov1, sensor2_expressedIn_prevSensor2, cov2, calib, 'zeros');
            calibGuess_dq = EulerStateToDualQuat(estimatedCalibration_euler');
        otherwise
            error('invalid initialGuessType');
    end
    
    % calibrate using the initial guess
    x0 = [ reshape(sensor1_expressedIn_prevSensor1_dq', [], 1); calibGuess_dq' ];
    [x, chisq, ~, J, iterations] = nrlm_dualquat(@(x,iter)ErrorFunc3(x, iter, sensor1_expressedIn_prevSensor1_dq, sensor2_expressedIn_prevSensor2_dq, sqrt_invS), x0, [1000 4 1e-5 doOutput]); 
    
    estimatedCalibration = x(end-7:end)';
    if (estimatedCalibration(1) < 0)
        estimatedCalibration = -estimatedCalibration;
    end
    
    optimOutput.x = x;
    optimOutput.chisq = chisq;
    zeroMaskJ = abs(J)<1e-6;
    J(zeroMaskJ) = 0;
    optimOutput.J = sparse(J);
    optimOutput.iterations = iterations;
    optimOutput.initialGuessType = initialGuessType;
    optimOutput.annealOutput = annealOutput;
        
    if doOutput == 1
        fprintf('               true calibration: %8.5f %8.5f %8.5f %8.5f | %8.5f %8.5f %8.5f %8.5f\n', calib_dq);
        fprintf('          estimated calibration: %8.5f %8.5f %8.5f %8.5f | %8.5f %8.5f %8.5f %8.5f\n', estimatedCalibration);
        fprintf('              calibration error: %8.5f %8.5f %8.5f %8.5f | %8.5f %8.5f %8.5f %8.5f\n', calib_dq - estimatedCalibration);
    end  

function [chisq] = DoMinimization(calib0_euler, sensor1_expressedIn_prevSensor1_dq, sensor2_expressedIn_prevSensor2_dq, sqrt_invS)
    x0 = [ reshape(sensor1_expressedIn_prevSensor1_dq', [], 1); EulerStateToDualQuat(calib0_euler)' ];
    [~, chisq, ~, ~, ~] = nrlm_dualquat(@(x,iter)ErrorFunc3(x, iter, sensor1_expressedIn_prevSensor1_dq, sensor2_expressedIn_prevSensor2_dq, sqrt_invS), x0, [1000 4 1e-5 0]); 
    
function [cost,J] = ErrorFunc3(params, iter, observed_sensor1_expressedIn_prevSensor1_dq, observed_sensor2_expressedIn_prevSensor2_dq, sqrt_invCov)
    
    %extract the values from the parameter vector
    N = size(observed_sensor1_expressedIn_prevSensor1_dq,1);
    estimated_sensor1_expressedIn_prevSensor1_dq = reshape(params(1:end-8), 8, N)';
    estimated_calib_dq = params(end-7:end)';
    
    estimated_sensor2_expressedIn_prevSensor2_dq = TransformDiffDualQuat(estimated_calib_dq, estimated_sensor1_expressedIn_prevSensor1_dq);
    
    d1 = DiffBetweenDualQuat(estimated_sensor1_expressedIn_prevSensor1_dq, observed_sensor1_expressedIn_prevSensor1_dq);
    d2 = DiffBetweenDualQuat(estimated_sensor2_expressedIn_prevSensor2_dq, observed_sensor2_expressedIn_prevSensor2_dq);

    cost = sqrt_invCov' * [d1;d2];
       
    if nargout > 1

        Jg = MakeEvaluatedJgDQLie(estimated_sensor1_expressedIn_prevSensor1_dq, observed_sensor1_expressedIn_prevSensor1_dq, ...
            estimated_sensor2_expressedIn_prevSensor2_dq, observed_sensor2_expressedIn_prevSensor2_dq, estimated_calib_dq);
        J = sqrt_invCov' * full(Jg);
        
%         if iter == 0
%             for r = 1:6:size(J,2)
%                 J(r:r+5,r+3:r+5) = 0;
%             end
%         end
%         x0 = zeros(6*N+6,1);
%         J2 = jacobianest(@(x)funJ(x, params, observed_sensor1_expressedIn_prevSensor1_dq, observed_sensor2_expressedIn_prevSensor2_dq, sqrt_invCov), x0);
        
    end
    
function cost = funJ(expoffset, params, ...
    observed_sensor1_expressedIn_prevSensor1_dq, observed_sensor2_expressedIn_prevSensor2_dq, sqrt_invCov)

    offset_params = UpdateDualQuatWithExpOffset(params, expoffset);   
    cost = ErrorFunc3(offset_params, observed_sensor1_expressedIn_prevSensor1_dq, observed_sensor2_expressedIn_prevSensor2_dq, sqrt_invCov);
             
      
    