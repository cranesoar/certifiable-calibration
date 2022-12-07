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
function [estimatedCalibration, optimOutput] = Calibrate3d(sensor1_expressedIn_prevSensor1, cov1, sensor2_expressedIn_prevSensor2, cov2, calib, initialGuessType, doOutput)

    addpath('../3dslam');
    
    if ( ~exist('doOutput', 'var') )
        doOutput = 1;
    end
    
    if ( ~exist('sensor1_expressedIn_prevSensor1', 'var') )
        load('simdata/calib2-001.mat')
        calib = run.true.calib;
        cov1 = run.true.cov1;
        cov2 = run.true.cov2;
        sensor1_expressedIn_prevSensor1 = run.observations{1}.sensor1_expressedIn_prevSensor1;
        sensor2_expressedIn_prevSensor2 = run.observations{1}.sensor2_expressedIn_prevSensor2;
        doOutput = 1;
        initialGuessType = 'zeros';
    end

    N = size(sensor1_expressedIn_prevSensor1,1);
    invS = MakeInvCov(cov1,cov2);
    sqrt_invS = sparse(chol(invS,'lower'));
    
    % perform the optimization, the state is x = [v1x v1y v1z v1r v1p v1w ... vNx vNy vNz vNr vNp vNw xs ys zs rs ps ws]    
    switch lower(initialGuessType)        
        case 'zeros' 
            x0 = [ reshape(sensor1_expressedIn_prevSensor1',6*N,1); [0 0 0 0 0 0]' ];
        case 'known'
            x0 = [ reshape(sensor1_expressedIn_prevSensor1',6*N,1); calib' ];
        case 'nearknown'
            calibGuessError = (rand(6,1)-0.5).*2.*[0.5 0.5 0.5 20/180*pi 20/180*pi 20/180*pi]';
            x0 = [ reshape(sensor1_expressedIn_prevSensor1',6*N,1); calib' + calibGuessError];
        case 'guess'
            calibGuess = InitialGuess(sensor1_expressedIn_prevSensor1, sensor2_expressedIn_prevSensor2);
            x0 = [ reshape(sensor1_expressedIn_prevSensor1',6*N,1); calibGuess' ];
        case 'anneal'
            if doOutput == 1
                options = saoptimset('StallIterLimit', 5, 'TolFun', 1e-3, 'PlotFcns', {@saplotbestx,@saplotbestf,@saplotx,@saplotf});
            else
                options = saoptimset('StallIterLimit', 5, 'TolFun', 1e-3);
            end
            calibGuess = simulannealbnd ( @(x)DoMinimization(x, sensor1_expressedIn_prevSensor1, sensor2_expressedIn_prevSensor2, sqrt_invS), [0 0 0 0 0 0], [-10 -10 -10 -pi -pi/2 -pi], [10 10 10 pi pi/2 pi], options);
            x0 = [ reshape(sensor1_expressedIn_prevSensor1',6*N,1); calibGuess' ];
        otherwise
            error('invalid initialGuessType');
    end
    if doOutput == 1
        options = optimset('Jacobian', 'on', 'Display', 'on', 'PlotFcns', {@optimplotstepsize, @optimplotresnorm, @plotcalib, @plotresiduals} );
    else
        options = optimset('Jacobian', 'on', 'Display', 'off');
    end

    [x,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin ( @(x)ErrorFunc3(x, sensor1_expressedIn_prevSensor1, sensor2_expressedIn_prevSensor2, sqrt_invS), ...
        x0, [], [], options);    
    
    optimOutput.x = x;
    optimOutput.x0 = x0;
    optimOutput.resnorm = resnorm;
    optimOutput.residual = residual;
    optimOutput.exitflag = exitflag;
    optimOutput.output = output;
    optimOutput.lambda = lambda;
    optimOutput.jacobian = jacobian;
    
    estimatedCalibration = x(end-5:end);

    if doOutput == 1
        fprintf('               true calibration: %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f\n', calib);
        fprintf('          estimated calibration: %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f\n', estimatedCalibration);
        fprintf('              calibration error: %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f\n', calib - estimatedCalibration');
    end
    
function [resnorm] = DoMinimization(calib0, sensor1_expressedIn_prevSensor1, sensor2_expressedIn_prevSensor2, sqrt_invS)
    N = size(sensor1_expressedIn_prevSensor1,1);
    x0 = [ reshape(sensor1_expressedIn_prevSensor1',6*N,1); calib0' ];
    options = optimset('Jacobian', 'on', 'Display', 'off', 'MaxFunEvals', 30);
    [x,resnorm] = lsqnonlin ( @(x)ErrorFunc3(x, sensor1_expressedIn_prevSensor1, sensor2_expressedIn_prevSensor2, sqrt_invS), ...
        x0, [], [], options); 


function [cost,J] = ErrorFunc3(params, observed_robot_expressedIn_prevRobot, observed_sensor_expressedIn_prevSensor, sqrt_invCov)

    %extract the values from the parameter vector
    N = size(observed_robot_expressedIn_prevRobot,1);
    true_robot_expressedIn_prevRobot = reshape(params(1:end-6), 6, N)';
    sensor_expressedIn_robot = GetHomoTransform(params(end-5:end));
       
    %calculate where the sensor would be if the robot were at true_robot_expressedIn_prevRobot
    expected_sensor_expressedIn_prevSensor = TransformDiffHomo(sensor_expressedIn_robot, true_robot_expressedIn_prevRobot);
    
    %find the error between our guesses and the observed data
    robotErr = true_robot_expressedIn_prevRobot - observed_robot_expressedIn_prevRobot;
    sensorErr = expected_sensor_expressedIn_prevSensor - observed_sensor_expressedIn_prevSensor;
    robotErr(:,4:6)  = asin(sin(robotErr(:,4:6)));
    sensorErr(:,4:6) = asin(sin(sensorErr(:,4:6)));
    
%     robotErr(:,1:3) = true_robot_expressedIn_prevRobot(:,1:3) - observed_robot_expressedIn_prevRobot(:,1:3);
%     robotErr(:,4:6) = QuatToRPY(RPYToQuat(true_robot_expressedIn_prevRobot(:,4:6))) - QuatToRPY(RPYToQuat(observed_robot_expressedIn_prevRobot(:,4:6)));
%     
%     sensorErr(:,1:3) = expected_sensor_expressedIn_prevSensor(:,1:3) - observed_sensor_expressedIn_prevSensor(:,1:3);
%     sensorErr(:,4:6) = QuatToRPY(RPYToQuat(expected_sensor_expressedIn_prevSensor(:,4:6))) - QuatToRPY(RPYToQuat(observed_sensor_expressedIn_prevSensor(:,4:6)));
    
    
    % dx = estimate - mean, S = covariance
    % cost = dx' * inv(S) * dx 
    % Let B*B' = inv(S) => B = chol(inv(S), 'lower');
    % cost = dx' * B * B' * dx
    % cost = (dx' * B) * (B' * dx)
    % Let h = B'*dx
    % cost = h' * h
    % we then note that lsqnonlin wants h as input.  B = sqrt_invCov
    % note that you cannot do err = sqrt(dx .* (invCov * dx)), b/c this may produce imaginary numbers
    
    dx = [reshape(robotErr', numel(robotErr), 1); reshape(sensorErr', numel(sensorErr), 1)];
    cost = sqrt_invCov' * dx;
    
    if nargout > 1
        analJ = MakeEvaluatedJg(true_robot_expressedIn_prevRobot, params(end-5:end));
        J = sqrt_invCov' * analJ;
        
%         alpha = J' * J;
%         beta = J' * dx;
%         da = alpha \ -beta
%         figure(10);
%         subplot(1,2,1);
%         imagesc(reshape(da,6,N+1)');
%         subplot(1,2,2);
%         imagesc([robotErr sensorErr])
%         %subplot(1,2,1);
%         %plot(da(1:end-6));
%         %subplot(1,2,2);
%         %plot(da(end-5:end));
%         %grid on;
    end
    
    if ( any(isnan(cost)) || any(isinf(cost)) )
        keyboard
    end
    
function stop = plotcalib(x,optimValues,state)

    stop = false;
    if ~strcmp(state, 'iter')
        return;
    end
    
    calib =  x(end-5:end)';

    global storedData;
    if optimValues.iteration == 0
        storedData = [];
    end
    storedData = [storedData; calib];
    plot(storedData);
    grid on;
    %legend('x','y','z','r','p','w', 'Location', 'BestOutside');
    
function stop = plotresiduals(x,optimValues,state)

    stop = false;
    if ~strcmp(state, 'iter')
        return;
    end

    global storedDataResiduals;
    if optimValues.iteration == 0
        storedDataResiduals = [];
    end
    storedDataResiduals = [storedDataResiduals; optimValues.residual'];
    if size(storedDataResiduals,1)>1
        imagesc(diff(storedDataResiduals,1,1));
    end
    

    