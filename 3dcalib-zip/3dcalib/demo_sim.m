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
function demo_sim()

    % This function demonstrates a 6D calibration using simulated data.
    % The optimization is performed in the DQ/Lie algebra.  The CRLB is
    % also calcualted in terms of DQ parameters.  Simulation noise parameters are
    % in Create3DSimulatedData.m.  Each simulation is a sampling from a
    % ground truth mean and covariance via Monte Carlo analysis.  A
    % calibration estimate is created for each sampling and statistics are
    % compared with the ground truth and true CRLB.
    
    addpath('..\3d-dualquat');  
    addpath('..\3d-euler');  
    addpath('..\3dslam');
    
    % simulate some data (noise parameters are in Create3DSimulatedData)
    if ~exist('run3.mat', 'file')
        [r,p,w]=bot_quat_to_roll_pitch_yaw(bot_angle_axis_to_quat(pi/5, [.4 .5 .6]));
        trueCalib_euler = [.06 .03 .04 r p w ];
        trueCalib_dq = EulerStateToDualQuat(trueCalib_euler);
        run = Create3DSimulatedData(trueCalib_euler);
        save run3.mat;
    else
        load run3.mat;
    end
    
    if ~exist('results3.mat', 'file')
        % set the number of estimations to perform
        N = min(200, size(run.observations,2));
        calibrationEstimates = zeros(N,8);
        
        parfor i = 1:N
            [calibrationEstimates(i,:), optimOutput{i}] = Calibrate3d_DualQuat3( ...
                run.observations{i}.sensor1_expressedIn_prevSensor1, run.true.cov1, ...
                run.observations{i}.sensor2_expressedIn_prevSensor2, run.true.cov2, ...
                [0 0 0 0 0 0], 'zeros', 0);
        end
        
        save results3.mat
    else
        load results3.mat
    end

    % calculate the mean and covariance of the estimates
    calibrationMean = mean(calibrationEstimates);
    calibrationCov = cov(calibrationEstimates);
    
    % convert the euler simulation to DQ/Lie states/covariances
    N = size(run.true.sensor1_expressedIn_prevSensor1,1);
    times = [1:N]'*0.1;
    [sensor1_expressedIn_prevSensor1_dq, cov1_dq, ~] = InterpolateStatesAndCovariances_DualQuat(times, run.true.sensor1_expressedIn_prevSensor1, run.true.cov1, times);
    [sensor2_expressedIn_prevSensor2_dq, cov2_dq, ~] = InterpolateStatesAndCovariances_DualQuat(times, run.true.sensor2_expressedIn_prevSensor2, run.true.cov2, times);

    % calculate the CRLB (using the known true values)  
    [crlb] = CRLB3d_DualQuat3(sensor1_expressedIn_prevSensor1_dq, sensor2_expressedIn_prevSensor2_dq, trueCalib_dq, 'cov1_dq', cov1_dq, 'cov2_dq', cov2_dq);
        
    % display some debug output
    fprintf('               true calibration: %8.5f %8.5f %8.5f %8.5f | %8.5f %8.5f %8.5f %8.5f\n', trueCalib_dq);
    fprintf('              mean of estimates: %8.5f %8.5f %8.5f %8.5f | %8.5f %8.5f %8.5f %8.5f\n', calibrationMean);
    fprintf('standard deviation of estimates: %8.5f %8.5f %8.5f %8.5f | %8.5f %8.5f %8.5f %8.5f\n', sqrt(diag(calibrationCov)));
    fprintf('                           crlb: %8.5f %8.5f %8.5f %8.5f | %8.5f %8.5f %8.5f %8.5f\n', sqrt(diag(crlb)));
    
    % display some debug output
    figure(1);
    c = sqrt(diag(crlb));
    for j = 1:8
        titleText{j} = sprintf(' \\mu_C=%0.4f, \\sigma_C=%0.4f', trueCalib_dq(j), c(j));
    end    
    vertlines = [trueCalib_dq' trueCalib_dq'+sqrt(diag(crlb)) trueCalib_dq'-sqrt(diag(crlb))];
    MakeHistograms(calibrationEstimates, {'s0', 's1', 's2', 's3', 's4', 's5', 's6', 's7'}, ...
        sqrt(size(calibrationEstimates,1)), 2, 4, 'showvar', 'showmean', ...
        'VertLines', vertlines, 'titletexts', titleText, 'FontSize', 7);
        