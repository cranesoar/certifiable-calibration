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
function demo()

    % This function demonstrates a 6D calibration using data from
    % two Xtion's and relative motion data from the KinectFusion algorithm.
    % The raw .oni files are upsidedown_sensor*.oni.  These files were
    % processed using KinectFusion and the resulting camera estimates are
    % in upsidedown_sensor*.oni_poses.txt for convenience.  The
    % KinectFusion does not readily provide uncertainty information and we
    % assume a constant covariance matrix in the DQ state space.  As a
    % result, the interpolation method is not used here (see demo_sim.m for
    % an example).

    addpath('..\3d-dualquat');  
    addpath('..\3d-euler');  
    addpath('..\3dslam');

    % load data from kinfu
    cam1_raw = dlmread('..\3d-experiment\logs\2012-05-13\upsidedown_sensor1.oni_poses.txt');
    cam2_raw = dlmread('..\3d-experiment\logs\2012-05-13\upsidedown_sensor2.oni_poses.txt');
    cam1_abs_euler = [cam1_raw(:,2:4) QuatToRPY([cam1_raw(:,8) cam1_raw(:,5:7)])];    
    cam2_abs_euler = [cam2_raw(:,2:4) QuatToRPY([cam2_raw(:,8) cam2_raw(:,5:7)])];  
    
    % known calibration (not used during optimization, only for comparison)
    calib_euler = [-0.04 0.025 0 -4/180*pi 0 pi];
    calib_dq = EulerStateToDualQuat(calib_euler);
       
    % generate the timestamps for the kinfu data from external timing
    sync1 = [4515 4559 4701 4978 5030 5217 5373 5508 5657 5784 5986 6202 6322 6449];
    sync2 = [4567 4619 4776 5075 5149 5358 5530 5680 5836 5978 6195 6419 6568 6725];
    
    cam1_abs_dq = EulerStateToDualQuat(cam1_abs_euler(sync1(1):sync1(end)-1,:));
    cam2_abs_dq = EulerStateToDualQuat(cam2_abs_euler(sync2(1):sync2(end)-1,:));
    
    fps = 30;
    times1 = zeros(max(sync1) - min(sync1) - 1,1);
    times2 = zeros(max(sync2) - min(sync2) - 1,1);
    for i = 1:length(sync1)-1
        this_index1 = [sync1(i):sync1(i+1)-1] - sync1(1) + 1;
        this_index2 = [sync2(i):sync2(i+1)-1] - sync2(1) + 1;
        
        this_times1 = this_index1 / fps;
        this_times2 = linspace(min(this_times1), max(this_times1), length(this_index2));
        
        times1(this_index1) = this_times1;
        times2(this_index2) = this_times2;
        
    end
       
    % downsample the data to 200 evenly spaced data points
    new_times1 = linspace(min(times1), max(times1), 200);
    new_times2 = linspace(min(times2), max(times2), 200);
    new_cam1_abs_dq = ResampleAbsStatesDualQuat(times1, cam1_abs_dq, new_times1');    
    new_cam2_abs_dq = ResampleAbsStatesDualQuat(times2, cam2_abs_dq, new_times2'); 
    diff_cam1_dq = MakeRelStatesDualQuat(new_cam1_abs_dq);
    diff_cam2_dq = MakeRelStatesDualQuat(new_cam2_abs_dq);    
    
    % assign an equally weighted covariance for all measurements in the lie algebra
    cov1_dq = eye(6*size(diff_cam1_dq,1)) * 1e-5;
    cov2_dq = eye(6*size(diff_cam1_dq,1)) * 1e-5;

    % perform the optimization
    [estimatedCalibration_dq, optimOutput] = Calibrate3d_DualQuat3(diff_cam1_dq, cov1_dq, diff_cam2_dq, cov2_dq, [], 'zeros', 0);
    estimatedCalibration_euler = DualQuatToEulerState(estimatedCalibration_dq);
    
    % display some results
    fprintf('               true calibration: %8.5f %8.5f %8.5f %8.5f | %8.5f %8.5f %8.5f %8.5f\n', calib_dq);
    fprintf('          estimated calibration: %8.5f %8.5f %8.5f %8.5f | %8.5f %8.5f %8.5f %8.5f\n', estimatedCalibration_dq);
    fprintf('              calibration error: %8.5f %8.5f %8.5f %8.5f | %8.5f %8.5f %8.5f %8.5f\n', calib_dq - estimatedCalibration_dq);
    
    fprintf('               true calibration: %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f\n', calib_euler);
    fprintf('          estimated calibration: %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f\n', estimatedCalibration_euler);
    fprintf('              calibration error: %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f\n', calib_euler - estimatedCalibration_euler);