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
function run = Create3DSimulatedData(calib, matFilename)

    addpath('../3dslam');

    if ( ~exist('calib', 'var') )
        [r,p,w]=bot_quat_to_roll_pitch_yaw(bot_angle_axis_to_quat(pi/5, [.4 .5 .6]));
        calib = [.02 .03 .04 r p w ];
    end
    
    sensor2_expressedIn_sensor1 = GetHomoTransform(calib);
    
    th = 0:0.05:pi;
    r = 1*cos(3*th);
    
    x = r.*cos(th);
    y = r.*sin(th);
    z = x.^2 + y.^3;    
    %z = -0.5*x.^2 + -0.5*y.^2;
    
    xaxis = [diff(x) x(1)-x(end); diff(y) y(1)-y(end); diff(z) z(1)-z(end)];
    yaxis = zeros(size(xaxis));
    zaxis = zeros(size(xaxis));
    normal = zeros(size(xaxis));
    
    dz_dx = 2*x;
    dz_dy = 3*y.^2;
    %dz_dx = -1 * x;
    %dz_dy = -1 * y;
    for i = 1:length(x)
        normal(:,i) = cross( [1 0 dz_dx(i)], [0 1 dz_dy(i) ] );
        normal(:,i) = normal(:,i) / norm(normal(:,i));
        xaxis(:,i) = xaxis(:,i) /  norm(xaxis(:,i));        
        yaxis(:,i) = cross ( normal(:,i), xaxis(:,i) );
        yaxis(:,i) = yaxis(:,i) /  norm(yaxis(:,i));
        zaxis(:,i) = cross ( xaxis(:,i), yaxis(:,i) );
        zaxis(:,i) = zaxis(:,i) /  norm(zaxis(:,i));
    end
    
    figure(1);
    plot3(x,y,z);
    hold on;
%     quiver3(x,y,z, zaxis(1,:), zaxis(2,:), zaxis(3,:), 0);
%     quiver3(x,y,z, xaxis(1,:), xaxis(2,:), xaxis(3,:), 0);
    hold off;
    axis equal;
    grid on;
    
    sensor1_expressedIn_world = zeros(length(x),6);
    sensor2_expressedIn_world = zeros(length(x),6);
    T = eye(4);
    for i = 1:length(x)
        T(1:3,1:3) = [xaxis(:,i) yaxis(:,i) zaxis(:,i)]; % * bot_quat_to_matrix(bot_angle_axis_to_quat(pi/3, [1 1 1]));
        T(1:3,4) = [x(i), y(i), z(i)];
        hold on;
        DrawAxis(T, 0.1, 'r', 'b', 'k');
        DrawAxis(T * sensor2_expressedIn_sensor1, 0.1, 'g', 'y', 'c');
        sensor1_expressedIn_world(i,:) = GetState(T);
        sensor2_expressedIn_world(i,:) = GetState(T * sensor2_expressedIn_sensor1);
    end
    hold off;

    sensor1_expressedIn_prevSensor1 = MakeRelState(sensor1_expressedIn_world);
    sensor2_expressedIn_prevSensor2 = TransformDiffHomo( sensor2_expressedIn_sensor1, sensor1_expressedIn_prevSensor1 );
    
    sensor2Initial_expressedIn_world = GetState(GetHomoTransform(sensor1_expressedIn_world(1,:)) * sensor2_expressedIn_sensor1);
    [~,sensor1check_expressedIn_world] = MakeAbsStates(sensor1_expressedIn_world(1,:), [], sensor1_expressedIn_prevSensor1, []);
    [~,sensor2check_expressedIn_world] = MakeAbsStates(sensor2Initial_expressedIn_world, [], sensor2_expressedIn_prevSensor2, []);
    
%     figure(2);
%     plot3(x,y,z);
%     axis equal;
%     grid on;
%     hold on;
%     for i = 1:size(sensor1check_expressedIn_world,1)
%         T1 = GetHomoTransform(sensor1check_expressedIn_world(i,:));
%         DrawAxis(T1, 0.1, 'r', 'b', 'k');
%         T2 = GetHomoTransform(sensor2check_expressedIn_world(i,:));
%         DrawAxis(T2, 0.1, 'g', 'y', 'c');
%     end
%     hold off;
    
    % generate random covariances
    %RandStream.setDefaultStream(RandStream('mt19937ar','seed',0)); % set the random seed so we always get the same random values
    
    std_sensor1_per_unit = [0.05 0.06 0.07 5/180*pi 3/180*pi 7/180*pi]*4;
    std_sensor2_per_unit = [0.07 0.04 0.03 2/180*pi 8/180*pi 5/180*pi]*4;
    
    cov1 = zeros(size(sensor1_expressedIn_prevSensor1,1), 36);
    cov2 = zeros(size(sensor1_expressedIn_prevSensor1,1), 36);
    for i = 1:size(sensor1_expressedIn_prevSensor1,1)
        
        motion1 = abs(sensor1_expressedIn_prevSensor1(i,:));       
        motion2 = abs(sensor2_expressedIn_prevSensor2(i,:));
        
        S1 = diag((motion1 .* std_sensor1_per_unit).^2+(5e-2)^2) + randn(6)*(1e-3)^2;
        S2 = diag((motion2 .* std_sensor2_per_unit).^2+(5e-2)^2) + randn(6)*(1e-3)^2;
        S1 = triu(S1)+triu(S1,1)';
        S2 = triu(S2)+triu(S2,1)';
        cov1(i,:) = reshape(S1,1,[]);
        cov2(i,:) = reshape(S2,1,[]);
    end
    
    numberOfRuns = 400;
    run.true.sensor1_expressedIn_prevSensor1 = sensor1_expressedIn_prevSensor1;
    run.true.sensor2_expressedIn_prevSensor2 = sensor2_expressedIn_prevSensor2;
    run.true.sensor1_expressedIn_world = sensor1_expressedIn_world;
    run.true.sensor2_expressedIn_world = sensor2_expressedIn_world;
    run.true.cov1 = cov1;
    run.true.cov2 = cov2;
    run.true.calib = calib;
    for i = 1:numberOfRuns
        run.observations{i}.sensor1_expressedIn_prevSensor1 = SampleVelocitiesWithCovariance(run.true.sensor1_expressedIn_prevSensor1, run.true.cov1);
        run.observations{i}.sensor2_expressedIn_prevSensor2 = SampleVelocitiesWithCovariance(run.true.sensor2_expressedIn_prevSensor2, run.true.cov2);
        
%         figure(100+i);
%         DisplayRun(run,0);
%         hold on;
%         DisplayRun(run,i);
%         hold off;
    end
        
    if ( exist('matFilename', 'var') )
        save(matFilename, 'run');
    end

function observed_pos_expressedIn_prevPos = SampleVelocitiesWithCovariance(true_pos_expressedIn_prevPos, true_pos_expressedIn_prevPos_cov)

    observed_pos_expressedIn_prevPos = zeros(size(true_pos_expressedIn_prevPos));
    for i = 1:size(observed_pos_expressedIn_prevPos,1)
        c = reshape(true_pos_expressedIn_prevPos_cov(i,:),6,6);
        observed_pos_expressedIn_prevPos(i,:) = mgd(1, 6, true_pos_expressedIn_prevPos(i,:), c);        
    end

    
