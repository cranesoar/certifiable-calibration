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
function x0 = InitialGuess(sensor1_expressedIn_prevSensor1, sensor2_expressedIn_prevSensor2)
   
    if ( ~exist('sensor1_expressedIn_prevSensor1', 'var') )
        load('simdata/calib2-015.mat')
        %calib = run.true.calib;
        sensor1_expressedIn_prevSensor1 = run.observations{50}.sensor1_expressedIn_prevSensor1;
        sensor2_expressedIn_prevSensor2 = run.observations{50}.sensor2_expressedIn_prevSensor2;
    end

    options = optimset('Display', 'off', 'Algorithm', 'levenberg-marquardt');
    guesses = zeros(size(sensor1_expressedIn_prevSensor1,1)-1,7);
    for i = 1:size(sensor1_expressedIn_prevSensor1,1)-1
        %sensor2_expressedIn_sensor1 = GetHomoTransform(calib);
        %expected_sensor2_expressedIn_prevSensor2 = TransformDiffHomo(sensor2_expressedIn_sensor1, sensor1_expressedIn_prevSensor1(i:i+1,:));
        [x,~,exitflag] = fsolve(@(x)System(x, sensor1_expressedIn_prevSensor1(i:i+1,:), sensor2_expressedIn_prevSensor2(i:i+1,:)), [0 0 0], options);
        x = GetFullState(x, sensor1_expressedIn_prevSensor1(i:i+1,:), sensor2_expressedIn_prevSensor2(i:i+1,:));
        guesses(i,:) = [x exitflag];
    end
    
    rpy=guesses(guesses(:,end)>0,4:6);
    q=mean(RPYToQuat(rpy));
    q(1) = sqrt(1- sum(q(2:4).^2));
    x0 = [median(guesses(guesses(:,end)>0,1:3)) QuatToRPY(q)];

    
function x = GetFullState(x, sensor1_expressedIn_prevSensor1, sensor2_expressedIn_prevSensor2)
    rs = x(1);
    ps = x(2);
    ws = x(3);
    
    r2Rr1x = sensor1_expressedIn_prevSensor1(1,1);
    r2Rr1y = sensor1_expressedIn_prevSensor1(1,2);
    r2Rr1z = sensor1_expressedIn_prevSensor1(1,3);
    r2Rr1r = sensor1_expressedIn_prevSensor1(1,4);
    r2Rr1p = sensor1_expressedIn_prevSensor1(1,5);
    r2Rr1w = sensor1_expressedIn_prevSensor1(1,6);
    
    r3Rr2x = sensor1_expressedIn_prevSensor1(2,1);
    r3Rr2y = sensor1_expressedIn_prevSensor1(2,2);
    r3Rr2z = sensor1_expressedIn_prevSensor1(2,3);
    r3Rr2r = sensor1_expressedIn_prevSensor1(2,4);
    r3Rr2p = sensor1_expressedIn_prevSensor1(2,5);
    r3Rr2w = sensor1_expressedIn_prevSensor1(2,6);
    
    s2Rs1x = sensor2_expressedIn_prevSensor2(1,1);
    s2Rs1y = sensor2_expressedIn_prevSensor2(1,2);
    s3Rs2z = sensor2_expressedIn_prevSensor2(2,3);
    
    [y]=GetXsYsZs(rs,ps,ws, r2Rr1x,r2Rr1y,r2Rr1z,r2Rr1r,r2Rr1p,r2Rr1w, r3Rr2x,r3Rr2y,r3Rr2z,r3Rr2r,r3Rr2p,r3Rr2w, s2Rs1x,s2Rs1y,s3Rs2z);
    x = [y x];
    
function F = System(x, sensor1_expressedIn_prevSensor1, sensor2_expressedIn_prevSensor2)

    x = GetFullState(x,sensor1_expressedIn_prevSensor1,sensor2_expressedIn_prevSensor2);
    
    sensor2_expressedIn_sensor1 = GetHomoTransform(x);

    expected_sensor2_expressedIn_prevSensor2 = TransformDiffHomo(sensor2_expressedIn_sensor1, sensor1_expressedIn_prevSensor1(1:2,:));
    
    F = expected_sensor2_expressedIn_prevSensor2 - sensor2_expressedIn_prevSensor2;
    F = [F(1,1:2) F(2,3) F(1,4:5) F(2,6)];
    
%     
%     r2Rr1 = GetHomoTransform(sensor1_expressedIn_prevSensor1(1,:));
%     sRr = GetHomoTransform(x);
%     s2Rs1 = GetState(inv(sRr) * r2Rr1 * sRr);
%     
%     r3Rr2 = GetHomoTransform(sensor1_expressedIn_prevSensor1(2,:));
%     s3Rs2 = GetState(inv(sRr) * r3Rr2 * sRr);
%     
%     %F = [s2Rs1(1:4) s3Rs2(5:6)] - [sensor2_expressedIn_prevSensor2(1, 1:4) sensor2_expressedIn_prevSensor2(2,5:6)];
%     %F = [s2Rs1(1:2) s3Rs2(3) s2Rs1(4:5) s3Rs2(6)] - ...
%     %[sensor2_expressedIn_prevSensor2(1, 1:2) sensor2_expressedIn_prevSensor2(2,3) sensor2_expressedIn_prevSensor2(1,4:5) sensor2_expressedIn_prevSensor2(2,6)];
%     F = [s2Rs1(1:2) s3Rs2(3:4) s2Rs1(5:6)] - [sensor2_expressedIn_prevSensor2(1,1:2) sensor2_expressedIn_prevSensor2(2,3:4) sensor2_expressedIn_prevSensor2(1,5:6)];

    