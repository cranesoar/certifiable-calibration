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
function [Jg,U] = MakeEvaluatedJgAndU(sensor1_expressedIn_prevSensor1, calibration)

    N = size(sensor1_expressedIn_prevSensor1,1);
    
    if size(sensor1_expressedIn_prevSensor1,2)==6
        sensor1_expressedIn_prevSensor1_dualquat = EulerStateToDualQuat(sensor1_expressedIn_prevSensor1);
    else
        sensor1_expressedIn_prevSensor1_dualquat = sensor1_expressedIn_prevSensor1;
    end
    if size(calibration,2)==6
        calib_dualquat = EulerStateToDualQuat(calibration);
    else
        calib_dualquat = calibration;
    end
    
%     t=[calib_dualquat; sensor1_expressedIn_prevSensor1_dualquat];
%     m1= sqrt(sum(t(:,1:4).^2,2));
%     m2= diag(t(:,1:4)*t(:,5:8)');
%     f = reshape([m1 m2]',[],1);
%     addpath('..\DERIVESTsuite');
%     jacobianest(
    
    
    Jg = zeros(2*6*N,8*N+8);
    U = zeros(8*N+8,6*N+6);
    
    s0 = calib_dualquat(1);
    s1 = calib_dualquat(2);
    s2 = calib_dualquat(3);
    s3 = calib_dualquat(4);
    s4 = calib_dualquat(5);
    s5 = calib_dualquat(6);
    s6 = calib_dualquat(7);
    s7 = calib_dualquat(8);
    
    for i = 1:N
        
        r2Rr10 = sensor1_expressedIn_prevSensor1_dualquat(i,1);
        r2Rr11 = sensor1_expressedIn_prevSensor1_dualquat(i,2);
        r2Rr12 = sensor1_expressedIn_prevSensor1_dualquat(i,3);
        r2Rr13 = sensor1_expressedIn_prevSensor1_dualquat(i,4);
        r2Rr14 = sensor1_expressedIn_prevSensor1_dualquat(i,5);
        r2Rr15 = sensor1_expressedIn_prevSensor1_dualquat(i,6);
        r2Rr16 = sensor1_expressedIn_prevSensor1_dualquat(i,7);
        r2Rr17 = sensor1_expressedIn_prevSensor1_dualquat(i,8);
        
        % fill-in Jg
        r = 6*(i-1)+1;
        c = 8*(i-1)+1;
        Jg(r:r+5, c:c+7) = Jgsubmat0(r2Rr10,r2Rr11,r2Rr12,r2Rr13,r2Rr14,r2Rr15,r2Rr16,r2Rr17,s0,s1,s2,s3,s4,s5,s6,s7);
        
        r = 6*N + 6*(i-1)+1;
        Jg(r:r+5, c:c+7) = Jgsubmat1(r2Rr10,r2Rr11,r2Rr12,r2Rr13,r2Rr14,r2Rr15,r2Rr16,r2Rr17,s0,s1,s2,s3,s4,s5,s6,s7);
        
        Jg(r:r+5, end-7:end) = Jgsubmat2(r2Rr10,r2Rr11,r2Rr12,r2Rr13,r2Rr14,r2Rr15,r2Rr16,r2Rr17,s0,s1,s2,s3,s4,s5,s6,s7);
        
        % fill-in U
        r = 8*(i-1)+1;
        c = size(U,2) - 6*i + 1;
        U(r:r+7, c:c+5) = Usubmat0(r2Rr10,r2Rr11,r2Rr12,r2Rr13,r2Rr14,r2Rr15,r2Rr16,r2Rr17,s0,s1,s2,s3,s4,s5,s6,s7);
        
    end
    
    U(end-7:end,1:6) = Usubmat1(r2Rr10,r2Rr11,r2Rr12,r2Rr13,r2Rr14,r2Rr15,r2Rr16,r2Rr17,s0,s1,s2,s3,s4,s5,s6,s7);
