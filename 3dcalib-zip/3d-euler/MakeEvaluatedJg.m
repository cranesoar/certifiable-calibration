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
function Jg = MakeEvaluatedJg(sensor1_expressedIn_prevSensor1, calibration)

    N = size(sensor1_expressedIn_prevSensor1,1);
    
    Jg = zeros(2*6*N,6*N+6);
    
    Jg(1:6*N,1:6*N) = eye(6*N);
    
    sx = calibration(1);
    sy = calibration(2);
    sz = calibration(3);
    sr = calibration(4);
    sp = calibration(5);
    sw = calibration(6);
    
    for i = 1:N
        
        r2Rr1x = sensor1_expressedIn_prevSensor1(i,1);
        r2Rr1y = sensor1_expressedIn_prevSensor1(i,2);
        r2Rr1z = sensor1_expressedIn_prevSensor1(i,3);
        r2Rr1r = sensor1_expressedIn_prevSensor1(i,4);
        r2Rr1p = sensor1_expressedIn_prevSensor1(i,5);
        r2Rr1w = sensor1_expressedIn_prevSensor1(i,6);
        
        r = 6*N + (i-1)*6 + 1;
        c = (i-1)*6 + 1;
        
        Jg(r:r+5,c:c+5) = submat1(sx,sy,sz,sr,sp,sw,r2Rr1x,r2Rr1y,r2Rr1z,r2Rr1r,r2Rr1p,r2Rr1w);
        Jg(r:r+5,end-5:end) = submat2(sx,sy,sz,sr,sp,sw,r2Rr1x,r2Rr1y,r2Rr1z,r2Rr1r,r2Rr1p,r2Rr1w);

    end