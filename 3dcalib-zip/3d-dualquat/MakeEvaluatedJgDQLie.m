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
function [J] = MakeEvaluatedJgDQLie(estimated_sensor1_expressedIn_prevSensor1_dq, observed_sensor1_expressedIn_prevSensor1_dq, ...
    estimated_sensor2_expressedIn_prevSensor2_dq, observed_sensor2_expressedIn_prevSensor2_dq, ...
    calib_dualquat)

    invZ_Gx1 = ConcatDualQuat(InverseDualQuat(observed_sensor1_expressedIn_prevSensor1_dq), estimated_sensor1_expressedIn_prevSensor1_dq);
    invZ_Gx2 = ConcatDualQuat(InverseDualQuat(observed_sensor2_expressedIn_prevSensor2_dq), estimated_sensor2_expressedIn_prevSensor2_dq);
    invZ_Gx = [invZ_Gx1; invZ_Gx2];
    eval_Jlog = JLog(invZ_Gx);
    
    N = size(estimated_sensor1_expressedIn_prevSensor1_dq,1);

    Jg = zeros(16*N,6*N+6);
    %Jg = sparse(16*N,6*N+6);
    
    s = num2cell(calib_dualquat);   
    
    for i = 1:N
        
        r2Rr1 = num2cell(estimated_sensor1_expressedIn_prevSensor1_dq(i,:));
        Zr2Rr1 = num2cell(observed_sensor1_expressedIn_prevSensor1_dq(i,:));
        s2Rs1 = num2cell(estimated_sensor2_expressedIn_prevSensor2_dq(i,:));
        Zs2Rs1 = num2cell(observed_sensor2_expressedIn_prevSensor2_dq(i,:));
        
        % fill-in Jg
        r = 8*(i-1)+1;
        c = 6*(i-1)+1;
        Jg(r:r+7, c:c+5) = Jg2submat0(r2Rr1{:}, Zr2Rr1{:});
        
        r = 8*N + 8*(i-1)+1;
        Jg(r:r+7, c:c+5) = Jg2submat1(r2Rr1{:}, Zs2Rs1{:}, s{:});
        
        Jg(r:r+7, end-5:end) = Jg2submat2(r2Rr1{:}, Zs2Rs1{:}, s{:});
        
    end

    J = 2 * eval_Jlog * sparse(Jg);