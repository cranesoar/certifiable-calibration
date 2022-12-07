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
function [J,eval_Jlog,Jg] = MakeJgFIM(sensor1_expressedIn_prevSensor1_dq, sensor2_expressedIn_prevSensor2_dq, calib_dualquat)

    %a_dq = EulerStateToDualQuat([1 1 1 pi pi/2 pi/3]);
    a_dq = [1 0 0 0, 0 0 0 0];

    Gx_dq = [sensor1_expressedIn_prevSensor1_dq; sensor2_expressedIn_prevSensor2_dq];
    Gx_dq_woffset = ConcatDualQuat(a_dq, Gx_dq);
    eval_Jlog = JLog(Gx_dq_woffset);
    
    N = size(sensor1_expressedIn_prevSensor1_dq,1);

    Jg = zeros(16*N,8*N+8);
    
    s = num2cell(calib_dualquat);   
    a = num2cell(a_dq);
    
    for i = 1:N
        
        r2Rr1 = num2cell(Gx_dq(i,:));
        
        % fill-in Jg
        r = 8*(i-1)+1;
        c = 8*(i-1)+1;
        Jg(r:r+7, c:c+7) = JgFIMsubmat0_withoff(r2Rr1{:}, s{:}, a{:});
        
        r = 8*N + 8*(i-1)+1;
        Jg(r:r+7, c:c+7) = JgFIMsubmat1_withoff(r2Rr1{:}, s{:}, a{:});
        
        Jg(r:r+7, end-7:end) = JgFIMsubmat2_withoff(r2Rr1{:}, s{:}, a{:});
        
    end

    J = 2 * eval_Jlog * Jg;
    
%     
%     Gx_dq = [sensor1_expressedIn_prevSensor1_dq; sensor2_expressedIn_prevSensor2_dq];
%     
%     eval_Jlog = JLog(Gx_dq);
%     
%     N = size(sensor1_expressedIn_prevSensor1_dq,1);
% 
%     Jg = zeros(16*N,8*N+8);
%     
%     s = num2cell(calib_dualquat);   
%     
%     for i = 1:N
%         
%         r2Rr1 = num2cell(Gx_dq(i,:));
%         
%         % fill-in Jg
%         r = 8*(i-1)+1;
%         c = 8*(i-1)+1;
%         Jg(r:r+7, c:c+7) = JgFIMsubmat0(r2Rr1{:}, s{:});
%         
%         r = 8*N + 8*(i-1)+1;
%         Jg(r:r+7, c:c+7) = JgFIMsubmat1(r2Rr1{:}, s{:});
%         
%         Jg(r:r+7, end-7:end) = JgFIMsubmat2(r2Rr1{:}, s{:});
%         
%     end
% 
%     J = 2 * eval_Jlog * Jg;    