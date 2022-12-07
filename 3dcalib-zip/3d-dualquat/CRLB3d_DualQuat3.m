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
function [crlb_calib,J,K,U,CRLB,JhJg] = CRLB3d_DualQuat3(sensor1_expressedIn_prevSensor1_dq, sensor2_expressedIn_prevSensor2_dq, calibration_dq, varargin)

    %this calculation of the CRLB uses the covariances expressed in the Lie
    %Algebra.

    p = inputParser;
    p.addRequired('sensor1_expressedIn_prevSensor1_dq', @(x)isnumeric(x))
    p.addRequired('sensor2_expressedIn_prevSensor2_dq', @(x)isnumeric(x))
    p.addRequired('calibration_dq', @(x)isnumeric(x))
    p.addParamValue('cov1_dq', [], @(x)isnumeric(x))
    p.addParamValue('cov2_dq', [], @(x)isnumeric(x))
    p.addParamValue('invS', [], @(x)isnumeric(x))
%     p.addParamValue('obs1', [], @(x)isnumeric(x))
%     p.addParamValue('obs2', [], @(x)isnumeric(x))
    p.parse(sensor1_expressedIn_prevSensor1_dq, sensor2_expressedIn_prevSensor2_dq, calibration_dq, varargin{:});
    cov1_dq=p.Results.cov1_dq;
    cov2_dq=p.Results.cov2_dq;
    invS=p.Results.invS;
%     observed_sensor1_expressedIn_prevSensor1_dq = p.Results.obs1;
%     observed_sensor2_expressedIn_prevSensor2_dq = p.Results.obs2;
    
    if isempty(invS)
        S = blkdiag(cov1_dq, cov2_dq);
        invS = inv(S);
    end
        
    [JhJg,Jh,Jg] = MakeJgFIM(sensor1_expressedIn_prevSensor1_dq, sensor2_expressedIn_prevSensor2_dq, calibration_dq);
        
    [~,U_old] = MakeEvaluatedJgAndU(sensor1_expressedIn_prevSensor1_dq, calibration_dq);
    N = size(sensor1_expressedIn_prevSensor1_dq,1);
    r2Rr1 = sensor1_expressedIn_prevSensor1_dq;
    s = calibration_dq;
    F = zeros( (1+N)*2, 8*(N+1) );
    for i = 1:N
        %r = (i-1)*2+1+2;
        r = (i-1)*2+1;
        c = (i-1)*8+1;
        F(r:r+1,c:c+7) = [2*r2Rr1(i,1:4) 0 0 0 0; r2Rr1(i,5:8) r2Rr1(i,1:4)];
    end
    %F(1:2, end-7:end) = [2*s(1:4) 0 0 0 0; s(5:8) s(1:4)];
    F(end-1:end, end-7:end) = [2*s(1:4) 0 0 0 0; s(5:8) s(1:4)];
    U = null(F);
    
    JgU=Jg*U;
    JhJgU=2*Jh*JgU;
    K=JhJgU' * invS * JhJgU;
    CRLB = U / K * U';
    crlb_calib = CRLB(end-7:end,end-7:end);
    
    J=[];
%     sqrt(diag(crlb_calib))'
%        
%     J = JhJg' * invS * JhJg;
%     K = U'*J*U;
%     CRLB = U / K * U';
%     crlb_calib = CRLB(end-7:end,end-7:end);
%     sqrt(diag(crlb_calib))'
%     
%     J = JhJg' * invS * JhJg;
%     K = U_old'*J*U_old;
%     CRLB = U_old / K * U_old';
%     crlb_calib = CRLB(end-7:end,end-7:end);
%     sqrt(diag(crlb_calib))'

% standard deviation of estimates:  0.16182  0.09017  0.46144  0.13627 | 0.20081  0.13928  0.21789  0.21393
%                                   0.0572   0.0276   0.0889   0.0515    0.0462   0.0187   0.0769   0.0770

%     sqrt_invS = sparse(inv(chol(S, 'lower')));
%     x0 = [reshape(sensor1_expressedIn_prevSensor1_dq',[],1); calibration_dq'];
% 	sqrtInvsJhJg = jacobianest(@(x)ErrorFunc3(x, observed_sensor1_expressedIn_prevSensor1_dq, observed_sensor2_expressedIn_prevSensor2_dq, sqrt_invS), x0);
%     sqrtInvsJhJg = real(sqrtInvsJhJg);
%     %sqrt_invS'*2*Jh*Jg
%     J2 = sqrtInvsJhJg' * sqrtInvsJhJg;
%     K2 = U'*J2*U;
%     CRLB2 = U / K2 * U';
%     sqrt(diag(CRLB2(end-7:end,end-7:end)))
    
function [cost] = ErrorFunc3(params, observed_sensor1_expressedIn_prevSensor1_dq, observed_sensor2_expressedIn_prevSensor2_dq, sqrt_invCov)
    
    %extract the values from the parameter vector
    N = size(observed_sensor1_expressedIn_prevSensor1_dq,1);
    estimated_sensor1_expressedIn_prevSensor1_dq = reshape(params(1:end-8), 8, N)';
    estimated_calib_dq = params(end-7:end)';
    
    estimated_sensor2_expressedIn_prevSensor2_dq = TransformDiffDualQuat(estimated_calib_dq, estimated_sensor1_expressedIn_prevSensor1_dq);
    
    d1 = DiffBetweenDualQuat(estimated_sensor1_expressedIn_prevSensor1_dq, observed_sensor1_expressedIn_prevSensor1_dq);
    d2 = DiffBetweenDualQuat(estimated_sensor2_expressedIn_prevSensor2_dq, observed_sensor2_expressedIn_prevSensor2_dq);

    cost = sqrt_invCov' * real([d1;d2]);

    

    

