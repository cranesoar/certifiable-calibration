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
function J=JLog(invZ_Gx_exp_d)

    if size(invZ_Gx_exp_d,2) ~= 8
        e = reshape(invZ_Gx_exp_d,8,[])';
    else
        e = invZ_Gx_exp_d;
    end
    
    N = size(e,1);
    
    J = zeros(6*N,8*N);
    %J = sparse(6*N,8*N);
    
    for i = 1:N
        s = num2cell(e(i,:));
        if norm(e(i,2:4)) < 1e-9
            y = zeros(6,8);
            y(1:3,2:4) = eye(3);
            y(4:6,6:8) = eye(3);
            %y = jacobianest(@(x)LogDualQuat(x), e(i,:));
%            y = JLogNoRot(s{:});
        else
            y = JLogWRot(s{:});
        end
        r = (i-1)*6+1;
        c = (i-1)*8+1;
        J(r:r+5,c:c+7) = y;
    end

    J = sparse(J);