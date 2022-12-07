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
function J = JAvg(b, ys, w) 

    b_dq = reshape(b,8,[])';
    
    N = size(b_dq,1);
    
    w = w/sum(abs(w));
    
    J = zeros(N*6,N*6);
    
    for i = 1:size(b_dq,1)

        thisB = b_dq(i,:);
        thisCellB = num2cell(thisB);
        
        idx = [0:7]+8*(i-1)+1;
        q_i = ys(idx,:)';
        
        r = (i-1)*6 + 1;
        
        err = ConcatDualQuat( repmat(InverseDualQuat(thisB),size(q_i,1),1), q_i );
                
        for j = 1:size(q_i,1)
            thisCellQ = num2cell(q_i(j,:));
            if abs(err(j,1)-1) > 1e-6
                thisJ = JAvgSubmat0(thisCellQ{:}, thisCellB{:}, w(j));
            else
                thisJ = JAvgSubmat1(thisCellQ{:}, thisCellB{:}, w(j));
            end
            J(r:r+5, r:r+5) = J(r:r+5, r:r+5) + thisJ;
        end
    end
        