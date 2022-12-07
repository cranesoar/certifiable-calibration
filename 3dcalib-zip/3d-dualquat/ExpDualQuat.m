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
function q = ExpDualQuat(d)

    N = size(d,1);
    
    w = [zeros(N,1) d(:,1:3)];
    v = [zeros(N,1) d(:,4:6)];
    s = [w v];
    nw = sqrt(sum(w.^2,2));
    cos_nw = cos(nw);
    sin_nw = sin(nw);
    sinc_nw = sinc(nw/pi);
    
    s2 = ConcatDualQuat(s,s);
    s3 = ConcatDualQuat(s2,s);
    
    t1 = bsxfun(@times,  0.5*(2*cos_nw + nw.*sin_nw), repmat([1 0 0 0 0 0 0 0], N, 1));
    t2 = bsxfun(@times, -0.5*(  cos_nw - 3*sinc_nw), s);    
    t3 = bsxfun(@times, 0.5*sinc_nw, s2);
    t4 = bsxfun(@times, -0.5 ./ (nw.^2) .* (cos_nw - sinc_nw), s3);
    
    q = t1+t2+t3+t4;
    
    badmask = nw < 1e-9;
    M = sum(badmask);
    q(badmask,:) = repmat([1 0 0 0 0 0 0 0], M, 1) + s(badmask,:) + 0.5 * s2(badmask,:) + 1/6*s3(badmask,:);
    
    if any( (sqrt(sum(q(:,1:4).^2,2))-1) > 1e-10) || any(abs(dot(q(:,1:4),q(:,5:8),2)) > 1e-10)
        error('bad dual quaternion!');
    end