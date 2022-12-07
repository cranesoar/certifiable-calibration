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
function d = LogDualQuat(g, threshold)

    if ~exist('threshold', 'var')
        threshold = 1e-9;
    end

    goodmask = abs(g(:,1)-1) > threshold;
    badmask = ~goodmask;
    d = zeros(size(g,1),6);
    
    s0 = g(goodmask,1);
    s1 = g(goodmask,2);
    s2 = g(goodmask,3);
    s3 = g(goodmask,4);
    s4 = g(goodmask,5);
    s5 = g(goodmask,6);
    s6 = g(goodmask,7);
    s7 = g(goodmask,8);
    d(goodmask,:) = [(1+(-1).*s0.^2).^(-1/2).*s1.*atan2((1+(-1).*s0.^2).^(1/2),s0), ...
        (1+(-1).*s0.^2).^(-1/2).*s2.*atan2((1+(-1).*s0.^2).^(1/2),s0), ...
        (1+(-1).*s0.^2).^(-1/2).*s3.*atan2((1+(-1).*s0.^2).^(1/2),s0), ...
        (1/2).*(1+(-1).*s0.^2).^(-3/2).*(2.*(1+(-1).*s0.^2).^(1/2).*s1.*((-1).*s0.^2.*s4+((-1)+s0.^2).*s4)+(-1).*(((-3)+3.*s0.^2+3.*s1.^2+s2.^2+s3.^2).*s5+2.*s1.*(s2.*s6+s3.*s7)).*atan2((1+(-1).*s0.^2).^(1/2),s0)), ...
        (1/2).*(1+(-1).*s0.^2).^(-3/2).*(2.*(1+(-1).*s0.^2).^(1/2).*s2.*((-1).*s0.^2.*s4+((-1)+s0.^2).*s4)+(-1).*(2.*s1.*s2.*s5+s1.^2.*s6+((-3)+3.*s0.^2+3.*s2.^2+s3.^2).*s6+2.*s2.*s3.*s7).*atan2((1+(-1).*s0.^2).^(1/2),s0)), ...
        (1/2).*(1+(-1).*s0.^2).^(-3/2).*(2.*(1+(-1).*s0.^2).^(1/2).*s3.*((-1).*s0.^2.*s4+((-1)+s0.^2).*s4)+(-1).*(2.*s1.*s3.*s5+2.*s2.*s3.*s6+s1.^2.*s7+s2.^2.*s7+3.*((-1)+s0.^2+s3.^2).*s7).*atan2((1+(-1).*s0.^2).^(1/2),s0))];
    
    d(badmask,:) = [ zeros(sum(badmask),3) g(badmask,6:8) ];
    
    