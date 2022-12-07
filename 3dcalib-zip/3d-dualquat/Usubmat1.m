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
function y=Usubmat1(r2Rr10,r2Rr11,r2Rr12,r2Rr13,r2Rr14,r2Rr15,r2Rr16,r2Rr17,s0,s1,s2,s3,s4,s5,s6,s7)
y=[(-1).*s1.*s3,(-1).*s1.*s2,(-1).*s1.^2,s0.*s1,s3.*s5+(-1).*s1.*s7,s2.*...
s5+(-1).*s1.*s6;s0.*s3,s0.*s2,s0.*s1,(-1).*s0.^2,(-1).*s3.*s4+s0.*s7,...
(-1).*s2.*s4+s0.*s6;0,0,0,0,0,s1.*s4+(-1).*s0.*s5;0,0,0,0,s1.*s4+(-1).*...
s0.*s5,0;0,0,0,(-1).*s1.*s4+s0.*s5,0,0;0,0,s1.*s4+(-1).*s0.*s5,0,0,0;0,...
s1.*s4+(-1).*s0.*s5,0,0,0,0;s1.*s4+(-1).*s0.*s5,0,0,0,0,0];
