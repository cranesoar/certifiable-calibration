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
function y=JgFIMsubmat0_withoff(r2Rr10,r2Rr11,r2Rr12,r2Rr13,r2Rr14,r2Rr15,r2Rr16,r2Rr17,s0,s1,s2,s3,s4,s5,s6,s7,a0,a1,a2,a3,a4,a5,a6,a7)
y=[a0,(-1).*a1,(-1).*a2,(-1).*a3,0,0,0,0;a1,a0,(-1).*a3,a2,0,0,0,0;a2,a3,...
a0,(-1).*a1,0,0,0,0;a3,(-1).*a2,a1,a0,0,0,0,0;a4,(-1).*a5,(-1).*a6,(-1)...
.*a7,a0,(-1).*a1,(-1).*a2,(-1).*a3;a5,a4,(-1).*a7,a6,a1,a0,(-1).*a3,a2;...
a6,a7,a4,(-1).*a5,a2,a3,a0,(-1).*a1;a7,(-1).*a6,a5,a4,a3,(-1).*a2,a1,...
a0];
