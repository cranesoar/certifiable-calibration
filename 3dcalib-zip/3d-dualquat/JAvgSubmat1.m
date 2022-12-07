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
function y=JAvgSubmat1(qa0,qa1,qa2,qa3,qa4,qa5,qa6,qa7,b0,b1,b2,b3,b4,b5,b6,b7,wa)
y=[(-1/2).*wa,0,0,0,0,0;0,(-1/2).*wa,0,0,0,0;0,0,(-1/2).*wa,0,0,0;0,(3/4)...
.*(b7.*qa0+(-1).*b6.*qa1+b5.*qa2+(-1).*b4.*qa3+qa3.*qa4+(-1).*qa2.*...
qa5+qa1.*qa6+(-1).*qa0.*qa7).*wa,(-3/4).*(b6.*qa0+b7.*qa1+(-1).*b4.*...
qa2+(-1).*b5.*qa3+qa2.*qa4+qa3.*qa5+(-1).*qa0.*qa6+(-1).*qa1.*qa7).*wa,...
(-3/4).*wa,0,0;(-3/4).*(b7.*qa0+(-1).*b6.*qa1+b5.*qa2+(-1).*b4.*...
qa3+qa3.*qa4+(-1).*qa2.*qa5+qa1.*qa6+(-1).*qa0.*qa7).*wa,0,(3/4).*(b5.*...
qa0+(-1).*b4.*qa1+(-1).*b7.*qa2+b6.*qa3+qa1.*qa4+(-1).*qa0.*qa5+(-1).*...
qa3.*qa6+qa2.*qa7).*wa,0,(-3/4).*wa,0;(3/4).*(b6.*qa0+b7.*qa1+(-1).*b4.*...
qa2+(-1).*b5.*qa3+qa2.*qa4+qa3.*qa5+(-1).*qa0.*qa6+(-1).*qa1.*qa7).*wa,...
(-3/4).*(b5.*qa0+(-1).*b4.*qa1+(-1).*b7.*qa2+b6.*qa3+qa1.*qa4+(-1).*...
qa0.*qa5+(-1).*qa3.*qa6+qa2.*qa7).*wa,0,0,0,(-3/4).*wa];
