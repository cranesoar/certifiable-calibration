%
% Copyright � 2012, The Massachusetts Institute of Technology. All rights reserved. 
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
function y=Jg2submat0(r2Rr10,r2Rr11,r2Rr12,r2Rr13,r2Rr14,r2Rr15,r2Rr16,r2Rr17,Zr2Rr10,Zr2Rr11,Zr2Rr12,Zr2Rr13,Zr2Rr14,Zr2Rr15,Zr2Rr16,Zr2Rr17)
y=[(1/8).*((-4).*r2Rr11.*Zr2Rr10+4.*r2Rr10.*Zr2Rr11+4.*r2Rr13.*...
Zr2Rr12+(-4).*r2Rr12.*Zr2Rr13),(1/2).*((-1).*r2Rr12.*Zr2Rr10+(-1).*...
r2Rr13.*Zr2Rr11+r2Rr10.*Zr2Rr12+r2Rr11.*Zr2Rr13),(1/2).*((-1).*r2Rr13.*...
Zr2Rr10+r2Rr12.*Zr2Rr11+(-1).*r2Rr11.*Zr2Rr12+r2Rr10.*Zr2Rr13),0,0,0;(...
1/8).*(4.*r2Rr10.*Zr2Rr10+4.*r2Rr11.*Zr2Rr11+4.*r2Rr12.*Zr2Rr12+4.*...
r2Rr13.*Zr2Rr13),(1/2).*((-1).*r2Rr13.*Zr2Rr10+r2Rr12.*Zr2Rr11+(-1).*...
r2Rr11.*Zr2Rr12+r2Rr10.*Zr2Rr13),(1/2).*(r2Rr12.*Zr2Rr10+r2Rr13.*...
Zr2Rr11+(-1).*r2Rr10.*Zr2Rr12+(-1).*r2Rr11.*Zr2Rr13),0,0,0;(1/8).*(4.*...
r2Rr13.*Zr2Rr10+(-4).*r2Rr12.*Zr2Rr11+4.*r2Rr11.*Zr2Rr12+(-4).*r2Rr10.*...
Zr2Rr13),(1/2).*(r2Rr10.*Zr2Rr10+r2Rr11.*Zr2Rr11+r2Rr12.*...
Zr2Rr12+r2Rr13.*Zr2Rr13),(1/2).*((-1).*r2Rr11.*Zr2Rr10+r2Rr10.*...
Zr2Rr11+r2Rr13.*Zr2Rr12+(-1).*r2Rr12.*Zr2Rr13),0,0,0;(1/8).*((-4).*...
r2Rr12.*Zr2Rr10+(-4).*r2Rr13.*Zr2Rr11+4.*r2Rr10.*Zr2Rr12+4.*r2Rr11.*...
Zr2Rr13),(1/2).*(r2Rr11.*Zr2Rr10+(-1).*r2Rr10.*Zr2Rr11+(-1).*r2Rr13.*...
Zr2Rr12+r2Rr12.*Zr2Rr13),(1/2).*(r2Rr10.*Zr2Rr10+r2Rr11.*...
Zr2Rr11+r2Rr12.*Zr2Rr12+r2Rr13.*Zr2Rr13),0,0,0;(1/8).*((-4).*r2Rr15.*...
Zr2Rr10+4.*r2Rr14.*Zr2Rr11+4.*r2Rr17.*Zr2Rr12+(-4).*r2Rr16.*Zr2Rr13+(-4)...
.*r2Rr11.*Zr2Rr14+4.*r2Rr10.*Zr2Rr15+4.*r2Rr13.*Zr2Rr16+(-4).*r2Rr12.*...
Zr2Rr17),(1/2).*((-1).*r2Rr16.*Zr2Rr10+(-1).*r2Rr17.*Zr2Rr11+r2Rr14.*...
Zr2Rr12+r2Rr15.*Zr2Rr13+(-1).*r2Rr12.*Zr2Rr14+(-1).*r2Rr13.*...
Zr2Rr15+r2Rr10.*Zr2Rr16+r2Rr11.*Zr2Rr17),(1/2).*((-1).*r2Rr17.*...
Zr2Rr10+r2Rr16.*Zr2Rr11+(-1).*r2Rr15.*Zr2Rr12+r2Rr14.*Zr2Rr13+(-1).*...
r2Rr13.*Zr2Rr14+r2Rr12.*Zr2Rr15+(-1).*r2Rr11.*Zr2Rr16+r2Rr10.*Zr2Rr17),(...
1/4).*((-2).*r2Rr11.*Zr2Rr10+2.*r2Rr10.*Zr2Rr11+2.*r2Rr13.*Zr2Rr12+(-2)...
.*r2Rr12.*Zr2Rr13),(1/2).*((-1).*r2Rr12.*Zr2Rr10+(-1).*r2Rr13.*...
Zr2Rr11+r2Rr10.*Zr2Rr12+r2Rr11.*Zr2Rr13),(1/2).*((-1).*r2Rr13.*...
Zr2Rr10+r2Rr12.*Zr2Rr11+(-1).*r2Rr11.*Zr2Rr12+r2Rr10.*Zr2Rr13);(1/8).*(...
4.*r2Rr14.*Zr2Rr10+4.*r2Rr15.*Zr2Rr11+4.*r2Rr16.*Zr2Rr12+4.*r2Rr17.*...
Zr2Rr13+4.*r2Rr10.*Zr2Rr14+4.*r2Rr11.*Zr2Rr15+4.*r2Rr12.*Zr2Rr16+4.*...
r2Rr13.*Zr2Rr17),(1/2).*((-1).*r2Rr17.*Zr2Rr10+r2Rr16.*Zr2Rr11+(-1).*...
r2Rr15.*Zr2Rr12+r2Rr14.*Zr2Rr13+(-1).*r2Rr13.*Zr2Rr14+r2Rr12.*...
Zr2Rr15+(-1).*r2Rr11.*Zr2Rr16+r2Rr10.*Zr2Rr17),(1/2).*(r2Rr16.*...
Zr2Rr10+r2Rr17.*Zr2Rr11+(-1).*r2Rr14.*Zr2Rr12+(-1).*r2Rr15.*...
Zr2Rr13+r2Rr12.*Zr2Rr14+r2Rr13.*Zr2Rr15+(-1).*r2Rr10.*Zr2Rr16+(-1).*...
r2Rr11.*Zr2Rr17),(1/4).*(2.*r2Rr10.*Zr2Rr10+2.*r2Rr11.*Zr2Rr11+2.*...
r2Rr12.*Zr2Rr12+2.*r2Rr13.*Zr2Rr13),(1/2).*((-1).*r2Rr13.*...
Zr2Rr10+r2Rr12.*Zr2Rr11+(-1).*r2Rr11.*Zr2Rr12+r2Rr10.*Zr2Rr13),(1/2).*(...
r2Rr12.*Zr2Rr10+r2Rr13.*Zr2Rr11+(-1).*r2Rr10.*Zr2Rr12+(-1).*r2Rr11.*...
Zr2Rr13);(1/8).*(4.*r2Rr17.*Zr2Rr10+(-4).*r2Rr16.*Zr2Rr11+4.*r2Rr15.*...
Zr2Rr12+(-4).*r2Rr14.*Zr2Rr13+4.*r2Rr13.*Zr2Rr14+(-4).*r2Rr12.*...
Zr2Rr15+4.*r2Rr11.*Zr2Rr16+(-4).*r2Rr10.*Zr2Rr17),(1/2).*(r2Rr14.*...
Zr2Rr10+r2Rr15.*Zr2Rr11+r2Rr16.*Zr2Rr12+r2Rr17.*Zr2Rr13+r2Rr10.*...
Zr2Rr14+r2Rr11.*Zr2Rr15+r2Rr12.*Zr2Rr16+r2Rr13.*Zr2Rr17),(1/2).*((-1).*...
r2Rr15.*Zr2Rr10+r2Rr14.*Zr2Rr11+r2Rr17.*Zr2Rr12+(-1).*r2Rr16.*...
Zr2Rr13+(-1).*r2Rr11.*Zr2Rr14+r2Rr10.*Zr2Rr15+r2Rr13.*Zr2Rr16+(-1).*...
r2Rr12.*Zr2Rr17),(1/4).*(2.*r2Rr13.*Zr2Rr10+(-2).*r2Rr12.*Zr2Rr11+2.*...
r2Rr11.*Zr2Rr12+(-2).*r2Rr10.*Zr2Rr13),(1/2).*(r2Rr10.*Zr2Rr10+r2Rr11.*...
Zr2Rr11+r2Rr12.*Zr2Rr12+r2Rr13.*Zr2Rr13),(1/2).*((-1).*r2Rr11.*...
Zr2Rr10+r2Rr10.*Zr2Rr11+r2Rr13.*Zr2Rr12+(-1).*r2Rr12.*Zr2Rr13);(1/8).*(...
(-4).*r2Rr16.*Zr2Rr10+(-4).*r2Rr17.*Zr2Rr11+4.*r2Rr14.*Zr2Rr12+4.*...
r2Rr15.*Zr2Rr13+(-4).*r2Rr12.*Zr2Rr14+(-4).*r2Rr13.*Zr2Rr15+4.*r2Rr10.*...
Zr2Rr16+4.*r2Rr11.*Zr2Rr17),(1/2).*(r2Rr15.*Zr2Rr10+(-1).*r2Rr14.*...
Zr2Rr11+(-1).*r2Rr17.*Zr2Rr12+r2Rr16.*Zr2Rr13+r2Rr11.*Zr2Rr14+(-1).*...
r2Rr10.*Zr2Rr15+(-1).*r2Rr13.*Zr2Rr16+r2Rr12.*Zr2Rr17),(1/2).*(r2Rr14.*...
Zr2Rr10+r2Rr15.*Zr2Rr11+r2Rr16.*Zr2Rr12+r2Rr17.*Zr2Rr13+r2Rr10.*...
Zr2Rr14+r2Rr11.*Zr2Rr15+r2Rr12.*Zr2Rr16+r2Rr13.*Zr2Rr17),(1/4).*((-2).*...
r2Rr12.*Zr2Rr10+(-2).*r2Rr13.*Zr2Rr11+2.*r2Rr10.*Zr2Rr12+2.*r2Rr11.*...
Zr2Rr13),(1/2).*(r2Rr11.*Zr2Rr10+(-1).*r2Rr10.*Zr2Rr11+(-1).*r2Rr13.*...
Zr2Rr12+r2Rr12.*Zr2Rr13),(1/2).*(r2Rr10.*Zr2Rr10+r2Rr11.*...
Zr2Rr11+r2Rr12.*Zr2Rr12+r2Rr13.*Zr2Rr13)];
