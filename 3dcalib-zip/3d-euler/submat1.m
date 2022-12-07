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
function y=submat1(xs,ys,zs,rs,ps,ws,r2Rr1x,r2Rr1y,r2Rr1z,r2Rr1r,r2Rr1p,r2Rr1w)
y=[cos(ps).*cos(ws),cos(ps).*sin(ws),(-1).*sin(ps),(-1).*cos(r2Rr1p).*sin(...
ps).*(ys.*cos(r2Rr1r)+(-1).*zs.*sin(r2Rr1r))+cos(ps).*(sin(r2Rr1r).*((...
-1).*zs.*cos(r2Rr1w+(-1).*ws).*sin(r2Rr1p)+ys.*sin(r2Rr1w+(-1).*ws))+...
cos(r2Rr1r).*(ys.*cos(r2Rr1w).*cos(ws).*sin(r2Rr1p)+zs.*sin(r2Rr1w+(-1)...
.*ws)+ys.*sin(r2Rr1p).*sin(r2Rr1w).*sin(ws))),cos(ps).*cos(r2Rr1w+(-1).*...
ws).*((-1).*xs.*sin(r2Rr1p)+cos(r2Rr1p).*(zs.*cos(r2Rr1r)+ys.*sin(...
r2Rr1r)))+sin(ps).*(xs.*cos(r2Rr1p)+sin(r2Rr1p).*(zs.*cos(r2Rr1r)+ys.*...
sin(r2Rr1r))),cos(ps).*(zs.*cos(r2Rr1w+(-1).*ws).*sin(r2Rr1r)+(-1).*(...
xs.*cos(r2Rr1p)+ys.*sin(r2Rr1p).*sin(r2Rr1r)).*sin(r2Rr1w+(-1).*ws)+(-1)...
.*cos(r2Rr1r).*(ys.*cos(r2Rr1w).*cos(ws)+zs.*sin(r2Rr1p).*sin(r2Rr1w+(...
-1).*ws)+ys.*sin(r2Rr1w).*sin(ws)));cos(ws).*sin(ps).*sin(rs)+(-1).*cos(...
rs).*sin(ws),cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws),cos(ps).*sin(rs)...
,cos(rs).*((-1).*cos(r2Rr1r).*(sin(r2Rr1w).*((-1).*ys.*cos(ws).*sin(...
r2Rr1p)+zs.*sin(ws))+cos(r2Rr1w).*(zs.*cos(ws)+ys.*sin(r2Rr1p).*sin(ws))...
)+(-1).*sin(r2Rr1r).*(sin(r2Rr1w).*(zs.*cos(ws).*sin(r2Rr1p)+ys.*sin(ws)...
)+cos(r2Rr1w).*(ys.*cos(ws)+(-1).*zs.*sin(r2Rr1p).*sin(ws))))+sin(rs).*(...
cos(ps).*cos(r2Rr1p).*(ys.*cos(r2Rr1r)+(-1).*zs.*sin(r2Rr1r))+sin(ps).*(...
cos(r2Rr1r).*(ys.*cos(r2Rr1w).*cos(ws).*sin(r2Rr1p)+zs.*cos(ws).*sin(...
r2Rr1w)+(-1).*zs.*cos(r2Rr1w).*sin(ws)+ys.*sin(r2Rr1p).*sin(r2Rr1w).*...
sin(ws))+(-1).*sin(r2Rr1r).*(zs.*cos(r2Rr1w).*cos(ws).*sin(r2Rr1p)+(-1)...
.*ys.*cos(ws).*sin(r2Rr1w)+ys.*cos(r2Rr1w).*sin(ws)+zs.*sin(r2Rr1p).*...
sin(r2Rr1w).*sin(ws)))),(-1).*cos(ps).*(xs.*cos(r2Rr1p)+sin(r2Rr1p).*(...
zs.*cos(r2Rr1r)+ys.*sin(r2Rr1r))).*sin(rs)+((-1).*xs.*sin(r2Rr1p)+cos(...
r2Rr1p).*(zs.*cos(r2Rr1r)+ys.*sin(r2Rr1r))).*(cos(r2Rr1w+(-1).*ws).*sin(...
ps).*sin(rs)+cos(rs).*sin(r2Rr1w+(-1).*ws)),sin(ps).*sin(rs).*((-1).*...
cos(ws).*(sin(r2Rr1r).*((-1).*zs.*cos(r2Rr1w)+ys.*sin(r2Rr1p).*sin(...
r2Rr1w))+cos(r2Rr1r).*(ys.*cos(r2Rr1w)+zs.*sin(r2Rr1p).*sin(r2Rr1w)))+(...
-1).*xs.*cos(r2Rr1p).*sin(r2Rr1w+(-1).*ws)+(cos(r2Rr1r).*(zs.*cos(...
r2Rr1w).*sin(r2Rr1p)+(-1).*ys.*sin(r2Rr1w))+sin(r2Rr1r).*(ys.*cos(...
r2Rr1w).*sin(r2Rr1p)+zs.*sin(r2Rr1w))).*sin(ws))+cos(rs).*(xs.*cos(...
r2Rr1p).*cos(r2Rr1w+(-1).*ws)+sin(r2Rr1r).*(ys.*cos(r2Rr1w).*cos(ws).*...
sin(r2Rr1p)+zs.*cos(ws).*sin(r2Rr1w)+(-1).*zs.*cos(r2Rr1w).*sin(ws)+ys.*...
sin(r2Rr1p).*sin(r2Rr1w).*sin(ws))+cos(r2Rr1r).*(zs.*cos(r2Rr1w).*cos(...
ws).*sin(r2Rr1p)+(-1).*ys.*cos(ws).*sin(r2Rr1w)+ys.*cos(r2Rr1w).*sin(ws)...
+zs.*sin(r2Rr1p).*sin(r2Rr1w).*sin(ws)));cos(rs).*cos(ws).*sin(ps)+sin(...
rs).*sin(ws),(-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws),cos(ps).*...
cos(rs),cos(ps).*cos(r2Rr1p).*cos(rs).*(ys.*cos(r2Rr1r)+(-1).*zs.*sin(...
r2Rr1r))+sin(rs).*(cos(r2Rr1r).*(sin(r2Rr1w).*((-1).*ys.*cos(ws).*sin(...
r2Rr1p)+zs.*sin(ws))+cos(r2Rr1w).*(zs.*cos(ws)+ys.*sin(r2Rr1p).*sin(ws))...
)+sin(r2Rr1r).*(sin(r2Rr1w).*(zs.*cos(ws).*sin(r2Rr1p)+ys.*sin(ws))+cos(...
r2Rr1w).*(ys.*cos(ws)+(-1).*zs.*sin(r2Rr1p).*sin(ws))))+cos(rs).*sin(ps)...
.*(cos(r2Rr1r).*(cos(r2Rr1w).*(ys.*cos(ws).*sin(r2Rr1p)+(-1).*zs.*sin(...
ws))+sin(r2Rr1w).*(zs.*cos(ws)+ys.*sin(r2Rr1p).*sin(ws)))+(-1).*sin(...
r2Rr1r).*(cos(r2Rr1w).*(zs.*cos(ws).*sin(r2Rr1p)+ys.*sin(ws))+sin(...
r2Rr1w).*((-1).*ys.*cos(ws)+zs.*sin(r2Rr1p).*sin(ws)))),(-1).*cos(ps).*...
cos(rs).*(xs.*cos(r2Rr1p)+sin(r2Rr1p).*(zs.*cos(r2Rr1r)+ys.*sin(r2Rr1r))...
)+((-1).*xs.*sin(r2Rr1p)+cos(r2Rr1p).*(zs.*cos(r2Rr1r)+ys.*sin(r2Rr1r)))...
.*(cos(rs).*cos(r2Rr1w+(-1).*ws).*sin(ps)+(-1).*sin(rs).*sin(r2Rr1w+(-1)...
.*ws)),cos(rs).*sin(ps).*((-1).*cos(ws).*((-1).*zs.*cos(r2Rr1w).*sin(...
r2Rr1r)+(xs.*cos(r2Rr1p)+ys.*sin(r2Rr1p).*sin(r2Rr1r)).*sin(r2Rr1w)+cos(...
r2Rr1r).*(ys.*cos(r2Rr1w)+zs.*sin(r2Rr1p).*sin(r2Rr1w)))+(xs.*cos(...
r2Rr1p).*cos(r2Rr1w)+cos(r2Rr1r).*(zs.*cos(r2Rr1w).*sin(r2Rr1p)+(-1).*...
ys.*sin(r2Rr1w))+sin(r2Rr1r).*(ys.*cos(r2Rr1w).*sin(r2Rr1p)+zs.*sin(...
r2Rr1w))).*sin(ws))+sin(rs).*((-1).*cos(ws).*(xs.*cos(r2Rr1p).*cos(...
r2Rr1w)+cos(r2Rr1r).*(zs.*cos(r2Rr1w).*sin(r2Rr1p)+(-1).*ys.*sin(r2Rr1w)...
)+sin(r2Rr1r).*(ys.*cos(r2Rr1w).*sin(r2Rr1p)+zs.*sin(r2Rr1w)))+(-1).*((...
-1).*zs.*cos(r2Rr1w).*sin(r2Rr1r)+(xs.*cos(r2Rr1p)+ys.*sin(r2Rr1p).*sin(...
r2Rr1r)).*sin(r2Rr1w)+cos(r2Rr1r).*(ys.*cos(r2Rr1w)+zs.*sin(r2Rr1p).*...
sin(r2Rr1w))).*sin(ws));0,0,0,((cos(ps).*cos(rs).*((-1).*cos(ps).*cos(...
r2Rr1p).*cos(rs).*sin(r2Rr1r)+(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*...
sin(r2Rr1r).*sin(r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*...
sin(ws))+((-1).*cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+cos(r2Rr1r).*sin(...
r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+((-1).*cos(ws).*...
sin(rs)+cos(rs).*sin(ps).*sin(ws)).*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*...
cos(rs)+(cos(r2Rr1w).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(...
r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*sin(ws))+(cos(...
r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*...
cos(ws).*sin(ps)+sin(rs).*sin(ws)))).*((-1).*(cos(rs).*cos(ws)+sin(ps).*...
sin(rs).*sin(ws)).*(cos(ps).*cos(r2Rr1p).*cos(rs).*sin(r2Rr1r)+(cos(...
r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(...
ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1w).*sin(r2Rr1p).*sin(...
r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(...
rs).*sin(ws)))+(-1).*cos(ps).*sin(rs).*(cos(ps).*cos(r2Rr1p).*cos(...
r2Rr1r).*cos(rs)+(cos(r2Rr1w).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(...
r2Rr1p).*sin(r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*sin(ws)...
)+(cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(...
cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+(-1).*(cos(ws).*sin(ps).*...
sin(rs)+(-1).*cos(rs).*sin(ws)).*((-1).*cos(ps).*cos(rs).*sin(r2Rr1p)+...
cos(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*...
sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))))+(...
cos(ps).*sin(rs).*((-1).*cos(ps).*cos(r2Rr1p).*cos(rs).*sin(r2Rr1r)+(...
cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*(cos(...
ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*sin(ws))+((-1).*cos(r2Rr1w).*sin(...
r2Rr1p).*sin(r2Rr1r)+cos(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(...
ps)+sin(rs).*sin(ws)))+(cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)).*(...
cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*cos(rs)+(cos(r2Rr1w).*sin(r2Rr1r)+(...
-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*...
cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(...
r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))).*(...
((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws)).*(cos(ps).*cos(...
r2Rr1p).*cos(rs).*sin(r2Rr1r)+(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*...
sin(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*...
sin(ws))+(cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(...
r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+cos(ps).*cos(rs)...
.*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*cos(rs)+(cos(r2Rr1w).*sin(r2Rr1r)+...
(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*...
cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(...
r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+(...
cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)).*((-1).*cos(ps).*cos(rs).*...
sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(ws).*sin(rs)+cos(rs).*...
sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(...
ws)))))).*(((cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)).*(cos(ps).*cos(...
r2Rr1p).*cos(rs).*sin(r2Rr1r)+(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*...
sin(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*...
sin(ws))+(cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(...
r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+cos(ps).*sin(rs)...
.*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*cos(rs)+(cos(r2Rr1w).*sin(r2Rr1r)+...
(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*...
cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(...
r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+(...
cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws)).*((-1).*cos(ps).*cos(...
rs).*sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(ws).*sin(rs)+cos(...
rs).*sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*...
sin(ws))))).^2+(((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws)).*(...
cos(ps).*cos(r2Rr1p).*cos(rs).*sin(r2Rr1r)+(cos(r2Rr1r).*cos(r2Rr1w)+...
sin(r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(ws).*sin(rs)+cos(rs)...
.*sin(ps).*sin(ws))+(cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*cos(...
r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+...
cos(ps).*cos(rs).*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*cos(rs)+(cos(...
r2Rr1w).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(...
ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w)...
.*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(...
rs).*sin(ws)))+(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)).*((-1).*cos(...
ps).*cos(rs).*sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(ws).*...
sin(rs)+cos(rs).*sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*sin(...
ps)+sin(rs).*sin(ws))))).^2).^(-1),(((cos(rs).*cos(ws)+sin(ps).*sin(rs)...
.*sin(ws)).*(cos(ps).*cos(r2Rr1p).*cos(rs).*sin(r2Rr1r)+(cos(r2Rr1r).*...
cos(r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(ws).*sin(...
rs)+cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(...
-1).*cos(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(...
ws)))+cos(ps).*sin(rs).*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*cos(rs)+(...
cos(r2Rr1w).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(...
cos(ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1r).*cos(...
r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(...
ps)+sin(rs).*sin(ws)))+(cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws)...
).*((-1).*cos(ps).*cos(rs).*sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w).*((-1)...
.*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*...
cos(ws).*sin(ps)+sin(rs).*sin(ws))))).^2+(((-1).*cos(ws).*sin(rs)+cos(...
rs).*sin(ps).*sin(ws)).*(cos(ps).*cos(r2Rr1p).*cos(rs).*sin(r2Rr1r)+(...
cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*((-1).*...
cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1w).*sin(r2Rr1p).*...
sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+...
sin(rs).*sin(ws)))+cos(ps).*cos(rs).*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r)...
.*cos(rs)+(cos(r2Rr1w).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(...
r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*sin(ws))+(cos(...
r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*...
cos(ws).*sin(ps)+sin(rs).*sin(ws)))+(cos(rs).*cos(ws).*sin(ps)+sin(rs).*...
sin(ws)).*((-1).*cos(ps).*cos(rs).*sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w)...
.*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(...
rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws))))).^2).^(-1).*((((-1).*cos(ws).*...
sin(rs)+cos(rs).*sin(ps).*sin(ws)).*(cos(ps).*cos(r2Rr1p).*cos(rs).*sin(...
r2Rr1r)+(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w))...
.*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1w).*sin(...
r2Rr1p).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws)...
.*sin(ps)+sin(rs).*sin(ws)))+cos(ps).*cos(rs).*(cos(ps).*cos(r2Rr1p).*...
cos(r2Rr1r).*cos(rs)+(cos(r2Rr1w).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(...
r2Rr1p).*sin(r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*sin(ws)...
)+(cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(...
cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+(cos(rs).*cos(ws).*sin(ps)+...
sin(rs).*sin(ws)).*((-1).*cos(ps).*cos(rs).*sin(r2Rr1p)+cos(r2Rr1p).*(...
sin(r2Rr1w).*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+cos(...
r2Rr1w).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws))))).*(cos(ps).*cos(...
r2Rr1r).*sin(rs).*((-1).*cos(ps).*cos(rs).*sin(r2Rr1p)+cos(r2Rr1p).*(...
sin(r2Rr1w).*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+cos(...
r2Rr1w).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws))))+sin(r2Rr1r).*(...
cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)).*((-1).*cos(ps).*cos(rs).*...
sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(ws).*sin(rs)+cos(rs).*...
sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(...
ws))))+(cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws)).*((-1).*cos(...
ps).*cos(r2Rr1p).*cos(rs)+(-1).*sin(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(...
ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*...
sin(ps)+sin(rs).*sin(ws)))))+((-1).*(cos(rs).*cos(ws)+sin(ps).*sin(rs).*...
sin(ws)).*(cos(ps).*cos(r2Rr1p).*cos(rs).*sin(r2Rr1r)+(cos(r2Rr1r).*cos(...
r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(ws).*sin(rs)+...
cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*...
cos(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))...
+(-1).*cos(ps).*sin(rs).*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*cos(rs)+(...
cos(r2Rr1w).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(...
cos(ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1r).*cos(...
r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(...
ps)+sin(rs).*sin(ws)))+(-1).*(cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*...
sin(ws)).*((-1).*cos(ps).*cos(rs).*sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w)...
.*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(...
rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws))))).*(cos(ps).*cos(r2Rr1r).*cos(...
rs).*((-1).*cos(ps).*cos(rs).*sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w).*((...
-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*...
cos(ws).*sin(ps)+sin(rs).*sin(ws))))+sin(r2Rr1r).*((-1).*cos(ws).*sin(...
rs)+cos(rs).*sin(ps).*sin(ws)).*((-1).*cos(ps).*cos(rs).*sin(r2Rr1p)+...
cos(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*...
sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws))))+(...
cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)).*((-1).*cos(ps).*cos(r2Rr1p)...
.*cos(rs)+(-1).*sin(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(ws).*sin(rs)+cos(...
rs).*sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*...
sin(ws)))))),(((-1).*cos(r2Rr1p).*(cos(r2Rr1w+(-1).*ws).*sin(rs)+cos(rs)...
.*sin(ps).*sin(r2Rr1w+(-1).*ws)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*...
sin(ws))+cos(ps).*cos(rs).*((-1).*(cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)...
+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*...
sin(ws))+(cos(r2Rr1w).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(...
r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+((-1).*cos(ws).*...
sin(rs)+cos(rs).*sin(ps).*sin(ws)).*((cos(r2Rr1w).*sin(r2Rr1p).*sin(...
r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(ws).*sin(rs)+cos(rs)...
.*sin(ps).*sin(ws))+(-1).*(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*sin(...
r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))).*(...
(-1).*(cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)).*(cos(ps).*cos(...
r2Rr1p).*cos(rs).*sin(r2Rr1r)+(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*...
sin(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*...
sin(ws))+(cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(...
r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+(-1).*cos(ps).*...
sin(rs).*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*cos(rs)+(cos(r2Rr1w).*sin(...
r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(ws).*sin(rs)+...
(-1).*cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+...
sin(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))...
+(-1).*(cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws)).*((-1).*cos(...
ps).*cos(rs).*sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(ws).*...
sin(rs)+cos(rs).*sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*sin(...
ps)+sin(rs).*sin(ws)))))+((-1).*cos(r2Rr1p).*(cos(r2Rr1w+(-1).*ws).*sin(...
rs)+cos(rs).*sin(ps).*sin(r2Rr1w+(-1).*ws)).*(cos(ws).*sin(ps).*sin(rs)+...
(-1).*cos(rs).*sin(ws))+cos(ps).*sin(rs).*((-1).*(cos(r2Rr1r).*cos(...
r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*...
cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1w).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*...
sin(r2Rr1p).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))...
+(cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)).*((cos(r2Rr1w).*sin(...
r2Rr1p).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(ws).*...
sin(rs)+cos(rs).*sin(ps).*sin(ws))+(-1).*(cos(r2Rr1r).*cos(r2Rr1w)+sin(...
r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*...
sin(ws)))).*(((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws)).*(cos(...
ps).*cos(r2Rr1p).*cos(rs).*sin(r2Rr1r)+(cos(r2Rr1r).*cos(r2Rr1w)+sin(...
r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(ws).*sin(rs)+cos(rs).*...
sin(ps).*sin(ws))+(cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*cos(...
r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+...
cos(ps).*cos(rs).*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*cos(rs)+(cos(...
r2Rr1w).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(...
ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w)...
.*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(...
rs).*sin(ws)))+(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)).*((-1).*cos(...
ps).*cos(rs).*sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(ws).*...
sin(rs)+cos(rs).*sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*sin(...
ps)+sin(rs).*sin(ws)))))).*(((cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)...
).*(cos(ps).*cos(r2Rr1p).*cos(rs).*sin(r2Rr1r)+(cos(r2Rr1r).*cos(r2Rr1w)...
+sin(r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(ws).*sin(rs)+cos(rs)...
.*sin(ps).*sin(ws))+(cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*cos(...
r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+...
cos(ps).*sin(rs).*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*cos(rs)+(cos(...
r2Rr1w).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(...
ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w)...
.*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(...
rs).*sin(ws)))+(cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws)).*((-1)...
.*cos(ps).*cos(rs).*sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(...
ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*...
sin(ps)+sin(rs).*sin(ws))))).^2+(((-1).*cos(ws).*sin(rs)+cos(rs).*sin(...
ps).*sin(ws)).*(cos(ps).*cos(r2Rr1p).*cos(rs).*sin(r2Rr1r)+(cos(r2Rr1r)...
.*cos(r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(ws).*...
sin(rs)+cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1w).*sin(r2Rr1p).*sin(...
r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(...
rs).*sin(ws)))+cos(ps).*cos(rs).*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*...
cos(rs)+(cos(r2Rr1w).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(...
r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*sin(ws))+(cos(...
r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*...
cos(ws).*sin(ps)+sin(rs).*sin(ws)))+(cos(rs).*cos(ws).*sin(ps)+sin(rs).*...
sin(ws)).*((-1).*cos(ps).*cos(rs).*sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w)...
.*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(...
rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws))))).^2).^(-1);0,0,0,(1/4).*(((...
cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)).*(cos(ps).*cos(r2Rr1p).*cos(...
rs).*sin(r2Rr1r)+(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*...
sin(r2Rr1w)).*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+(cos(...
r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1w)).*(cos(...
rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+cos(ps).*sin(rs).*(cos(ps).*...
cos(r2Rr1p).*cos(r2Rr1r).*cos(rs)+(cos(r2Rr1w).*sin(r2Rr1r)+(-1).*cos(...
r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*cos(rs).*...
sin(ps).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r).*...
sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+(cos(ws).*...
sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws)).*((-1).*cos(ps).*cos(rs).*sin(...
r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(...
ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))...
)).^2+(((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws)).*(cos(ps).*...
cos(r2Rr1p).*cos(rs).*sin(r2Rr1r)+(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p)...
.*sin(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*...
sin(ws))+(cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(...
r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+cos(ps).*cos(rs)...
.*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*cos(rs)+(cos(r2Rr1w).*sin(r2Rr1r)+...
(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*...
cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(...
r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+(...
cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)).*((-1).*cos(ps).*cos(rs).*...
sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(ws).*sin(rs)+cos(rs).*...
sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(...
ws))))).^2).^(-1/2).*(2.*cos(ps).^2.*cos(rs).*(sin(r2Rr1p).*sin(r2Rr1r)...
.*sin(r2Rr1w).*sin(ws)+(-1).*cos(r2Rr1r).*(cos(ws).*sin(r2Rr1w)+2.*cos(...
r2Rr1p).*sin(ws))+cos(r2Rr1w).*(cos(ws).*sin(r2Rr1p).*sin(r2Rr1r)+cos(...
r2Rr1r).*sin(ws)))+2.*sin(r2Rr1w).*(sin(r2Rr1p).*sin(r2Rr1r).*(2.*cos(...
ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*(1+sin(ps).^2).*sin(ws))+cos(...
r2Rr1r).*(cos(rs).*cos(ws).*(1+sin(ps).^2)+2.*sin(ps).*sin(rs).*sin(ws))...
)+cos(r2Rr1w).*(4.*sin(ps).*sin(rs).*(cos(r2Rr1r).*cos(ws)+(-1).*sin(...
r2Rr1p).*sin(r2Rr1r).*sin(ws))+cos(rs).*(((-3)+cos(2.*ps)).*cos(ws).*...
sin(r2Rr1p).*sin(r2Rr1r)+2.*sin(ws).*((-1).*cos(r2Rr1r).*(1+sin(ps).^2)+...
sin(2.*ps).*sin(r2Rr1r).*sin(ws))))+(-2).*cos(ps).*(2.*cos(r2Rr1p).*cos(...
rs).*sin(ps).*sin(r2Rr1r)+2.*cos(r2Rr1r).*sin(r2Rr1p).*(cos(rs).*cos(...
r2Rr1w+(-1).*ws).*sin(ps)+(-1).*sin(rs).*sin(r2Rr1w+(-1).*ws)).*sin(ws)+...
sin(r2Rr1r).*(2.*sin(r2Rr1w).*sin(rs).*sin(ws).^2+cos(rs).*sin(ps).*sin(...
r2Rr1w).*sin(2.*ws)+cos(r2Rr1w).*sin(rs).*sin(2.*ws)))),(1/8).*(((cos(...
rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)).*(cos(ps).*cos(r2Rr1p).*cos(rs)...
.*sin(r2Rr1r)+(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*sin(...
r2Rr1w)).*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+(cos(...
r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1w)).*(cos(...
rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+cos(ps).*sin(rs).*(cos(ps).*...
cos(r2Rr1p).*cos(r2Rr1r).*cos(rs)+(cos(r2Rr1w).*sin(r2Rr1r)+(-1).*cos(...
r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*cos(rs).*...
sin(ps).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r).*...
sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+(cos(ws).*...
sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws)).*((-1).*cos(ps).*cos(rs).*sin(...
r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(...
ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))...
)).^2+(((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws)).*(cos(ps).*...
cos(r2Rr1p).*cos(rs).*sin(r2Rr1r)+(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p)...
.*sin(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*...
sin(ws))+(cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(...
r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+cos(ps).*cos(rs)...
.*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*cos(rs)+(cos(r2Rr1w).*sin(r2Rr1r)+...
(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*...
cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(...
r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+(...
cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)).*((-1).*cos(ps).*cos(rs).*...
sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(ws).*sin(rs)+cos(rs).*...
sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(...
ws))))).^2).^(-1/2).*(cos(r2Rr1p).*(8.*sin(rs).*sin(r2Rr1w+(-1).*ws).*((...
-1).*cos(r2Rr1r).*sin(ps)+cos(ps).*sin(r2Rr1r).*sin(ws))+cos(rs).*(4.*...
cos(ps).*cos(ws).^2.*sin(ps).*sin(r2Rr1r).*sin(r2Rr1w)+sin(r2Rr1w).*(((...
-3)+cos(2.*ws)).*sin(2.*ps).*sin(r2Rr1r)+8.*cos(r2Rr1r).*sin(ps).^2.*...
sin(ws))+(-4).*cos(ws).*((-1)+(1/2).*((-3)+cos(2.*ps)).*cos(r2Rr1r).*...
cos(r2Rr1w)+cos(ps).^2.*((-1)+cos(r2Rr1r).*cos(r2Rr1w))+sin(ps).^2+cos(...
r2Rr1w).*sin(2.*ps).*sin(r2Rr1r).*sin(ws))))+4.*cos(ps).*sin(r2Rr1p).*((...
-2).*cos(r2Rr1r).*cos(rs).*sin(ps)+(-1).*sin(r2Rr1w).*sin(rs)+(-1).*cos(...
ws).^2.*sin(r2Rr1w).*sin(rs)+2.*cos(ps).*cos(rs).*sin(r2Rr1r).*sin(ws)+...
sin(r2Rr1w).*sin(rs).*sin(ws).^2+cos(rs).*sin(ps).*sin(r2Rr1w).*sin(2.*...
ws)+cos(r2Rr1w).*(2.*cos(rs).*cos(ws).^2.*sin(ps)+sin(rs).*sin(2.*ws))))...
,(1/8).*(cos(r2Rr1w).*(cos(rs).*(4.*cos(ps).*cos(ws).^2.*sin(ps).*sin(...
r2Rr1p).*sin(r2Rr1r)+8.*cos(ws).*sin(ps).*(sin(ps).*sin(r2Rr1r)+cos(ps)...
.*((-1).*cos(r2Rr1p)+cos(r2Rr1r)).*sin(ws))+sin(r2Rr1p).*(((-3)+cos(2.*...
ws)).*sin(2.*ps).*sin(r2Rr1r)+8.*cos(r2Rr1r).*sin(ps).^2.*sin(ws)))+8.*...
sin(rs).*(sin(ps).*((-1).*cos(r2Rr1r).*cos(ws).*sin(r2Rr1p)+sin(r2Rr1r)...
.*sin(ws))+cos(ps).*(cos(r2Rr1p).*cos(ws).^2+cos(ws).*sin(r2Rr1p).*sin(...
r2Rr1r).*sin(ws)+cos(r2Rr1r).*sin(ws).^2)))+8.*sin(r2Rr1w).*((-1).*sin(...
ps).*(sin(r2Rr1r).*(cos(ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*sin(ws))+...
cos(r2Rr1r).*sin(r2Rr1p).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+...
cos(ps).*(cos(r2Rr1p).*cos(ws).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(...
ws))+sin(ws).*(cos(rs).*sin(ps).*(cos(ws).*sin(r2Rr1p).*sin(r2Rr1r)+cos(...
r2Rr1r).*sin(ws))+sin(rs).*((-1).*cos(r2Rr1r).*cos(ws)+sin(r2Rr1p).*sin(...
r2Rr1r).*sin(ws)))))).*(((cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)).*(...
cos(ps).*cos(r2Rr1p).*cos(rs).*sin(r2Rr1r)+(cos(r2Rr1r).*cos(r2Rr1w)+...
sin(r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(ws).*sin(rs)+cos(rs)...
.*sin(ps).*sin(ws))+(cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*cos(...
r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws)))+...
cos(ps).*sin(rs).*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*cos(rs)+(cos(...
r2Rr1w).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(...
ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w)...
.*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(...
rs).*sin(ws)))+(cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws)).*((-1)...
.*cos(ps).*cos(rs).*sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w).*((-1).*cos(...
ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(rs).*cos(ws).*...
sin(ps)+sin(rs).*sin(ws))))).^2+(((-1).*cos(ws).*sin(rs)+cos(rs).*sin(...
ps).*sin(ws)).*(cos(ps).*cos(r2Rr1p).*cos(rs).*sin(r2Rr1r)+(cos(r2Rr1r)...
.*cos(r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*((-1).*cos(ws).*...
sin(rs)+cos(rs).*sin(ps).*sin(ws))+(cos(r2Rr1w).*sin(r2Rr1p).*sin(...
r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws).*sin(ps)+sin(...
rs).*sin(ws)))+cos(ps).*cos(rs).*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*...
cos(rs)+(cos(r2Rr1w).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(...
r2Rr1w)).*(cos(ws).*sin(rs)+(-1).*cos(rs).*sin(ps).*sin(ws))+(cos(...
r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*...
cos(ws).*sin(ps)+sin(rs).*sin(ws)))+(cos(rs).*cos(ws).*sin(ps)+sin(rs).*...
sin(ws)).*((-1).*cos(ps).*cos(rs).*sin(r2Rr1p)+cos(r2Rr1p).*(sin(r2Rr1w)...
.*((-1).*cos(ws).*sin(rs)+cos(rs).*sin(ps).*sin(ws))+cos(r2Rr1w).*(cos(...
rs).*cos(ws).*sin(ps)+sin(rs).*sin(ws))))).^2).^(-1/2);0,0,0,(((-1).*...
cos(r2Rr1p).*sin(ps).*(sin(ps).*sin(r2Rr1r)+cos(ps).*cos(r2Rr1r).*sin(...
ws))+cos(ps).*(cos(r2Rr1w+(-1).*ws).*sin(r2Rr1p).*(sin(ps).*sin(r2Rr1r)+...
cos(ps).*cos(r2Rr1r).*sin(ws))+sin(r2Rr1w+(-1).*ws).*((-1).*cos(r2Rr1r)...
.*sin(ps)+cos(ps).*sin(r2Rr1r).*sin(ws)))).*(cos(ps).*cos(ws).*(cos(ps)...
.*sin(r2Rr1p).*sin(rs)+(-1).*cos(r2Rr1p).*(cos(r2Rr1w+(-1).*ws).*sin(ps)...
.*sin(rs)+cos(rs).*sin(r2Rr1w+(-1).*ws)))+sin(ps).*(cos(ps).*cos(r2Rr1p)...
.*cos(r2Rr1r).*sin(rs)+(cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(...
r2Rr1r).*sin(r2Rr1w)).*(cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws)...
)+((-1).*cos(r2Rr1w).*sin(r2Rr1r)+cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w))...
.*(cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)))+(-1).*cos(ps).*sin(ws).*...
(cos(ps).*cos(r2Rr1p).*sin(r2Rr1r).*sin(rs)+(cos(r2Rr1w).*sin(r2Rr1p).*...
sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1w)).*(cos(ws).*sin(ps).*sin(rs)+...
(-1).*cos(rs).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*sin(...
r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws))))+(...
cos(ps).*sin(ws).*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*sin(rs)+(cos(...
r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(ws).*...
sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws))+((-1).*cos(r2Rr1w).*sin(r2Rr1r)...
+cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(rs).*cos(ws)+sin(ps).*sin(...
rs).*sin(ws)))+(-1).*sin(ps).*((-1).*cos(ps).*cos(r2Rr1p).*sin(r2Rr1r).*...
sin(rs)+((-1).*cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+cos(r2Rr1r).*sin(...
r2Rr1w)).*(cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws))+(-1).*(cos(...
r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*...
cos(ws)+sin(ps).*sin(rs).*sin(ws)))).*(cos(r2Rr1p).*cos(r2Rr1r).*sin(ps)...
.^2+(-1).*cos(ps).*sin(ps).*(cos(r2Rr1r).*cos(r2Rr1w+(-1).*ws).*sin(...
r2Rr1p)+(-1).*cos(ws).*sin(r2Rr1p)+sin(r2Rr1r).*(sin(r2Rr1w+(-1).*ws)+...
cos(r2Rr1p).*sin(ws)))+cos(ps).^2.*(cos(r2Rr1p).*cos(r2Rr1w+(-1).*ws).*...
cos(ws)+sin(ws).*(cos(r2Rr1w).*(cos(ws).*sin(r2Rr1p).*sin(r2Rr1r)+cos(...
r2Rr1r).*sin(ws))+sin(r2Rr1w).*((-1).*cos(r2Rr1r).*cos(ws)+sin(r2Rr1p).*...
sin(r2Rr1r).*sin(ws)))))).*((cos(ps).*cos(ws).*((-1).*cos(ps).*sin(...
r2Rr1p).*sin(rs)+cos(r2Rr1p).*(cos(r2Rr1w+(-1).*ws).*sin(ps).*sin(rs)+...
cos(rs).*sin(r2Rr1w+(-1).*ws)))+(-1).*sin(ps).*(cos(ps).*cos(r2Rr1p).*...
cos(r2Rr1r).*sin(rs)+(cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r)...
.*sin(r2Rr1w)).*(cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws))+((-1)...
.*cos(r2Rr1w).*sin(r2Rr1r)+cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(...
rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)))+cos(ps).*sin(ws).*(cos(ps).*...
cos(r2Rr1p).*sin(r2Rr1r).*sin(rs)+(cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)...
+(-1).*cos(r2Rr1r).*sin(r2Rr1w)).*(cos(ws).*sin(ps).*sin(rs)+(-1).*cos(...
rs).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*sin(...
r2Rr1w)).*(cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)))).^2+(cos(r2Rr1p)...
.*cos(r2Rr1r).*sin(ps).^2+(-1).*cos(ps).*sin(ps).*(cos(r2Rr1r).*cos(...
r2Rr1w+(-1).*ws).*sin(r2Rr1p)+(-1).*cos(ws).*sin(r2Rr1p)+sin(r2Rr1r).*(...
sin(r2Rr1w+(-1).*ws)+cos(r2Rr1p).*sin(ws)))+cos(ps).^2.*(cos(r2Rr1p).*...
cos(r2Rr1w+(-1).*ws).*cos(ws)+sin(ws).*(cos(r2Rr1w).*(cos(ws).*sin(...
r2Rr1p).*sin(r2Rr1r)+cos(r2Rr1r).*sin(ws))+sin(r2Rr1w).*((-1).*cos(...
r2Rr1r).*cos(ws)+sin(r2Rr1p).*sin(r2Rr1r).*sin(ws))))).^2).^(-1),(((-1)...
.*cos(r2Rr1r).*sin(ps).^2.*sin(r2Rr1p)+cos(ps).^2.*cos(r2Rr1w+(-1).*ws)...
.*((-1).*cos(ws).*sin(r2Rr1p)+cos(r2Rr1p).*sin(r2Rr1r).*sin(ws))+cos(ps)...
.*sin(ps).*(cos(r2Rr1p).*((-1).*cos(r2Rr1r).*cos(r2Rr1w+(-1).*ws)+cos(...
ws))+sin(r2Rr1p).*sin(r2Rr1r).*sin(ws))).*(cos(ps).*cos(ws).*(cos(ps).*...
sin(r2Rr1p).*sin(rs)+(-1).*cos(r2Rr1p).*(cos(r2Rr1w+(-1).*ws).*sin(ps).*...
sin(rs)+cos(rs).*sin(r2Rr1w+(-1).*ws)))+sin(ps).*(cos(ps).*cos(r2Rr1p).*...
cos(r2Rr1r).*sin(rs)+(cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r)...
.*sin(r2Rr1w)).*(cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws))+((-1)...
.*cos(r2Rr1w).*sin(r2Rr1r)+cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(...
rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)))+(-1).*cos(ps).*sin(ws).*(cos(...
ps).*cos(r2Rr1p).*sin(r2Rr1r).*sin(rs)+(cos(r2Rr1w).*sin(r2Rr1p).*sin(...
r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1w)).*(cos(ws).*sin(ps).*sin(rs)+(-1)...
.*cos(rs).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*...
sin(r2Rr1w)).*(cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws))))+((-1).*cos(...
r2Rr1p).*cos(r2Rr1r).*sin(ps).*(cos(r2Rr1w+(-1).*ws).*sin(ps).*sin(rs)+...
cos(rs).*sin(r2Rr1w+(-1).*ws))+(-1).*cos(ps).^2.*sin(rs).*(cos(r2Rr1p).*...
cos(ws)+sin(r2Rr1p).*sin(r2Rr1r).*sin(ws))+cos(ps).*(cos(r2Rr1r).*sin(...
ps).*sin(r2Rr1p).*sin(rs)+(-1).*(cos(r2Rr1w+(-1).*ws).*sin(ps).*sin(rs)+...
cos(rs).*sin(r2Rr1w+(-1).*ws)).*(cos(ws).*sin(r2Rr1p)+(-1).*cos(r2Rr1p)...
.*sin(r2Rr1r).*sin(ws)))).*(cos(r2Rr1p).*cos(r2Rr1r).*sin(ps).^2+(-1).*...
cos(ps).*sin(ps).*(cos(r2Rr1r).*cos(r2Rr1w+(-1).*ws).*sin(r2Rr1p)+(-1).*...
cos(ws).*sin(r2Rr1p)+sin(r2Rr1r).*(sin(r2Rr1w+(-1).*ws)+cos(r2Rr1p).*...
sin(ws)))+cos(ps).^2.*(cos(r2Rr1p).*cos(r2Rr1w+(-1).*ws).*cos(ws)+sin(...
ws).*(cos(r2Rr1w).*(cos(ws).*sin(r2Rr1p).*sin(r2Rr1r)+cos(r2Rr1r).*sin(...
ws))+sin(r2Rr1w).*((-1).*cos(r2Rr1r).*cos(ws)+sin(r2Rr1p).*sin(r2Rr1r).*...
sin(ws)))))).*((cos(ps).*cos(ws).*((-1).*cos(ps).*sin(r2Rr1p).*sin(rs)+...
cos(r2Rr1p).*(cos(r2Rr1w+(-1).*ws).*sin(ps).*sin(rs)+cos(rs).*sin(...
r2Rr1w+(-1).*ws)))+(-1).*sin(ps).*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*...
sin(rs)+(cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w))...
.*(cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws))+((-1).*cos(r2Rr1w)...
.*sin(r2Rr1r)+cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(rs).*cos(ws)+...
sin(ps).*sin(rs).*sin(ws)))+cos(ps).*sin(ws).*(cos(ps).*cos(r2Rr1p).*...
sin(r2Rr1r).*sin(rs)+(cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*cos(...
r2Rr1r).*sin(r2Rr1w)).*(cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws)...
)+(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*(...
cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)))).^2+(cos(r2Rr1p).*cos(...
r2Rr1r).*sin(ps).^2+(-1).*cos(ps).*sin(ps).*(cos(r2Rr1r).*cos(r2Rr1w+(...
-1).*ws).*sin(r2Rr1p)+(-1).*cos(ws).*sin(r2Rr1p)+sin(r2Rr1r).*(sin(...
r2Rr1w+(-1).*ws)+cos(r2Rr1p).*sin(ws)))+cos(ps).^2.*(cos(r2Rr1p).*cos(...
r2Rr1w+(-1).*ws).*cos(ws)+sin(ws).*(cos(r2Rr1w).*(cos(ws).*sin(r2Rr1p).*...
sin(r2Rr1r)+cos(r2Rr1r).*sin(ws))+sin(r2Rr1w).*((-1).*cos(r2Rr1r).*cos(...
ws)+sin(r2Rr1p).*sin(r2Rr1r).*sin(ws))))).^2).^(-1),(cos(ps).*((-1).*...
cos(r2Rr1w+(-1).*ws).*(sin(ps).*sin(r2Rr1r)+cos(ps).*cos(r2Rr1r).*sin(...
ws))+sin(r2Rr1w+(-1).*ws).*(cos(r2Rr1r).*sin(ps).*sin(r2Rr1p)+(-1).*cos(...
ps).*(cos(r2Rr1p).*cos(ws)+sin(r2Rr1p).*sin(r2Rr1r).*sin(ws)))).*(cos(...
ps).*cos(ws).*(cos(ps).*sin(r2Rr1p).*sin(rs)+(-1).*cos(r2Rr1p).*(cos(...
r2Rr1w+(-1).*ws).*sin(ps).*sin(rs)+cos(rs).*sin(r2Rr1w+(-1).*ws)))+sin(...
ps).*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*sin(rs)+(cos(r2Rr1r).*cos(...
r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(ws).*sin(ps).*sin(...
rs)+(-1).*cos(rs).*sin(ws))+((-1).*cos(r2Rr1w).*sin(r2Rr1r)+cos(r2Rr1r)...
.*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)...
))+(-1).*cos(ps).*sin(ws).*(cos(ps).*cos(r2Rr1p).*sin(r2Rr1r).*sin(rs)+(...
cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1w)).*(...
cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws))+(cos(r2Rr1r).*cos(...
r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws)+sin(...
ps).*sin(rs).*sin(ws))))+(cos(ps).*cos(r2Rr1p).*cos(ws).*(cos(rs).*cos(...
r2Rr1w+(-1).*ws)+(-1).*sin(ps).*sin(rs).*sin(r2Rr1w+(-1).*ws))+cos(ps).*...
sin(ws).*((-1).*(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*sin(...
r2Rr1w)).*(cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws))+(cos(...
r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1w)).*(cos(...
rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)))+(-1).*sin(ps).*((cos(r2Rr1w).*...
sin(r2Rr1r)+(-1).*cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(ws).*sin(...
ps).*sin(rs)+(-1).*cos(rs).*sin(ws))+(cos(r2Rr1r).*cos(r2Rr1w).*sin(...
r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(rs).*cos(ws)+sin(ps).*sin(rs).*...
sin(ws)))).*(cos(r2Rr1p).*cos(r2Rr1r).*sin(ps).^2+(-1).*cos(ps).*sin(ps)...
.*(cos(r2Rr1r).*cos(r2Rr1w+(-1).*ws).*sin(r2Rr1p)+(-1).*cos(ws).*sin(...
r2Rr1p)+sin(r2Rr1r).*(sin(r2Rr1w+(-1).*ws)+cos(r2Rr1p).*sin(ws)))+cos(...
ps).^2.*(cos(r2Rr1p).*cos(r2Rr1w+(-1).*ws).*cos(ws)+sin(ws).*(cos(...
r2Rr1w).*(cos(ws).*sin(r2Rr1p).*sin(r2Rr1r)+cos(r2Rr1r).*sin(ws))+sin(...
r2Rr1w).*((-1).*cos(r2Rr1r).*cos(ws)+sin(r2Rr1p).*sin(r2Rr1r).*sin(ws)))...
))).*((cos(ps).*cos(ws).*((-1).*cos(ps).*sin(r2Rr1p).*sin(rs)+cos(...
r2Rr1p).*(cos(r2Rr1w+(-1).*ws).*sin(ps).*sin(rs)+cos(rs).*sin(r2Rr1w+(...
-1).*ws)))+(-1).*sin(ps).*(cos(ps).*cos(r2Rr1p).*cos(r2Rr1r).*sin(rs)+(...
cos(r2Rr1r).*cos(r2Rr1w).*sin(r2Rr1p)+sin(r2Rr1r).*sin(r2Rr1w)).*(cos(...
ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws))+((-1).*cos(r2Rr1w).*sin(...
r2Rr1r)+cos(r2Rr1r).*sin(r2Rr1p).*sin(r2Rr1w)).*(cos(rs).*cos(ws)+sin(...
ps).*sin(rs).*sin(ws)))+cos(ps).*sin(ws).*(cos(ps).*cos(r2Rr1p).*sin(...
r2Rr1r).*sin(rs)+(cos(r2Rr1w).*sin(r2Rr1p).*sin(r2Rr1r)+(-1).*cos(...
r2Rr1r).*sin(r2Rr1w)).*(cos(ws).*sin(ps).*sin(rs)+(-1).*cos(rs).*sin(ws)...
)+(cos(r2Rr1r).*cos(r2Rr1w)+sin(r2Rr1p).*sin(r2Rr1r).*sin(r2Rr1w)).*(...
cos(rs).*cos(ws)+sin(ps).*sin(rs).*sin(ws)))).^2+(cos(r2Rr1p).*cos(...
r2Rr1r).*sin(ps).^2+(-1).*cos(ps).*sin(ps).*(cos(r2Rr1r).*cos(r2Rr1w+(...
-1).*ws).*sin(r2Rr1p)+(-1).*cos(ws).*sin(r2Rr1p)+sin(r2Rr1r).*(sin(...
r2Rr1w+(-1).*ws)+cos(r2Rr1p).*sin(ws)))+cos(ps).^2.*(cos(r2Rr1p).*cos(...
r2Rr1w+(-1).*ws).*cos(ws)+sin(ws).*(cos(r2Rr1w).*(cos(ws).*sin(r2Rr1p).*...
sin(r2Rr1r)+cos(r2Rr1r).*sin(ws))+sin(r2Rr1w).*((-1).*cos(r2Rr1r).*cos(...
ws)+sin(r2Rr1p).*sin(r2Rr1r).*sin(ws))))).^2).^(-1)];
