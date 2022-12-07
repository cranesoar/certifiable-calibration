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
function newStates = ResampleAbsStatesDualQuat(stateTimes, absStates, newStateTimes)

    N = length(stateTimes);

    interpIndexes = interp1(stateTimes, 1:length(stateTimes), newStateTimes, 'linear', 'extrap');
    index1 = floor(interpIndexes);
    index2 = ceil(interpIndexes);
    frac = interpIndexes - index1;
    
    lowmask=(index1<1);
    index1(lowmask) = 1;
    index2(lowmask) = 2;
    frac(lowmask) = (newStateTimes(lowmask) - stateTimes(index1(lowmask))) ./ ( stateTimes(index2(lowmask)) - stateTimes(index1(lowmask)) );
    
    highmask = (index2>N);
    index1(highmask) = N-1;
    index2(highmask) = N;
    frac(highmask) = (newStateTimes(highmask) - stateTimes(index1(highmask))) ./ ( stateTimes(index2(highmask)) - stateTimes(index1(highmask)) );
    
%     if any(lowmask)
%         fprintf('warning: extrapolating times (the lowmask code above may not be well tested)\n');
%         keyboard;
%     end
       
    newStates = SlerpDualQuat(absStates(index1,:), absStates(index2,:), frac);
