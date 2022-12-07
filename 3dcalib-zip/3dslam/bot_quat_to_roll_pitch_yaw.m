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
function [r,p,y] = bot_quat_to_roll_pitch_yaw (q) 
    roll_a = 2 * (q(1)*q(2) + q(3)*q(4));
    roll_b = 1 - 2 * (q(2)*q(2) + q(3)*q(3));
    r = atan2 (roll_a, roll_b);

    pitch_sin = 2 * (q(1)*q(3) - q(4)*q(2));
    p = asin (pitch_sin);

    yaw_a = 2 * (q(1)*q(4) + q(2)*q(3));
    yaw_b = 1 - 2 * (q(3)*q(3) + q(4)*q(4));
    y = atan2 (yaw_a, yaw_b);