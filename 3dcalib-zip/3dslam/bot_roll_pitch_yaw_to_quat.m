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
function q = bot_roll_pitch_yaw_to_quat (rpy)

    roll = rpy(1);
    pitch = rpy(2);
    yaw = rpy(3);

    halfroll = roll / 2;
    halfpitch = pitch / 2;
    halfyaw = yaw / 2;

    sin_r2 = sin (halfroll);
    sin_p2 = sin (halfpitch);
    sin_y2 = sin (halfyaw);

    cos_r2 = cos (halfroll);
    cos_p2 = cos (halfpitch);
    cos_y2 = cos (halfyaw);

    q(1) = cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2;
    q(2) = sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2;
    q(3) = cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2;
    q(4) = cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2;
