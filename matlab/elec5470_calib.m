%% Example usage of egomotion_calibration.m
close all; clear; clc;

% Read data matrices

load('../data/sim_ri_05.mat');
R_bc = tform2rotm(T_bc); 
t_bc = tform2trvec(T_bc);

motion_point = length(camera_T);

A_body_R = zeros(3,3,motion_point);
B_sensor_R = zeros(3,3,motion_point);
A_body_T = zeros(3,motion_point);
B_sensor_T = zeros(3,motion_point);

for point = 1 : motion_point-1
    K_cam_last= [camera_R(:,:,point) camera_T(:,point); 0 0 0 1];
    K_cam = [camera_R(:,:,point+1) camera_T(:,point+1); 0 0 0 1];

%     K_leg_last= [robot_gt_R(:,:,point) robot_gt_T(:,point); 0 0 0 1];
%     K_leg = [robot_gt_R(:,:,point+1) robot_gt_T(:,point+1); 0 0 0 1];

%     K_cam_last= [robot_gt_R(:,:,point) robot_gt_T(:,point); 0 0 0 1];
%     K_cam = [robot_gt_R(:,:,point+1) robot_gt_T(:,point+1); 0 0 0 1];

    K_leg_last= [robot_cam_R(:,:,point) robot_cam_T(:,point); 0 0 0 1];
    K_leg = [robot_cam_R(:,:,point+1) robot_cam_T(:,point+1); 0 0 0 1];
    
%     dat_T_cam = inv(K_cam_last) * K_cam;
%     dat_T_leg = inv(K_leg_last) * K_leg;    
    
    dat_T_cam = K_cam_last \ K_cam;
    dat_T_leg = K_leg_last \ K_leg;   
    
    A_body_R(:,:,point) = tform2rotm(dat_T_leg);
    A_body_T(:,point) = tform2trvec(dat_T_leg)';
    B_sensor_R(:,:,point) = tform2rotm(dat_T_cam);
    B_sensor_T(:,point) = tform2trvec(dat_T_cam)'; 
end

[R_cal, t_cal] = egomotion_calibration(B_sensor_R,B_sensor_T,A_body_R,A_body_T);
% Solve
disp('Estimate:');
R_cal
t_cal
disp('Ground Truth:');
R_bc
t_bc