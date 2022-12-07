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
function [y, Y, meanIters] = UnscentedTransform_DualQuat(func, x, P, inPlusFunc, MeanOfSigmaPointsDualQuat, outMinusFunc)
    % adapted from Tim Bailey's UKF code at http://www-personal.acfr.usyd.edu.au/tbailey/software/matlab_utilities.htm
    % changed to implement the SUT of Simon Julier

    % Set up some values
    D = length(x);  % state dimension
    N = D*2 + 1;    % number of samples

    alpha = 10^-2;
    kappa = 0;
    beta = 2;
    lambda = alpha^2*(D+kappa)-D;  %NB: for kappa=0, and small alpha, lambda ~= -D.  the result is that teh sqrt(lambda+D) is small.  so increasing alpha increases the sigma point's distance from the mean

    Wm0 = lambda/(D+lambda);
    Wmi = 1/2/(D+lambda);

    Wc0 = lambda/(D+lambda) + (1-alpha^2+beta);
    Wci = Wmi;

    % Create samples
    Ps = sqrt_posdef(P) * sqrt(D+lambda); 
    ss = [x, inPlusFunc(repvec(x,D),Ps), inPlusFunc(repvec(x,D),-Ps)];

    % Transform samples according to function 'func'
    ys = feval(func, ss);  % compute (possibly discontinuous) transform

    % Calculate predicted observation mean
    %idx = 2:N; 
    %y = Wm0*ys(:,1) + Wmi*sum(ys(:,idx), 2);
    [y, meanIters] = MeanOfSigmaPointsDualQuat(ys, [Wm0 Wmi*ones(1,N-1)]);

    % Calculate new unscented covariance
    %dy = ys - repvec(y,N);
    %Y  = Wc0*dy(:,1)*dy(:,1)' + Wci*dy(:,idx)*dy(:,idx)';
    dy = outMinusFunc(ys, repvec(y,N));
    idx = 2:N;
    Y = Wc0*dy(:,1)*dy(:,1)' + Wci*dy(:,idx)*dy(:,idx)';

function x = repvec(x,N)
    x = x(:, ones(1,N));
    
function R = sqrt_posdef(P)
    [U,D,~] = svd(P);
    R = U*sqrt(D);    