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
function [states_at_new_times, covariances_at_new_times, new_times, meanIters, S, subMatRConds] = InterpolateStatesAndCovariances_DualQuat(times, relPositions_at_times, covariances_at_times, new_times)

    % relPositions_at_times Nx6 vector of euler states
    % covariances_at_times Nx36 covariances of euler states
    
    % states_at_new_times Nx8 vector of interpolated dual quaternion states
    % covariances_at_new_times 6Nx6N covariance matrix in lie algebra

    %if ~exist('times', 'var')
    %    Test_Interpolation();
    %    return;
    %end
       
    state = reshape(relPositions_at_times', [], 1);
 
    % construct observation covariance matrix
    S = zeros(size(covariances_at_times,1)*6, size(covariances_at_times,1)*6);
    for i = 1:size(covariances_at_times,1)
        r1=(i-1)*6+1;
        S(r1:r1+5, r1:r1+5) = reshape(covariances_at_times(i,:), 6, 6);
    end
    S = triu(S,0)+triu(S,1)'; %force S to be symmetric
   
    [states_dq, covariances_at_new_times, meanIters] = UnscentedTransform_DualQuat(@(x)func(x, times, new_times), state, S, @plusR, @MeanOfSigmaPointsDualQuat, @minusDQ);
    states_at_new_times = reshape(states_dq, 8, [])';
    
    subMatRConds = zeros(size(covariances_at_times,1),2);
    for i = 1:size(covariances_at_times,1)
        r1=(i-1)*6+1;
        %subMatRConds(i,:) = [rcond(S(r1:r1+5, r1:r1+5)) rcond(covariances_at_new_times(r1:r1+5, r1:r1+5))];
        subMatRConds(i,:) = rcond(S(r1:r1+5, r1:r1+5));
    end
    
    %sfprintf('rcond(cov_euler)=%g, rcond(cov_dq)=%g\n', rcond(S), rcond(covariances_at_new_times));
    
function s = plusR(a, b)
    s = a + b;
    
function d = minusDQ(a, b)    
    % a is a N*8 x M matrix where each column is a separate state
    % b is a N*8 x M matrix where each column is a separate state
    d = zeros(size(a,1)/8*6, size(a,2));
    parfor i = 1:size(a,2)
        d(:,i) = DiffBetweenDualQuat(a(:,i), b(:,i));
    end    
    
function [e,J] = MeanErr(b, iter, ys, w) 

    b_dq = reshape(b,8,[])';

    e = zeros(size(b_dq,1),6);
    for i = 1:size(b_dq,1)

        idx = [0:7]+8*(i-1)+1;
        q_i = ys(idx,:)';
        b_i = InverseDualQuat(b_dq(i,:));
        p_i = ConcatDualQuat( repmat(b_i, size(q_i,1), 1), q_i );
        e_i = sum(bsxfun(@times, w'/sum(abs(w)), LogDualQuat(p_i)),1);

        e(i,:) = e_i;
    end
    e = reshape(e',[],1);
    
    if nargout > 1
        J = JAvg(b, ys, w);
        %x0 = zeros(6*size(b_dq,1),1);
        %J = jacobianest(@(x)funJ(x, b, iter, ys, w), x0);
    end

% function cost = funJ(expoffset, b, iter, ys, w)
%     offset_b = UpdateDualQuatWithExpOffset(b, expoffset);   
%     cost = MeanErr(offset_b, iter, ys, w);
    
function [mu, iter] = MeanOfSigmaPointsDualQuat(ys, w)

    %b = DualQuatLinearBlend(w,ys);
    %[mu, chisq, ~, ~, iter] = nrlm_dualquat(@(x,iter)MeanErr(x, iter, ys, w), b, [1000 4 1e-5 1]);
    [mu, chisq, ~, ~, iter] = nrlm_dualquat(@(x,iter)MeanErr(x, iter, ys, w), ys(:,1), [1000 4 1e-5 1]); 
    fprintf('completed mean in %i iterations with err %g\n', iter, chisq);
                   
function y = func(x, times, new_times)      
    
    initialTime = times(1)-mean(diff(times));
    initialNewTime = new_times(1)-mean(diff(new_times));
    
    N = size(x, 2);
    
    y = zeros(8*length(new_times), N);
    
    parfor i = 1:N
        relPositions_dq_at_times = EulerStateToDualQuat(reshape( x(:,i), 6, [] )');
        
        [absState_dq_times, absState_qd] = MakeAbsStatesDualQuat([1 0 0 0 0 0 0 0], initialTime, relPositions_dq_at_times, times);        
        absStates_dq_at_new_times = ResampleAbsStatesDualQuat(absState_dq_times, absState_qd, [initialNewTime; new_times]);        
        relStates_at_new_times =  MakeRelStatesDualQuat(absStates_dq_at_new_times);
        y(:,i) = reshape(relStates_at_new_times', [], 1);

    end
    
function relStates_dq = MakeRelStatesDualQuat(absStates_dq)

    relStates_dq = zeros(size(absStates_dq,1)-1,8);

    prevPose = absStates_dq(1,:);
    for i = 2:size(absStates_dq,1)
        thisPose = absStates_dq(i,:);
        relStates_dq(i-1,:) = ConcatDualQuat(InverseDualQuat(prevPose), thisPose);       
        
        prevPose = thisPose;
    end
     