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
function [ x chisq alpha J iter ] = nrlm_dualquat( fun, x0, params, plot )

done = 0;
alamda = .001;
x = x0;

if nargin > 2
    ITMAX = params(1);
    NDONE = params(2);
    tol = params(3);
    doOutput = params(4);
else
    ITMAX = 1000;
    NDONE = 4;
    tol = 1.e-6;
    doOutput = 1;
end

if nargin > 3
    plot(x);
end

tStart = tic;

% Initialization
[r J] = feval(fun,x,0);
chisq = sum(r.^2);
ochisq = chisq;

% Create matrix and vector
DOF = size(J,2);
alpha = sparse(DOF, DOF);

for iter=1:ITMAX
    if nargin > 3
        plot(x);
    end
    
    % Print status
    if doOutput == 1
        fprintf('%d       \t%12.20f\t', iter, full(ochisq(1,1)));
        toc(tStart);
        tStart = tic;
    end
    
    % Last pass. Use zero alamda
    if (done == NDONE)
        alamda = 0.;
    end;
    
    % Alter linearized fitting matrix, by 
    % augmenting diagonal elements
    alpha = J' * J + alamda * sparse(eye(DOF));
    beta = J' * r;
    
    % Matrix solution (CHOLMOD)
    da = alpha \ -beta; 
    
    % alternative (CSparse)
    %da = cs_cholsol(alpha, full(-beta));
   
    % Converged. Return
    if (done == NDONE)
        return;
    end;
    
    % Did the trial succeed?
    %x_new = x + da;
    x_new = UpdateDualQuatWithExpOffset(x,da);
    
    [r_new J_new] = feval(fun, x_new, iter);
    chisq = sum(r_new.^2);
    if (abs(chisq - ochisq) < max(tol, tol * chisq))
        done = done + 1;
        if nargout < 3 && done == NDONE
            % Converged. alpha not requested. Update x and return
            if chisq < ochisq
                x = x_new;
            end
            return;
        end
    end;
    
    if (chisq < ochisq) % Success, accept new solution
        alamda = alamda * .1;%sqrt(.1);
        ochisq = chisq;
        r = r_new;
        J = J_new;
        x = x_new;
    else % Failure, increase lambda
        alamda = alamda * 10.;%sqrt(10.);
        chisq = ochisq;
    end;
end;
end

