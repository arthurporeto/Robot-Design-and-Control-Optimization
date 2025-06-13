function [c, ceq] = nonlcon(w, init)
% Set any non-linear inequality constraints
c = [];

% Set any linear equality constraints, i.e., 
% enforce dynamics at all knot points
% Get required values h, N, m, n, x0 and xf from init
% foo = init.foo;
h = init.h;
N = init.N;
m = init.m;
n = init.n;
x0 = init.x0;
xf = init.xf;

% We already know that all the state and control are appended together as
% on large vector and passed to the optimiser
% But to enforce specific constraints, we need to resolve this large vector
% into respective states and controls using index slicing

% Resolve all the states and controls into an index sliced matrix
% First, extract only the elements relevant to control
% Outputs Nx2 matrix where each row is one discretisation of time
tempu = w(n*N+1:end);
% Now reshape tempu such that each row corresponds to control signal at a
% selected time
u = reshape(tempu,m,N)';

% Similarly repeat to extract the state variable values
tempz = w(1:n*N);
% Output Nx4 matrix where each row is one discretisation of time
z = reshape(tempz,n ,N )';
% Enforce the dynamics constraint at each of the knot points using the
% 2R-robot dynamics given in the file `dyn2R`

% Now read about the output of a non-linear constrain, i.e., [c, ceq]
% In this file we only have equality constraints given out as c
% these equality constraints enforce dynamics constraints at the knot points

% Equality constraints at t0 to ensure the initial state
% From the extracted state variables, we first want to constraint the
% first state variables (states at t=0) to be the same as the inital
% conditions
% For example, for the first state, the constraint will be
ceq(1) = z(1,1) - x0(1);
% Similarly enforce the initial constraints to all the states
ceq(2) = z(1,2) - x0(2);
ceq(3) = z(1,3) - x0(3);
ceq(4) = z(1,4) - x0(4);

% Dynamics constraint at all intermediate steps
% the symbolic dynamics of the robot for a given z and u at an instant 'i' can be extracted
% by calling the function dyn2R(z(i,:)',u(i,:)',init);
size(z(1,:));
size(u(1,:));
% Now enforce dynamics constraints for all N-1 times steps
for k=1:N-3
    % Get dynamics function values at k+1 and k
    fk1 = dyn2R(z(k+1,:)',u(k+1,:)',init);
    fk = dyn2R(z(k,:)',u(k,:)',init);
    %Now add the constraints
    %ceq(end+1, 1) = ;
    %Do the same for all 4 states
    %ceq(end+1, 1) = ...
    ceq(end+1) = 0.5*h*(fk1(1)+fk(1))+z(k,1)-z(k+1,1);
    ceq(end+1) = 0.5*h*(fk1(2)+fk(2))+z(k,2)-z(k+1,2);
    ceq(end+1) = 0.5*h*(fk1(3)+fk(3))+z(k,3)-z(k+1,3);
    ceq(end+1) = 0.5*h*(fk1(4)+fk(4))+z(k,4)-z(k+1,4);
end
fk1;
% Equality constraints at tf to ensure the final state
% This is same as the enforced inital constraints
% We want to make sure that the last state to be same as the chosen final
% % state xf
ceq(end+1) = z(N,1) - xf(1);
ceq(end+1) = z(N,2) - xf(2);
ceq(end+1) = z(N,3) - xf(3);
ceq(end+1) = z(N,4) - xf(4);
% 
% 
% % Similarly constrain all the final states
% 
% %% Constraint more last states to not just reach but reach and stay at xf
% % To ensure that the final state is reached not just at the last moment, we
% % constrain also a few states before the final state to xf
% 
% % Similar to above constraint not just z(N, 1) but also for time steps N-1
% % and N-2 to the final state xf this ensures that the solution holds the final state
ceq(end+1) = z(N-1, 1) - xf(1);
ceq(end+1) = z(N-1, 2) - xf(2);
ceq(end+1) = z(N-1, 3) - xf(3);
ceq(end+1) = z(N-1, 4) - xf(4);


ceq(end+1) = z(N-2, 1) - xf(1);
ceq(end+1) = z(N-2, 2) - xf(2);
ceq(end+1) = z(N-2, 3) - xf(3);
ceq(end+1) = z(N-2, 4) - xf(4);

% 
