function [J] = obj(w,init)
% The only input is the entire vector of the appended control and state
% Isolate only the control variables and compute the cost
% Get required variables h, N and m from init
% foo = init.foo
h = init.h;
N = init.N;
m = init.m;
n = init.n;

% Retrieve the list of control variables
tempu = w(n*N+1:end);
% reshape the control list into a mxN matrix
u = reshape(tempu,m,N);
% Now create an objective function that is equivalent to integrating the
% control effort (u) over the entire period of time of motion
% Hint: use trapezoidal integration
% Compute the objective function value
J = 0;
for k=1:N-1
    A = 0.5*h*(u(:,k+1).^2+u(:,k).^2);
    J = J+sum(A);
end
