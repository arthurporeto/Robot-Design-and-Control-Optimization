%% Trajectory optimisation: Direct transcription
% Using trapezoidal collocation
clc
clear all
close all

% Declare variables for inital and final time
t0 = 0;
tf = 5;

% Start with the initial values for the variables: 
% l1=0.35, l2=0.3, m1=1.5, m2=0.5, me=1;
% Declare these variables as done yesterday using an "init" struct
% init.g = ;

init = struct('l1',0.35,'l2',0.3,'m1',1.5,'m2',0.5,'me',1);

% We also need to setup additional design parameters for the optimisation
% problem
% N: number of total time steps for discretisation
% h: the step size of time resulting from t0, tf and number of steps N
% x0: inital state of the robot represented as [theta1(0), theta2(0), dtheta1(0), dtheta2(0)]
% xf: final desired state of the robot
% n: number of the state variables
% m: number of control variables
% Now choose the above values appropriately and add them to the init struct
% init.N = ;

init.N = 100;
init.h = (tf-t0)/init.N;
init.x0 = [0,0,0,0];
init.xf = [pi,0,0,0];
init.n = 4;
init.m = 2;
init.g = 1;

h = init.h;
N = init.N;
m = init.m;
n = init.n;
x0 = init.x0;
xf = init.xf;
g = init.g;

% The idea behind this optimisation problem is to break the problem into
% multiple smaller problems and solve them together using knot constraints
% see the slides to understand how we discretise the problem resulting in
% the increase of the number of state and control design variables
% Set an initial guess for all the state values
% use the n and N saved in the struct init to compute the size of this
% vector

% Set initial guess for all the states of the problem -- size = (n*N, 1)
%z0 = 0.1*ones(n*N , 1);
z0 = 0.1*ones(n*N,1);
%z0 = linspace(0,pi,n*N)';
% Similar to above set bounds on all the states of the problem
% use init to compute the size
pattern_l = [-pi;-pi;-100;-100]; % lower bounds
pattern_u = [pi;pi;100;100]; % upper bounds
repeat = n*N/length(pattern_l);
zl = repmat(pattern_l,repeat,1);
zu=repmat(pattern_u,repeat,1);

% Set initial guess for all control inputs -- size = (m*N, 1)
u0 = zeros(m*N,1);
% Set bounds on control
ul = -50.*ones(m*N,1);
uu = 50.*ones(m*N,1);

%% Optimization setup

% Setup options for the optimisation problem, read about `TolX`, `TolFun`,
% `MaxFunEval`, `MaxIter` and use them in the options
% Q1. How does modifying these variables effect the solution obtained?
% %optNLP = optimset('TolX', 1e-6, ...    % Tolerance for x
%                    'TolFun', 1e-4, ...  % Tolerance for function value
%                    'MaxFunEvals', 3e6, ... % Max function evaluations
%                    'MaxIter', 100, ... % Max iterations
%                    'Display', 'iter'); % Show iteration details
optNLP = optimset('MaxIter',5000,'TolFun',1e-4,'MaxFunEvals',3e6,'Display','iter','Algorithm','sqp');
% Write a function to enforce constraint at the `knot points` in a
% function called `nlconst`
% HINT: To pass parameters into the objective function or the constraints
% you can use the function handler. For example if you have an objective function
% obj(x, init), then you can tell the optimiser that only x is a variable
% by giving it the input as @(x)obj(x, init) and similarly for a constraint
% too.

% Now complete the problem setup and solve it using fmincon
A=[];
b=[];
Aeq=[];
beq=[];
w0 = [z0;u0];
lb=[zl;ul];
ub=[zu;uu];
tic 
[opt, fval, exitflag,output] = fmincon(@(w) obj(w,init),w0,A,b,Aeq,beq,lb,ub,@(w) nonlcon(w,init),optNLP);
toc
% NOTE: Be extremely careful when you use the reshape function to convert
% the long vector of states and control into the right shape for plotting

%% Questions

% Q1. Solve the problem using fmincon and provide reasons for your observations
% Q2. Solve the problem using different algorithms available, `sqp`,
% `active-set`, `interior point` and note the following:
%     1. Number of iterations
%     2. Time taken
%     3. Number of function evaluations
%     4. Objective function value at optima
% and also the difference in the obtained solution by plotting the
% corresponding states and torques on the same plot
% tic
% optNLP_sqp = optimset('MaxIter',5000,'TolFun',1e-3,'MaxFunEvals',3e6,'Display','iter','Algorithm','sqp');
% [opt_sqp, fval_sqp,exitflag_sqp,output_sqp] = fmincon(@(w) obj(w,init),w0,A,b,Aeq,beq,lb,ub,@(w) nonlcon(w,init),optNLP_sqp)
% toc
% 
% tic
% optNLP_active = optimset('MaxIter',5000,'TolFun',1e-3,'MaxFunEvals',3e6,'Display','iter','Algorithm','active-set');
% [opt_active, fval_active,exitflat_active,output_active] = fmincon(@(w) obj(w,init),w0,A,b,Aeq,beq,lb,ub,@(w) nonlcon(w,init),optNLP_active)
% toc


% tf = 20;
% init.h = (tf-t0)/init.N;
% 
% tic
% optNLP_sqp_tf = optimset('MaxIter',5000,'TolFun',1e-3,'MaxFunEvals',3e6,'Display','iter','Algorithm','sqp');
% [opt_sqp_tf, fval_sqp_tf,exitflag_sqp_tf,output_sqp_tf] = fmincon(@(w) obj(w,init),w0,A,b,Aeq,beq,lb,ub,@(w) nonlcon(w,init),optNLP_sqp_tf);
% toc
% 
% init.g = 1;
% tic
% optNLP_sqp_g = optimset('MaxIter',5000,'TolFun',1e-3,'MaxFunEvals',3e6,'Display','iter','Algorithm','sqp');
% [opt_sqp_g, fval_sqp_g,exitflag_sqp_g,output_sqp_g = fmincon(@(w) obj(w,init),w0,A,b,Aeq,beq,lb,ub,@(w) nonlcon(w,init),optNLP_sqp_g);
% toc
% Q3. How does the final solution change when:
%     1. tf is changed
%     2. N is changed
%     3. x0 is changed
%     4. xf is changed
%     5. g is changed
%     6. l1, l2, m1, m2 are changed
% and also the difference in the obtained solution by plotting the
% corresponding states and torques on the same plot
% 
% Q4. Give a better initial guess for the states and see how the simulation
% time and iterations and the solution quality changes
% 
% Q5. Modify the bounds on states and control and see how the behavior of
% the solution changes? can you find a solution for a small value of
% control bounds?
% 
% Q6(Bonus). Modify the discretisation scheme from Euler to
% Hermite-Simphson or another and see how system behavior changes 
% (See additional resource from moodle on a tutorial on trajectory optimisation)
% 
% Q7(Bonus). Give the obtained control input to the system and observe the 
% system performance? Does the robot actually reach the final position as expected? 

%% Plotting Assistant: Plotting the optimal control and states
% -------------------------------------- 
% THE FUNCTION BELOW IS FOR PLOTTING YOUR SOLUTION
% YOU MIGHT HAVE TO CHECK AND ADAPT IT BASED ON YOUR DECLARATION OF THE VARIABLES
% THE CODE BELOW IS ONLY A TEMPLATE AND NEEDS TO BE MODIFY AS NEEDED
%  -------------------------------------- 


% Extract optimal values from the optimiser stored in the variable "opt"
tempu = opt(n*N+1:end);
% tempu_sqp = opt_sqp(n*N+1:end);
% tempu_active = opt_active(n*N+1:end);
% tempu_sqp_tf = opt_sqp_tf(n*N+1:end);
% tempu_sqp_g = opt_sqp_g(n*N+1:end);

% Now reshape the long extracted vector into the right shape for control
u = reshape(tempu,m ,N ); 
u(:,end+1) = u(:,end); %adding another column
%u_sqp = reshape(tempu_sqp,m ,N ); 
%u_sqp(:,end+1) = u_sqp(:,end); %adding another column
% u_active = reshape(tempu_active,m ,N ); 
% u_active(:,end+1) = u_active(:,end); %adding another column
% u_sqp_tf = reshape(tempu_sqp_tf,m ,N ); 
% u_sqp_tf(:,end+1) = u_sqp_tf(:,end); %adding another column
% u_sqp_g = reshape(tempu_sqp_g,m ,N ); 
% u_sqp_g(:,end+1) = u_sqp_g(:,end); %adding another column

% Similarly extract the state variables from the optimiser also from the variable "opt"
tempz = opt(1:n*N);
%tempz_sqp = opt_sqp(1:n*N);
%tempz_active = opt_active(1:n*N);
% Now reshape the long extracted vector into the right shape for states
z = reshape(tempz, n,N );
z(:,end+1) = z(:,end);
%z_sqp = reshape(tempz_sqp, n,N );
%z_sqp(:,end+1) = z_sqp(:,end);
%z_active = reshape(tempz_active, n,N );
%z_active(:,end+1) = z_active(:,end);
% z_sqp_tf = reshape(tempz_sqp_tf, n,N );
% z_sqp_tf(:,end+1) = z_sqp_tf(:,end);
% z_sqp_g = reshape(tempz_sqp_g, n,N );
% z_sqp_g(:,end+1) = z_sqp_g(:,end);

% Plotting the results
t = t0:init.h:tf;      % Discretize time based on step size
% Plotting the control actions
figure(1);clf;
stairs(t,u(1,:));
xlabel('Time steps','FontSize', 12);
ylabel('Control input u','FontSize', 12);
hold on
stairs(t,u(2,:));
legend('u_1','u_2')
saveas(gcf, 'Control_inputs.png');  % Save as PNG
% plotting the states
figure(2);clf;
plot(t,z(1,:),'b');
hold on;
plot(t,z(2,:),'r');
xlabel('Time steps','FontSize', 12);
ylabel('States','FontSize', 12);
plot(t,z(3,:),'--b');
plot(t,z(4,:),'--r');
legend({'$\theta_1$', '$\theta_2$', '$\dot{\theta}_1$', '$\dot{\theta}_2$'}, ...
       'Interpreter', 'latex', 'Location', 'best');
saveas(gcf, 'States.png');  % Save as PNG

% figure(3);clf;
% stairs(t,u(1,:));
% xlabel('Time steps','FontSize', 12);
% ylabel('Control input u','FontSize', 12);
% hold on
% stairs(t,u(2,:));
% stairs(t,u_sqp(1,:))
% stairs(t,u_sqp(2,:))
% stairs(t,u_active(1,:))
% stairs(t,u_active(2,:))
% legend({'$u_{1_{interior point}}$','$u_{2_{interior point}}$','$u_{1_{sqp}}$','$u_{2_{interior sqp}}$','$u_{1_{active set}}$','$u_{2_{active set}}$'},'Interpreter','latex')
% %legend({'interior point', 'sqp','active set'}, 'Location', 'best');
% saveas(gcf, 'Control_inputs_comparisson.png');  % Save as PNG
% 
% figure(4);clf;
% plot(t,z(1,:),'b');
% hold on;
% plot(t,z(2,:),'r');
% xlabel('Time steps','FontSize', 12);
% ylabel('States','FontSize', 12);
% plot(t,z(3,:),'--b');
% plot(t,z(4,:),'--r');
% plot(t,z_sqp(1,:),'m')
% plot(t,z_sqp(2,:),'c')
% plot(t,z_sqp(3,:),'--m')
% plot(t,z_sqp(4,:),'--c')
% plot(t,z_active(1,:),'g')
% plot(t,z_active(2,:),'k')
% plot(t,z_active(3,:),'--g')
% plot(t,z_active(4,:),'--k')
% 
% legend({'$\theta_{1_{interior point}}$', '$\theta_{2_{interior point}}$', '$\dot{\theta}_{1_{interior point}}$', '$\dot{\theta}_{2_{interior point}}$', ...
%     '$\theta_{1_{sqp}}$', '$\theta_{2_{sqp}}$', '$\dot{\theta}_{1_{sqp}}$', '$\dot{\theta}_{2_{sqp}}$', ...
%     '$\theta_{1_{active set}}$', '$\theta_{2_{active set}}$', '$\dot{\theta}_{1_{active set}}$', '$\dot{\theta}_{2_{active set}}$'}, ...
%     'Interpreter', 'latex', 'Location', 'best','NumColumns',3);
% saveas(gcf, 'States_comparisson.png');  % Save as PNG

