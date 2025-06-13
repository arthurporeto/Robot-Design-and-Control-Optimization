% Assuming the robot is in a vertical plane, set g = 0 if it is horizontal
% Deriving the dynamics using Euler-Lagrange formulation
syms th1(t) th2(t) l1 l2 m1 m2 me g u1 u2

I1 = [0,0,0;0,0,0;0,0,m1*l1^2/12];
I2 = [0,0,0;0,0,0;0,0,m2*l2^2/12];
Ie = [0,0,0;0,0,0;0,0,0];

q = [th1(t);th2(t)];
u = [u1;u2];

l1_bar = [l1*cos(th1(t)), l1*sin(th1(t))];
l2_bar = [l2*cos(th2(t)), l2*sin(th2(t))];

% Get center of masses
pc1 = l1_bar/2;
pc2 = l1_bar+l2_bar/2;
pc3 = l1_bar+l2_bar;

% Potential vector
V = pc1(2)*m1*g+pc2(2)*m2*g+pc3(2)*me*g;
G = jacobian(V, q).';

% Computing the linear velocity Jacobian matrix
Jv1 = jacobian(pc1, q);
Jv2 = jacobian(pc2, q);
Jve = jacobian(pc3, q);
% Computing the angular velocity Jacobian matrix
Jw1 = [0,0;0,0;1,0];
Jw2 = [0,0;0,0;1,1];
Jwe = [0,0;0,0;1,1];
% Mass matrix
M = simplify(m1*(Jv1.'*Jv1)+m2*(Jv2.'*Jv2)+me*(Jve.'*Jve)+(Jw1.'*I1*Jw1)+(Jw2.'*I2*Jw2)+(Jwe.'*Ie*Jwe));
% Check for the M matrix
% A = M-M.'

% Coriolis matrix
n = length(q);
tempC = sym(zeros(n,n));
for i=1:n
    for j=1:n
        for k=1:n
            tempC(i,j) = tempC(i,j)+1/2*(diff(M(i,j),q(k))+diff(M(i,k),q(j))-diff(M(j,k),q(i)))*diff(q(k),t);
        end
    end
end

C = simplify(tempC);
% Check for the C matrix
% A = simplify(diff(M,t)-2*C)
% A+A.'

LHS = M*diff(q,t,2)+C*diff(q,t)+G;
qdd = inv(M)*(u-(G+C*diff(q,t)));
% Verified the dynamics with the Mathematica code
matlabFunction(qdd,'file','dynamics_2R_function.m')


% Convert the system into state-space


