function [sys,x0,str,ts] = ctrl(t,x,u,flag)
% refenrence Robot Control System Design And Matlab Simulation
switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1
        sys = mdlDerivatives(t,x,u);
    case 3
        sys = mdlOutputs(t,x,u);
    case {2,4,9}
        sys = [];
    otherwise
        error(['Unhandled flag = ', num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes
global node c b Fai
node = 7;
c = 0.1 * [-1.5, -1, -0.5, 0, 0.5, 1, 1.5;
        -1.5, -1, -0.5, 0, 0.5, 1, 1.5;
        -1.5, -1, -0.5, 0, 0.5, 1, 1.5;
        -1.5, -1, -0.5, 0, 0.5, 1, 1.5;
        -1.5, -1, -0.5, 0, 0.5, 1, 1.5];
b = 10;
Fai = 5 * eye(6);

sizes = simsizes;
sizes.NumContStates = 6 * node;
sizes.NumDiscStates = 0;
sizes.NumOutputs    = 6;
sizes.NumInputs     = 30;
sizes.DirFeedthrough= 1;
sizes.NumSampleTimes= 0;
sys = simsizes(sizes);
x0 = 0.1 * ones(1, 6 * node);
str = [];
ts = [];

function sys = mdlDerivatives(t, x, u)
global node c b Fai
q1_d = u(1); q2_d = u(2); q3_d = u(3); q4_d = u(4); q5_d = u(5); q6_d = u(6);
dq1_d = u(7); dq2_d = u(8); dq3_d = u(9); dq4_d = u(10); dq5_d = u(11); dq6_d = u(12);
ddq1_d = u(13); ddq2_d = u(14); ddq3_d = u(15); ddq4_d = u(16); ddq5_d = u(17); ddq6_d = u(18);

q1 = u(19); q2 = u(20); q3 = u(21); q4 = u(22); q5 = u(23); q6 = u(24);
dq1 = u(25); dq2 = u(26); dq3 = u(27); dq4 = u(28); dq5 = u(29); dq6 = u(30);

q_d = [q1_d; q2_d; q3_d; q4_d; q5_d; q6_d];
dq_d = [dq1_d; dq2_d; dq3_d; dq4_d; dq5_d; dq6_d];
ddq_d = [ddq1_d; ddq2_d; ddq3_d; ddq4_d; ddq5_d; ddq6_d];

q = [q1; q2; q3; q4; q5; q6];
dq = [dq1; dq2; dq3; dq4; dq5; dq6];

e = q_d - q;
de = dq_d - dq;
r = de + Fai * e;

dqr = dq_d + Fai * e;
ddqr = ddq_d + Fai * de;

z1 = [e(1); de(1); q_d(1); dq_d(1); ddq_d(1)];
z2 = [e(2); de(2); q_d(2); dq_d(2); ddq_d(2)];
z3 = [e(3); de(3); q_d(3); dq_d(3); ddq_d(3)];
z4 = [e(4); de(4); q_d(4); dq_d(4); ddq_d(4)];
z5 = [e(5); de(5); q_d(5); dq_d(5); ddq_d(5)];
z6 = [e(6); de(6); q_d(6); dq_d(6); ddq_d(6)];

for j = 1:1:node
    h1(j) = exp(- norm(z1 - c(:,j))^2/(b*b));
    h2(j) = exp(- norm(z2 - c(:,j))^2/(b*b));
    h3(j) = exp(- norm(z3 - c(:,j))^2/(b*b));
    h4(j) = exp(- norm(z4 - c(:,j))^2/(b*b));
    h5(j) = exp(- norm(z5 - c(:,j))^2/(b*b));
    h6(j) = exp(- norm(z6 - c(:,j))^2/(b*b));
end

F = 1.5 * eye(node);

for i = 1:1:node
    sys(i) = 1.5 * h1(i) * r(1);
    sys(i + node) = 1.5 * h2(i) * r(2);
    sys(i + 2 * node) = 1.5 * h3(i) * r(3);
    sys(i + 3 * node) = 1.5 * h4(i) * r(4);
    sys(i + 4 * node) = 1.5 * h5(i) * r(5);
    sys(i + 5 * node) = 1.5 * h6(i) * r(6);
end

function sys = mdlOutputs(t, x, u)
global node c b Fai

q1_d = u(1); q2_d = u(2); q3_d = u(3); q4_d = u(4); q5_d = u(5); q6_d = u(6);
dq1_d = u(7); dq2_d = u(8); dq3_d = u(9); dq4_d = u(10); dq5_d = u(11); dq6_d = u(12);
ddq1_d = u(13); ddq2_d = u(14); ddq3_d = u(15); ddq4_d = u(16); ddq5_d = u(17); ddq6_d = u(18);

q1 = u(19); q2 = u(20); q3 = u(21); q4 = u(22); q5 = u(23); q6 = u(24);
dq1 = u(25); dq2 = u(26); dq3 = u(27); dq4 = u(28); dq5 = u(29); dq6 = u(30);

q_d = [q1_d; q2_d; q3_d; q4_d; q5_d; q6_d];
dq_d = [dq1_d; dq2_d; dq3_d; dq4_d; dq5_d; dq6_d];
ddq_d = [ddq1_d; ddq2_d; ddq3_d; ddq4_d; ddq5_d; ddq6_d];

q = [q1; q2; q3; q4; q5; q6];
dq = [dq1; dq2; dq3; dq4; dq5; dq6];

e = q_d - q;
de = dq_d - dq;
r = de + Fai * e;

dqr = dq_d + Fai * e;
ddqr = ddq_d + Fai * de;

W_f1 = [x(1:node)]';
W_f2 = [x(node + 1:node*2)]';
W_f3 = [x(node*2 + 1:node*3)]';
W_f4 = [x(node*3 + 1:node*4)]';
W_f5 = [x(node*4 + 1:node*5)]';
W_f6 = [x(node*5 + 1:node*6)]';

z1 = [e(1); de(1); q_d(1); dq_d(1); ddq_d(1)];
z2 = [e(2); de(2); q_d(2); dq_d(2); ddq_d(2)];
z3 = [e(3); de(3); q_d(3); dq_d(3); ddq_d(3)];
z4 = [e(4); de(4); q_d(4); dq_d(4); ddq_d(4)];
z5 = [e(5); de(5); q_d(5); dq_d(5); ddq_d(5)];
z6 = [e(6); de(6); q_d(6); dq_d(6); ddq_d(6)];
for j = 1:1:node
    h1(j) = exp(- norm(z1 - c(:,j))^2/(b*b));
    h2(j) = exp(- norm(z2 - c(:,j))^2/(b*b));
    h3(j) = exp(- norm(z3 - c(:,j))^2/(b*b));
    h4(j) = exp(- norm(z4 - c(:,j))^2/(b*b));
    h5(j) = exp(- norm(z5 - c(:,j))^2/(b*b));
    h6(j) = exp(- norm(z6 - c(:,j))^2/(b*b));
end

fn = [W_f1 * h1';
    W_f2 * h2';
    W_f3 * h3';
    W_f4 * h4';
    W_f5 * h5';
    W_f6 * h6';];
Kv = 20 * eye(6);

ep = 0.20;
bd = 0.1;
v = -(ep + bd) * sign(r);
tol = fn + Kv * r - v;

sys(1) = tol(1);
sys(2) = tol(2);
sys(3) = tol(3);
sys(4) = tol(4);
sys(5) = tol(5);
sys(6) = tol(6);