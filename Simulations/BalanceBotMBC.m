
clear all 
%% Symbolics space
% syms th_ddot th_dot th x_ddot x_dot V
M_p = .297;    % Mass in kg
M_w = .034;      % Mass of wheels in kg
r = .065/2;       % Wheel radius
g = 9.8;    % gravity
l = 0.0381;      % Length from axle to COM
I_p = M_p*(l^2);      % Inertia
I_w = M_w * r^2;
R = 25.8;      % Motor winding resistance
L = .003;      % Motor inductance
N = 50;      % Gear ratio !!!!!!!!!!!!!!!!!!!Do we need this or do motor constants take care? No
bm = .1;
k_m = .19;     % Torque constant
k_e = .2864;    % V_peak / w_noLoad

% eqn1 = (I_p + M_p*(l^2))*th_ddot - ((2 * k_m * k_e) / (R * r))*x_dot + (2 * k_m / R)*V + M_p * g * l * th == -M_p * l * x_ddot;
% eqn2 = ( (2*k_m) / (R*r))*V == (2*M_w + (2*I_w)/(r^2) + M_p)*x_ddot + ((2*k_m*k_e)/(R*(r^2)))*x_dot + M_p*l*th_ddot;
% 
% s_th_ddot = solve(eqn1, th_ddot)
% s_x_ddot = solve(eqn2, x_ddot)
% 
% f_th_ddot = subs(s_th_ddot, x_ddot, s_x_ddot)
% f_x_ddot = subs(s_x_ddot, th_ddot, s_th_ddot)
% 
% state_th_ddot = solve(th_ddot == f_th_ddot, th_ddot)
% state_x_ddot = solve(x_ddot == f_x_ddot, x_ddot)

%% State-Space Matrices
% A = [0 1 0 0;
%     0 -20.0245 5.1153 0;
%     0 0 0 1;
%     0 413.3172 -195.7387 0];
% B = [0; 2.2723; 0; -46.9023];
% C = [0 0 1 0];
% D = 0;
% 
% pend_ss = ss(A,B,C,D);
% 
% step(pend_ss);
% 
% eig(A)
% 
% [num,den] = ss2tf(A,B,C,D);
% sys = tf(num(1,:),den);
% rlocus(sys)

%% Paper Matrices
beta = 2*M_w + 2 * I_w / (r^2) + M_p;
alpha = I_p * beta + 2 * M_p * (l^2) * (M_w + I_w / (r^2));

A = [((2 * k_m * k_e *(M_p * l * r - I_p - M_p * l^2)) / (R * alpha *r^2)) (M_p^2 * (g * l^2) / alpha) 0;
     0 0 1;
     ( 2 * k_m * k_e *(r * beta - M_p * l)) / (R * alpha * r^2) M_p * g * l * beta / alpha 0];

B = [
    ( 2 * k_m * ( I_p + M_p * (l^2) - M_p * l * r)) / (R * r *alpha);
    0;
    (2 * k_m * (M_p * l - r * beta)) / (R * r * alpha)];

C = [ 0 1 0];
D = 0;

dt = 0.001;

Dd = D;
Cd = C;
A_big = [A B;
    zeros(1,4)] * dt;
A_big_d = expm(A_big);
Ad = A_big_d(1:3, 1:3);
Bd = A_big_d(1:3, 4);


des_p_ctrl = [-12 + 5i; -12 - 5i;-1.5];
des_p_obs = [ -20; -18; -19];

% des_p_ctrl_d = expm([des_p_ctrl zeros(3,2)].* dt);
% des_p_ctrl_d = des_p_ctrl_d(:,1);
% des_p_ctrl_d = exp(des_p_ctrl* dt);
des_p_ctrl_d = exp(des_p_ctrl * dt);
des_p_obs_d = exp(des_p_obs * dt); 
% des_p_obs_d = des_p_obs_d(:,1);

% Values from australian paper
% A = [-0.0097 11.1594 0;
%     0 0 1;
%     -0.0293 172.1160 0];
% B = [0.0815; 0; 0.2456];

Ad = expm(A*dt);
Bd = A^-1*(Ad-eye(size(A)))*B
Ad = expm(A*dt);
Bd = A^-1*(Ad-eye(size(A)))*B;
% cl_poles_disc = exp(cl_poles*dt);
Kd = place(Ad,Bd,des_p_ctrl_d);
% Observer Design

Ld = place(Ad',(Cd*Ad)',des_p_obs_d)'
% System Model
Ad_cl_obs = [Ad -Bd*Kd; Ld*C*Ad Ad-Bd*Kd-Ld*C*(Ad-Bd*Kd)];
Bd_cl = Bd*(Kd-(B'*B)^-1*B'*A)*[1;0;0]; % the [1;0;0] reects a unit step in state 1
Bd_cl_obs = [Bd_cl;Bd_cl];
cl_sys_obs = ss(Ad_cl_obs,Bd_cl_obs,[C zeros(size(C))],D,dt);
cl_step_info = stepinfo(cl_sys_obs)



[y, tout, x] = step(cl_sys_obs)
% figure(1)
subplot(2,1,1)
step(cl_sys_obs)
% subplot(2,1,2)
% margin(cl_sys_obs)
%% Simulation


%% Define Desired Poles

Tr = 1;
OS = 0.03;

zeta = sqrt( (log(OS)^2) / (pi^2 + log(OS)^2));
wn = (1.53 + 2.31 * zeta^2) / (Tr);

Tpeak = (2 * pi) / wn;

% [~, p, ~] = besself(3, (2 * pi) / 1.3);

des_p = [-zeta*wn + j*wn*sqrt(1 - zeta^2);
     -zeta*wn - j*wn*sqrt(1 - zeta^2);
     -5*zeta*wn + j*wn*sqrt(1 - zeta^2);
     -5*zeta*wn - j*wn*sqrt(1 - zeta^2)];




%% DC MOTOR CHARACTERISTICS


R = 25.8;      % Motor winding resistance
L = .003;      % Motor inductance
N = 50;      % Gear ratio
bm = .1;


J = 0.005;      % Motor stator inertia (Do wheels get added to this?)
Kt = .19;     % Torque constant
I_stall = .2; % Amps
tau_stall = 0.32;
I_noLoad = .045;
rpm_noLoad = 200;

Am = [-bm/J   Kt/J
    -Kt/L   -R/L];
Bm = [0
    1/L];
Cm = [1   0];
Dm = 0;
motor_ss = ss(Am,Bm,Cm,Dm);

step(motor_ss);

%%

p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];
% Our system will be,
[num, den] = ss2tf(A,B,C,D);

desired_poles = [-18+5i; -18 - 5i; -1; -1.5];
K = place(A,B,des_p);

% (M + m)x_ddot + b*x_dot + m*L*th_ddot*cos(th) - m*L*(th_dot^2 * sin(th) =
% J*phi_ddot
% (I + mL^2)th_ddot + m*g*L* sin(th) = -m*L*x_Ddot*cos(th)

[num,den] = ss2tf(A,B,C,D);
[numCL, denCL] = ss2tf((A-B*K), B, C, D);

 
sys = tf(num(1,:),den);
sysCL = tf(numCL(1,:), denCL);
sysCL_th = tf(numCL(2,:), denCL);



t = 0:0.01:10;
figure(1)
hold on
subplot(3,1,1)
rlocus(sysCL);
% step(sysCL,t);
subplot(3,1,2)
step(sysCL)
subplot(3,1,3)
step(sysCL_th)