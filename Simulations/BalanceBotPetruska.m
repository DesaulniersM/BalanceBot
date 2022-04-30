%% Parameters
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

dt = .001;
%% Matrices


W = [-1/r 0 0 0;
    0 -1/r 0 0;
    0 0 1 0;
    0 0 0 1];

beta = 2*M_w + 2 * I_w / (r^2) + M_p;
alpha = I_p * beta + 2 * M_p * (l^2) * (M_w + I_w / (r^2));

Ap = [ 0 1 0 0;
    0 ((2 * k_m * k_e *(M_p * l * r - I_p - M_p * l^2)) / (R * alpha *r^2)) (M_p^2 * (g * l^2) / alpha) 0;
     0 0 0 1;
     0 ( 2 * k_m * k_e *(r * beta - M_p * l)) / (R * alpha * r^2) M_p * g * l * beta / alpha 0];

Bp = [ 0;
    ( 2 * k_m * ( I_p + M_p * (l^2) - M_p * l * r)) / (R * r *alpha);
    0;
    (2 * k_m * (M_p * l - r * beta)) / (R * r * alpha)];

A = W*Ap*inv(W);
B = W*Bp;

Ge = 1; %fix...

C_row1 = [0 0 -g 0]+[0 0 0 l]*A;
C = [ C_row1;
    0 0 1 0;
    0 0 0 1;
    Ge 0 -Ge 0];
D_row1 = [0 0 0 l]*B ;
D = [ D_row1;
    0;
    0;
    0];

[~,p_ctrl,~] = besself(4,14);
p_obs = 1.5 * p_ctrl;

%% Test controllability and observibility
% controllability = rank(ctrb(A,B))
% Observability = rank(obsv(A',C'))
%% Discrete Time State-Space

ToDiscrete = expm([A B; zeros(1, 4) zeros(1,1)]*dt);
Ad = ToDiscrete(1:4,1:4);
Bd = ToDiscrete(1:4,5);
Cd = C;
Dd = D;
p_ctrld = exp(p_ctrl*dt);
p_obsvd = exp(p_obs*dt);

K = place(A, B, p_ctrld);
L = place(Ad', (Cd*Ad)', p_obsvd);
%% Simulations

% 1. Design the system dyn in cont time
% 2. convert it to discrete time
% 3. calculate the gains
% 4. implement

Ado = (Ad - L*C*Ad);
Bdo = (eye(rank(L*C)) - L*C)*Bd;
x_now = Ado*x_last + Bdo*u_last + L*y_meas + D*u_last;
u_now = K * (x_now_desired - x_now);


