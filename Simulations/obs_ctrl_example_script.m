% Testing Dr. P's Matlab Script

A = [0 1 0; 0 0 1; -15 -7 -3;];
B = [0;0;23];
C = [1 0 0];
D = 0;
%% Continuous Time Design
zeta = sqrt(log(0.03)^2/(pi^2+log(0.03)^2));
wn = (1.53+2.31*zeta^2)/0.5;
cl_poles = [-5*wn*zeta;-wn*(zeta+[1;-1]*sqrt(1-zeta^2)*1j)];
%% Discrete Time Design
dt = 0.05;
Ad = expm(A*dt);
Bd = A^-1*(Ad-eye(size(A)))*B;
cl_poles_disc = exp(cl_poles*dt);
Kd = place(Ad,Bd,cl_poles_disc);
%% Observer Design
w_filter = 10; % 10 rad/s cut-o frequency
[~,P,~] = butter(size(A,1),w_filter,'s');
obs_poles = exp(P*dt)
Ld = place(Ad',(C*Ad)',obs_poles)'
%% System Model
Ad_cl_obs = [Ad -Bd*Kd; Ld*C*Ad Ad-Bd*Kd-Ld*C*(Ad-Bd*Kd)];
Bd_cl = Bd*(Kd-(B'*B)^-1*B'*A)*[1;0;0]; % the [1;0;0] reects a unit step in state 1
Bd_cl_obs = [Bd_cl;Bd_cl];
cl_sys_obs = ss(Ad_cl_obs,Bd_cl_obs,[C zeros(size(C))],D,dt);
cl_step_info = stepinfo(cl_sys_obs)
figure(1)
subplot(2,1,1)
step(cl_sys_obs)
subplot(2,1,2)
margin(cl_sys_obs)