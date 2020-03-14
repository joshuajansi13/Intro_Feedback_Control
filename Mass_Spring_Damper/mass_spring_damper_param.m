% parameters for animation
P.m = 5;      %Mass of object, kg
P.k = 3;       %spring constant, N/m
P.b = 0.5;      % damping, N-s/m

P.F_max = 5;

% Initial Conditions
P.z0 = 0.0;                % initial mass position, m
P.zdot0 = 0.0;             % initial mass velocity, m/s

% Simulation Parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 10.0;   % End time of simulation
P.Ts = 0.01;      % sample time for simulation
P.t_plot = 0.1;   % the plotting and animation is updated at this rate


% sample rate
P.Ts = 0.01;
% dirty derivative gain
P.sigma = 0.05;
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain
% equalibrium torque
% P.theta_e = 0*pi/180; %change
% P.tau_e   = P.m*P.g*P.ell/2*cos(P.theta_e); %change
%  tuning parameters
tr = 2;  % tuned to get fastest possible rise time before saturation.
zeta = 0.707;
P.ki = 4;  % integrator gain
% desired closed loop polynomial
wn = 2.2/tr;
Delta_cl_d = [1, 2*zeta*wn, wn^2];
% select PD gains
P.kp = (Delta_cl_d(3)*P.m - P.k); %change
P.kd = ((2*Delta_cl_d(2)*P.m) - P.b); %change

