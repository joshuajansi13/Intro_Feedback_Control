% parameters for animation
Par.m1 = 0.35;      % mass of ball, kg
Par.m2 = 2;     % mass of rod, kg
Par.L = 0.5;      % length of beam 
Par.g = 9.81;

% Initial Conditions
Par.z_0 = 0.25;                % initial mass position, m
Par.theta_0 = 0.0*pi/180;             %initial theta position of beam, m
Par.zdot_0 = 0.0;             % initial mass velocity, m/s
Par.thetadot_0 = 0.0;          %inittial angular velocity of beam, rad/s

% Simulation Parameters
Par.t_start = 0.0;  % Start time of simulation
Par.t_end = 10.0;   % End time of simulation
Par.Ts = 0.01;      % sample time for simulation
Par.t_plot = 0.1;   % the plotting and animation is updated at this rate


% dirty derivative parameters
Par.sigma = 0.05; % cutoff freq for dirty derivative
Par.beta = (2*Par.sigma-Par.Ts)/(2*Par.sigma+Par.Ts); % dirty derivative gain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       PD Control: Time Design Strategy (HW_06)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tuning parameters
M = 10;         % time scale separation between inner and outer loop
zeta_z   = 0.7;   % damping ratio for outer loop
zeta_th  = 0.7;   % damping ratio for inner loop
tr_theta = 0.12;    %0.12 for HW_6  % rise time for inner loop
tr_z = 1.1;  %tr_theta*M for HW_6
wn_th    = 2.2/tr_theta; % natural frequency for inner loop
wn_z = 2.2/tr_z; % natural frequency for outer loop
integrator_pole = -1;
% tuning parameters for observer 
wn_th_obs = 20*wn_th; %15
wn_z_obs = 20*wn_z;   %10
dist_obsv_pole = -2;

% saturation limits
Par.F_max = 30;                   % Max Force, N
% error_max = 1;        		   % Max step size,m
Par.theta_max = 30.0*pi/180.0;  % Max theta, rads
Par.z_e = Par.L/2;

%---------------------------------------------------
%                    Inner Loop (HW_6)
%---------------------------------------------------
% gains for inner loop
Par.kp_th  = (wn_th^2*(Par.m1*(Par.L^2/4)+Par.m2*(Par.L^2/3)))/Par.L; % kp - inner loop
Par.kd_th  = (2*zeta_th*wn_th*(Par.m1*(Par.L^2/4)+Par.m2*(Par.L^2/3)))/Par.L; % kd - inner loop
% DC gain for inner loop
k_DC_gain = 1;

%---------------------------------------------------
%                    Outer Loop (HW_6)
%---------------------------------------------------

%PD design for outer loop
Par.kp_z = -(wn_z^2)/(k_DC_gain*Par.g);
Par.kd_z = -(2*wn_z*zeta_z)/(k_DC_gain*Par.g);
Par.ki_z = -0.00001;    % select integrator gain

%-------------- State Feedback Stuff (HW_7) ----------------
Par.a = Par.m1*(Par.L^2/4) + Par.m2*(Par.L^2/3);
Par.A = [0 0 1 0;
    0 0 0 1;
    0 -Par.g 0 0;
    (-Par.m1*Par.g)/Par.a 0 0 0];
Par.B = [0; 0; 0; Par.L/Par.a];
Par.C = [1 0 0 0;
    0 1 0 0];
% gain calculation

% %-------------------without integrator--------------------
% des_char_poly = conv([1, 2*zeta_z*wn_z, wn_z^2],...
%                 [1, 2*zeta_th*wn_th, wn_th^2]);
% des_poles = roots(des_char_poly);
% %---------------------------------------------------------

%--------------------with an integrator---------------------
des_char_poly = conv(...
                conv([1, 2*zeta_z*wn_z, wn_z^2],...
                [1, 2*zeta_th*wn_th, wn_th^2]),...
                poly(integrator_pole));
des_poles = roots(des_char_poly);
%-----------------------------------------------------------

% form augmented system
Cr = [1,0,0,0];
A1 = [Par.A, zeros(4,1);
      -Cr, 0];
B1 = [Par.B; 0];


% %----Compute gains if system is controllable (without integrator)----
% if rank(ctrb(A,B)) ~= 4
%     disp('The system is not controllable')
% else
%     Par.K = place(A, B, des_poles);
%     Par.kr = -1.0/(Cr*inv(A-B*Par.K)*B);
% end
% %--------------------------------------------------------------------

%-------Compute gains if system is controllable (with integrator)------
if rank(ctrb(A1,B1)) ~= 5
    disp('The system is not controllable')
else
    K1 = place(A1, B1, des_poles);
    Par.K = K1(1:4);
    Par.ki = K1(5);
end
%-----------------------------------------------------------------------


%--------------------Observer Stuff (HW 8)--------------------------%

% %--------compute observer gains (without disturbance observer)----------%
% des_obsv_char_poly = conv(...
%     [1,2*zeta_z*wn_z_obs,wn_z_obs^2],...
%     [1,2*zeta_th*wn_th_obs,wn_th_obs^2]);
% des_obsv_poles = roots(des_obsv_char_poly);
% %----------------------------------------------------------------------%
% 
% %------is the system obervable? (without disturbance observer)---------%
% if rank(obsv(Par.A,Par.C)) ~= 4
%     disp('System not Observable');
% else % if so, compute the gains
%     Par.L_gains = place(Par.A', Par.C', des_obsv_poles)';
% end
% %----------------------------------------------------------------------%

%--------compute observer gains (with disturbance observer)----------%
% form augmented system for disturbance observer
A2 = [Par.A, Par.B; zeros(1,4), zeros(1,1)];
C2 = [Par.C, zeros(2,1)];

des_obsv_char_poly = conv(...
    conv([1,2*zeta_z*wn_z_obs,wn_z_obs^2],...
        [1,2*zeta_th*wn_th_obs,wn_th_obs^2]),...
        poly(dist_obsv_pole));
des_obsv_poles = roots(des_obsv_char_poly);

%------is the system obervable? (with disturbance observer)---------%
if rank(obsv(A2,C2)) ~= 5
    disp('System not Observable');
else % if so, compute the gains
    L2 = place(A2', C2',des_obsv_poles)';
    Par.L_gains = L2(1:4,:);
    Par.Ld_gain = L2(5,:);
end
%----------------------------------------------------------------------%
