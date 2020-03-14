% parameters for animation
Pa.mc = 1;
Pa.Jc = 0.0042;
Pa.mw = 0.25;
Pa.d = 0.3;
Pa.mu = 0.1;
Pa.g = 9.81;

% Initial Conditions
Pa.z0 = 0.0;                % initial mass position
Pa.h0 = 0.0;
Pa.theta0 = 0.0;             %initial theta position of beam, m
Pa.zdot0 = 0.0;             % initial mass velocity, m/s
Pa.hdot0 = 0.0;
Pa.thetadot0 = 0.0;          %inittial angular velocity of beam, rad/s

% Simulation Parameters
Pa.t_start = 0.0;  % Start time of simulation
Pa.t_end = 100.0;   % End time of simulation
Pa.Ts = 0.01;      % sample time for simulation
Pa.t_plot = 0.5;   % the plotting and animation is updated at this rate

% dirty derivative parameters
Pa.sigma = 0.05; % cutoff freq for dirty derivative
Pa.beta = (2*Pa.sigma-Pa.Ts)/(2*Pa.sigma+Pa.Ts); % dirty derivative gain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       PD Control: Time Design Strategy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% saturation limits
Pa.F_max = 15;                   % Max Force, N
% error_max = 1;        		   % Max step size,m
% Pa.theta_max = 30.0*pi/180.0;  % Max theta, rads

M = 10;

% Tuning Parameters
t_r_h = 0.6;  % 1 for HW_06 
wn_h = 2.2/t_r_h;
zeta_h = 0.7;
tr_th = 0.25;  % 0.9 for HW_06 
wn_th    = 2.2/tr_th; % natural frequency for inner loop
zeta_th = 0.7;
tr_z = 1.2;  % M*tr_th for HW_06
wn_z = 2.2/tr_z; % natural frequency for outer loop
zeta_z = 0.7;
integrator_pole_z = -0.3;
integrator_pole_h = -1;
% tuning parameters for observer
wn_h_obs = wn_h*10;
wn_th_obs = wn_th*10;
wn_z_obs = wn_z*10;
dist_obsv_pole_z = -1;
dist_obsv_pole_h = -1;

%--------------------------------------------------
%               Longitudinal Control (HW_06)
%--------------------------------------------------
Pa.kp_h = wn_h^2*(Pa.mc+2*Pa.mw);
Pa.kd_h = 2*zeta_h*wn_h*(Pa.mc+2*Pa.mw);
Pa.ki_h = 2;

%---------------------------------------------------
%                    Inner Loop (HW_06)
%---------------------------------------------------
% gains for inner loop
Pa.kp_th  = (2*Pa.d^2*Pa.mw+Pa.Jc)*wn_th^2; % kp - inner loop
Pa.kd_th  = (2*Pa.d^2*Pa.mw+Pa.Jc)*2*zeta_th*wn_th; % kd - inner loop
% DC gain for inner loop
k_DC_gain = 1;

%---------------------------------------------------
%                    Outer Loop (HW_06)
%---------------------------------------------------

%PD design for outer loop
Pa.kp_z = -wn_z^2/Pa.g;
Pa.kd_z = (Pa.mu-(2*zeta_z*wn_z*(Pa.mc+2*Pa.mw)))/((Pa.mc+2*Pa.mw)*Pa.g);
Pa.ki_z = -0.0001;

%------------------------HW_07--------------------------
%----------------Longitudinal-------------------------% 
Pa.a0 = Pa.mc+2*Pa.mw;
Pa.A_h = [0 1;
       0 0];
Pa.B_h = [0; 1/Pa.a0];
Pa.C_h = [1 0];

% %---------gain calculation (without integrator)------------%
% des_char_poly_h = [1, 2*zeta_h*wn_h, wn_h^2];
% des_poles_h = roots(des_char_poly_h);
% %----------------------------------------------------------%

%------------gain calculation (with integrator)--------------%
des_char_poly_h = conv([1, 2*zeta_h*wn_h, wn_h^2],...
                       poly(integrator_pole_h));
des_poles_h = roots(des_char_poly_h);
%------------------------------------------------------------%

% form augmented system
A1 = [Pa.A_h, zeros(2,1);
      -Pa.C_h, 0];
B1 = [Pa.B_h; 0];

% %----Compute gains if system is controllable (without integrator)----%
% if rank(ctrb(Pa.A_h,Pa.B_h)) ~= 2
%     disp('The system is not controllable')
% else
%     Pa.K_h = place(Pa.A_h, Pa.B_h, des_poles_h);
%     Pa.kr_h = -1.0/(Pa.C_h*inv(Pa.A_h-Pa.B_h*Pa.K_h)*Pa.B_h);
% end
% %--------------------------------------------------------------------%

%-----Compute gains if system is controllable (with integrator)--------%
if rank(ctrb(A1,B1)) ~= 3
    disp('The system is not controllable')
else
    K1 = place(A1, B1, des_poles_h);
    Pa.K_h = K1(1:2);
    Pa.ki_h = K1(3);
end
%---------------------------------------------------------------------%

%-------------------------Lateral-----------------------------%
Pa.b0 = 2*Pa.d^2*Pa.mw + Pa.Jc;
Pa.A_z = [0 0 1 0;
       0 0 0 1;
       0 -Pa.g -Pa.mu/Pa.a0 0;
       0 0 0 0];
Pa.B_z = [0; 0; 0; 1/Pa.b0];
Pa.C_z = [1 0 0 0;
       0 1 0 0];

% %-----------gain calculation (without integrator)-------------%
% des_char_poly_z = conv([1, 2*zeta_z*wn_z, wn_z^2],...
%                        [1, 2*zeta_th*wn_th, wn_th^2]);
% des_poles_z = roots(des_char_poly_z);
% %-------------------------------------------------------------%

%--------------gain calculation (with an integrator)------------%
des_char_poly_z = conv(...
                conv([1, 2*zeta_z*wn_z, wn_z^2],...
                [1, 2*zeta_th*wn_th, wn_th^2]),...
                poly(integrator_pole_z));
des_poles_z = roots(des_char_poly_z);
%---------------------------------------------------------------%

% form augmented system
C_zr = [1,0,0,0];
A2 = [Pa.A_z, zeros(4,1);
      -C_zr, 0];
B2 = [Pa.B_z; 0];

% %----Compute gains if system is controllable (without integrator)----%
% if rank(ctrb(Pa.A_z,Pa.B_z)) ~= 4
%     disp('The system is not controllable')
% else
%     Pa.K_z = place(Pa.A_z, Pa.B_z, des_poles_z);
%     Pa.kr_z = -1.0/(Pa.C_zr*inv(Pa.A_z-Pa.B_z*Pa.K_z)*Pa.B_z);
% end
% %--------------------------------------------------------------------%

%------Compute gains if system is controllable (with integrator)-------%
if rank(ctrb(A2,B2)) ~= 5
    disp('The system is not controllable')
else
    K2 = place(A2, B2, des_poles_z);
    Pa.K_z = K2(1:4);
    Pa.ki_z = K2(5);
end
%---------------------------------------------------------------------%

%-------------------------HW 08 (observers)---------------------------%

% %-----------calculate obs. poles (without dist. obsv LON)-----------------%
% des_obsv_char_poly_h = [1, 2*zeta_h*wn_h_obs, wn_h_obs^2];
% des_obsv_poles_h = roots(des_obsv_char_poly_h);
% 
% %--------------is the system observable? (without dist. obsv)-------------%
% if rank(obsv(Pa.A_h,Pa.C_h)) ~= 2
%     disp('System not Observable')
% else
%     Pa.L_h_gains = place(Pa.A_h', Pa.C_h', des_obsv_poles_h)';
% end
% %------------------------------------------------------------------------%
% 
% 
% %-----------calculate obs. poles (without dist. obsv LAT)-----------------%
% des_obsv_char_poly_z = conv(...
%     [1, 2*zeta_z*wn_z_obs, wn_z_obs^2],...
%     [1, 2*zeta_th*wn_th_obs, wn_th_obs^2]);
% des_obsv_poles_z = roots(des_obsv_char_poly_z);
% 
% %--------------is the system observable? (without dist. obsv)-------------%
% if rank(obsv(Pa.A_z,Pa.C_z)) ~= 4
%     disp('System not Observable')
% else
%     Pa.L_z_gains = place(Pa.A_z', Pa.C_z', des_obsv_poles_z)';
% end
% %------------------------------------------------------------------------%


%--------compute observer gains (with disturbance observer LAT)----------%
% form augmented system for disturbance observer
A2_obs = [Pa.A_z, Pa.B_z; zeros(1,4), zeros(1,1)];
C2_obs = [Pa.C_z, zeros(2,1)];

des_obsv_char_poly_z = conv(...
    conv([1,2*zeta_z*wn_z_obs,wn_z_obs^2],...
        [1,2*zeta_th*wn_th_obs,wn_th_obs^2]),...
        poly(dist_obsv_pole_z));
des_obsv_poles_z = roots(des_obsv_char_poly_z);

%------is the system obervable? (with disturbance observer LAT)---------%
if rank(obsv(A2_obs,C2_obs)) ~= 5
    disp('System not Observable');
else % if so, compute the gains
    L2_obs = place(A2_obs', C2_obs',des_obsv_poles_z)';
    Pa.L_z_gains = L2_obs(1:4,:);
    Pa.Ld_z_gain = L2_obs(5,:);
end
%----------------------------------------------------------------------%

%--------compute observer gains (with disturbance observer LON)----------%
% form augmented system for disturbance observer
A1_obs = [Pa.A_h, Pa.B_h; zeros(1,2), zeros(1,1)];
C1_obs = [Pa.C_h, zeros(1,1)];

des_obsv_char_poly_h = conv([1,2*zeta_h*wn_h_obs,wn_h_obs^2],...
        poly(dist_obsv_pole_h));
des_obsv_poles_h = roots(des_obsv_char_poly_h);

%------is the system obervable? (with disturbance observer LON)---------%
if rank(obsv(A1_obs,C1_obs)) ~= 3
    disp('System not Observable');
else % if so, compute the gains
    L1_obs = place(A1_obs', C1_obs',des_obsv_poles_h)';
    Pa.L_h_gains = L1_obs(1:2,:);
    Pa.Ld_h_gain = L1_obs(3,:);
end
%----------------------------------------------------------------------%
