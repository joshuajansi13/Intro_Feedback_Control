addpath ./.. % adds the parent directory to the path
planar_vtol_param

% %--------------------------HW 15----------------------------
% % Longitudinal 
% Plon = tf([0.667], [1,0,0]);
% figure(1), clf, bode(Plon), grid on
% title('Planar VTOL, Longitudinal')
% 
% % Lateral
% % Inner Loop
% Plat_in = tf([20.325], [1,0,0]);
% figure(2), clf, bode(Plat_in), grid on
% title('Planar VTOL, Lateral, Inner Loop')
% 
% % Outer Loop
% Plat_out = tf([-14.715], [1.5,0.1,0]);
% figure(3), clf, bode(Plat_out), grid on
% title('Planar VTOL, Lateral, Outer Loop')

%-----------------------HW 16--------------------------------
% P_lon = tf([1], [(Pa.mc+2*Pa.mw), 0, 0]);
% P_lat_in = tf([1], [(2*Pa.d^2*Pa.mw+Pa.Jc), 0, 0]);
% P_lat_out = tf([-(Pa.mc+2*Pa.mw)*Pa.g],[(Pa.mc+2*Pa.mw), Pa.mu, 0]);
% 
% C_lon = tf([(Pa.kd_h+Pa.kp_h*Pa.sigma),(Pa.kp_h+Pa.ki_h*Pa.sigma),Pa.ki_h],[Pa.sigma,1,0]);
% C_lat_in = tf([(Pa.kd_th+Pa.sigma*Pa.kp_th), Pa.kp_th], [Pa.sigma, 1]);
% C_lat_out = tf([(Pa.kd_z+Pa.kp_z*Pa.sigma),(Pa.kp_z+Pa.ki_z*Pa.sigma),Pa.ki_z],[Pa.sigma,1,0]);
% 
% figure(1), clf, 
% bode(P_lon), grid on
% hold on
% bode(series(C_lon,P_lon))
% legend('No control', 'PID')
% title('Planar VTOL Longitudinal')
% 
% figure(2), clf, 
% bode(P_lat_in), grid on
% hold on
% bode(series(C_lat_in,P_lat_in))
% legend('No control', 'PD')
% title('Planar VTOL Lateral, Inner Loop')
% 
% figure(3), clf, 
% bode(P_lat_out), grid on
% hold on
% bode(series(C_lat_out,P_lat_out))
% legend('No control', 'PID')
% title('Planar VTOL Lateral, Outer Loop')

%-----------------------HW 17--------------------------------
P_lon = tf([1], [(Pa.mc+2*Pa.mw), 0, 0]);
P_lat_in = tf([1], [(2*Pa.d^2*Pa.mw+Pa.Jc), 0, 0]);
P_lat_out = tf([-(Pa.mc+2*Pa.mw)*Pa.g],[(Pa.mc+2*Pa.mw), Pa.mu, 0]);

C_lon = tf([(Pa.kd_h+Pa.kp_h*Pa.sigma),(Pa.kp_h+Pa.ki_h*Pa.sigma),Pa.ki_h],[Pa.sigma,1,0]);
C_lat_in = tf([(Pa.kd_th+Pa.sigma*Pa.kp_th), Pa.kp_th], [Pa.sigma, 1]);
C_lat_out = tf([(Pa.kd_z+Pa.kp_z*Pa.sigma),(Pa.kp_z+Pa.ki_z*Pa.sigma),Pa.ki_z],[Pa.sigma,1,0]);

% margin and bode plots 
figure(1), clf, margin(P_lon*C_lon), grid on, hold on
bode(P_lon*C_lon/(1+P_lon*C_lon)) 
legend('Open Loop Longitudinal', 'Closed Loop Longitudinal')
figure(2), clf, margin(P_lat_in*C_lat_in), grid on, hold on
bode(P_lat_in*C_lat_in/(1+P_lat_in*C_lat_in)) 
legend('Open Loop-Inner Lateral', 'Closed Loop-Inner Lateral')
figure(3), clf, margin(P_lat_out*C_lat_out), grid on, hold on
bode(P_lat_out*C_lat_out/(1+P_lat_out*C_lat_out))
legend('Open Loop-Outer Lateral', 'Closed Loop-Outer Lateral')

