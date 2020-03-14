addpath ./.. % adds the parent directory to the path
ball_on_beam_param 


% %------------------------HW 15-------------------------
% % Bode plots for Inner and Outer Loops
% Pin = tf([2.652], [1, 0, 0]);
% figure(1), clf, bode(Pin), grid on
% title('Ball on Beam, Inner Loop')
% 
% Pout = tf([-9.81], [1, 0, 0]);
% figure(2), clf, bode(Pout), grid on
% title('Ball on Beam, Outer Loop')


% %--------------------------HW 16-------------------------
% P_in = tf([Par.L], [(Par.m1*(Par.L/2)^2+Par.m2*Par.L^2*(1/3)), 0, 0]);
% P_out = tf([-Par.g], [1,0,0]);
% 
% C_in = tf([(Par.kd_th+Par.sigma*Par.kp_th), Par.kp_th], [Par.sigma, 1]);
% C_out = tf([(Par.kd_z+Par.kp_z*Par.sigma),(Par.kp_z+Par.ki_z*Par.sigma),Par.ki_z],[Par.sigma,1,0]);
% 
% figure(1), clf, 
% bode(P_in), grid on
% hold on
% bode(series(C_in,P_in))
% hold on 
% bode(tf([1],[1,0,0]))
% legend('No control', 'PD', '1/s^2')
% title('Ball On Beam, Inner Loop')
% 
% figure(2), clf, 
% bode(P_out), grid on
% hold on
% bode(series(C_out,P_out))
% hold on
% bode(tf([1],[1,0,0]))
% legend('No control', 'PID', '1/s^2')
% title('Ball On Beam, Outer Loop')

%--------------------------HW 17-------------------------
% Compute inner and outer open-loop transfer functions
P_in = tf([Par.L], [(Par.m1*(Par.L/2)^2+Par.m2*Par.L^2*(1/3)), 0, 0]);
P_out = tf([-Par.g], [1,0,0]);

% Compute inner and outer closed-loop transfer functions
C_in = tf([(Par.kd_th+Par.sigma*Par.kp_th), Par.kp_th], [Par.sigma, 1]);
C_out = tf([(Par.kd_z+Par.kp_z*Par.sigma),(Par.kp_z+Par.ki_z*Par.sigma),Par.ki_z],[Par.sigma,1,0]);

% margin and bode plots 
figure(1), clf, margin(P_in*C_in), grid on, hold on
bode(P_in*C_in/(1+P_in*C_in)) 
legend('Open Loop-Inner', 'Closed Loop-Inner')
figure(2), clf, margin(P_out*C_out), grid on, hold on
bode(P_out*C_out/(1+P_out*C_out))
legend('Open Loop-Outer', 'Closed Loop-Outer')

