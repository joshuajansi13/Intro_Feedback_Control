% %----------------------------HW_6 (PID)--------------------------%
% function F=ball_on_beam_ctrl(in,Par)
%     z_r   = in(1);
%     z     = in(2);
%     theta = in(3);
%     t     = in(4);
%     
%     % set persistent flag to initialize integrators and 
%     % differentiators at the start of the simulation
%     persistent flag
%     if t<Par.Ts
%         flag = 1;
%     else
%         flag = 0;
%     end
%     
%     % compute the desired angled angle using the outer loop control
%     theta_r = PID_z(z_r,z,flag,Par.kp_z,Par.ki_z,Par.kd_z,...
%                     Par.theta_max,Par.Ts,Par.sigma);
%     % compute the force using the inner loop
%     F       = PD_th(theta_r,theta,flag,Par.kp_th,Par.kd_th,...
%                      Par.F_max,Par.Ts,Par.sigma);
%     
% end
% 
% %------------------------------------------------------------
% % PID control for position
% function u = PID_z(z_c,z,flag,kp,ki,kd,limit,Ts,sigma)
%     % declare persistent variables
%     persistent integrator
%     persistent zdot
%     persistent error_d1
%     persistent z_d1
%     % reset persistent variables at start of simulation
%     if flag==1
%         integrator  = 0;
%         zdot    = 0;
%         error_d1    = 0;
%         z_d1    =  z;
%     end
%     
%     % compute the error
%     error = z_c-z;
%     % update derivative of z
%     zdot = (2*sigma-Ts)/(2*sigma+Ts)*zdot...
%            + 2/(2*sigma+Ts)*(z-z_d1);
%     % update integral of error
%     if abs(zdot)<.005
%         integrator = integrator + (Ts/2)*(error+error_d1);
%     end
%     % update delayed variables for next time through the loop
%     error_d1 = error;
%     z_d1     = z;
% 
%     % compute the pid control signal
%     u_unsat = kp*error + ki*integrator - kd*zdot;
%     u = sat(u_unsat,limit);
%     
%     % integrator anti-windup
% %     if ki~=0
% %         integrator = integrator + Ts/ki*(u-u_unsat);
% %     end
% end
% 
% 
% %------------------------------------------------------------
% % PID control for angle theta
% function u = PD_th(theta_c,theta,flag,kp,kd,limit,Ts,sigma)
%     % declare persistent variables
%     persistent thetadot
%     persistent theta_d1
%     % reset persistent variables at start of simulation
%     if flag==1
%         thetadot    = 0;
%         theta_d1    = 0;
%     end
%     
%     % compute the error
%     error = theta_c-theta;
%     % update derivative of y
%     thetadot = (2*sigma-Ts)/(2*sigma+Ts)*thetadot...
%                + 2/(2*sigma+Ts)*(theta-theta_d1);
%     % update delayed variables for next time through the loop
% 
%     theta_d1 = theta;
% 
%     % compute the pid control signal
%     u_unsat = kp*error - kd*thetadot;
%     u = sat(u_unsat,limit);
%     
% end
% 
% %-----------------------------------------------------------------
% % saturation function
% function out = sat(in,limit)
%     if     in > limit,      out = limit;
%     elseif in < -limit,     out = -limit;
%     else                    out = in;
%     end
% end

% %--------------------------HW_7 (State Feedback)----------------------%
% function F=ball_on_beam_ctrl(in,Par)
%     z_r   = in(1);
%     z     = in(2);
%     theta = in(3);
%     t     = in(4);
%     
%     persistent zdot
%     persistent z_dl
%     persistent thetadot
%     persistent theta_dl
% 
%     if t<Par.Ts
%         zdot = 0;
%         z_dl = z;
%         thetadot = 0;
%         theta_dl = theta;
%     end
%     
%     % dirty derivative
%     zdot = Par.beta*zdot...
%         + (1-Par.beta)*(z-z_dl)/Par.Ts;
%     thetadot = Par.beta*thetadot...
%         + (1-Par.beta)*(theta-theta_dl)/Par.Ts;
%     z_dl = z;
%     theta_dl = theta;
%     
%     % integrator (with integrator)
%     error = z_r - z;
%     persistent integrator
%     persistent error_d1
%     % reset persistent variables at t=0
%     if t<Par.Ts
%         integrator  = 0;
%         error_d1    = 0;
%     end
%     integrator = integrator...
%         + (Par.Ts/2)*(error+error_d1);
%     error_d1 = error;
% 
%     
%     % construct the state
%     x = [z; theta; zdot; thetadot];
%     % compute the state feedback controller
%     
%     F_e = Par.m1*Par.g*Par.z_e/Par.L + Par.m2*Par.g/2;
%     
% %     % without integrator
% %     F_unsat = -Par.K*x + Par.kr*z_r + F_e;
% %     F = sat(F_unsat, Par.F_max);
%     
%     % with an integrator
%     F_unsat = -Par.K*x - Par.ki*integrator + F_e;
%     F = sat(F_unsat, Par.F_max);
%     
%     % integrator anti-windup
%     if Par.ki~=0,
%        integrator = integrator...
%            + Par.Ts/Par.ki*(F-F_unsat);
%     end
% 
% end

% %-------------------------HW 8 (Observer)-------------------------%
% function out=ball_on_beam_ctrl(in,Par)
%     z_r    = in(1);
%     z_m     = in(2);
%     theta_m = in(3);
%     t      = in(4);
%     
%     % implement observer
%     persistent xhat  % estimated state
%     persistent dhat  % estimated disturbance
%     persistent F     % delayed input
%     if t<Par.Ts
%         xhat = [0;0;0;0];
%         dhat = 0;
%         F    = 0;
%     end
%     N = 10;
% %     %-----------------without disturbance obsv------------------%
% %     for i=1:N
% %         xhat = xhat + ...
% %             Par.Ts/N*(Par.A*xhat+Par.B*F...
% %               +Par.L_gains*([z_m;theta_m]-Par.C*xhat));
% %     end
% %     zhat = xhat(1);
% %     %----------------------------------------------------------%
%     
%     %--------------------with distrubance obsv-------------------%
%     for i=1:N
%         xhat = xhat + ...
%             Par.Ts/N*(Par.A*xhat+Par.B*(F+dhat)...
%               +Par.L_gains*([z_m;theta_m]-Par.C*xhat));
%         dhat = dhat + ...
%             Par.Ts/N*Par.Ld_gain*([z_m;theta_m]-Par.C*xhat);
%     end
%     zhat = xhat(1);
%     %------------------------------------------------------------%
%     
%     F_e = Par.m1*Par.g*Par.z_e/Par.L + Par.m2*Par.g/2;
% 
%     % integrator
%     error = z_r - zhat;
%     persistent integrator
%     persistent error_d1
%     % reset persistent variables at t=0
%     if t<Par.Ts==1
%         integrator  = 0;
%         error_d1    = 0;
%     end
%     integrator = integrator...
%         + (Par.Ts/2)*(error+error_d1);
%     error_d1 = error;
% 
% %     %---compute the state feedback controller (without dist. obvs.)---%
% %     F_unsat = -Par.K*xhat - Par.ki*integrator + F_e;
% %     %-----------------------------------------------------------------%
%     
%     %---compute the state feedback controller (with dist. obvs.)---%
%     F_unsat = -Par.K*xhat - Par.ki*integrator - dhat + F_e;
%     %-----------------------------------------------------------------%
%     
%     F = sat(F_unsat, Par.F_max);
%     
%     % integrator anti-windup
%     if Par.ki~=0
%        integrator = integrator...
%            + Par.Ts/Par.ki*(F-F_unsat);
%     end
%     
%     out = [F; xhat(1); xhat(2)];
% 
% end
% 

%------------------------HW 18-----------------------%
function F=ball_on_beam_ctrl(in,Par)
    z_r   = in(1);
    z     = in(2);
    theta = in(3);
    t     = in(4);
    
    % initialize controller state z
    persistent xout_C
    persistent xout_F
    persistent xin_C
    if t<Par.Ts
          xout_C = zeros(size(Par.Aout_C,1),1);
          xout_F = zeros(size(Par.Aout_F,1),1); 
          xin_C  = zeros(size(Par.Ain_C,1),1);
    end
    
    % prefilter the reference command for outer loop
    % solve differential equation defining prefilter
    F_e = Par.m1*Par.g*Par.z_e/Par.L + Par.m2*Par.g/2;
    N = 10; % number of Euler integration steps for each sample
    for i=1:N
        xout_F = xout_F + Par.Ts/N*( Par.Aout_F*xout_F + Par.Bout_F*z_r );
        % output equation for the prefilter
        z_r_filtered = Par.Cout_F*xout_F + Par.Dout_F*z_r;
        % error signal for outer loop
        error_out = z_r_filtered - z;
        xout_C = xout_C + Par.Ts/N*( Par.Aout_C*xout_C + Par.Bout_C*error_out );
        % output equation for the controller
        theta_r = Par.Cout_C*xout_C + Par.Dout_C*error_out;

        % error signal for inner loop
        error_in = theta_r - theta;
        % state space equations for C
        xin_C = xin_C + Par.Ts/N*( Par.Ain_C*xin_C + Par.Bin_C*error_in );
        % output equation for the controller
        F = Par.Cin_C*xin_C + Par.Din_C*error_in + F_e;
    end
  
end


%--------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end
