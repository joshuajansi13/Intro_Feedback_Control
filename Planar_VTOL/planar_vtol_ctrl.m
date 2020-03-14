% %-------------------------HW_06 (PID)---------------------- 
% function Tau=planar_vtol_ctrl(in,Pa)
%     h_r   = in(1);
%     h     = in(2);
%     z_r   = in(3);
%     z     = in(4);
%     theta = in(5);
%     t     = in(6);
%     
%     % set persistent flag to initialize integrators and 
%     % differentiators at the start of the simulation
%     persistent flag
%     if t<Pa.Ts
%         flag = 1;
%     else
%         flag = 0;
%     end
%     
%     F = PID_h(h_r,h,flag,Pa.kp_h,Pa.ki_h,Pa.kd_h,...
%                     Pa.F_max,Pa.Ts,Pa.sigma);
%     theta_r = PID_z(z_r,z,flag,Pa.kp_z,Pa.ki_z,Pa.kd_z,...
%                     Pa.Ts,Pa.sigma);
%     Tau     = PD_th(theta_r,theta,flag,Pa.kp_th,Pa.kd_th,...
%                     Pa.Ts,Pa.sigma,F);
%     
% end
% 
% %------------------------------------------------------------
% % PID control for longitudinal position
% function u = PID_h(h_c,h,flag,kp,ki,kd,limit,Ts,sigma)
%     % declare persistent variables
%     persistent integrator
%     persistent hdot
%     persistent error_d1
%     persistent h_dl
%     % reset persistent variables at start of simulation
%     if flag==1
%         integrator  = 0;
%         hdot    = 0;
%         error_d1    = 0;
%         h_dl    = h;
%     end
%     
%     % compute the error
%     error = h_c-h;
%     % update derivative of h
%     hdot = (2*sigma-Ts)/(2*sigma+Ts)*hdot...
%            + 2/(2*sigma+Ts)*(h-h_dl);
%     % update integral of error
%     if abs(hdot)<.001
%         integrator = integrator + (Ts/2)*(error+error_d1);
%     end
%     % update delayed variables for next time through the loop
%     error_d1 = error;
%     h_dl     = h;
% 
%     % compute the pid control signal
%     u_unsat = kp*error + ki*integrator - kd*hdot;
%     u = sat(u_unsat,limit);
%     
%     % integrator anti-windup
% %     if ki~=0
% %         integrator = integrator + Ts/ki*(u-u_unsat);
% %     end
% end
% 
% %------------------------------------------------------------
% % PID control for longitudinal position
% function u = PID_z(z_c,z,flag,kp,ki,kd,Ts,sigma)
%     % declare persistent variables
%     persistent integrator
%     persistent zdot
%     persistent error_d1
%     persistent z_dl
%     % reset persistent variables at start of simulation
%     if flag==1
%         integrator  = 0;
%         zdot    = 0;
%         error_d1    = 0;
%         z_dl    = z;
%     end
%     
%     % compute the error
%     error = z_c-z;
%     % update derivative of h
%     zdot = (2*sigma-Ts)/(2*sigma+Ts)*zdot...
%            + 2/(2*sigma+Ts)*(z-z_dl);
%     % update integral of error
%     if abs(zdot)<.001
%         integrator = integrator + (Ts/2)*(error+error_d1);
%     end
%     % update delayed variables for next time through the loop
%     error_d1 = error;
%     z_dl     = z;
% 
%     % compute the pid control signal
%     u = kp*error + ki*integrator - kd*zdot;
% %     u = sat(u_unsat,limit);
%     
% %     % integrator anti-windup
% %     if ki~=0
% %         integrator = integrator + Ts/ki*(u-u_unsat);
% %     end
% end
% 
% %------------------------------------------------------------
% % PID control for angle theta
% function u = PD_th(theta_c,theta,flag,kp,kd,Ts,sigma,F)
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
%     u = [F; u_unsat];
% %     u = sat(u_unsat,limit);
%     
% end
% 
% 
% %-----------------------------------------------------------------
% % saturation function
% function out = sat(in,limit)
%     if     in > limit,      out = limit;
%     elseif in < -limit,     out = -limit;
%     else                    out = in;
%     end
% end

% %-----------------------HW_07 (State Feedback)--------------------------
% function Tau=planar_vtol_ctrl(in,Pa)
%     h_r   = in(1);
%     h     = in(2);
%     z_r   = in(3);
%     z     = in(4);
%     theta = in(5);
%     t     = in(6);
%     
%     persistent zdot
%     persistent z_dl
%     persistent thetadot
%     persistent theta_dl
% 
%     if t<Pa.Ts
%         zdot = 0;
%         z_dl = z;
%         thetadot = 0;
%         theta_dl = theta;
%     end
%     
%     % dirty derivative
%     zdot = Pa.beta*zdot...
%         + (1-Pa.beta)*(z-z_dl)/Pa.Ts;
%     thetadot = Pa.beta*thetadot...
%         + (1-Pa.beta)*(theta-theta_dl)/Pa.Ts;
%     z_dl = z;
%     theta_dl = theta;
%     
%     % integrator (with integrator)
%     error = z_r - z;
%     persistent integrator
%     persistent error_d1
%     % reset persistent variables at t=0
%     if t<Pa.Ts
%         integrator  = 0;
%         error_d1    = 0;
%     end
%     integrator = integrator...
%         + (Pa.Ts/2)*(error+error_d1);
%     error_d1 = error;
%     
%     % construct the state
%     x = [z; theta; zdot; thetadot];
%     
%     % compute the state feedback controller
% %     Tau_unsat = -Pa.K_z*x+Pa.kr_z*z_r;  % without integrator
%     Tau_unsat = -Pa.K_z*x-Pa.ki_z*integrator;  % with integrator
%     
% %     F = state_FB(h_r, h, Pa.F_max, Pa.K_h, Pa.kr_h, t, Pa.Ts, Pa.beta); % without integrator
%     F = state_FB(h_r, h, Pa.F_max, Pa.K_h, Pa.ki_h, t, Pa.Ts, Pa.beta); % with integrator
%     Tau = [F, Tau_unsat];
% end

% function u=state_FB(h_r, h, limit, K, kr, t, ts, beta) % without integrator
%     persistent hdot
%     persistent h_dl
%     
%     if t<ts
%         hdot = 0;
%         h_dl = h;
%     end
%     % dirty derivative
%     hdot = beta*hdot...
%         + (1-beta)*(h-h_dl)/ts;
%     h_dl = h;
%     
%     x = [h; hdot];
%     u = sat(-K*x+kr*h_r, limit);
% end

% function u=state_FB(h_r, h, limit, K, ki, t, ts, beta) % with integrator
%     persistent hdot
%     persistent h_dl
%     
%     if t<ts
%         hdot = 0;
%         h_dl = h;
%     end
%     % dirty derivative
%     hdot = beta*hdot...
%         + (1-beta)*(h-h_dl)/ts;
%     h_dl = h;
%     
%     % integrator (with integrator)
%     error = h_r - h;
%     persistent integrator
%     persistent error_d1
%     % reset persistent variables at t=0
%     if t<ts==1,
%         integrator  = 0;
%         error_d1    = 0;
%     end
%     integrator = integrator...
%         + (ts/2)*(error+error_d1);
%     error_d1 = error;
%     
%     x = [h; hdot];
%     u_unsat = -K*x - ki*integrator;
%     u = sat(u_unsat, limit);
% end

% %-------------------------HW 8 (Observer)-------------------------%
% function out=planar_vtol_ctrl(in,Pa)
%     h_r     = in(1);
%     h_m     = in(2);
%     z_r     = in(3);
%     z_m     = in(4);
%     theta_m = in(5);
%     t       = in(6);
%     
%     % implement observer
%     persistent xhat  % estimated state
%     persistent dhat  % estimated disturbance
%     persistent Tau     % delayed input
%     if t<Pa.Ts
%         xhat = [0;0;0;0];
%         dhat = 0;
%         Tau    = 0;
%     end
%     N = 10;
% %     %-----------------without disturbance obsv------------------%
% %     for i=1:N
% %         xhat = xhat + ...
% %             Pa.Ts/N*(Pa.A_z*xhat+Pa.B_z*Tau...
% %               +Pa.L_z_gains*([z_m;theta_m]-Pa.C_z*xhat));
% %     end
% %     zhat = xhat(1);
% %     %----------------------------------------------------------%
%     
%     %--------------------with distrubance obsv-------------------%
%     for i=1:N
%         xhat = xhat + ...
%             Pa.Ts/N*(Pa.A_z*xhat+Pa.B_z*(Tau+dhat)...
%               +Pa.L_z_gains*([z_m;theta_m]-Pa.C_z*xhat));
%       dhat = dhat + ...
%         Pa.Ts/N*Pa.Ld_z_gain*([z_m;theta_m]-Pa.C_z*xhat);
%     end
%     zhat = xhat(1);
%     %------------------------------------------------------------%
%     
% 
%     % integrator
%     error = z_r - zhat;
%     persistent integrator
%     persistent error_d1
%     % reset persistent variables at t=0
%     if t<Pa.Ts==1
%         integrator  = 0;
%         error_d1    = 0;
%     end
%     integrator = integrator...
%         + (Pa.Ts/2)*(error+error_d1);
%     error_d1 = error;
% 
% %     %---compute the state feedback controller (without dist. obvs.)---%
% %     Tau_unsat = -Pa.K_z*xhat - Pa.ki_z*integrator;
% %     %-----------------------------------------------------------------%
%     
%     %---compute the state feedback controller (with dist. obvs.)---%
%     Tau_unsat = -Pa.K_z*xhat - Pa.ki_z*integrator - dhat;
%     %-----------------------------------------------------------------%
%     
%     Tau = Tau_unsat;
% %     F = observer(h_r,h_m,Pa.F_max,Pa.K_h,Pa.ki_h,Pa.L_h_gains,t,Pa.Ts,Pa.A_h,Pa.B_h,Pa.C_h); % without dist. obsv
%     F = observer(h_r,h_m,Pa.F_max,Pa.K_h,Pa.ki_h,Pa.L_h_gains,Pa.Ld_h_gain,t,Pa.Ts,Pa.A_h,Pa.B_h,Pa.C_h); % with dist. obsv
% 
%     out = [F(1), Tau, xhat(1), xhat(2), F(2)];
% 
% end
% 
% % function u=observer(y_r,y_m,limit,K,ki,L,t,Ts,A,B,C) %without dist obsv
% function u=observer(y_r,y_m,limit,K,ki,L,Ld,t,Ts,A,B,C) %with dist obsv
% 
%     persistent xhat
%     persistent dhat
%     persistent F
%     
%     if t<Ts
%         xhat = [0;0];
%         dhat = 0;
%         F = 0;
%     end
%     
%     N = 10;
% %     %---------------without disturbance obsv.-------------------%
% %     for i=1:N
% %         xhat = xhat + ...
% %             Ts/N*(A*xhat+B*F...
% %               +L*(y_m-C*xhat));
% %     end
% %     hhat = xhat(1);
% %     %-----------------------------------------------------------%
%     
%     %------------------with disturbance obsv.-------------------%
%     for i=1:N
%         xhat = xhat + ...
%             Ts/N*(A*xhat+B*(F+dhat)...
%               +L*(y_m-C*xhat));
%         dhat = dhat + ...
%             Ts/N*Ld*(y_m-C*xhat);
%     end
%     hhat = xhat(1);
%     %-----------------------------------------------------------%
% 
%     
%     % integrator
%     error = y_r - hhat;
%     persistent integrator
%     persistent error_d1
%     % reset persistent variables at t=0
%     if t<Ts==1
%         integrator  = 0;
%         error_d1    = 0;
%     end
%     integrator = integrator...
%         + (Ts/2)*(error+error_d1);
%     error_d1 = error;
% 
% %     %---compute the state feedback controller (without dist. obvs.)---%
% %     F_unsat = -K*xhat - ki*integrator;
% %     %-----------------------------------------------------------------%
%     
%     %---compute the state feedback controller (with dist. obvs.)---%
%     F_unsat = -K*xhat - ki*integrator - dhat;
%     %-----------------------------------------------------------------%
% 
%     F = sat(F_unsat, limit);
%     u = [F,hhat];
% end


%------------------------HW 18-----------------------%
function Tau=planar_vtol_ctrl(in,Pa)
    h_r   = in(1);
    h     = in(2);
    z_r   = in(3);
    z     = in(4);
    theta = in(5);
    t     = in(6);

    % initialize controller state z
    persistent xout_C
    persistent xout_F
    persistent xin_C
    if t<Pa.Ts
          xout_C = zeros(size(Pa.A_C_lat_out,1),1);
          xout_F = zeros(size(Pa.A_F_lat_out,1),1); 
          xin_C  = zeros(size(Pa.A_C_lat_in,1),1);
    end
    
    
    % prefilter the reference command for outer loop
    % solve differential equation defining prefilter
    N = 10; % number of Euler integration steps for each sample
    for i=1:N
        xout_F = xout_F + Pa.Ts/N*(Pa.A_F_lat_out*xout_F + Pa.B_F_lat_out*z_r );
        % output equation for the prefilter
        z_r_filtered = Pa.C_F_lat_out*xout_F + Pa.D_F_lat_out*z_r;
        % error signal for outer loop
        error_out = z_r_filtered - z;
        xout_C = xout_C + Pa.Ts/N*(Pa.A_C_lat_out*xout_C + Pa.B_C_lat_out*error_out );
        % output equation for the controller
        theta_r = Pa.C_C_lat_out*xout_C + Pa.D_C_lat_out*error_out;

        % error signal for inner loop
        error_in = theta_r - theta;
        % state space equations for C
        xin_C = xin_C + Pa.Ts/N*(Pa.A_C_lat_in*xin_C + Pa.B_C_lat_in*error_in);
        % output equation for the controller
        F_unsat = loop_shape(h_r, h, t, Pa.Ts, Pa.A_C_lon,Pa.B_C_lon,Pa.C_C_lon,Pa.D_C_lon,Pa.A_F_lon, Pa.B_F_lon, Pa.C_F_lon, Pa.D_F_lon);
        Tau_unsat = Pa.C_C_lat_in*xin_C + Pa.D_C_lat_in*error_in;
        
        Tau = [F_unsat, Tau_unsat];
    end
  
end

function u=loop_shape(h_r, h, t, ts, A_C, B_C, C_C, D_C, A_F, B_F, C_F, D_F) 
    % initialize controller state z
    persistent xlon_C
    persistent xlon_F
    if t<ts
          xlon_C = zeros(size(A_C,1),1);
          xlon_F = zeros(size(A_F,1),1); 
    end
    
    F_e = 14.715;
    
    % prefilter the reference command for outer loop
    % solve differential equation defining prefilter
    N = 10; % number of Euler integration steps for each sample
    for i=1:N
        xlon_F = xlon_F + ts/N*(A_F*xlon_F + B_F*h_r );
        % output equation for the prefilter
        h_r_filtered = C_F*xlon_F + D_F*h_r;
        % error signal for outer loop
        error_out = h_r_filtered - h;
        xlon_C = xlon_C + ts/N*(A_C*xlon_C + B_C*error_out );
        u = C_C*xlon_C + D_C*error_out + F_e;
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


