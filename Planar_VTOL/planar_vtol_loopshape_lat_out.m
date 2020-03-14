addpath ./.. % adds the parent directory to the path
planar_vtol_param

Pa.P_lat_out = tf([-(Pa.mc+2*Pa.mw)*Pa.g],[(Pa.mc+2*Pa.mw), Pa.mu, 0]);
Pa.C_lat_out = tf([(Pa.kd_z+Pa.kp_z*Pa.sigma),(Pa.kp_z+Pa.ki_z*Pa.sigma),Pa.ki_z],[Pa.sigma,1,0]);
Plant = Pa.P_lat_out*Pa.C_lat_out;

figure(2), clf, bode(Plant,logspace(-4,4)), hold on, grid on
figure(3), clf, bodemag(Plant), hold on, grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Define Design Specifications
% add_spec_tracking(0.01, 0.1);
add_spec_noise(10^-5, 100);    
% add_spec_disturbance(0.1, 0.07, Plant*C_pid);
% add_spec_tracking_step(0.01);
% add_spec_tracking_ramp(0.03);
% add_spec_tracking_parabola(0.010

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Design
C = tf(1, 1);
C = add_control_lead(C, 0.5, 35);
C = add_control_lead(C, 1, 10);
C = add_control_lag(C, 10, 10);     
% C = add_control_proportional(C, -10);
C = add_control_lpf(C, 0.009);
% C = add_control_integral(C, -0.9);
% 
figure(2), bode(Plant * C)
figure(3), margin(Plant * C)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prefilter Design
F = tf(1, 1);
F = add_control_notch(F, 2.5, 1);
% F = add_control_lpf(F, 0.9);
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create plots for analysis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Open-loop tranfer function 
  OPEN = Plant*C;
% closed loop transfer function from R to Y
  CLOSED_R_to_Y = (Plant*C/(1+Plant*C));
% closed loop transfer function from R to U
  CLOSED_R_to_U = (C/(1+C*Plant));
figure(4), clf
    subplot(3,1,1), 
        bodemag(CLOSED_R_to_Y), hold on
        bodemag(CLOSED_R_to_Y*F)
        bodemag(Pa.P_lat_out*Pa.C_lat_out/(1+Pa.P_lat_out*Pa.C_lat_out))
        title('Closed Loop Bode Plot'), grid on
        legend
    subplot(3,1,2), 
        step(CLOSED_R_to_Y), hold on
        step(CLOSED_R_to_Y*F)
        title('Closed loop step response'), grid on
    subplot(3,1,3), 
        step(CLOSED_R_to_U), hold on
        step(CLOSED_R_to_U*F)
        title('Control effort for step response'), grid on
%print('../../../figures/hw_satellite_compensator_out_design_5','-dpdf','-bestfit')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to state space equations for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Pa.C_lateral_out = Pa.C_lat_out * C;
[Pa.num_C_lat_out, Pa.den_C_lat_out] = tfdata(Pa.C_lateral_out,'v');
[Pa.A_C_lat_out,Pa.B_C_lat_out,Pa.C_C_lat_out,Pa.D_C_lat_out]=tf2ss(Pa.num_C_lat_out, Pa.den_C_lat_out);

[Pa.num_F_lat_out,Pa.den_F_lat_out] = tfdata(F,'v');
[Pa.A_F_lat_out, Pa.B_F_lat_out, Pa.C_F_lat_out, Pa.D_F_lat_out] = tf2ss(Pa.num_F_lat_out,Pa.den_F_lat_out);


%--- general tracking specification ---
% track references above omega_r by gamma_r
function add_spec_tracking(gamma_r, omega_r)
	w = logspace(log10(omega_r)-2,log10(omega_r));
	plot(w,20*log10(1/gamma_r)*ones(size(w)),'g')
end

%--- steady state tracking of step ---
% track step to within gamma_r
function add_spec_tracking_step(gamma_r)
    w = logspace(-5, 0);
    plot(w, 20*log10(1/gamma_r -1)*ones(size(w)),'g')
end

%--- steady state tracking of ramp ---
% track ramp to within gamma_r
function add_spec_tracking_ramp(gamma_r)
    w = logspace(-4, 0);
    plot(w,20*log10(1/gamma_r)-20*log10(w),'g')
end

%--- steady state tracking of parabola ---
% track ramp to within gamma_r
function add_spec_tracking_parabola(gamma_r)
    w = logspace(-5, 0);
    plot(w,20*log10(1/gamma_r)-40*log10(w),'g')
end

%--- input disturbance specification ---
% reject distubance below omega_d by gamma_d
function add_spec_disturbance(gamma_d, omega_d, Plant)
    w = logspace(log10(omega_d)-2, log10(omega_d));
    Pmag=bode(Plant,w);
    for i=1:size(Pmag,3), Pmag_(i)=Pmag(1,1,i); end
    plot(w,20*log10(1/gamma_d)*ones(1,length(Pmag_))+20*log10(Pmag_),'g')
end

%--- noise specification ---
% attenuate noise about omega_n by gamma_n
function add_spec_noise(gamma_n, omega_n)
	w = logspace(log10(omega_n),2+log10(omega_n));
	plot(w,20*log10(gamma_n)*ones(size(w)),'g')
end

% proportional control: change cross over frequency
% proportional gain kp
function Cnew = add_control_proportional(C, kp)
	Cnew = C*kp;
end

% integral control: increase steady state tracking and dist rejection
% ki: frequency at which integral action ends
function Cnew = add_control_integral(C, ki) 
	Integrator = tf([1, ki], [1, 0]);
	Cnew = C*Integrator;
end

% phase lag: add gain at low frequency (tracking, dist rejection)
% z: frequency at which lag ends
% M: separation between pole and zero
function Cnew = add_control_lag(C, z, M)
    Lag = tf([1, z], [1, z/M]);
	Cnew = C * Lag;
end

% low pass filter: decrease gain at high frequency (noise)
% p:  low pass filter frequency
function Cnew = add_control_lpf(C, p)
    LPF = tf(p,[1, p]);
    Cnew = C * LPF;
end

% phase lead: increase PM (stability)
% wmax: location of maximum frequency bump
% M: separation between zero and pole
function Cnew = add_control_lead(C, omega_L, M)
    gain = (1+sqrt(M))/(1+1/sqrt(M));
    Lead =tf(gain * [1, omega_L/sqrt(M)], [1, omega_L*sqrt(M)]);
	Cnew = C * Lead;
end

% notch filter: reduce closed loop peaking at cross over
% ws: frequency to start the notch
% M: width of the notch
function Cnew = add_control_notch(C, ws, M)
    Notch = tf([1,2*sqrt(M)*ws,M*ws^2],[1,(M+1)*ws,M*ws^2]);
	Cnew = Notch * C;
end
