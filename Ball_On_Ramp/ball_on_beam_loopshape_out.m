addpath ./.. % adds the parent directory to the path
ball_on_beam_param 

addpath ./..
ball_on_beam_loopshape_in

% transfer functions
Par.P_out = tf([-Par.g],[1,0,0]);
Par.C_pid_out = tf([(Par.kd_z+Par.kp_z*Par.sigma),(Par.kp_z+Par.ki_z*Par.sigma),Par.ki_z],[Par.sigma,1,0]);
Plant = minreal(Par.P_out*((Par.P_in*Par.C_pd_in)/(1+Par.P_in*Par.C_pd_in)));
figure(2), clf, bode(Plant,'r'), hold on, grid on
figure(3), clf, bodemag(Plant,logspace(-3,5),'r'), hold on, grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Define Design Specifications
%     add_spec_disturbance(0.1, 10^-2, Plant);
add_spec_noise(0.001, 100);    
add_spec_tracking(0.01, 0.1);
% add_spec_tracking_step(0.01);
% add_spec_tracking_ramp(0.03);
% add_spec_tracking_parabola(0.010

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Design
C = tf(1, 1);
%     C = add_control_lag(C, 0.2, 10);
%     % C = add_control_proportional(C, k);
%     % C = add_control_proportional(C, 1);    
    C = add_control_lead(C, 26, 0.001);
%     % C = add_control_integral(C, 0.2);
    C = add_control_lpf(C, 9);
%     % C = add_control_proportional(C, 3);
%     
    figure(2), bode(Plant * C)
    figure(3), margin(Plant * C)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prefilter Design
F = tf(1, 1);
F = add_control_notch(F, 27, 10);
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
        bodemag(Par.P_in*Par.C_pd_in/(1+Par.P_in*Par.C_pd_in))
        title('Closed Loop Bode Plot'), grid on
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
C=minreal(C*Par.C_pid_out);
[Par.num_Cout,Par.den_Cout] = tfdata(C,'v');
[Par.Aout_C,Par.Bout_C,Par.Cout_C,Par.Dout_C]=tf2ss(Par.num_Cout,Par.den_Cout);

[Par.num_Fout,Par.den_Fout] = tfdata(F,'v');
[Par.Aout_F, Par.Bout_F, Par.Cout_F, Par.Dout_F] = tf2ss(Par.num_Fout,Par.den_Fout);

% Par.C_out=C;
    

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
    Lead =tf([gain * 1, gain * omega_L/sqrt(M)],...
        [1, omega_L*sqrt(M)]);
	Cnew = C * Lead;
end

% notch filter: reduce closed loop peaking at cross over
% ws: frequency to start the notch
% M: width of the notch
function Cnew = add_control_notch(C, ws, M)
    Notch = tf([1,2*sqrt(M)*ws,M*ws^2],[1,(M+1)*ws,M*ws^2]);
	Cnew = Notch * C;
end
