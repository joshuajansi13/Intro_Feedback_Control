function F = mass_spring_damper_ctrl(in,P)
    z_c = in(1); %-----change
    z   = in(2); %-----change
    t   = in(3); %-----change
    
    % set persistent flag to initialize integrator and 
    % differentiator at the start of the simulation
    persistent flag
    if t<P.Ts,
        flag = 1;
    else
        flag = 0;
    end
      
    % compute equilibrium force F_e
%     F_e = P.k * z; %-------change

    F = PID_z(z_c,z,flag,P.kp,P.ki,P.kd,...
                       P.F_max,P.Ts,P.beta); %-----change

%     F = F_e + F_tilde; %-----change
    
end
%------------------------------------------------------------
% PID control for z
function u = PID_z(z_c,z,flag,kp,ki,kd,limit,Ts,beta) %-----change
    % declare persistent variables
    persistent integrator
    persistent zdot
    persistent error_d1
    persistent z_d1
    % reset persistent variables at start of simulation
    if flag==1,
        integrator  = 0;
        zdot    = 0;
        error_d1    = 0;
        z_d1    = 0;
    end
    % compute the error
    error = z_c-z; 
    % update derivative of y
    zdot = beta*zdot + (1-beta)*((z-z_d1)/Ts);
    % update integral of error
    if abs(zdot)<0.05,
    integrator = integrator + (Ts/2)*(error+error_d1);
    end
    % update delayed variables for next time through the loop
    error_d1 = error;
    z_d1 = z;
    % compute the pid control signal
    u_unsat = kp*error + ki*integrator - kd*zdot;
    u = sat(u_unsat,limit);
    
    % integrator anti-windup
    if ki~=0,
        integrator = integrator + Ts/ki*(u-u_unsat);
    end
end
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end