function ball_on_beam_animation(u)

    % process inputs to function
    z        = u(1);
%     zdot     = u(2);
    theta    = u(2);
%     thetadot = u(4);
    t        = u(3);
    
    % drawing parameters
    L = 1;
%     gap = 0.01;
    radius = 0.25;
    width = 2.0;
    height = .3;
    
    % define persistent variables 
    persistent base_handle
    persistent ball_handle
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        track_width=2;
        plot([-track_width,track_width],[0,0],'k'); % plot track
        hold on
        base_handle = drawBeam(theta, width, []);
        ball_handle = drawBall(z,height,theta,radius,[]);
        axis([-track_width, track_width, -L, 2*track_width-L]); %sets axis
    
        
    % at every other time step, redraw base and rod
    else 
        drawBeam(theta, width, base_handle);
        drawBall(z, height, theta, radius, ball_handle);
    end
end

   
%
%=======================================================================
% drawBase
% draw the base of the pendulum
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawBall(z, height, theta, radius, handle)
  th = 0:pi/5:2*pi;
  xunit = radius * cos(th);
  yunit = radius * sin(th); 
  Pb = [z; height];
  B = [xunit; yunit];
  rot_mat = [cos(theta) -sin(theta);
             sin(theta) cos(theta)];
  B = B + rot_mat*Pb;
  if isempty(handle)
    handle = plot(B(1,:), B(2,:));
  else
    set(handle,'XData', B(1,:), 'YData', B(2,:));
    drawnow
  end
end

function handle = drawBeam(theta, width, handle)
    X = [0, width*cos(theta)];
    Y = [0, width * sin(theta)];
    if isempty(handle)
        handle = plot(X, Y, 'k', 'LineWidth', 3);
    else
        set(handle,'XData', X, 'YData', Y);
        drawnow
    end
end
