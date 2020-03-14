function planarVtolAnimation(u)

    % process inputs to function
%     zt        = u(1);
    zv        = u(1);
%     zvdot     = u(2);
    h     = u(2);
%     thetadot  = u(4);
    theta         = u(3);
%     hdot      = u(6);
    t         = u(4);
    
    % drawing parameters
%     base_width = 0.3;
%     base_height = 0.2;
    radius = 0.25;
    wing_length = 0.3;
    
    
    % define persistent variables 
%     persistent base_handle
    persistent left_line_handle
    persistent right_line_handle
    persistent left_wing_handle
    persistent right_wing_handle
    persistent body_handle
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        track_width=2;
        plot([0,2*track_width],[0,0],'k'); % plot track
        hold on
%         base_handle = drawBase(zt, base_width, base_height, []);
        body_handle = drawBody(zv, h, radius, []);
        right_wing_handle = drawRightWing(zv, h, theta, radius, wing_length, []);
        left_wing_handle = drawLeftWing(zv, h, theta, radius, wing_length, []);
        left_line_handle = drawLeftLine(zv, h, theta, radius, wing_length, []);
        right_line_handle = drawRightLine(zv, h, theta, radius, wing_length, []);
        axis([0, 2*track_width, 0, 2*track_width]); %sets axis
    
        
    % at every other time step, redraw base and rod
    else 
%         drawBase(zt, base_width, base_height, base_handle);
        drawBody(zv, h, radius, body_handle);
        drawRightWing(zv, h, theta, radius, wing_length, right_wing_handle);
        drawLeftWing(zv, h, theta, radius, wing_length, left_wing_handle);
        drawLeftLine(zv, h, theta, radius, wing_length, left_line_handle);
        drawRightLine(zv, h, theta, radius, wing_length, right_line_handle)
    end
end

%Function handles

function handle = drawRightLine(zv, h, theta, radius, wing_length, handle)
  X = [zv+(radius+wing_length-.1)*cos(theta), zv+radius*cos(theta)];
  Y = [h+(radius+wing_length-.1)*sin(theta), h+radius*sin(theta)];
    
    if isempty(handle)
        handle = plot(X, Y, 'k');
    else
        set(handle, 'XData', X, 'YData', Y);
        drawnow
    end
end

function handle = drawLeftLine(zv, h, theta, radius, wing_length, handle)
  X = [zv-(radius+wing_length-.1)*cos(-theta), zv - radius*cos(-theta)];
  Y = [h+(radius+wing_length-.1)*sin(-theta), h+radius*sin(-theta)];
    
    if isempty(handle)
        handle = plot(X, Y, 'k');
    else
        set(handle, 'XData', X, 'YData', Y);
        drawnow
    end
end

function handle = drawRightWing(zv, h, theta, radius, wing_length, handle)
    X = zv + (radius + wing_length)*cos(theta);
    Y = h + (radius + wing_length)*sin(theta);
    
    if isempty(handle)
        handle = plot(X,Y,'-ko', 'MarkerSize', 20);
    else
        set(handle, 'XData', X, 'YData', Y);
    end
end

function handle = drawLeftWing(zv, h, theta, radius, wing_length, handle)
    X = zv - (radius + wing_length)*cos(-theta);
    Y = h + (radius + wing_length)*sin(-theta);
    
    if isempty(handle)
        handle = plot(X,Y,'-ro', 'MarkerSize', 20);
    else
        set(handle, 'XData', X, 'YData', Y);
    end
end

%
%=======================================================================
% drawBase 
% this is for the ground target
%=======================================================================
%
% function handle = drawBase(zt, base_width, base_height, handle)
%   pts = [...
%       zt-base_width/2, 0;...
%       zt+base_width/2, 0;...
%       zt+base_width/2, base_height;...
%       zt-base_width/2, base_height;...
%       ];
%   if isempty(handle)
%     handle = fill(pts(:,1), pts(:,2), 'b');
%   else
%     set(handle,'XData',pts(:,1));
%     drawnow
%   end
% end

function handle = drawBody(zv, h, radius, handle) 
  th = 0:pi/5:2*pi;
  xunit = zv + (radius * cos(th));
  yunit = h + (radius * sin(th)); 
  if isempty(handle)
    handle = plot(xunit, yunit);
  else
    set(handle,'XData', xunit, 'YData', yunit);
    drawnow
  end
end
    
