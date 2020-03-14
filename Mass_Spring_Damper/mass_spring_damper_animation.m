function massSpringDamperAnimation(u)

    % process inputs to function
    z        = u(1);
%     zdot     = u(2);
    t        = u(2);
    
    % drawing parameters
    L = 1;
    gap = 0.01;
    width = 1.0;
    height = 0.75;
    
    % define persistent variables 
    persistent base_handle
    persistent spring_handle
    persistent damper_handle
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        track_width=2;
        plot([-track_width,track_width],[0,0],'k'); % plot track
        hold on
        base_handle = drawMass(z, width, height, gap, []);
        spring_handle = drawSpring(z, width, height, []);
        damper_handle = drawDamper(z, width, height, []);
        
        axis([-track_width, track_width, -L, 2*track_width-L]); %sets axis
    
        
    % at every other time step, redraw base and rod
    else 
        drawMass(z, width, height, gap, base_handle);
        drawSpring(z, width, height, spring_handle);
        drawDamper(z, width, height, damper_handle);
    end
end

   
%
%=======================================================================
% drawBase
% draw the base of the pendulum
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawMass(z, width, height, gap, handle)
  
  pts = [...
      z-width/2, gap;...
      z+width/2, gap;...
      z+width/2, gap+height;...
      z-width/2, gap+height;...
      ];

  if isempty(handle)
    handle = fill(pts(:,1),pts(:,2),'b');
  else
    set(handle,'XData',pts(:,1));
    drawnow
  end
end

function handle = drawSpring(z, width, height, handle)
    X = [-2 , z - width/2];
    Y = [height - .25, height - .25];
        
    if isempty(handle)
        handle = plot(X, Y, 'r', 'LineWidth', 2);
    else
        set(handle,'XData',X,'YData',Y);
        drawnow
    end 
end  

function handle = drawDamper(z, width, height, handle)
    X = [-2 , z - width/2];
    Y = [height - .5, height - .5];
        
    if isempty(handle)
        handle = plot(X, Y, 'k', 'LineWidth', 2);
    else
        set(handle,'XData',X,'YData',Y);
        drawnow
    end 
end  