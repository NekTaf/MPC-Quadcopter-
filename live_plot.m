function live_plot(y,intervals,fps,xy_lim,z_lim,sim_time)

% LIVE_PLOT - Plot UAV Path
%
%
% Inputs:
%    y - UAV Output states
%    intervals - intervals between each simulation time step
%    fps - annimation speed 
%    xy_lim -  x y axis graph limits
%    z_lim - z axis graph limits


% Time between each frame
dt=1/fps;

figure('Name','UAV Simulation') 


for i=1:intervals:size(y,1)
    
    % Roll Pitch Yaw
    roll=y(i,7);
    pitch=y(i,8);
    yaw=y(i,9);
    
    % Rotational Matrix 
    R = [cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll); ...
     sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll); ...
     -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)];

    % uvw speed vectors along xyz
    u = [y(i,4) 0 0];
    v = [0  y(i,5) 0];
    w = [0 0 y(i,6)];

    % Vector Rotation body-->ground
    u_rotated = R*u';
    v_rotated = R*v';
    w_rotated = R*w';

    % Delete previous annotations  
    if i > 1
        delete(h1);
        delete(h2);
        delete(h3);
        delete(h4);
        delete(h5);
        delete(text);
    end

    % Plot vectors 
    h1=quiver3(y(i,1), y(i,2), y(i,3), u_rotated(1), u_rotated(2), u_rotated(3), 'r','LineWidth', 2, 'MaxHeadSize', 10);
    hold on;
    h2=quiver3(y(i,1), y(i,2), y(i,3), v_rotated(1), v_rotated(2), v_rotated(3), 'g','LineWidth', 2, 'MaxHeadSize', 10);
    hold on
    h3=quiver3(y(i,1), y(i,2), y(i,3), w_rotated(1), w_rotated(2), w_rotated(3), 'b','LineWidth', 2, 'MaxHeadSize', 10);
    hold on 
    h5=quiver3(y(i,1), y(i,2), y(i,3), u_rotated(1)+v_rotated(1), u_rotated(2)+v_rotated(2), u_rotated(3)+v_rotated(3), 'm','LineWidth', 2, 'MaxHeadSize', 10);
    hold on 
    h4=plot3(y(1:5:i,1), y(1:5:i,2), y(1:5:i,3), 'k.');
    legend('u', 'v','w','ω','UAV pos');
    txt = ['Simulation Time: ' num2str(sim_time(i)) ' s'];
    text = annotation('textbox', [0.7 0.1 0.2 0.1], 'String', txt, 'FitBoxToText', 'on', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
    xlim([-xy_lim*1.5 +xy_lim*1.5]);
    ylim([-xy_lim*1.5 +xy_lim*1.5]);
    zlim([0 z_lim+2]);
    drawnow;

    pause(dt)

end
end




