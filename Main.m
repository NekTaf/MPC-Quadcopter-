%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Author: Nektarios Aristeidis Tafanidis
%%% Date: 30/04/23
%%% About:  MPC Controller design for UAV tracking circular trajectory at
%%%         a height of 5m with an angular velocity of 0.1 rad/sec
%%%         while maintaining 0 heading.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

% Define the parameters
g = 9.81;               % Acceleration due to gravity (m/s^2)
m = 1;                  % Mass (kg)
Ixx = 9.3e-3;           % Moment of inertia about x-axis (kg*m^2)
Iyy = 9.2e-3;           % Moment of inertia about y-axis (kg*m^2)
Izz = 15.1e-3;          % Moment of inertia about z-axis (kg*m^2)
w = 0.1;                % Angular velocity (rad/s)
radius = 5;             % Radius (m)
height = 5;             % Height (m)

% MPC controller
dt = 0.1;               % sample time (sec)
Np = 40;                % prediction horizon
Nc = 10;                % control horizon
T_sim = 300;             % Total simulation time (sec)

% Simulation time
t = 0:dt:T_sim;
Nt = length(t);

 
% State Matrix 
A = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, -g, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, g, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];


% Control Matrix 
B = [0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    1/m, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 1/Ixx, 0, 0;
    0, 0, 1/Iyy, 0;
    0, 0, 0, 1/Izz];

% Output Matrix 
C=[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

    
% Feed-forward Matrix 
D = zeros(12,4);

% UAV State Space Model
quad_model = ss(A, B, C, D);

% I/O names
quad_model.OutputName = {'x','y','z','x_dot','y_dot','z_dot',...
    'phi','theta','psi','phi_dot','theta_dot','psi_dot'};

quad_model.InputName = {'T','t_phi','t_theta','t_psi'};

% number of states & inputs
Nx = size(A,1); 
Nu = size(B,2); 

% MPC controller
mpcobj = mpc(quad_model, dt, Np, Nc);
% Initial conditions
mpcobj.Model.Nominal.X = [zeros(1,12)];
% Controller Weights
mpcobj.Weights = struct('MV',[1 1 1 1],...
    'MVRate',[0.5 1 1 1],'OV',[1.2 1.2 0.5 0.8 0.8 zeros(1,3) 1 zeros(1,3)]);


% UAV states used in simulation 
xc = mpcstate(mpcobj);

% Set constraints to controller outputs
F = [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;    % Yaw angle
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0];    % Altitude

% Constraint constant for I/O
G = [0;height];

% MV constraint constant
E = zeros(4);

% Set hard constraints 
V=[0;0;0;0];

% Apply constraints 
setconstraint(mpcobj,E,F,G,V)

% I/O simulation states
y = zeros(Nt,Nx);
u = zeros(Nt,Nu);

% Distance of UAV from origin
d_centre=zeros(1,Nt);

% Generate circle for refrence 
[circle_x,circle_y] = create_circle(10000,radius);

% Generate random number to choose starting point 
rng(fix(sum(clock)));
random_index = randi([1, numel(circle_x)], 1);

% Initial angular position 
theta = atan2(circle_y(random_index),circle_x(random_index));


for i = 1:Nt

    % Update model states
    y(i,:) = xc.Plant';

    % Distance from origin
    dx_centre = 0-y(i,1);
    dy_centre = 0-y(i,2);
    d_centre(i) = sqrt((dx_centre)^2 + (dy_centre)^2);
    
    % Distance from circle
    d1 = sqrt((y(i,1)-circle_x(random_index))^2 + ((y(i,2)-circle_y(random_index))^2));
    
    % Calculate x and y coordinates of next position
    x_r = radius*cos(theta);
    y_r = radius*sin(theta);
    
    % Calculate x and y speed vectors to guide point to next position
    vx = (x_r - y(i,1))/dt;
    vy = (y_r - y(i,2))/dt;
    
    theta = theta + w*dt;

    % Refrence
    r = [x_r y_r height vx vy 0 0 0 0 0 0 0]; 
        
    % Compute optimal control action and update controller states
    [u(i,:),info(i)] = mpcmove(mpcobj,xc,y(i,:),r,[]);
    
end

%% Live Plot
close all
live_plot(y,5,100,radius,height,t);



%% Graphs and Results 
close all

% Results & Evaluate preformance
review(mpcobj)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name','Response') 

% Plot waypoints
plot3(y(:,1), y(:,2), y(:,3), 'go');
hold on
plot3(circle_x,circle_y,height*ones(size(circle_x)),'k','LineWidth',2)

legend('UAV path', 'Refrence path');

% axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
zlim([0 inf])
title('Response');
grid on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure ('Name','Distance form origin') 

% Plot the points
plot(t, d_centre)

% Add labels and title
xlabel('Time (sec)')
ylabel('distance (m)')
title('Distance form origin')
grid on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure ('Name','Altitude') 

% Plot the points
plot(t, y(:,3))

% Add labels and title
xlabel('Time (sec)')
ylabel('Z (m)')
title('Altitude')
grid on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure ('Name','Angular speed') 

% Plot the points
plot(t, sqrt(y(:,4).^2 + y(:,5).^2)./radius)

% Add labels and title
xlabel('Time (sec)')
ylabel('W (rad/s)')
title('Angular speed')
grid on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure ('Name','Angles') 


sgtitle('Euler Angles')


% Plot the points
subplot(3, 1, 1); % create first subplot
plot(t, 180/pi*(y(:,7)))

% Add labels and title
xlabel('Time (sec)')
ylabel('φ (deg)')
title('Roll Angle')
grid on


subplot(3, 1, 2); % create first subplot
plot(t, 180/pi*(y(:,8)))

% Add labels and title
xlabel('Time (sec)')
ylabel('θ (deg)')
title('Pitch Angle')
grid on


subplot(3, 1, 3); % create first subplot
plot(t, 180/pi*(y(:,9)))

% Add labels and title
xlabel('Time (sec)')
ylabel('ψ (deg)')
title('Yaw Angle')
grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure ('Name','Angular Velocities') 

sgtitle('Angular Velocities')


% Plot the points
subplot(3, 1, 1); % create first subplot
plot(t, 180/pi*(y(:,10)))

% Add labels and title
xlabel('Time (sec)')
ylabel('P (deg/sec)')
title('Roll Rate')
grid on


subplot(3, 1, 2); % create first subplot
plot(t, 180/pi*(y(:,11)))

% Add labels and title
xlabel('Time (sec)')
ylabel('Q (deg/sec)')
title('Pitch Rate')
grid on


subplot(3, 1, 3); % create first subplot
plot(t, 180/pi*(y(:,12)))

% Add labels and title
xlabel('Time (sec)')
ylabel('R (deg/sec)')
title('Yaw Rate')
grid on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure ('Name','Velocities (Body axis)') 

sgtitle('Velocities (Body axis)')

% Plot the points
subplot(3, 1, 1); % create first subplot
plot(t, y(:,4))

% Add labels and title
xlabel('Time (sec)')
ylabel('u (m/sec)')
title('X')
grid on


subplot(3, 1, 2); % create first subplot
plot(t, y(:,5))

% Add labels and title
xlabel('Time (sec)')
ylabel('v (m/sec)')
title('Y')
grid on


subplot(3, 1, 3); % create first subplot
plot(t,y(:,6))

% Add labels and title
xlabel('Time (sec)')
ylabel('w (m/sec)')
title('Z')
grid on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure ('Name','Control inputs') 

sgtitle('Control Inputs')


% Plot the points
subplot(4, 1, 1); % create first subplot
plot(t, u(:,1))

% Add labels and title
xlabel('Time (sec)')
ylabel('T (N)')
title('Total Upwards force (F-gm)')
grid on


subplot(4, 1, 2); % create first subplot
plot(t, u(:,2))

% Add labels and title
xlabel('Time (sec)')
ylabel('t_φ (Nm)')
title('Pitch Torque')
grid on


subplot(4, 1, 3); % create first subplot
plot(t,u(:,3))

% Add labels and title
xlabel('Time (sec)')
ylabel('t_θ (Nm)')
title('Roll Torque')
grid on


subplot(4, 1, 4); % create first subplot
plot(t,u(:,4))

% Add labels and title
xlabel('Time (sec)')
ylabel('t_ψ (Nm)')
title('Yaw Torque')
grid on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%








 
