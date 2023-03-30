function new_state = integrate_kinematics(state, dt, lin_velocity, ang_velocity)
%INTEGRATE_KINEMATICS integrate the kinematics of the robot

%   state is the current state, and has the form [x;y;theta]
%   dt is the length of time for which to integrate
%   lin_velocity is the (forward) linear velocity of the robot
%   ang_velocity is the angular velocity of the robot

%   new_state is the state after integration, also in the form [x;y;theta]

   
x = state(1);
y = state(2);
theta = state(3);

if ang_velocity ~= 0
    x_new = x + 1/ang_velocity * (sin(theta+dt*ang_velocity) - sin(theta)) * lin_velocity;
    y_new = y + 1/ang_velocity * (-cos(theta + dt*ang_velocity) + cos(theta)) * lin_velocity;
else
    x_new = x + dt*cos(theta)*lin_velocity;
    y_new = y + dt*sin(theta)*lin_velocity;
end

theta_new = theta + dt * ang_velocity;

new_state = [x_new; y_new; theta_new];
end