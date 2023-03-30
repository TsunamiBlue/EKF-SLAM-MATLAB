function [wl, wr] = inverse_kinematics(u, q)
% Compute the left and right wheel velocities (wl, wr) required for the robot
% to achieve a forward speed u and angular speed q.

% The scale parameter and wheel track required to solve this are provided here.
% You can find these values in the robot simulator as well.
% In real-life, you would have to measure or calibrate them!
%scale_parameter = 4.50e-3;
%wheel_track = 0.12;

scale_parameter = 5.33e-3;
wheel_track = 0.156;

% for test
%scale_parameter = 0.005332606917967;
%wheel_track = 0.155797269290760;

A = [[1/scale_parameter,-wheel_track/(2*scale_parameter)] ; [1/scale_parameter,wheel_track/(2*scale_parameter)]];

ans_m = A * [u;q];
%disp(ans_m)
wr = ans_m(2);
wl = ans_m(1);

%wl = (u - wheel_track/2 * q) / scale_parameter;
%wr = (u + wheel_track/2 * q) / scale_parameter;

end