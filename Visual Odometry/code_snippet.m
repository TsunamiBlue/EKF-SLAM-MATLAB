% if isempty(rows)||isempty(cols)
%    u = 0.1;
%    [wl, wr] = inverse_kinematics(u, 0);
%    pb.setVelocity([wl, wr], 0.3);
%    break;
% end

t = 0:0.01:pi/2;
xTrace = [1,3,3+0.5*sin(t),4-0.5*cos(t),4+0.5*sin(2*t),4,3,...
    3-0.5*sin(t),2+0.5*cos(t),2-0.5*sin(t)];
yTrace = [1,1,1.5-0.5*cos(t),1.5+0.5*sin(t),2.5-0.5*cos(2*t),3,3,...
    2.5+0.5*cos(t),2.5-0.5*sin(t),1.5+0.5*cos(t)];

[lmx,lmy] = meshgrid(0.5:(4/3):4.5);
landmarks = [lmx(:)'; lmy(:)'];

num = size(landmarks,2);





        simTimeStep = 0.1; % The duration of each step of the simulator (s).
        robotWheelVelScale = 5.33e-3; % The scaling applied to wheel velocities (tk/s).
        robotWheelVelNoise = 2.0/50; % Variance of the Gaussian noise added to the wheel velocities.
        robotMeasureNoisePosition = 0.005; % Variance of the Gaussian noise added to position measurements.
        robotMeasureNoiseAngle = 0.08; % Variance of the Gaussian noise added to angle measurements.
        robotWheelTrack = 0.156; % The distance between the robot wheels (m).
        robotTimeNoise = 0.1; % Variance of the Gaussion noise added to the duration of vel commands.
        worldBoundaries = [0,5;0,5]; % The world is a 5m x 5m square.
        simRobotTrailLimit = 5000; % The maximum entries in the trail. Reduce to save memory.
        robotCameraK = [200 0 200; 0 200 0; 0 0 1]; % The robot camera matrix.
        robotCameraHeight = 0.1; % The height of the camera from the ground (m).
        robotCameraR = [0,-1,0;0,0,-1;1,0,0]'; % The orienation of the camera w.r.t. the robot.
        robotCameraRef = imref2d([200,400]); % Size of the camera output image.
        worldARUCOSize = 0.08; % Size of ARUCO landmarks (m).
        worldARUCONoise = 0.002; % Noise on projection measurement of ARUCO landmarks.
        worldLandmarkMinDist = 0.2; % Minimum distance between generated landmarks.
        % The following three parameters are used to determine which
        % landmarks are in view of the robot.
        robotLandmarkViewAngle = 45 * pi/180; % Maximum absolute angle of landmarks from the robot x axis (rad).
        robotLandmarkDistMin = 0.1; % Minimum distance of landmarks from the robot (m)
        robotLandmarkDistMax = 3.0; % Maximum distance of landmarks from the robot (m)