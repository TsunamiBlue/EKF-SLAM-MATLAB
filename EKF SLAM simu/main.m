% Always begin by using addpath
addpath("../simulator")

% For testing, we can tell the simulator where we want to place our
% landmarks. Let's try a grid formation
[lmx,lmy] = meshgrid(0.5:(4/3):4.5);
landmarks = [lmx(:)'; lmy(:)'];

% Now, we can start the simulator with these landmarks.
% You can also try set your own landmarks, or, if you leave it blank, the
% simulator will generate landmarks randomly.
pb = piBotSim("floor_spiral.jpg", landmarks);
pb.showLandmarks(true); % Use this to turn on/off showing the landmarks.
pb.showViewRange(true); % Use this to turn on/off showing the pibot view.

% Your final solution should not use any of the above.
% We will set up the simulator for you and choose the landmark positions.
% Your algorithm will not know where the landmarks are initially, and
% will have to estimate their positions based on the measurements.

% Place your robot at the centre of the room
% NOTICE it should be the same as the state vector
pb.place([2.5;2.5],0);
x = 0; y = 0; theta = 0;

% interval
dt = 0.1;
% state vector x_0
state_vector = [x;y;theta];

% tells which element of the measurement corresponds to which id
state_ids = []; 

% pose estimation
estimated_trails = [];

% direct integration
di = [x;y;theta];
direct_integrations = [];



% Which landmarks can we measure? Try the measurement function
[landmarkPoints, landmarkIds] = pb.measureLandmarks();

% Now you can use these measurements in your slam system!
% You will also need to use your known input velocity to the robot.
% I strongly suggest you try some simple examples before you try to follow
% a line, e.g. drive in a straight line for a few seconds.
% This will also help to evaluate your solution!
% figure;
% camAxes = axes();

% show the figure
figure
trail_axes = gca();

hold on
title("EKF-SLAM estiamtion")
xlim(trail_axes, [0 5]);
ylim(trail_axes, [0 5]);
axis(trail_axes, "manual");
grid on
grid minor

% Initialise your EKF class
EKF = ekf_slam();
% aRMSE(landmarks, landmarks + randn(size(landmarks)));

while(true)
    
    % line follow module
    img = pb.getImage();
%     imshow(img, "Parent", camAxes); % Check the video  
    default_speed = 2;
    gry_img = rgb2gray(img);
    bin_img = ~imbinarize(gry_img, 0.2);
    bin_img = bin_img(end-49:end-20, :);
    if ~any(bin_img)
        break;
    end
    [rows, cols] = find(bin_img);
    line_centre = (mean(cols)-400/2)/200;
    u = 0.1*default_speed*(1-line_centre^2);
    q = -2*line_centre;
    [wl, wr] = inverse_kinematics(u,q);
    pb.setVelocity(wl,wr);




    % EKF
    EKF.input_velocity(dt,u,q);
    di =  di + dt * [cos(di(3)) * u;
                sin(di(3)) * u;
                      q];

    [state_vector,cov] = EKF.output_robot();

    % measure landmarks
    [landmarkPoints, landmarkIds] = pb.measureLandmarks();
    if ~isempty(landmarkIds) && ~any(isnan(landmarkPoints(1,:)))
        % run EKF functions
        EKF.input_measurements(landmarkPoints, landmarkIds);

        EKF.add_new_landmarks(landmarkPoints,landmarkIds);
        [state_vector,cov] = EKF.output_landmarks();
    end




    % END
    % normalization theta
    while state_vector(3) > 2 * pi
        state_vector(3) = state_vector(3) - 2 * pi;
    end
    while state_vector(3) < 0
        state_vector(3) = state_vector(3) + 2 * pi;
    end

    estimated_trails = [estimated_trails,state_vector(1:3)];
    direct_integrations = [direct_integrations,di];




    % ##########Plot##########
    plot(estimated_trails(1,:),estimated_trails(2,:),'b-','parent',trail_axes);
    title("EKF-SLAM-Visual")
%     grid on
%     grid minor
    hold on
    % plot the integrated trajectory
    plot(direct_integrations(1,:), direct_integrations(2,:),'r-','parent',trail_axes);
    state_ids = EKF.state_ids;
    for i = 1:numel(state_ids)
        color = state_ids(i);
        scatter(state_vector(3+2*i-1),state_vector(3+2*i),8,'MarkerFaceColor'...
            ,cmap(color),'MarkerEdgeColor','none','parent',trail_axes);
        
        text(state_vector(3+2*i-1)+0.1, state_vector(3+2*i)+0.1, cellstr(num2str(color))...
            , 'Color',cmap(color), 'FontSize', 12, 'Parent', trail_axes);
        
        [e1,e2] = elpse_cal(state_vector(3+2*i-1:3+2*i),cov(3+2*i-1:3+2*i,3+2*i-1:3+2*i));
        plot(e1,e2,'Parent',trail_axes,'Color',cmap(color));

%         e0 = plot_ellipses(state_vector(3+2*i-1:3+2*i),cov(3+2*i-1:3+2*i,3+2*i-1:3+2*i)...
%             ,cmap(color),trail_axes);
        
    end
    
    % plot the visiable landmarks
    for i = 1:numel(landmarkIds)
        ilm = find(state_ids == landmarkIds(i));
        plot([state_vector(1),state_vector(3+2*ilm-1)], [state_vector(2),state_vector(3+2*ilm)]...
            ,'r-','Parent',trail_axes);
    end
    
    % plot ellipse

%     e1 = plot_ellipses(state_vector(1:2),cov(1:2,1:2),'b',trail_axes);
    [e11,e12] = elpse_cal(state_vector(1:2),cov(1:2,1:2));
    plot(e11,e12,'Parent',trail_axes,'Color',cmap(color));
    xlim(trail_axes, [-5 5]);
    ylim(trail_axes, [-5 5]);
    axis(trail_axes, 'manual');
    hold off
end

% compute the estimated landmarks matrix
estimated_landmarks = [];
for i = 1:(numel(state_vector) - 3)/2
    index = find(state_ids == i);
    estimated_landmarks = [estimated_landmarks, state_vector(3+2*index-1:3+2*index)];
end

legend('Direct Integration','EKF-SLAM')







RMSE_ALL(estimated_landmarks, estimated_trails, landmarks)

function RMSE_ALL(estimated_landmarks, estimated_trajectory, landmark_positions)
% estimate the RMSE and ARMSE of the robot_trail and estimated landmarks
% #inputs:
% estimated_landmarks: estimated landmarks position
% estimated_trajectory: estimated robrot trajectory over time
    assert(all(size(estimated_landmarks) == size(landmark_positions)), "Estimated and true landmarks are not the same size.");
    assert(size(estimated_trajectory,1) == 3, "Estimated trajectory does not have 3 rows.");
    % Load the true robot trajectory
    load("robot_trail.mat", "simRobotTrail");
    simRobotTrailRows = ~any(isnan(simRobotTrail), 1);
    simRobotTrail = simRobotTrail(:, simRobotTrailRows);
    
    % Match the trajectories. Note they are assumed to start at the same time.
    % The estimated trajectory MUST start as soon as the robot begins simulation.
    trajectory_length = min(size(simRobotTrail, 2), size(estimated_trajectory,2));
    simRobotTrail = simRobotTrail(:,1:trajectory_length);
    estimated_trajectory = estimated_trajectory(:,1:trajectory_length);
    
    % Compute the trajectory error
    estimated_positions = estimated_trajectory(1:2,:);
    true_positions = simRobotTrail(1:2,:);
    trajectory_armse = aRMSE(true_positions, estimated_positions);
    
    % Compute the landmark error
    landmarks_armse = aRMSE(landmark_positions, estimated_landmarks);
    
    % Print the results
    disp("Trajectory RMSE:" + num2str(trajectory_armse));
    disp("Landmark RMSE:" + num2str(landmarks_armse));
end

function armse = aRMSE(points1, points2)
% Compute the aligned RMSE between two matched sets of 2D points
n = size(points1,2);
assert(all(size(points1)==[2,n]));
assert(all(size(points2)==[2,n]));

mu1 = mean(points1,2);
mu2 = mean(points2,2);

Sig = 1/n * (points2-mu2) * (points1-mu1)';

[U,~,V] = svd(Sig);
A = eye(2);
if det(Sig) < 0
    A(2,2) = -1;
end

R = V * A * U';
x = mu1 - R * mu2;

points1_aligned = R' * (points1 - x);

armse = real(sqrt(1/n * sum((points1_aligned - points2).^2, 'all')));

end


function [e1,e2] = elpse_cal(centre, cov)
fac = 10;

[V,D] = eig(cov);
ang = linspace(0,2*pi);
ellipse = fac*0.5*V*sqrt(D)*[cos(ang);sin(ang)] + centre;
e1 = ellipse(1,:);
e2 = ellipse(2,:);
end

