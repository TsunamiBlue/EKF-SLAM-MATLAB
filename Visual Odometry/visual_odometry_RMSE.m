% Always begin by using addpath
%addpath("D:\ENGN6627-Robotics\MatlabSimu\simulator\")
% Determine where your m-file's folder is.
folder = fileparts(which("D:\ENGN6627-Robotics\MatlabSimu\")); 
% Add that folder plus all subfolders to the path.
addpath(genpath(folder));


% Create a window to visualise the robot camera
%figure();
%camAxes = axes();


% For testing, we can tell the simulator where we want to place our
% landmarks. Let's try a grid formation
[lmx,lmy] = meshgrid(0.5:(4/3):4.5);
landmarks = [lmx(:)'; lmy(:)'];

% Now, we can start the simulator with these landmarks.
% You can also try set your own landmarks, or, if you leave it blank, the
% simulator will generate landmarks randomly.
pb = piBotSim("floor_course.jpg", landmarks);
pb.showLandmarks(true); % Use this to turn on/off showing the landmarks.
pb.showViewRange(true); % Use this to turn on/off showing the pibot view.

% Place your robot at the centre of the room
x0 = 1;
y0 = 1;
theta0 = 0;
pb.place([x0;y0],theta0);

img = pb.getImage();
%imshow(img, "Parent", camAxes); % Check the video

% Which landmarks can we measure? Try the measurement function
[landmarkPoints, landmarkIds] = pb.measureLandmarks();
% How accurate are these measurements? While testing, you can know where 
% the landmarks are, so you can check what kind of error the measurements
% have. How is the error distributed?


% Now you can use these measurements in your odometry observer!
% You will also need to use your known input velocity to the robot.
% I strongly suggest you try some simple examples before you try to follow
% a line, e.g. drive in a straight line for a few seconds.
% This will also help to evaluate your solution!


% plot the ground truth track
figure();
track_axes = gca();
hold on
title("Visual odometry")
xlim(track_axes,[0,5]);
ylim(track_axes,[0,5]);
axis(track_axes,'manual');
%init all params
%total num of landmarks
num_lm = size(landmarks,2);
% mark observed landmark
olm_indicator = zeros(1,num_lm);
% time interval
dt = 0.1;
pose0 = [cos(theta0) -sin(theta0) x0;
        sin(theta0) cos(theta0) y0;
        0 0 1];
% landmark_coords
pbar = zeros(3,num_lm);
track = [];
gain_k0 = 0.3;
gain_ki = [];
ci = [];

t = 0;


while true
    img = pb.getImage();
    [u,q,eol] = line_follow(img, 2.0);
    if eol
        disp("END OF LINE WARNING!!!")
        break;
    end
    
    [wl,wr] = inverse_kinematics(u,q);
    
    pb.setVelocity(wl,wr);
    t = t + dt;
    
    % ^B_A W _B
    W = [0 -q u;
         q 0 0;
         0 0 0];
    
    
    [observed_landmarks, olm_ids] = pb.measureLandmarks();

    % homogeneous coords expressed in B of landmarks
    y = [observed_landmarks;ones(1,size(olm_ids,2));olm_ids];

    ybar = y(1:3,:);
    
    % indicator update
    if ~isempty(y)
        olm_indicator(1,y(4,:)) = olm_indicator(1,y(4,:)) + 1;
    end
    
    posex = pose0 * ybar; 
    pObserve = pbar(:,y(4,:));
    % init for 1st case ignore the rest
    for k = 1:size(y,2)
        if pObserve(3,k) == 0
            pObserve(:,k) = posex(:,k);
        end
    end
    
    % calculate the error
    ebar = pose0 * ybar - pObserve;
    
    % update gain 
    [gain_k0, gain_ki, ci] = compute_gain(ebar, dt, y, olm_indicator);
    
    [pose0,posex] = observer(pose0,pObserve,ebar,ybar,dt,W,gain_k0,gain_ki,ci);
    
    %the estimated p position
    pbar(:,y(4,:)) = posex;
     
    % plot the track
    track = [track, pose0(1:2,3)];
    plot(track(1,:),track(2,:),'c-','Parent',track_axes);
    drawnow;
    % plot the pose
    for j = 1:size(y,2)
        % group by ID
        scatter(posex(1,j),posex(2,j),6,'MarkerFaceColor',cmap(y(4,j)),'MarkerEdgeColor','none'); 
        hold on;
    end
    if t>100
        break;
    end
end

rmse = RMSE(pbar,landmarks); 
disp("RMSE: "+num2str(rmse));
armse = aligned_RMSE(pbar,landmarks); 
disp("aligned RMSE: "+num2str(armse));
plot(pbar(1,:),pbar(2,:),'r*','Parent',track_axes);


% helper method

function [pose0,posex] = observer(pose0,posex,ebar,ybar,dt,W,gain_k0,gain_ki,ci)
% see L06C imple P40
    pose0 = pose0 * expm(dt*(W-gain_k0*Pse(pose0'*ebar*ci*ybar')));
    posex = posex + dt*(1-gain_k0)*ebar*gain_ki;
end

function W = Pse(A)
    W = zeros(3);
    W(1:2,1:2) = 0.5*(A(1:2,1:2)-A(1:2,1:2)');
    W(1:2,3) = A(1:2,3); 
end

