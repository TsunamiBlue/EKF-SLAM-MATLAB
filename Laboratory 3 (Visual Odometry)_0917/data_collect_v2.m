%% data_collection
close all
clear all
clc
%addpath("D:\ENGN6627-Robotics\MatlabSimu\Laboratory 3 (Visual Odometry)_0917\data\simulator");
% Determine where your m-file's folder is.
folder = fileparts(which("D:\ENGN6627-Robotics\MatlabSimu\")); 
% Add that folder plus all subfolders to the path.
addpath(genpath(folder));

% Create a window to visualise the robot camera
figure;
camAxes = axes();

% landmarks grid
[lmx,lmy] = meshgrid(0.5:(4/3):4.5);
landmarks = [lmx(:)'; lmy(:)'];

save('D:\ENGN6627-Robotics\MatlabSimu\Laboratory 3 (Visual Odometry)_0917\data\landmarks_sample.mat','landmarks');

pb = piBotSim("floor_course.jpg",landmarks);

start_point = [1;1];
theta0 = 0;

pb.place(start_point,theta0);

default_speed = 5;

Collected_data = [];
start_time = tic;
while true
    img = pb.getImage();
    % Binarise the image by picking some sensible threshold
    imshow(img, "Parent", camAxes); % Check the video

    gry_img = rgb2gray(img);
    bin_img = ~imbinarize(gry_img, 0.2);
    bin_img = bin_img(end-49:end-20, :);
    % Check the close line is visible
    if ~any(bin_img)
        break;
    end
    % find the close line coordinates in the image
    [rows, cols] = find(bin_img);
    
    line_centre = (mean(cols)-400/2)/200;
    u = 0.1*default_speed*(1-line_centre^2);
    q = -2*line_centre;

    [wl,wr] = inverse_kinematics(u, q);   
    [lms, ids] = pb.measureLandmarks();
   
    clc
    % evaluate each time step
    time_duration = toc(start_time)
    pb.setVelocity(wl, wr);
    start_time = tic;
    
    data.velocity = [u;q];
    data.wheelVelocity = [wl;wr];
    data.landmarks = [lms;ids];
    data.time = time_duration;
    Collected_data = [Collected_data,data];
end

% the timestep correction
len = size(Collected_data,2);
for i=1:len-1
    Collected_data(i).time = Collected_data(i+1).time;
end
Collected_data = Collected_data(1:end-1);

save('D:\ENGN6627-Robotics\MatlabSimu\Laboratory 3 (Visual Odometry)_0917\data\data_curve.mat', 'Collected_data');
save('D:\ENGN6627-Robotics\MatlabSimu\Laboratory 3 (Visual Odometry)_0917\data\landmarks_curve.mat','landmarks');