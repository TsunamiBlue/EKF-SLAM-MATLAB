%% data_collection
close all
clear all
clc
%addpath("D:\ENGN6627-Robotics\MatlabSimu\Laboratory 3 (Visual Odometry)_0917\data\simulator");
% Determine where your m-file's folder is.
folder = fileparts(which("D:\ENGN6627-Robotics\MatlabSimu\")); 
% Add that folder plus all subfolders to the path.
addpath(genpath(folder));

% landmarks alone the course
% landmarks1 = [(1:0.5:3.5);ones(1,6)];
% landmarks2 = [[3.5;1.5],[4;2],[4.5;2.5]];
% landmarks3 = [(4.5:-0.5:2.5);3*ones(1,5)];
% landmarks4 = [(1.5:0.5:2.5);(1.5:0.5:2.5)];
% landmarks5 = [4.5;2];
% landmarks = [landmarks1,landmarks2,landmarks3,landmarks4,landmarks5];

% landmarks alone the course less
% landmarks1 = [2 3;ones(1,2)];
% landmarks2 = [[3.5;1.5],[4;2],[4.5;2.5]];
% landmarks3 = [(4.1:-1:3.1);3*ones(1,2)];
% landmarks4 = [(2.5:-0.5:1.5);(2.5:-0.5:1.5)];
% landmarks = [landmarks1,landmarks2,landmarks3,landmarks4];

% landmarks alone the curve
% landmarks1 = [2;1];
% landmarks2 = 0.5*[cos(-pi/3:pi/6:0);sin(-pi/3:pi/6:0)] + [3;1.5];
% landmarks3 = 0.5*[cos(-pi/2:pi/6:pi/2);sin(-pi/2:pi/6:pi/2)] + [4;2.5];
% landmarks4 = 0.5*[cos(pi/2:pi/6:pi);sin(pi/2:pi/6:pi)] + [3;2.5];
% landmarks5 = 0.5*[cos(pi/2:pi/6:pi);sin(pi/2:pi/6:pi)] + [2;1.5];
% landmarks = [landmarks1,landmarks2,landmarks3,landmarks4,landmarks5];

% landmarks along the line
% landmarks1 = [1:0.5:3;ones(1,5)];
% landmarks2 = 0.5*[cos(3*pi/4);sin(3*pi/4)] + [4;1.5];
% landmarks3 = [4.5;2.5];
% landmarks4 = [2.5:0.5:4;3*ones(1,4)];
% landmarks5 = [2;2];
% landmarks = [landmarks1,landmarks2,landmarks3,landmarks4,landmarks5];

% landmarks alone the course (more)
landmarks1 = [1.5:1:2.5;ones(1,2)];
landmarks2 = 0.5*[cos(-pi/3:pi/6:0);sin(-pi/3:pi/6:0)] + [3;1.5];
landmarks3 = 0.5*[cos(-pi/2:pi/6:pi/2);sin(-pi/2:pi/6:pi/2)] + [4;2.5];
landmarks4 = 0.5*[cos(pi/2:pi/6:pi);sin(pi/2:pi/6:pi)] + [3;2.5];
landmarks5 = 0.5*[cos(pi/2:pi/6:pi);sin(pi/2:pi/6:pi)] + [2;1.5];
landmarks = [landmarks1,landmarks2,landmarks3,landmarks4,landmarks5];

% landmarks grid
% [lmx,lmy] = meshgrid(0.5:(4/3):4.5);
% landmarks = [lmx(:)'; lmy(:)'];

save('D:\ENGN6627-Robotics\MatlabSimu\Laboratory 3 (Visual Odometry)_0917\data\landmarks_sample.mat','landmarks');

pb = piBotSim("floor_course.jpg",landmarks);

start_point = [1;1];
theta0 = 0;

pb.place(start_point,theta0);

speed = 2;

Collected_data = [];
start_time = tic;
while true
    img = pb.getImage();
    gray = rgb2gray(img);
    bin_img = ~imbinarize(gray, 0.2);
    
    imgB = bin_img(end-59:end-30,:);
    [rows, cols] = find(imgB);
    
    if isempty(rows)||isempty(cols)
       u = 0.1;
       [wl, wr] = inverse_kinematics(u, 0);
       pb.setVelocity([wl, wr], 0.3);
       break;
    end
    
    line_centre = (mean(cols)-200)/200;
    u = 0.1*speed*(1-line_centre^2);
    q = -pi*line_centre;
    [wl,wr] = inverse_kinematics(u, q);   
    [lms, ids] = pb.measureLandmarks();
   
    clc
    % evaluate each time step
    elapsed_time = toc(start_time)
    pb.setVelocity(wl, wr);
    start_time = tic;
    
    data.velocity = [u;q];
    data.wheelVelocity = [wl;wr];
    data.landmarks = [lms;ones(1,numel(ids));ids];
    data.time = elapsed_time;
    Collected_data = [Collected_data,data];
end

% the timestep is one step behind 
size = size(Collected_data,2);
for i=1:size-1
    Collected_data(i).time = Collected_data(i+1).time;
end
Collected_data = Collected_data(1:end-1);

save('D:\ENGN6627-Robotics\MatlabSimu\Laboratory 3 (Visual Odometry)_0917\data\data_curve.mat', 'Collected_data');
save('D:\ENGN6627-Robotics\MatlabSimu\Laboratory 3 (Visual Odometry)_0917\data\landmarks_curve.mat','landmarks');