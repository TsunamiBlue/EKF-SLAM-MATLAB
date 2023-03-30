% Calibrate the scale parameter and wheel track of the robot
addpath("../simulator/"); % Add the simulator to the MATLAB path.
pb = piBotSim("floor.jpg");

% pb = PiBot("192.168.50.1"); % Use this command instead if using PiBot.

scale_parameter = 0.0;
wheel_track = 0.0; 

% Write your code to compute scale_parameter and wheel_track below.
% HINTS:
% - In simulator: Start by placing your robot (pb.place). Then, drive forward for a known
% time, and measure the robot position (pb.measure) to compute the
% velocity. This will let you solve for the scale parameter.
% - For PiBots: Put a mark on the ground and place your robot on the mark.
% Let it run for certain time and measure the pose, or let it run for certain distance and
% and measure the time.
% - Using multiple trials with different speeds is key to your success!


% Place the robot at the start of a square
pb.place([0.5;0.5], 0);
v_list = [10,20,30,40,50,60,70];
wl = 50;
wr = 50;
sample_scale_list = [];
sample_track_list = [];

lr_scale_X = [];
lr_scale_Y = [];
lr_track_X = [];
lr_track_Y = [];

for j=1:10
    for i=1:7
        % calibrate the scale factor
        pb.place([0.5;0.5],0);
        [startpoint,~] = pb.measure();
        pb.setVelocity([v_list(i),v_list(i)],5);
        [endpoint,~] = pb.measure();
        current_u = (endpoint(1)-startpoint(1))/5;
        current_scale = current_u/v_list(i);
        sample_scale_list = [sample_scale_list,current_scale];

        lr_scale_X = [lr_scale_X;v_list(i)];
        lr_scale_Y = [lr_scale_Y;current_u];
    
        % calibrate the wheel_track
        [~,start_ang] = pb.measure();
        pb.setVelocity([-v_list(i),v_list(i)],1);
        [~,end_ang] = pb.measure();
        %disp(start_ang);
        %disp(end_ang);
        delta_ang = end_ang-start_ang;
        current_track = 2*(v_list(i)*current_scale)/delta_ang;
        %current_track = 2*v_list(i)/delta_ang;
        sample_track_list = [sample_track_list,current_track];

        lr_track_X = [lr_track_X;delta_ang];
        lr_track_Y = [lr_track_Y;2*(v_list(i)*current_scale)];
    
        pb.place([0.5;0.5], 0);
    end
end
% 1-D linear regression in L2 is the same as mean of all samples

% find scale_factor
scale_parameter = mean(sample_scale_list);
disp(scale_parameter);

% find wheel_track
wheel_track = mean(sample_track_list);
disp(wheel_track);

mdl_scale = fitlm(lr_scale_X,lr_scale_Y);
mdl_track = fitlm(lr_track_X,lr_track_Y);

mdl_scale.Coefficients;
mdl_track.Coefficients;

%plot(mdl_scale);
plot(mdl_track);
disp("DONE.");

