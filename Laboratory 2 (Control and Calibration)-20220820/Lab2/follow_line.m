% Calibrate the scale parameter and wheel track of the robot
addpath("../simulator/"); % Add the simulator to the MATLAB path.
pb = piBotSim("floor_spiral.jpg");

% Start by placing your robot at the start of the line
pb.place([2.5;2.5], 0.6421);

% pb = PiBot("192.168.50.1"); % Use this command instead if using PiBot.

% Create a window to visualise the robot camera
figure;
camAxes = axes();

% Follow the line in a loop
xes = [];
yes = [];
init_state = [2.5,2.5, 0.6421];
current_state = init_state;
while true
    start_time = tic;
    
    % First, get the current camera frame
    img = pb.getImage();
    imshow(img, "Parent", camAxes); % Check the video
    
    
    default_speed = 5;
    % Binarise the image by picking some sensible threshold
    gry_img = rgb2gray(img);
    bin_img = ~imbinarize(gry_img, 0.2);
    bin_img = bin_img(end-49:end-20, :);

    % Check the close line is visible
    if ~any(bin_img)
        break;
    end

    % find the close line coordinates in the image
    [rows, cols] = find(bin_img);
    
    %Normalizing and Find the centre of the line to follow
    line_centre = (mean(cols)-400/2)/200;
    
    
    
    % If you have reached the end of the line, you need to stop by breaking
    % the loop.
    end_of_line = false;
    if (end_of_line)
        break;
    end

    % Use the line centre to compute a velocity command
    u = 0.1*default_speed*(1-line_centre^2);
    q = -2*line_centre;

    % Compute the required wheel velocities
    [wl, wr] = inverse_kinematics(u,q);
    
    % Apply the wheel velocities
    pb.setVelocity(wl,wr);
    end_time = toc(start_time);
    delta_t = end_time;
    new_state = integrate_kinematics(current_state,delta_t,u,q);
    current_state = new_state;
    xes = [xes;new_state(1)];
    yes = [yes;new_state(2)];
end


% Save the trajectory of the robot to a file.
% Don't use this if you are using PiBot.
pb.saveTrail();