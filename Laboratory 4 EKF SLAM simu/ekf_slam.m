classdef ekf_slam < handle
    %EKF_SLAM The EKF algorithm for SLAM
    
    properties
        x = zeros(3,1); % The estimated state vector
        P = zeros(3,3); % The estimated state covariance
        
		% The covariance values provided here are NOT correct!
        sigxy = 0.1; % The covariance of linear velocity
        sigth = 0.01; % The covariance of angular velocity
        siglm = 0.01; % The covariance of landmark measurements
        
        idx2num = []; % The map from state vector index to landmark id.
        
        dt = 0.1; %Suppose dt is a constant 0.1 for now
        tmp = 0;
        state_ids = []
    end
    
    methods
        function input_velocity(obj, dt, lin_velocity, ang_velocity)
            % Perform the update step of the EKF. This involves updating
            % the state and covariance estimates using the input velocity,
            % the time step, and the covariance of the update step.
            
            %Step 1: Compute At and Bt
            landmark_num = (length(obj.x)-3)/2; %number of landmarks in state estimate
            At = jac_zeta(obj.x, lin_velocity, dt, landmark_num);
            Bt = jac_vel(obj.x, dt, landmark_num);
            
            %Step 2: Covariance prediction
            Qt = [obj.sigxy 0;0 obj.sigth];
            obj.tmp = Bt;
            obj.P = At * obj.P * At' + Bt * Qt * Bt'; % Qt represents noise(covariance) in velocities input
            obj.P = obj.P;
            
            %Step 3: State prediction
            state_new = obj.x + dt * [lin_velocity*cos(obj.x(3,1));
                                      lin_velocity*sin(obj.x(3,1));
                                      ang_velocity;
                                      zeros(2*landmark_num,1)];
            obj.x = state_new;

            % 4 direct integration
            %obj.int =  dt * [cos(obj.int(3)) * u;
            %         sin(obj.int(3)) * u;
            %         q];
        end
        
        function input_measurements(obj, measurements, nums)
            % Perform the innovation step of the EKF. This involves adding
            % new (not previously seen) landmarks to the state vector and
            % implementing the EKF innovation equations. You will need the
            % landmark measurements and you will need to be careful about
            % matching up landmark nums with their indices in the state
            % vector and covariance matrix.

            for i = 1:numel(nums)
               lm = measurements(:,i);
               id = nums(i);
               
               % if the id is in our state_ids we don't need to add anything
               if any(id == obj.state_ids)
                   continue
               end
               
               % Augment
               % if the observed landmark is new, add to the state
               inertial_lm = convert2inertial(obj.x(1:3),lm);
               
               % compute G matrix for obj.P expansion
               G_xi_0 = [1,0,-sin(obj.x(3))*lm(1)-cos(obj.x(3))*lm(2);
                        0,1,cos(obj.x(3))*lm(1)-sin(obj.x(3))*lm(2)];
               G_xi = [G_xi_0,zeros(2,numel(obj.x)-3)];
                
               G_z = [cos(obj.x(3)),-sin(obj.x(3));
                      sin(obj.x(3)),cos(obj.x(3))];
                  
               % expand the state when new landmark added
               obj.state_ids = [obj.state_ids, id];
               obj.x = [obj.x; inertial_lm];
               
               
               
               % expand cov mat
               Qt = [obj.sigxy 0;0 obj.sigth];
               obj.P = [obj.P, obj.P*G_xi';
                        G_xi*obj.P, G_xi*obj.P*G_xi'+G_z*Qt*G_z'];
            end
            
            
            %Complete remaining steps of EKF           
        end
        
        function add_new_landmarks(obj, y, nums)
            % Add a new (not seen before) landmark to the state vector and
            % covariance matrix. You will need to associate the landmark's
            % id number with its index in the state vector.
            % y:lms nums:lm ids

            N = numel(nums);
            length = numel(obj.state_ids); %number of landmarks in state
            Q = eye(2*N) * obj.siglm;
            C = []; % intialise C matrix
            z_err = []; % error matrix
            
            for i = 1:N
               z_i = y(:,i);
               id = nums(i);
               % find the position of the measurement in the array
               index = find(obj.state_ids == id);
               
               % compute the C matrix
               lm = obj.x(3+index*2-1: 3+index*2); % landmark position in state_vector 
               lx = lm(1); ly = lm(2);
               th = obj.x(3); % theta
               % grad of measure
               % first three non-zero terms
               C_0 = [-cos(th),-sin(th),sin(th)*(obj.x(1)-lx)-cos(th)*(obj.x(2)-ly);
                      sin(th),-cos(th),cos(th)*(obj.x(1)-lx)+sin(th)*(obj.x(2)-ly)];
               % non-zero terms associate with landmark index
               C_lm = [cos(th),sin(th);
                       -sin(th),cos(th)];
               % ith C matrix
               Ci = [C_0,zeros(2,2*(index-1)),C_lm,zeros(2,2*(length-index))];
               % overall constructed C matrix
               C = [C;Ci]; %#ok<*AGROW>
               
               % Compute z_err
               rotation_matrix = [cos(th), -sin(th);
                                  sin(th), cos(th)];
               zHat = rotation_matrix' * (lm - obj.x(1:2));
               z_err = [z_err; zHat - z_i];
            end
               % Kalman gain
               
               K = obj.P * C' /(C * obj.P * C' + Q);
               % Update obj.P
               obj.P = (eye(numel(obj.x)) - K * C) * obj.P;
               obj.x = obj.x - K * z_err;
           
            %Complete remaining steps of EKF   
            
        end
        
        function [robot, cov] = output_robot(obj)
            % Suggested: output the part of the state vector and covariance
            % matrix corresponding only to the robot.
            robot = obj.x;
            cov = obj.P;
            

        end
        
        function [landmarks, cov] = output_landmarks(obj)
            % Suggested: output the part of the state vector and covariance
            % matrix corresponding only to the landmarks.
            landmarks = obj.x;
            cov = obj.P;

        end
        
    end
end

 % Jacobians and System Functions
 
function x1 = f(x0,u)
    % integrate the input u from the state x0 to obtain x1.

end

function At = jac_zeta(x,u,dt, landmark_num)
    % Given the state x0 and input signal u, compute the A_t matrix.
    % Landmark_num is the number of landmarks in the state
    
    A1 = [1 0 -dt * u * sin(x(3,1));
          0 1 dt * u * cos(x(3,1));
          0 0 1];
    zero_matrix = zeros(3,2*landmark_num);  
    identity_m = eye(2*landmark_num);
    
    At = [         A1 zero_matrix;
          zero_matrix' identity_m];
end

function Bt = jac_vel (x, dt, landmark_num)
    % Given the state x, compute the B_t matrix.
    % Landmark_num is the number of landmarks in the state
        Bt = dt .* [cos(x(3,1)) 0;
                    sin(x(3,1)) 0;
                    0 1;
                    zeros(2*landmark_num,2)]; 
end

function y = h(x, idx)
    % Given the state x and a list of indices idx, compute the state
    % measurement y.

end

function H = jac_h(x, idx)
    % Given the state x and a list of indices idx, compute the Jacobian of
    % the measurement function h.

end

function inertial_lm = convert2inertial(pose, lm)
% convert the landmark measurement to the inertial frame
% #inputs:
% pose: robot pose
% lm: landmark position (x;y) in BFF
% #output:
% inertial_lm: landmark position (x;y) in inertial frame
    th = pose(3); %theta
    R = [cos(th),-sin(th);
         sin(th),cos(th)];
    t = pose(1:2,:);
    inertial_lm = R * lm + t;
end