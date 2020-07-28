% CREATE_AB_NONLINEAR
% 16-833 Spring 2020 - *Stub* Provided
% Computes the A and b matrices for the 2D nonlinear SLAM problem
%
% Arguments: 
%     x       - Current estimate of the state vector
%     odom    - Matrix that contains the odometry measurements
%               between consecutive poses. Each row corresponds to
%               a measurement. 
%                 odom(:,1) - x-value of odometry measurement
%                 odom(:,2) - y-value of odometry measurement
%     obs     - Matrix that contains the landmark measurements and
%               relevant information. Each row corresponds to a
%               measurement.
%                 obs(:,1) - idx of pose at which measurement was 
%                   made
%                 obs(:,2) - idx of landmark being observed
%                 obs(:,3) - bearing theta of landmark measurement
%                 obs(:,4) - range d of landmark measurement
%     sigma_o - Covariance matrix corresponding to the odometry
%               measurements
%     sigma_l - Covariance matrix corresponding to the landmark
%               measurements
% Returns:
%     A       - MxN matrix
%     b       - Mx1 vector
%
function [As, b] = create_Ab_nonlinear(x, odom, obs, sigma_o, sigma_l)
%% Extract useful constants which you may wish to use
n_poses = size(odom, 1) + 1;                % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;                                  % pose dimension
l_dim = 2;                                  % landmark dimension
o_dim = size(odom, 2);                      % odometry dimension
m_dim = size(obs(1, 3:end), 2);             % landmark measurement dimension

% A matrix is MxN, b is Mx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;         % +1 for prior on the first pose

%% Initialize matrices
A = zeros(M, N);
b = zeros(M, 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A(1,1)=sqrt(1/sigma_o(1,1));
A(2,2)=sqrt(1/sigma_o(2,2));
b(1)=0;
b(2)=0;
% for all other measrements
col=1;
for i=3:2:o_dim*(n_odom+1)
    %disp(i);
    predicted_odom_meas=meas_odom(x(col),x(col+1),x(col+2),x(col+3));
    b(i)=sqrt(1/sigma_o(1,1))*(odom(floor(i/2),1)-predicted_odom_meas(1));
    b(i+1)=sqrt(1/sigma_o(2,2))*(odom(floor(i/2),2)-predicted_odom_meas(2));
    
    A(i,col)=-sqrt(1/sigma_o(1,1));
    A(i+1,col+1)=-sqrt(1/sigma_o(2,2));
    A(i,col+2)=sqrt(1/sigma_o(1,1));
    A(i+1,col+3)=sqrt(1/sigma_o(2,2));
    col=col+2;
end

x_offset= 2*n_poses;
y_offset= 2*(n_odom+1);
for obs_index = 0 : n_obs - 1
    pose_index = obs(obs_index + 1, 1);
    landmark_index = obs(obs_index + 1, 2);
    measurement = obs(obs_index + 1, 3 : 4);
    
    H=meas_landmark_jacobian(x(2*pose_index-1),x(2*pose_index),x(x_offset+2*landmark_index-1),x(x_offset+2*landmark_index));
    
    pose_est = x(pose_index * 2 - 1 : pose_index * 2);
    landmark_est = x(x_offset + landmark_index* 2 - 1 : x_offset + landmark_index* 2);
    predicted_landmark_meas = meas_landmark(pose_est(1), pose_est(2), landmark_est(1), landmark_est(2));
    
    b(y_offset + obs_index * 2 + 1)  = sqrt(1/sigma_l(1,1)) * (wrapToPi(measurement(1)-predicted_landmark_meas(1)));
    b(y_offset + obs_index * 2 + 2) = sqrt(1/sigma_l(2,2)) * (measurement(2)-predicted_landmark_meas(2));
    
    A(y_offset+2*obs_index+1, 2*pose_index-1)=H(1,1)*sqrt(1/sigma_l(1,1));
    A(y_offset+2*obs_index+1, 2*pose_index)=H(1,2)*sqrt(1/sigma_l(2,2));
    A(y_offset+2*obs_index+1,x_offset+2*landmark_index-1)=H(1,3)*sqrt(1/sigma_l(1,1));
    A(y_offset+2*obs_index+1,x_offset+2*landmark_index)=H(1,4)*sqrt(1/sigma_l(2,2));
    
    A(y_offset+2*obs_index+2, 2*pose_index-1)=H(2,1)*sqrt(1/sigma_l(1,1));
    A(y_offset+2*obs_index+2, 2*pose_index)=H(2,2)*sqrt(1/sigma_l(2,2));
    A(y_offset+2*obs_index+2,x_offset+2*landmark_index-1)=H(2,3)*sqrt(1/sigma_l(1,1));
    A(y_offset+2*obs_index+2,x_offset+2*landmark_index)=H(2,4)*sqrt(1/sigma_l(2,2));
    
    
end

%% Make A a sparse matrix 
As = sparse(A);
