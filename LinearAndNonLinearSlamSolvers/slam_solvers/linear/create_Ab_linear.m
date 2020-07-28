% CREATE_AB_LINEAR
% 16-833 Spring 2020 - *Stub* Provided
% Computes the A and b matrices for the 2D linear SLAM problem
%
% Arguments: 
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
%                 obs(:,3) - x-value of landmark measurement
%                 obs(:,4) - y-value of landmark measurement
%     sigma_o - Covariance matrix corresponding to the odometry
%               measurements
%     sigma_l - Covariance matrix corresponding to the landmark
%               measurements
% Returns:
%     A       - MxN matrix
%     b       - Mx1 vector
%
function [As, b] = create_Ab_linear(odom, obs, sigma_o, sigma_l)


% Useful Constants
n_poses = size(odom, 1) + 1; % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;
l_dim = 2;
o_dim = size(odom, 2);
m_dim = size(obs(1, 3:end), 2);

% A matrix is MxN, b is Mx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;     % +1 for prior on the first pose

% Initialize matrices
A = zeros(M, N);
b = zeros(M, 1);

% Add odometry and landmark measurements to A, b - including prior on first
% pose

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% for prior
A(1,1)=sqrt(1/sigma_o(1,1));
A(2,2)=sqrt(1/sigma_o(2,2));
b(1)=0;
b(2)=0;
% for all other measrements
col=1;
for i=3:2:o_dim*(n_odom+1)
    %disp(i);
    b(i)=sqrt(1/sigma_o(1,1))*odom(floor(i/2),1);
    b(i+1)=sqrt(1/sigma_o(2,2))*odom(floor(i/2),2);
    
    A(i,col)=-sqrt(1/sigma_o(1,1));
    A(i+1,col+1)=-sqrt(1/sigma_o(2,2));
    A(i,col+2)=sqrt(1/sigma_o(1,1));
    A(i+1,col+3)=sqrt(1/sigma_o(2,2));
    col=col+2;
end

x_offset= 2*n_poses;
y_offset= 2*(n_odom+1);


for obs_index = 0 : n_obs - 1
    % retrieve infomation
    pose_index = obs(obs_index + 1, 1);
    landmark_index = obs(obs_index + 1, 2);
    measurement = obs(obs_index + 1, 3 : 4);
   
    b(y_offset + obs_index * 2 + 1)  = sqrt(1/sigma_l(1,1)) * measurement(1);
    b(y_offset + obs_index * 2 + 2) = sqrt(1/sigma_l(2,2)) * measurement(2);
    
    A(y_offset+2*obs_index+1, 2*pose_index-1)=-(sqrt(1/sigma_l(1,1)));
    A(y_offset+2*obs_index+2, 2*pose_index)=-(sqrt(1/sigma_l(2,2)));
    A(y_offset+2*obs_index+1,x_offset+2*landmark_index-1)=(sqrt(1/sigma_l(1,1)));
    A(y_offset+2*obs_index+2,x_offset+2*landmark_index)=(sqrt(1/sigma_l(2,2)));

end

%% Make A a sparse matrix 
As = sparse(A);