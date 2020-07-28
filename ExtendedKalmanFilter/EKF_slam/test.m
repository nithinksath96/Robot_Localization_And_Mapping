
clear;
close all;

%==== TEST: Setup uncertianty parameters (try different values!) ===
%  Motion Uncertainty
sig_x = 0.25;
sig_y = 0.1;
sig_alpha = 0.1;
% Sensor uncertainty
sig_beta = 0.01;
sig_r = 0.08;

%==== Generate sigma^2 from sigma ===
sig_x2 = sig_x^2;
sig_y2 = sig_y^2;
sig_alpha2 = sig_alpha^2;
sig_beta2 = sig_beta^2;
sig_r2 = sig_r^2;

%==== Open data file ====
fid = fopen('../data/data.txt');

%==== Read first measurement data ====
tline = fgets(fid);
arr = str2num(tline);
measure = arr';
t = 1;
 
%==== Setup control and measurement covariances ===
control_cov = diag([sig_x2, sig_y2, sig_alpha2]);
measure_cov = diag([sig_beta2, sig_r2]);

%==== Setup initial pose vector and pose uncertainty ====
pose = [0 ; 0 ; 0];
pose_cov = diag([0.02^2, 0.02^2, 0.1^2]);

%==== TODO: Setup initial landmark vector landmark[] and covariance matrix landmark_cov[] ====
%==== (Hint: use initial pose with uncertainty and first measurement) ====
num_landmark = length(measure) / 2.0;
landmark = zeros(num_landmark * 2.0, 1);
landmark_cov = zeros(2 * num_landmark, 2 * num_landmark);
% calculating initial landmark position and its corresponding covariance
for index_landmark = 0 : num_landmark - 1
    beta = measure(index_landmark * 2 + 1);
    distance = measure(index_landmark * 2 + 2);
    % position estimation
    landmark(index_landmark * 2 + 1) = pose(1) + distance * cos(pose(3) + beta);
    landmark(index_landmark * 2 + 2) = pose(2) + distance * sin(pose(3) + beta);
    % covariance estimation
    initH = [1 0 -distance * sin(pose(3) + beta); 0 1 distance * cos(pose(3) + beta)];
    initQ = [-distance * sin(pose(3) + beta) cos(pose(3) + beta); distance * cos(pose(3) + beta) sin(pose(3) + beta)];
    landmark_cov(index_landmark * 2 + 1 : index_landmark * 2 + 2, index_landmark * 2 + 1 : index_landmark * 2 + 2) = initH * pose_cov * initH' + initQ * measure_cov * initQ';
end

%==== Setup state vector x with pose and landmark vector ====
x = [pose ; landmark];

%==== Setup covariance matrix P with pose and landmark covariances ====
P = [pose_cov zeros(3, 2*num_landmark) ; zeros(2*num_landmark, 3) landmark_cov];

%==== Plot initial state and conariance ====
last_x = x;
drawTrajAndMap(x, last_x, P, 0);

%==== Read control data ====
tline = fgets(fid);
while ischar(tline)
    arr = str2num(tline);
    d = arr(1);
    alpha = arr(2);
    
    %==== TODO: Predict Step ====
    %==== (Notice: predict state x_pre[] and covariance P_pre[] using input control data and control_cov[]) ====
    
    F = [eye(3) zeros(3, 2 * num_landmark)];
    G = eye(3 + 2 * num_landmark) + F' * [0 0 -(d * sin(x(3))); 0 0 (d * cos(x(3))); 0 0 0] * F;
    V = [cos(x(3)) -sin(x(3)) 0; sin(x(3)) cos(x(3)) 0; 0 0 1];
    % convert the control covariance into world frame
    R = V * control_cov * V';
    % predict the mean @ t + 1 (landmark position is not updated by nature)
    x_pre = x + F' * [d * cos(x(3)); d * sin(x(3)); alpha];
    % predict the covariance @ t + 1 (P is the covariance of previous timestamp and R is the control covariance expressed in world frame)
    P_pre = G * P * G' + F' * R * F;
    %==== Draw predicted state x_pre[] and covariance P_pre[] ====
    drawTrajPre(x_pre, P_pre);
    %==== Read measurement data ====
    tline = fgets(fid);
    arr = str2num(tline);
    measure = arr';
    
    %==== TODO: Update Step ====
    %==== (Notice: update state x[] and covariance P[] using input measurement data and measure_cov[]) ====
    
    % since we have num_landmark measurements
    for meas_index = 0 : num_landmark - 1
        meas_saw = measure(meas_index * 2 + 1 : meas_index * 2 + 2);
        % assuming known correspondance here
        landmark_saw = x_pre(3 + meas_index * 2 + 1 : 3 + meas_index * 2 + 2);
        % predict the measurement using predicted state
        delta_x = landmark_saw(1) - x_pre(1);
        delta_y = landmark_saw(2) - x_pre(2);
        q = (delta_x ^ 2 + delta_y ^ 2) ^ 0.5;
        meas_pre = [wrapToPi(atan2(delta_y, delta_x) - x_pre(3)); q];

        % predict the covariance of the measurement
        Fm = zeros(5, 3 + 2 * num_landmark);
        Fm(1 : 3, 1 : 3) = eye(3);
        Fm(4 : 5, 3 + meas_index * 2 + 1 : 3 + meas_index * 2 + 2) = eye(2);
        
        H = [delta_y / (q ^ 2) -delta_x / (q ^ 2) -1 -delta_y / (q ^ 2) delta_x / (q ^ 2); 
            -delta_x / q -delta_y / q 0 delta_x / q delta_y / q] * Fm;
        S = H * P_pre * H' + measure_cov;
        
        % since we have both the predicted measurement and the actual
        % measurement, we can do the update accordingly
        K = P_pre * H' * inv(S);
        x_pre = x_pre + K * (meas_saw - meas_pre);
        P_pre = (eye(3 + 2 * num_landmark) - K * H) * P_pre;
    end
    
    % update @ this timestamp is done, update the state
    x = x_pre;
    P = P_pre;
    
    %==== Plot ====   
    drawTrajAndMap(x, last_x, P, t);
    last_x = x;
    
    %==== Iteration & read next control data ===
    t = t + 1;
    tline = fgets(fid);
end

% %==== EVAL: Plot ground truth landmarks ====
% 
% % Plot the ground truth
% ground_truth = [3 6 3 12 7 8 7 14 11 6 11 12];
% for gt_index = 0 : num_landmark - 1
%     landmark_gt = ground_truth(gt_index * 2 + 1 : gt_index * 2 + 2);
%     plot(landmark_gt(1), landmark_gt(2), '*r');
% end
% 
% % Calculate the Euclidean and Mahalanobis distance of each landmark
% landmark_estimated = x(4 : end);
% landmark_cov = P(4 : end, 4 : end);
% for index = 0 : num_landmark - 1
%     gt = ground_truth(index * 2 + 1 : index * 2 + 2)';
%     est = landmark_estimated(index * 2 + 1 : index * 2 + 2);
%     
%     % euclidean distance
%     delta_x = gt(1) - est(1);
%     delta_y = gt(2) - est(2);
%     euclidean_dist = (delta_x ^ 2 + delta_y ^ 2) ^ 0.5;
%     
%     % mahalanobis distance
%     est_cov = landmark_cov(index * 2 + 1 : index * 2 + 2, index * 2 + 1 : index * 2 + 2);
%     mahalanobis_dist = ((gt' - est') * est_cov * (gt - est)) ^ 0.5;
%     
%     % disp
%     disp(['[LANDMARK_' num2str(index + 1) ']:']);
%     disp(['Euclidean: ' num2str(euclidean_dist) '; Mahalanobis: ' num2str(mahalanobis_dist)]);
% end


