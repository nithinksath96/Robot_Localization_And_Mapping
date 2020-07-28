%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  16833 Robot Localization and Mapping  % 
%  Assignment #2                         %
%  EKF-SLAM                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
clc;

%==== TEST: Setup uncertainity parameters (try different values!) ===
sig_x = 0.25;
sig_y = 0.1;
sig_alpha = 0.1;
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

% Write your code here...
k=6; %No of landmarks
landmark = zeros(k*2,1);
for i=1:2:2*k-1
  
    landmark(i:i+1,1)= pose(1:2,1) + [measure(i+1)*cos(pose(2)+measure(i));measure(i+1)*sin(pose(2)+measure(i))];
end
a=0.4;
landmark_cov = diag([a,a,a,a,a,a,a,a,a,a,a,a]);
% num_landmark = length(measure) / 2.0;
% landmark = zeros(num_landmark * 2.0, 1);
% landmark_cov = zeros(2 * num_landmark, 2 * num_landmark);
% calculating initial landmark position and its corresponding covariance
% for index_landmark = 0 : num_landmark - 1
%     beta = measure(index_landmark * 2 + 1);
%     distance = measure(index_landmark * 2 + 2);
%     % position estimation
%     landmark(index_landmark * 2 + 1) = pose(1) + distance * cos(pose(3) + beta);
%     landmark(index_landmark * 2 + 2) = pose(2) + distance * sin(pose(3) + beta);
%     % covariance estimation
%     initH = [1 0 -distance * sin(pose(3) + beta); 0 1 distance * cos(pose(3) + beta)];
%     initQ = [-distance * sin(pose(3) + beta) cos(pose(3) + beta); distance * cos(pose(3) + beta) sin(pose(3) + beta)];
%     landmark_cov(index_landmark * 2 + 1 : index_landmark * 2 + 2, index_landmark * 2 + 1 : index_landmark * 2 + 2) = initH * pose_cov * initH' + initQ * measure_cov * initQ';
% end

%==== Setup state vector x with pose and landmark vector ====
x = [pose ; landmark];

%==== Setup covariance matrix P with pose and landmark covariances ====
P = [pose_cov zeros(3, 2*k) ; zeros(2*k, 3) landmark_cov];

%==== Plot initial state and conariance ====
last_x = x;
%drawTrajAndMap(x, last_x, P, 0);

%==== Read control data ====
tline = fgets(fid);


x_pre = zeros(2*k+3,1);
%P_pre = zeros(2*k+3,2*k+3);
P_pre = [zeros(3) zeros(3, 2*k) ; zeros(2*k, 3) landmark_cov];
while ischar(tline)
    arr = str2num(tline);
    d = arr(1);
    alpha = arr(2);
    
    %==== TODO: Predict Step ====
    %==== (Notice: predict state x_pre[] and covariance P_pre[] using input control data and control_cov[]) ====
    
    % Write your code here...
    x_pre(1:3,1)=x(1:3,1) + [d*cos(x(3,1));d*sin(x(3,1));x(3,1)+alpha];
    G=[1,0,-1*d*sin(x(3));0,1,d*cos(x(3));0,0,1];
    e_x=normrnd(0,sig_x);
    e_y=normrnd(0,sig_y);
    %F=[cos(x(3)),-sin(x(3)),-e_x*sin(x(3))-e_y*cos(x(3)); sin(x(3)),cos(x(3)),e_x*cos(x(3))-e_y*sin(x(3));0,0,1];
    F=[cos(x(3)),-sin(x(3)),0; sin(x(3)),cos(x(3)),0;0,0,1];
    
    P_pre(1:3,1:3) = G*pose_cov*G' + F*control_cov*F';
    
    
    
    %==== Draw predicted state x_pre[] and covariance P_pre[] ====
    drawTrajPre(x_pre, P_pre);
    
    %==== Read measurement data ====
    tline = fgets(fid);
    arr = str2num(tline);
    measure = arr';
    
    %==== TODO: Update Step ====
    %==== (Notice: update state x[] and covariance P[] using input measurement data and measure_cov[]) ====
    
    % Write your code here...
    for i=1:2:2*k-1
        
        delta_x = x_pre(3+i)-x_pre(1);
        delta_y = x_pre(3+i+1)-x_pre(2);
        delta = [delta_x;delta_y];
        q=(delta'*delta)^0.5;
        beta_pred = wrapToPi(atan2(delta_y, delta_x) - x_pre(3));
        z_pred = [q;beta_pred];
        
        F = zeros(5,3+2*k);
        F(1:3,1:3)=eye(3);
        F(4:5, 3+i:3+i+1)=eye(2);
        
        
        H=[-q*delta_x,-q*delta_y,0,q*delta_x,q*delta_y; delta_y,-delta_x,-q,-delta_y,delta_x];
        H=H/(q^2);
        H=H*F;
        
        temp = inv(H*P_pre*H' + measure_cov);
        K=P_pre*H'*temp;
        
        z_actual = [measure(i+1);measure(i)];
        
        x_pre = x_pre + K*(z_actual-z_pred);
        P_pre= (eye(3+2*k)-K*H)*P_pre;
        
        
%         robot_cov=P_pre(1:3,1:3);
%         landmark_j_cov=P_pre(3+i:3+i+1,3+i:3+i+1);
%         P_correction=eye(5);
%         P_correction(1:3,1:3)=P_correction(1:3,1:3)*robot_cov;  
%         P_correction(4:5,4:5)=P_correction(4:5,4:5)*landmark_j_cov;
%         
%         temp = inv(H*P_correction*H' + measure_cov);
%         K=P_correction*H'*temp;
%         
%         z_actual = [measure(i+1);measure(i)];
%         
%         x_pre = x_pre + K*(z_actual-z_pred);
%         P_pre=
        
        
        
        
        
        
        
        
        
    
    
    end
    x = x_pre;
    P = P_pre;
    
    
    
    %==== Plot ====   
    drawTrajAndMap(x, last_x, P, t);
    last_x = x;
    
    %==== Iteration & read next control data ===
    t = t + 1;
    tline = fgets(fid);
end

%==== EVAL: Plot ground truth landmarks ====

% Write your code here...
    

%==== Close data file ====
fclose(fid);
