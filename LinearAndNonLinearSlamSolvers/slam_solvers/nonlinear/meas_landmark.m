% MEAS_LANDMARK
% 16-833 Spring 2020 - *Stub* Provided
% Simple function to predict a nonlinear landmark measurement from a pose
%
% Arguments: 
%     rx    - robot's x position
%     ry    - robot's y position
%     lx    - landmark's x position
%     ly    - landmark's y position
%
% Returns:
%     h     - odometry measurement prediction 
%
function h = meas_landmark(rx, ry, lx, ly)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h=[atan2(ly-ry, lx-rx);((lx-rx)^2 +(ly-ry)^2)^0.5];
