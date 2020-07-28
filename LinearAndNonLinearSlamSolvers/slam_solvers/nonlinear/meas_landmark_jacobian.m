% MEAS_LANDMARK_JACOBIAN
% 16-833 Spring 2020 - *Stub* Provided
% Compute the Jacobian of the measurement function
%
% Arguments: 
%     rx    - robot's x position
%     ry    - robot's y position
%     lx    - landmark's x position
%     ly    - landmark's y position
%
% Returns:
%     H     - Jacobian of the measurement fuction
%
function H = meas_landmark_jacobian(rx, ry, lx, ly)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta_x=lx-rx;
delta_y=ly-ry;
q=delta_x^2 + delta_y^2;
H=[delta_y,-delta_x,-delta_y,delta_x;-(q^0.5)*delta_x,-(q^0.5)*delta_y,(q^0.5)*delta_x,(q^0.5)*delta_y];
H=H/q;