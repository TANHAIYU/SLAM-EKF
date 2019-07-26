function [ state_pred, cov_pred] = slam_prediction(state, cov,odom, noise_odom)
%SLAM_PREDICTION The prediction step of the extended kalman filter
% Input:
%  state: the current state 
%  cov: the current covariance matrix
%  odom: The delta motion of the agent
%  noise_odom: The covariance matrix for the delta motion
%
% Output: 
%  state_pred: The state after the prediction
%  cov_pred: The covariance after the prediction
cosphi = cos(state(3));
sinphi = sin(state(3));
state_pred = state;
cov_pred = cov;
A = [cosphi -sinphi 0;
    sinphi cosphi 0;
    0 0 1];
state_pred(1:3) = state(1:3)+ A*odom;
F = [1,0,-sinphi*odom(1)-cosphi*odom(2); 
    0,1,cosphi*odom(1)-sinphi*odom(2);
    0,0,1];
cov_pred(1:3,1:3) = F*cov(1:3,1:3)*F'+noise_odom;

%%% Implement your code here
end

