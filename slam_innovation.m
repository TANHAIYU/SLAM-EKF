function [ state_est, cov_est] = slam_innovation(state, cov, observations, noise_observation)
% SLAM_INNOVATION The innovation of the EKF-SLAM
% Input
%  state: The current (predicted) state
%  cov: The current (predicted) covariance matrix
%  observations: The list of observations as [ID_1, x_1, y_1; ... ID_N,
%                x_N, y_N].
%  noise_observation: The variance of the measurement (a scalar).
%
% Output
%  state_est: The estimated state
%  cov_est: The estimated covariance

num_landmarks = (length(state)-3)/2;

%%% Initialization code

if (length(observations) == 0 )
    return;
end


H = zeros(2*length(observations), length(state));

observation_vec = zeros(length(observations)*2,1);
obs_model = zeros(length(observations)*2,1);

cosphi = cos(state(3));
sinphi = sin(state(3));
for i=1:length(observations);
    id = observations(i).id;
    if (id > num_landmarks)        
        %%% Implement your code here (1. Dynamic extension of state, covariance matrix, oberservation matrix H)
        nowstate = (id-num_landmarks)*2;
        state = [state;zeros(nowstate,1)];
        cov = [cov,zeros(2*num_landmarks+3,nowstate),
            zeros(nowstate,2*num_landmarks+3),1e10*eye(nowstate)];
        H = [H zeros(size(H,1),nowstate)];
        num_landmarks = (length(state)-3)/2;
    end
    
    %%% Implement your code here (2. Observation matrix H (Jacobian of observation model))
    d = state(2+2*id:3+2*id)-state(1:2);    
    H((i*2)-1, 1:3) = [-cosphi -sinphi -sinphi*d(1)+cosphi*d(2)];
    H((i*2), 1:3) = [sinphi -cosphi -cosphi*d(1)-sinphi*d(2)]; 
    H((i*2)-1, 2+id*2:3+id*2) = [cosphi sinphi];
    H((i*2), 2+id*2:3+id*2) = [-sinphi cosphi];
    
    observation_vec(i*2-1:i*2) = observations(i).pos;
    obs_model(i*2-1:i*2) = [cosphi*d(1)+sinphi*d(2); -sinphi*d(1)+cosphi*d(2) ];
end

%%% Implement your code here (3. Innovation of EKF)
K = cov*H'*(H*cov*H'+noise_observation*eye(2*length(observations)))^-1;
state_est = state+K*(observation_vec-obs_model);
cov_est = (eye(3+2*num_landmarks)-K*H)*cov;
end

