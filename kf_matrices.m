% This file builds the matrices needed for the Kalman filter.
% I put them all in one place so I don't have to repeat the same
% matrix code in both kalman_filter.m and rts_smoother.m
%
% NOTE: These matrices follow standard kinematic physics and align with
% the loosely-coupled GNSS/IMU fusion method described in:
% "Application of Machine Learning to GNSS/IMU
% Integration for High Precision Positioning on Smartphones",
% ION GNSS+ 2022. (See Eq. 1-2 in the paper)
%
% How to use:
%   F = kf_matrices('F', dt)           -> state transition matrix
%   B = kf_matrices('B', dt)           -> control input matrix (IMU)
%   Q = kf_matrices('Q', dt, sigma)    -> process noise matrix
%   H = kf_matrices('H')               -> measurement matrix

function out = kf_matrices(which_matrix, dt, sigma_acc)

    if nargin < 3
        sigma_acc = 0; % default, only needed for Q
    end

    if strcmp(which_matrix, 'F')
        out = make_F(dt);

    elseif strcmp(which_matrix, 'B')
        out = make_B(dt);

    elseif strcmp(which_matrix, 'Q')
        out = make_Q(dt, sigma_acc);

    elseif strcmp(which_matrix, 'H')
        out = make_H();

    else
        error('Unknown matrix type. Use F, B, Q or H.');
    end

end


% F matrix, describes how the state moves forward in time
% State is [px, vx, py, vy]
% Position updates using velocity: px_new = px + vx*dt
% Velocity stays the same
function F = make_F(dt)
    F = [1, dt, 0,  0;
         0,  1, 0,  0;
         0,  0, 1, dt;
         0,  0, 0,  1];
end


% B matrix, maps IMU acceleration to position/velocity change
% Position changes by 0.5*dt^2*a, velocity changes by dt*a
function B = make_B(dt)
    B = [0.5*dt^2,        0;
         dt,              0;
         0,        0.5*dt^2;
         0,              dt];
end


% Q matrix, process noise covariance
% This controls how much we trust the kinematic model
% Higher sigma_acc = more uncertainty in the prediction
function Q = make_Q(dt, sigma_acc)
    q = sigma_acc^2;
    Q = q * [dt^4/4,  dt^3/2,       0,       0;
             dt^3/2,    dt^2,       0,       0;
                  0,       0, dt^4/4, dt^3/2;
                  0,       0, dt^3/2,   dt^2];
end


% H matrix, measurement matrix
% We only measure position from GPS, not velocity
% So H picks out px and py from the state vector
function H = make_H()
    H = [1, 0, 0, 0;
         0, 0, 1, 0];
end
