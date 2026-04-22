% Forward KF: predicts with IMU, updates with GNSS. Skips update during GNSS dropouts when gps_mask=false.

function [xf, Pf, xp, Pp] = kalman_filter(data, cfg, gps_mask)
    % data, loaded data struct from load_data.m
    % cfg, settings struct from main.m
    % gps_mask, array of true/false, true means GPS is working at that step

    n = data.N;
    dt = cfg.dt;
    sig_gnss = cfg.sigma_gnss;
    sig_acc = cfg.sigma_acc;

    gx = data.gnss_x;
    gy = data.gnss_y;

    % get the matrices from kf_matrices.m
    F = kf_matrices('F', dt);
    B = kf_matrices('B', dt);
    H = kf_matrices('H');
    Q = kf_matrices('Q', dt, sig_acc);
    R = eye(2) * sig_gnss^2;   % GPS measurement noise

    xf = zeros(n, 4);   % filtered state estimates (N x 4), columns = [px, vx, py, vy]
    xp = zeros(n, 4);   % predicted state estimates before GPS correction (N x 4)
    Pf = zeros(4, 4, n);    % filtered covariance matrices (4 x 4 x N)
    Pp = zeros(4, 4, n);    % predicted covariance matrices (4 x 4 x N)

    % initialise the state using the first GPS reading
    vx_init = (gx(min(2, n)) - gx(1)) / dt;
    vy_init = (gy(min(2, n)) - gy(1)) / dt;

    x = [gx(1); vx_init; gy(1); vy_init];
    P = diag([sig_gnss^2, 100^2, sig_gnss^2, 100^2]);
    % the 100^2 for velocity means we are very uncertain about initial velocity

    % main filter loop
    for k = 1:n

        % predict step
        % use IMU reading as control input
        u = [data.ax_corr(k); data.ay_corr(k)];
        x_pred = F * x + B * u;
        P_pred = F * P * F' + Q;

        % save predictions before the update
        xp(k,:) = x_pred';
        Pp(:,:,k) = P_pred;

        % update step, when GNSS is available
        if gps_mask(k)
            z = [gx(k); gy(k)];         % GPS measurement
            innov = z - H * x_pred;     % difference between GPS and prediction
            S = H * P_pred * H' + R;    % innovation covariance
            K = P_pred * H' / S;        % Kalman gain
            x = x_pred + K * innov;     % updated state
            P = (eye(4) - K * H) * P_pred; % updated covariance
            P = 0.5 * (P + P');         % keep it symmetric (numerical fix)
        else
            % no GPS, just use the prediction, dead reckoning with IMU
            x = x_pred;
            P = P_pred;
        end

        xf(k,:) = x';
        Pf(:,:,k) = P;
    end

end
