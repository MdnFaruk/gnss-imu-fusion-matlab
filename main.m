% This is the main script to run the GNSS/IMU fusion project.

clc;
clear;
close all;

% path to the data file
cfg.data_path = 'df_total.csv';

% noise values for the filter
cfg.sigma_gnss = 10.0;  % sigma_gnss = how noisy the GPS is
cfg.sigma_acc = 5.0;   % sigma_acc  = how much we trust the IMU acceleration reading

% time between each sample (1 second)
cfg.dt = 1.0;

% gravity constant
cfg.gravity = 9.81;

% Picked steps 400 to 430 to simulate a GPS signal loss like a tunnel
cfg.outage_start = 400;
cfg.outage_len = 30;

% colours for the plots green, red, blue, purple, orange
cfg.colors.truth = [0.153 0.682 0.376];
cfg.colors.gnss = [0.906 0.298 0.235];
cfg.colors.kf = [0.161 0.502 0.725];
cfg.colors.rts = [0.557 0.267 0.678];
cfg.colors.dr = [0.902 0.494 0.133];

% Load the data from the CSV file
fprintf('Loading data from file...\n');
data = load_data(cfg.data_path, cfg.dt, cfg.gravity);

fprintf('  Number of samples : %d\n', data.N);
fprintf('  Total distance    : %.0f m (about %.1f km)\n', data.path_len, data.path_len/1000);
fprintf('  Average speed     : %.1f m/s (%.1f km/h)\n', data.avg_speed, data.avg_speed*3.6);
fprintf('  Raw GNSS error    : mean = %.2f m,  RMS = %.2f m\n', ...
        data.gnss_err_mean, data.gnss_err_rms);

% all GPS readings are available here (no outage mask)
gps_available = true(data.N, 1);

% Run the Kalman filter (forward pass)
fprintf('Running Kalman Filter...\n');
[xf, Pf, xp, Pp] = kalman_filter(data, cfg, gps_available);
fprintf('  Kalman filter done.\n\n');

% Run the RTS smoother (backward pass)
fprintf('Running RTS Smoother...\n');
[xs, ~] = rts_smoother(xf, Pf, xp, Pp, cfg);
fprintf('  RTS smoother done.\n\n');

% Compute errors and print results
fprintf('Computing accuracy results:\n');
metrics = evaluate(data, xf, xs, cfg);

% Make all the plots
fprintf('\nGenerating plots...\n');
visualize(data, xf, xs, metrics, cfg);

% This shows how the fused RTS positions could be used for simple
% obstacle avoidance and path planning on the highway route.
% Note: obstacles are synthetic (made up for the demo).

fprintf('\nRunning navigation demo...\n');
navigation_demo(data, xs, cfg);
 
fprintf('\nAll done!\n');