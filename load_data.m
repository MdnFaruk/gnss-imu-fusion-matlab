% Reads the CSV file and organises in a struct.

function data = load_data(filepath, dt, gravity)

    % read the CSV file
    raw = readtable(filepath);

    n = height(raw);    % number of samples
    data.N = n;         % store the number of rows

    % create a time axis starting at 0
    data.time = (0 : n-1)' * dt;

    % ground truth positions from GPS
    data.true_x = raw.true_x;
    data.true_y = raw.true_y;

    % raw GPS positions from the smartphone, noisy data
    data.gnss_x = raw.gnss_x;
    data.gnss_y = raw.gnss_y;

    % ax is already fine, no gravity on this axis
    % ay needs gravity removed because it reads ~9.81 m/s2 at rest
    data.ax_corr = raw.ax;
    data.ay_corr = raw.ay - gravity;

    % calculate some basic stats about the dataset
    dx = diff(data.true_x);
    dy = diff(data.true_y);
    total_dist = sum(sqrt(dx.^2 + dy.^2));

    data.path_len = total_dist;
    data.avg_speed = total_dist / (n * dt);

    % calculate the baseline GPS error
    gps_err = sqrt((data.gnss_x - data.true_x).^2 + ...
                   (data.gnss_y - data.true_y).^2);

    data.gnss_err_mean = mean(gps_err);
    data.gnss_err_rms = sqrt(mean(gps_err.^2));
    data.gnss_err_std = std(gps_err);

end
