% Makes all the plots for the project.

function visualize(data, xf, xs, metrics, cfg)

    cT = cfg.colors.truth;   % green, ground truth
    cG = cfg.colors.gnss;    % red, GPS only
    cK = cfg.colors.kf;      % blue, Kalman filter
    cR = cfg.colors.rts;     % purple, RTS smoother
    cD = cfg.colors.dr;      % orange, IMU / dead reckoning

    plot_trajectories(data, xf, xs, cT, cG, cK, cR);
    plot_error_time(data, xf, xs, cG, cK, cR);
    plot_outage(data, metrics, cfg, cT, cG, cK, cR);
    plot_summary(data, xf, xs, metrics, cT, cG, cK, cR);
    plot_sensitivity(data, cfg, cG, cK);

end

% Figure 1, Trajectory Comparison
function plot_trajectories(data, xf, xs, cT, cG, cK, cR)

    tx = data.true_x;   ty = data.true_y;
    gx = data.gnss_x;   gy = data.gnss_y;
    kx = xf(:,1);       ky = xf(:,3);
    rx = xs(:,1);       ry = xs(:,3);
    N  = data.N;

    figure('Name', 'Figure 1, Trajectory', 'NumberTitle', 'off', ...
           'Position', [50 50 1400 580]);
    sgtitle({'GNSS/IMU Sensor Fusion - Trajectory Comparison', ...
             'Highway Driving | Kalman Filter + RTS Smoother'}, ...
            'FontSize', 13, 'FontWeight', 'bold');

    % left subplot, full path
    subplot(1,2,1);
    hold on; grid on; axis equal;
    plot(tx, ty, '-',  'Color', cT, 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
    plot(gx, gy, '-',  'Color', cG, 'LineWidth', 1.0, 'DisplayName', 'GPS Only');
    plot(kx, ky, '-',  'Color', cK, 'LineWidth', 1.4, 'DisplayName', 'Kalman Filter');
    plot(rx, ry, '--', 'Color', cR, 'LineWidth', 2.0, 'DisplayName', 'RTS Smoother');
    xlabel('X position [m]', 'FontSize', 11);
    ylabel('Y position [m]', 'FontSize', 11);
    title('Full Trajectory (1200 steps, ~43 km)', 'FontSize', 11);
    legend('Location', 'best', 'FontSize', 9);

    % right subplot, zoomed in on first 250 steps
    sl = 1:250;
    subplot(1,2,2);
    hold on; grid on; axis equal;
    plot(tx(sl), ty(sl), '-',  'Color', cT, 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
    plot(gx(sl), gy(sl), '-',  'Color', cG, 'LineWidth', 1.0, 'DisplayName', 'GPS Only');
    plot(kx(sl), ky(sl), '-',  'Color', cK, 'LineWidth', 1.4, 'DisplayName', 'Kalman Filter');
    plot(rx(sl), ry(sl), '--', 'Color', cR, 'LineWidth', 2.0, 'DisplayName', 'RTS Smoother');
    xlabel('X position [m]', 'FontSize', 11);
    ylabel('Y position [m]', 'FontSize', 11);
    title('Zoomed In, First 250 Steps', 'FontSize', 11);
    legend('Location', 'best', 'FontSize', 9);

    fprintf('Figure 1 done\n');
end

% Figure 2, Error Over Time
function plot_error_time(data, xf, xs, cG, cK, cR)

    t = data.time;
    tx = data.true_x;   ty = data.true_y;

    eg = sqrt((data.gnss_x - tx).^2 + (data.gnss_y - ty).^2);
    ek = sqrt((xf(:,1)     - tx).^2 + (xf(:,3)     - ty).^2);
    er = sqrt((xs(:,1)     - tx).^2 + (xs(:,3)     - ty).^2);

    all_errors = {eg, ek, er};
    all_colors = {cG, cK, cR};
    all_labels = {'GPS Only', 'Kalman Filter', 'RTS Smoother'};

    figure('Name', 'Figure 2, Error Over Time', 'NumberTitle', 'off', ...
           'Position', [50 50 1200 750]);
    sgtitle('Positioning Error Over Time', 'FontSize', 13, 'FontWeight', 'bold');

    for i = 1:3
        e = all_errors{i};
        col = all_colors{i};

        subplot(3, 1, i);
        hold on; grid on;

        % shaded area under the error curve
        fill([t; flipud(t)], [e; zeros(size(e))], col, ...
             'FaceAlpha', 0.15, 'EdgeColor', 'none');
        plot(t, e, '-', 'Color', col, 'LineWidth', 0.8);

        % dashed lines for mean and 95th percentile
        yline(mean(e), '--', sprintf('Mean = %.2f m', mean(e)), ...
              'Color', col, 'LineWidth', 2.0, 'LabelHorizontalAlignment', 'right');
        yline(prctile(e, 95), ':', sprintf('95th = %.2f m', prctile(e,95)), ...
              'Color', [0.4 0.4 0.4], 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'right');

        ylabel('Error [m]', 'FontSize', 10);
        title(all_labels{i}, 'FontSize', 11);
        ylim([0, max(e)*1.05]);
    end
    xlabel('Time [s]', 'FontSize', 11);
    fprintf('Figure 2 done\n');
end

% Figure 3, GPS Outage Demo
function plot_outage(data, metrics, cfg, cT, cG, cK, cR)

    tx = data.true_x;   ty = data.true_y;
    gx = data.gnss_x;   gy = data.gnss_y;
    t  = data.time;
    N  = data.N;
    os = cfg.outage_start;
    ol = cfg.outage_len;
    oe = os + ol;

    xf_out    = metrics.xf_out;
    xs_out    = metrics.xs_out;
    gx_frozen = metrics.gx_frozen;
    gy_frozen = metrics.gy_frozen;
    eg_mean   = metrics.outage_gnss_err;
    er_mean   = metrics.outage_rts_err;
    imp       = (eg_mean - er_mean) / eg_mean * 100;

    err_frozen  = sqrt((gx_frozen - tx).^2 + (gy_frozen - ty).^2);
    err_kf_out  = sqrt((xf_out(:,1) - tx).^2 + (xf_out(:,3) - ty).^2);
    err_rts_out = sqrt((xs_out(:,1) - tx).^2 + (xs_out(:,3) - ty).^2);

    % window around the outage (with some context before/after)
    pad = 20;
    sl  = max(1, os-pad) : min(N, oe+pad);

    figure('Name', 'Figure 3, GPS Outage', 'NumberTitle', 'off', ...
           'Position', [50 50 1350 600]);
    sgtitle(sprintf('GPS Outage Demo, Signal lost from step %d to %d (%d seconds)', ...
                    os, oe-1, ol), 'FontSize', 13, 'FontWeight', 'bold');

    % left plot, trajectory around the outage
    subplot(1,2,1);
    hold on; grid on; axis equal;
    plot(tx(sl), ty(sl), '-',  'Color', cT, 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
    plot(gx(sl), gy(sl), '-',  'Color', cG, 'LineWidth', 1.0, 'DisplayName', 'GPS (frozen)');
    plot(xf_out(sl,1), xf_out(sl,3), '-',  'Color', cK, 'LineWidth', 1.5, 'DisplayName', 'KF with IMU');
    plot(xs_out(sl,1), xs_out(sl,3), '--', 'Color', cR, 'LineWidth', 2.0, 'DisplayName', 'RTS Smoother');
    yl = ylim;
    patch([tx(os) tx(min(oe-1,N)) tx(min(oe-1,N)) tx(os)], ...
          [yl(1) yl(1) yl(2) yl(2)], 'r', ...
          'FaceAlpha', 0.07, 'EdgeColor', 'none', 'DisplayName', 'Outage region');
    xlabel('X [m]');   ylabel('Y [m]');
    title('Path During Outage', 'FontSize', 11);
    legend('Location', 'best', 'FontSize', 8);

    % right plot - error over time during outage
    subplot(1,2,2);
    hold on; grid on;
    t_sl = t(sl);

    fill([t_sl; flipud(t_sl)], [err_frozen(sl); zeros(numel(sl),1)], cG, ...
         'FaceAlpha', 0.12, 'EdgeColor', 'none');
    fill([t_sl; flipud(t_sl)], [err_rts_out(sl); zeros(numel(sl),1)], cR, ...
         'FaceAlpha', 0.12, 'EdgeColor', 'none');

    plot(t_sl, err_frozen(sl),  '-',  'Color', cG, 'LineWidth', 1.5, ...
         'DisplayName', sprintf('GPS frozen (mean=%.0fm)', eg_mean));
    plot(t_sl, err_kf_out(sl),  '-',  'Color', cK, 'LineWidth', 1.3, ...
         'DisplayName', sprintf('KF+IMU (mean=%.0fm)', mean(err_kf_out(os:oe-1))));
    plot(t_sl, err_rts_out(sl), '--', 'Color', cR, 'LineWidth', 2.0, ...
         'DisplayName', sprintf('RTS (mean=%.0fm)', er_mean));

    yl2 = ylim;
    patch([t(os) t(min(oe-1,N)) t(min(oe-1,N)) t(os)], ...
          [yl2(1) yl2(1) yl2(2) yl2(2)], 'r', ...
          'FaceAlpha', 0.07, 'EdgeColor', 'none', ...
          'DisplayName', sprintf('Outage (%ds)', ol));

    t_mid = t(os + floor(ol/2));
    text(t_mid, (eg_mean + er_mean)/2, ...
         sprintf('%.0f%% less error\nusing IMU', imp), ...
         'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', ...
         'Color', cR, 'BackgroundColor', 'w', 'EdgeColor', cR);

    xlabel('Time [s]');   ylabel('Position Error [m]');
    title('Error During GPS Outage', 'FontSize', 11);
    legend('Location', 'best', 'FontSize', 9);

    fprintf('Figure 3 done\n');
end


% Figure 4, Summary Bar Charts
function plot_summary(data, xf, xs, metrics, cT, cG, cK, cR)

    means = [metrics.gnss.mean, metrics.kf.mean, metrics.rts.mean];
    rmses = [metrics.gnss.rms,  metrics.kf.rms,  metrics.rts.rms];
    p95s  = [metrics.gnss.p95,  metrics.kf.p95,  metrics.rts.p95];

    jitters = [metrics.jitter_truth, metrics.jitter_gnss, ...
               metrics.jitter_kf,   metrics.jitter_rts];

    figure('Name', 'Figure 4, Summary', 'NumberTitle', 'off', ...
           'Position', [50 50 1200 520]);
    sgtitle('Summary - Accuracy and Path Smoothness', ...
            'FontSize', 13, 'FontWeight', 'bold');

    % left - accuracy bars
    subplot(1,2,1);
    hold on; grid on;
    xpos  = 1:3;
    w     = 0.26;
    cdata = [cG; cK; cR];

    b1 = bar(xpos - w, means, w);
    b1.FaceColor = 'flat';   b1.CData = cdata;   b1.FaceAlpha = 0.85;

    b2 = bar(xpos,     rmses, w);
    b2.FaceColor = 'flat';   b2.CData = cdata;   b2.FaceAlpha = 0.85;

    b3 = bar(xpos + w, p95s,  w);
    b3.FaceColor = 'flat';   b3.CData = cdata;   b3.FaceAlpha = 0.50;

    for i = 1:3
        text(xpos(i)-w, means(i)+0.5, sprintf('%.1f', means(i)), ...
             'HorizontalAlignment', 'center', 'FontSize', 7.5);
        text(xpos(i),   rmses(i)+0.5, sprintf('%.1f', rmses(i)), ...
             'HorizontalAlignment', 'center', 'FontSize', 7.5);
        text(xpos(i)+w, p95s(i)+0.5,  sprintf('%.1f', p95s(i)), ...
             'HorizontalAlignment', 'center', 'FontSize', 7.5);
    end

    set(gca, 'XTick', xpos, 'XTickLabel', {'GPS Only', 'Kalman Filter', 'RTS Smoother'});
    ylabel('Position Error [m]', 'FontSize', 11);
    title('Accuracy Comparison (lower is better)', 'FontSize', 11);
    legend({'Mean', 'RMS', '95th pct'}, 'FontSize', 9, 'Location', 'northwest');

    % right - smoothness bars
    subplot(1,2,2);
    hold on; grid on;
    b = bar(1:4, jitters, 0.5);
    b.FaceColor = 'flat';
    b.CData     = [cT; cG; cK; cR];
    b.FaceAlpha = 0.82;

    for i = 1:4
        text(i, jitters(i)+0.003, sprintf('%.3f', jitters(i)), ...
             'HorizontalAlignment', 'center', 'FontSize', 9);
    end

    jg  = jitters(2);
    jr  = jitters(4);
    imp = (jg - jr) / jg * 100;
    if imp > 0
        arrow_txt = sprintf('%.1f%% smoother\nvs GPS', abs(imp));
    else
        arrow_txt = sprintf('%.1f%% less smooth\nvs GPS', abs(imp));
    end
    text(4, jr + 0.03, arrow_txt, ...
         'HorizontalAlignment', 'center', 'Color', cR, ...
         'FontWeight', 'bold', 'FontSize', 10);

    set(gca, 'XTick', 1:4, ...
        'XTickLabel', {'Ground Truth', 'GPS Only', 'Kalman Filter', 'RTS Smoother'});
    ylabel('Mean 2nd-Difference [m]  (lower = smoother)', 'FontSize', 11);
    title('Path Smoothness (lower is better)', 'FontSize', 11);

    fprintf('Figure 4 done\n');
end


% Figure 5, Parameter Sensitivity
function plot_sensitivity(data, cfg, cG, cK)

    gnss_vals = [1, 3, 5, 10, 15, 20, 30, 40];
    acc_vals  = [0.5, 1, 2, 5, 8, 10, 15, 20];
    full_mask = true(data.N, 1);
    tx = data.true_x;
    ty = data.true_y;

    rms_gnss = zeros(size(gnss_vals));
    rms_acc  = zeros(size(acc_vals));

    % test different GPS noise values
    tmp = cfg;
    for i = 1:numel(gnss_vals)
        tmp.sigma_gnss = gnss_vals(i);
        tmp.sigma_acc  = cfg.sigma_acc;
        [xf_t, Pf_t, xp_t, Pp_t] = kalman_filter(data, tmp, full_mask);
        [xs_t, ~] = rts_smoother(xf_t, Pf_t, xp_t, Pp_t, tmp);
        e = sqrt((xs_t(:,1)-tx).^2 + (xs_t(:,3)-ty).^2);
        rms_gnss(i) = sqrt(mean(e.^2));
    end

    % test different IMU noise values
    for i = 1:numel(acc_vals)
        tmp.sigma_gnss = cfg.sigma_gnss;
        tmp.sigma_acc  = acc_vals(i);
        [xf_t, Pf_t, xp_t, Pp_t] = kalman_filter(data, tmp, full_mask);
        [xs_t, ~] = rts_smoother(xf_t, Pf_t, xp_t, Pp_t, tmp);
        e = sqrt((xs_t(:,1)-tx).^2 + (xs_t(:,3)-ty).^2);
        rms_acc(i) = sqrt(mean(e.^2));
    end

    figure('Name', 'Figure 5, Sensitivity', 'NumberTitle', 'off', ...
           'Position', [50 50 1200 500]);
    sgtitle('Effect of Noise Parameters on RTS Smoother Accuracy', ...
            'FontSize', 13, 'FontWeight', 'bold');

    all_x    = {gnss_vals, acc_vals};
    all_rms  = {rms_gnss,  rms_acc};
    all_xl   = {'\sigma_{GNSS} [m]', '\sigma_{acc} [m/s^2]'};
    all_cols = {cG, cK};

    for p = 1:2
        xv  = all_x{p};
        yv  = all_rms{p};
        col = all_cols{p};
        [~, best_i] = min(yv);

        subplot(1,2,p);
        hold on; grid on;
        plot(xv, yv, 'o-', 'Color', col, 'LineWidth', 2.2, ...
             'MarkerSize', 9, 'MarkerFaceColor', col);
        plot(xv(best_i), yv(best_i), 'p', 'MarkerSize', 16, ...
             'MarkerFaceColor', [1.0 0.84 0.0], 'MarkerEdgeColor', 'k', ...
             'DisplayName', sprintf('Best: %.1f  (RMS=%.2f m)', xv(best_i), yv(best_i)));
        xlabel(all_xl{p}, 'FontSize', 11);
        ylabel('RMS Error [m]', 'FontSize', 11);
        legend('show', 'FontSize', 10, 'Location', 'best');
    end

    fprintf('Figure 5 done\n');
end
