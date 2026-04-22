% Computes the positioning errors and prints a summary to the console.
% Also runs the GPS outage simulation to see how well IMU handles it.
%
% Returns a metrics struct that visualize.m uses for the plots.

function metrics = evaluate(data, xf, xs, cfg)

    tx = data.true_x;
    ty = data.true_y;
    gx = data.gnss_x;
    gy = data.gnss_y;

    % calculate horizontal distance error for each method
    metrics.eg = calc_error(tx, ty, gx,      gy);       % raw GPS
    metrics.ek = calc_error(tx, ty, xf(:,1), xf(:,3));  % Kalman filter
    metrics.er = calc_error(tx, ty, xs(:,1), xs(:,3));  % RTS smoother

    % calculate summary stats for each method
    metrics.gnss = get_stats(metrics.eg);
    metrics.kf = get_stats(metrics.ek);
    metrics.rts = get_stats(metrics.er);

    % path smoothness, lower means smoother path
    % I use the mean of second differences, like measuring curvature
    metrics.jitter_truth = smoothness(tx, ty);
    metrics.jitter_gnss = smoothness(gx, gy);
    metrics.jitter_kf = smoothness(xf(:,1), xf(:,3));
    metrics.jitter_rts = smoothness(xs(:,1), xs(:,3));

    % GPS outage test, simulate losing GPS signal for 30 steps
    os = cfg.outage_start;
    ol = cfg.outage_len;
    oe = os + ol;

    % create a mask where GPS is turned off during the outage window
    gps_mask = true(data.N, 1);
    gps_mask(os : oe-1) = false;

    [xf_out, Pf_out, xp_out, Pp_out] = kalman_filter(data, cfg, gps_mask);
    [xs_out, ~] = rts_smoother(xf_out, Pf_out, xp_out, Pp_out, cfg);

    % frozen GPS baseline, what happens if we just hold the last GPS fix
    gx_frozen = gx;
    gy_frozen = gy;
    gx_frozen(os : oe-1) = gx(os-1);
    gy_frozen(os : oe-1) = gy(os-1);

    err_frozen = calc_error(tx, ty, gx_frozen, gy_frozen);
    err_rts_out = calc_error(tx, ty, xs_out(:,1), xs_out(:,3));

    metrics.outage_gnss_err = mean(err_frozen(os : oe-1));
    metrics.outage_rts_err = mean(err_rts_out(os : oe-1));

    % save these for the outage plot
    metrics.xf_out = xf_out;
    metrics.xs_out = xs_out;
    metrics.gx_frozen = gx_frozen;
    metrics.gy_frozen = gy_frozen;

    print_row('GPS only (no filter)',   metrics.gnss);
    print_row('Kalman Filter result',   metrics.kf);
    print_row('RTS Smoother result',    metrics.rts);

    fprintf('\n  Path smoothness (2nd difference, lower = smoother path):\n');
    fprintf('    Ground truth : %.4f m\n', metrics.jitter_truth);
    fprintf('    GPS only     : %.4f m\n', metrics.jitter_gnss);

    jr = metrics.jitter_rts;
    jg = metrics.jitter_gnss;
    if jr < jg
        fprintf('    RTS smoother : %.4f m  (%.1f%% better than GPS)\n', jr, (jg-jr)/jg*100);
    else
        fprintf('    RTS smoother : %.4f m  (%.1f%% worse than GPS)\n',  jr, (jr-jg)/jg*100);
    end

    fprintf('%-30s  GNSS=%.3fm   KF=%.3fm   RTS=%.3fm   (RTS is %.1f%% smoother than GPS)\n', ...
        'Path Jitter (2nd-diff)', ...
        metrics.jitter_gnss, metrics.jitter_kf, metrics.jitter_rts, ...
        (metrics.jitter_gnss - metrics.jitter_rts) / metrics.jitter_gnss * 100);

    imp = (metrics.outage_gnss_err - metrics.outage_rts_err) / metrics.outage_gnss_err * 100;
    fprintf('\n  GPS outage test (steps %d to %d):\n', os, oe-1);
    fprintf('    Frozen GPS error  : %.2f m\n', metrics.outage_gnss_err);
    fprintf('    KF + IMU error    : %.2f m  (%.1f%% improvement)\n', metrics.outage_rts_err, imp);

end

% calculates straight-line distance error between estimate and truth
function e = calc_error(tx, ty, ex, ey)
    e = sqrt((ex - tx).^2 + (ey - ty).^2);
end

% path smoothness using mean second difference magnitude
function j = smoothness(x, y)
    j = mean(sqrt(diff(x,2).^2 + diff(y,2).^2));
end

% collect mean, rms, median, 95th percentile, max in a struct
function s = get_stats(e)
    s.mean = mean(e);
    s.rms = sqrt(mean(e.^2));
    s.median = median(e);
    s.p95 = prctile(e, 95);
    s.max = max(e);
end

function print_row(label, s)
    fprintf('%-30s  mean=%6.2fm  RMS=%6.2fm  median=%6.2fm  p95=%6.2fm  max=%6.2fm\n', ...
            label, s.mean, s.rms, s.median, s.p95, s.max);
end
