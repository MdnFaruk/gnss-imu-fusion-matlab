% This runs the RTS (Rauch-Tung-Striebel) smoother after the Kalman filter.
%
% The idea is the Kalman filter only uses past data at each step.
% The RTS smoother goes backwards through all the data and improves the
% estimates by also using future measurements. So every point gets refined.
%
% I learned about this from the Rauch et al. 1965 paper and from
% the reference paper by Siemuri et al. (ION GNSS+ 2022).

function [xs, Ps] = rts_smoother(xf, Pf, xp, Pp, cfg)
    % xf, Pf, filtered states/covariances from kalman_filter.m
    % xp, Pp, predicted states/covariances from kalman_filter.m
    % cfg, struct from main.m, needs dt and sigma_acc

    n = size(xf, 1);
    dt = cfg.dt;
    sig_acc = cfg.sigma_acc;

    F = kf_matrices('F', dt);

    xs = xf;    % Smoothed state estimates (N x 4)
    Ps = Pf;    % Smoothed covariance matrices (4 x 4 x N)

    % go backwards from second to last
    for k = n-1 : -1 : 1

        % predicted covariance at k+1 from the forward pass
        Pp_next = Pp(:,:,k+1);

        % compute the smoother gain
        try
            Ck = Pf(:,:,k) * F' / Pp_next;
        catch
            % if the matrix is singular, use pseudo-inverse
            Ck = Pf(:,:,k) * F' * pinv(Pp_next);
        end

        % update the smoothed state estimate
        xs(k,:) = xf(k,:) + (Ck * (xs(k+1,:)' - xp(k+1,:)'))';

        % update the smoothed covariance
        diff_P = Ps(:,:,k+1) - Pp_next;
        Ps_new = Pf(:,:,k) + Ck * diff_P * Ck';
        Ps(:,:,k) = 0.5 * (Ps_new + Ps_new');
    end
end
