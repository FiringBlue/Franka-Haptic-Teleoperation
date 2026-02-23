%============================================================================
%Name        : train_gmm_gmr_from_segments.m
%Author      : Wenhao Zhao (wenhao.zhao@tum.de)
%Version     : Feb 2026
%Description : Train GMM + GMR for Part1 and Part2 on segmented throw demonstrations.
%============================================================================

clear; close all; clc;

% ========= User config =========
dataset_path = '/home/rmilnuc2/catkin_ws/src/franka_ros/franka_teleop_lmt/TeleopData/SegmentDemo';
out_path     = '/home/rmilnuc2/catkin_ws/src/franka_ros/franka_teleop_lmt/TeleopData/SegmentDemo_learned';    
nbSamples    = 6;               


nbD_map.part1 = 200;
nbD_map.part2 = 400;


% GMM components
nbStates = 5;


parts = {'part1','part2'};


do_plot = true;


% ========= Train each part =========
for pi = 1:numel(parts)
    part = parts{pi};
    nbD  = nbD_map.(part);

    fprintf('\n==== Training %s (nbD=%d, nbStates=%d) ====\n', part, nbD, nbStates);

    % 1) Load + interpolate demos, build Data = [x; v]
    [demos, Data] = load_segmented_demos(dataset_path, nbSamples, part, nbD);

    % 2) Fit GMM on [x v]
    opts = statset('MaxIter',500, 'Display','off');
    GMModel = fitgmdist(Data, nbStates, ...
        'RegularizationValue', 1e-6, ...
        'Replicates', 3, ...
        'Options',opts);

    % 3) Reproduce with GMR rollout (conditioning on x -> v, then integrate)
    x_repro = gmr_rollout_pos(GMModel, nbStates, nbD, demos);

    % 4) Save learned trajectory (3xN position, compatible with your LfD.cpp)
    learned_file = fullfile(out_path, ['learned_' part '.csv']);
    Nplay = compute_play_length_1khz(dataset_path, nbSamples, part);
    x_interp = spline(1:size(x_repro,2), x_repro, linspace(1,size(x_repro,2), Nplay));
    writematrix(x_interp, learned_file);

    fprintf('Saved: %s\n', learned_file);

    % 5) Plot
    if do_plot
        figure('Name', ['Reproduction ' part]); hold on; grid on;
        for n = 1:nbSamples
            plot3(demos(n).pos(1,:), demos(n).pos(2,:), demos(n).pos(3,:), 'LineWidth', 1.5);
        end
        plot3(x_repro(1,:), x_repro(2,:), x_repro(3,:), 'k--', 'LineWidth', 2.5);
        xlabel('x'); ylabel('y'); zlabel('z');
        title(['Part ' part ' demos + reproduction']);
        legend([arrayfun(@(k) sprintf('demo %d',k), 1:nbSamples, 'UniformOutput', false), {'repro'}], ...
               'Location','best');
        view(3);
    end
end


% ====================== Functions ======================

function [demos, Data] = load_segmented_demos(dataset_path, nbSamples, part, nbD)
% Load follower_i_partX.csv, take Xf_x Xf_y Xf_z, spline to nbD,
% compute velocity by finite diff, stack as rows for fitgmdist.

Data = [];
demos = struct([]);

for n = 1:nbSamples
    fileName = fullfile(dataset_path, sprintf('follower_%d_%s.csv', n, part));
    if ~isfile(fileName)
        error('Missing file: %s', fileName);
    end

    T = readtable(fileName);

    % --- REQUIRED columns (adjust here if your column names differ) ---
    cols = T.Properties.VariableNames;
    need = {'Xf_x','Xf_y','Xf_z'};
    if ~all(ismember(need, cols))
        error('File %s missing required columns. Found: %s', fileName, strjoin(cols, ', '));
    end

    pos = [T.Xf_x'; T.Xf_y'; T.Xf_z'];  % 3 x T
    % remove NaNs if any
    good = all(isfinite(pos),1);
    pos = pos(:,good);

    % spline interpolate to nbD
    pos = spline(1:size(pos,2), pos, linspace(1, size(pos,2), nbD));

    % velocity in m/s (1 kHz sampling)
    dt = 0.001;
    vel = [diff(pos,1,2)/dt, zeros(3,1)];

    demos(n).pos = pos;
    demos(n).vel = vel;

    % each row is [x v]
    Data = [Data; [pos' vel']];
end
end


function x_repro = gmr_rollout_pos(GMModel, nbStates, nbD, demos)
% Rollout: start at mean of demo starts, then for t=2..nbD:
% compute responsibilities h(k) based on x, compute E[v|x], integrate.

x_repro = zeros(3, nbD);

% start = mean of first points
nbSamples = numel(demos);
start_points = zeros(3, nbSamples);
for n = 1:nbSamples
    start_points(:,n) = demos(n).pos(:,1);
end
x_repro(:,1) = mean(start_points, 2);

dt = 0.001; % or 1 : discrete step in "index time"

for t = 2:nbD
    x_t = x_repro(:,t-1);
    h = zeros(1, nbStates);

    % responsibilities (unweighted by priors; ok for start; can be improved)
    for k = 1:nbStates
        mu_x = GMModel.mu(k,1:3);
        Sigma_x = GMModel.Sigma(1:3,1:3,k);
        h(k) = mvnpdf(x_t', mu_x, Sigma_x);
    end
    h = h / (sum(h) + realmin);

    % conditional mean E[v|x]
    v_t = zeros(3,1);
    for k = 1:nbStates
        mu_x = GMModel.mu(k,1:3)';
        mu_v = GMModel.mu(k,4:6)';
        Sigma_x  = GMModel.Sigma(1:3,1:3,k);
        Sigma_vx = GMModel.Sigma(4:6,1:3,k);

        v_k = mu_v + Sigma_vx / Sigma_x * (x_t - mu_x);
        v_t = v_t + h(k) * v_k;
    end

    x_repro(:,t) = x_repro(:,t-1) + v_t * dt;
end
end


% ========= read real time duration =========
function Nplay = compute_play_length_1khz(dataset_path, nbSamples, part)
% returns Nplay in samples at 1kHz using median duration
Ts = zeros(nbSamples,1);
for n = 1:nbSamples
    fn = fullfile(dataset_path, sprintf('follower_%d_%s.csv', n, part));
    T = readtable(fn);
    if ~ismember('time', T.Properties.VariableNames)
        error('Missing time column in %s', fn);
    end
    Ts(n) = T.time(end) - T.time(1);
end
Tref = median(Ts);          % robust
Nplay = max(2, round(Tref * 1000));
fprintf('[%s] durations(s): min=%.3f med=%.3f max=%.3f -> Nplay=%d\n', ...
        part, min(Ts), median(Ts), max(Ts), Nplay);
end
