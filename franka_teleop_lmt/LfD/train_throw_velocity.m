%============================================================================
%Name        : train_throw_velocity.m
%Author      : Wenhao Zhao (wenhao.zhao@tum.de)
%Version     : Feb 2026
%Description : Train GMM for Part3 on throw velocity only (no position).
%============================================================================

clear; close all; clc;

% ========= config =========
dataset_path = '/home/rmilnuc2/catkin_ws/src/franka_ros/franka_teleop_lmt/TeleopData/SegmentDemo';
nbSamples = 6;
nbD = 100;          % throw part 
nbStates = 3;

Data = [];


% ========= load demos =========
for n = 1:nbSamples
    fileName = fullfile(dataset_path, sprintf('follower_%d_part3.csv', n));
    T = readtable(fileName);

    % velocity directly from follower 
    vel = [T.Vf_x'; T.Vf_y'; T.Vf_z'];

    % remove NaN
    good = all(isfinite(vel),1);
    vel = vel(:,good);

    % spline to unified length
    vel = spline(1:size(vel,2), vel, linspace(1,size(vel,2), nbD));

    demos(n).vel = vel;

    % only velocity：Data = v
    Data = [Data; vel'];
end

% ========= GMM on velocity only =========
opts = statset('MaxIter',500, 'Display','off');
GMModel = fitgmdist(Data, nbStates, ...
        'RegularizationValue', 1e-6, ...
        'Replicates', 3, ...
        'Options',opts);

% ========= reproduce velocity =========
v_repro = zeros(3, nbD);

% initial vel = demo average at t=1
v0 = zeros(3,1);
for n = 1:nbSamples
    v0 = v0 + demos(n).vel(:,1);
end
v_repro(:,1) = v0 / nbSamples;

for t = 2:nbD
    v_t = v_repro(:,t-1);

    h = zeros(1, nbStates);
    for k = 1:nbStates
        mu = GMModel.mu(k,:)';
        Sigma = GMModel.Sigma(:,:,k);
        h(k) = mvnpdf(v_t', mu', Sigma);
    end
    h = h / (sum(h) + realmin);

    v_next = zeros(3,1);
    for k = 1:nbStates
        v_next = v_next + h(k) * GMModel.mu(k,:)';
    end

    v_repro(:,t) = v_next;
end

% ========= save =========
out_file = fullfile(dataset_path, 'learned_part3_vel.csv');
Nplay = compute_play_length_1khz(dataset_path, nbSamples, 'part3');
v_interp = spline(1:size(v_repro,2), v_repro, linspace(1,size(v_repro,2), Nplay));
writematrix(v_interp, out_file);

fprintf('Saved velocity primitive: %s\n', out_file);

% ========= plot =========
figure; hold on; grid on;
for n = 1:nbSamples
    plot(demos(n).vel(1,:), 'r');
end
plot(v_repro(1,:), 'k', 'LineWidth', 2);
title('Throw velocity X');



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