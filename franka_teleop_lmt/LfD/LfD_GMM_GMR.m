clear all;
close all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RMIL @TU München 2025
% Learning from demonstration using GMM
% GMM/GMR Based Reproduction Using P(v, x) 
% Basak Gulecyuz
% Conditioning on position x to predict velocity v
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Parameters
model.nbStates = 5;                         % Number of GMM states
model.nbD = 100;                            % Number of datapoints (per demo)
model.nbSamples = 3;                        % Number of demonstrations
dataset_path = '/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/TeleopData/';                  % Path to demonstration files

%% Load Demonstrations
Data = [];
for n = 1:model.nbSamples
    % Load and interpolate demo
    fileName = [dataset_path 'follower_' num2str(n) '.txt'];
    pos = loadDemos(fileName)';
    pos = spline(1:size(pos,2), pos, linspace(1, size(pos,2), model.nbD));

    % Compute velocity using finite differences
    vel = [diff(pos,1,2), zeros(3,1)];

    % Store and stack data as [x; v]
    demos(n).pos = pos;
    demos(n).vel = vel;
    Data = [Data, [pos; vel]];
end
Data = Data';  % Now each row is [x, v]

%% Frames

% Define transformation matrices for each frame from SDF pose
start_pose = [0.67 -0.32 0.40 0 0 -0.8708];   % x, y, z, roll, pitch, yaw
end_pose   = [0.41  0.15 0.40 0 0  1.5708];   % x, y, z, roll, pitch, yaw

% Convert to structure with .A (rotation matrix) and .b (translation)
p(1).A = eul2rotm(start_pose(4:6), 'XYZ');  % Rotation matrix
p(1).b = start_pose(1:3)';                 % Column vector
p(1).color =  [0.0, 1.0, 0.0];

p(2).A = eul2rotm(end_pose(4:6), 'XYZ');
p(2).b = end_pose(1:3)';
p(2).color = [1.0, 0.0, 0.0];   % Bright red (RedGlow)

%% Visualize Demonstrations
figure
hold on; grid on;

for n = 1:model.nbSamples
    h_demo(n) = plot3(demos(n).pos(1,:), demos(n).pos(2,:), demos(n).pos(3,:), 'LineWidth', 2.0);
end

% Plot frames (returns patch handles)
h_frames = plotFrames3D(p);

% Set labels and view
xlabel('x'); ylabel('y'); zlabel('z');
title('Demonstrations with Start/End Frames');
% view(3);

% Combine all handles into legend
legend({'demo 1', 'demo 2'});


%% ==== To-Do: Fit GMM to [x; v] ====
% Your task: Fit a Gaussian Mixture Model (GMM) to the data. 
% Hint: See fitgmdist()
% Our goal is to model P(x,v) using GMM
% Using a regularization value helps improve learning (e.g 1e-6)
K = model.nbStates;

GMModel = fitgmdist(Data, K, 'RegularizationValue', 1e-6);


%% Visualize GMM 
figure
hold on; grid on;

for n = 1:model.nbSamples
    h_demo(n) = plot3(demos(n).pos(1,:), demos(n).pos(2,:), demos(n).pos(3,:), 'LineWidth', 2.0);
end

% Plot frames (returns patch handles)
h_frames = plotFrames3D(p);

% Set labels and view
xlabel('x'); ylabel('y'); zlabel('z');
title('Demonstrations with Start/End Frames');
view(135, 30); % Top view


% Plot Gaussian states
GMM3D_plot(GMModel.mu(:,1:3)', GMModel.Sigma(1:3,1:3,:),1);

% Set labels and view
xlabel('x'); ylabel('y'); zlabel('z');

view(3);  % Top view


%% ==== To-Do: Model Selection using BIC ====
% Your task: Implement model selection to find the optimal number of GMM states.
% Follow these steps:

% 1. Define a range of possible state counts (e.g., from 2 to 20).
% 2. For each number of states K:
%    - Fit a GMM to the dataset using fitgmdist()
%    - Use a regularization value (e.g., 1e-6) and set 'Replicates=3' and 'MaxIter=500' for stability
%    - Store the Bayesian Information Criterion (BIC) for each K (Hint:
%    gitgmdist already returns BIC.)
% 3. Plot the BIC values against the number of states.
% 4. Find the number of states K with the minimum BIC value.
BICs = zeros(1,20);
for i=2:20
    gm = fitgmdist(Data, K, 'RegularizationValue', 1e-9);
    BICs(i) = gm.BIC;
end
[~, K_opt] = min(BICs);
% Plot
figure;
plot(2:20, BICs(2:20), '-o', 'LineWidth', 2);
xlabel('Number of States');
ylabel('BICs');
title('BIC vs Number of Gaussian Components');
grid on;



%% ====To-Do: Fit GMM to [x; v] with the number of states computed via BIC ====
fitgmdist(Data, K_opt, 'RegularizationValue', 1e-6);




%% Reproduce Trajectory via GMR Rollout
% To-Do: Use GMR to obtain the conditional dist. P(v|x)
% We will use the E[P(v|x)] at each sampling time t to compute the learned
% velocity. We will then integrate using the velocity and compute the
% learned trajectory.
% See lecture slides for GMR. Please follow the guideline step by step to 
% complete the task.

% Learned trajectory to be reproduced
x_repro = zeros(3, model.nbD);

% Extract mean of initial demo points as starting position
start_points = zeros(3, model.nbSamples);
for n = 1:model.nbSamples
    start_points(:,n) = demos(n).pos(:,1);
end
x_repro(:,1) = mean(start_points, 2);

% Sampling period
dt = 1;

for t = 2:model.nbD
    x_t = x_repro(:,t-1);
    v_t = zeros(3,1);
    h = zeros(1, model.nbStates); % state responsibily (or probability)

    % Computing h(k)
    for k = 1:model.nbStates
        mu_x = GMModel.mu(k,1:3)';
        sigma_x = GMModel.Sigma(1:3,1:3,k);

        %% ==== To-Do: Compute Responsibilities h(k) ====
        % Your task: For each GMM component, compute the probability of x_t under that component.
        % Hint: use mvnpdf(x_t', mu_x', sigma_x)
        h(k)=mvnpdf(x_t', mu_x', sigma_x);
      
    end
    h = h / (sum(h) + realmin);


    % Computing E(v|x)
    for k = 1:model.nbStates
        mu_x = GMModel.mu(k,1:3)';
        mu_v = GMModel.mu(k,4:6)';
        sigma_x = GMModel.Sigma(1:3,1:3,k);
        sigma_vx = GMModel.Sigma(4:6,1:3,k);

        %% ==== To-Do: Compute Conditional Mean of v | x ====
        % Your task: Use the responsibilities h(k) to compute the expected velocity v_t.
        % Hint: use mu_v + sigma_vx / sigma_x * (x_t - mu_x)
        vel = mu_v + sigma_vx / sigma_x * (x_t - mu_x);
        v_t = v_t + h(k) * vel;
        
    end

    % trajectory to be reproduced
    x_repro(:,t) = x_repro(:,t-1) + v_t * dt;
end

%% Plot Result
figure
for n = 1:model.nbSamples
    plot3(demos(n).pos(1,:), demos(n).pos(2,:), demos(n).pos(3,:), 'LineWidth', 2.0);
    hold on;
end
plot3(x_repro(1,:), x_repro(2,:), x_repro(3,:), 'k--', 'LineWidth', 2.0);
xlabel('x'); ylabel('y'); zlabel('z'); grid on;
% Plot frames (returns patch handles)
h_frames = plotFrames3D(p);
legend('Demo 1', 'Demo 2', 'Reproduction');
view(3);

%% Save Learned Trajectory
x_interp = spline(1:size(x_repro,2), x_repro, linspace(1,size(x_repro,2),model.nbD*12));
csvwrite([dataset_path 'learned.csv'], x_interp);
