clear all;
close all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RMIL @TU München 2025
% Learning from demonstration using TP-GMM
% GMM/GMR Based Reproduction Using P(v, x) 
% Basak Gulecyuz
% Conditioning on position x to predict velocity v
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Parameters
model.nbStates = 6; % ==== To-Do: You might want to tune the nbStates
model.nbD = 100;
model.nbSamples = 2;
model.dt=0.1;
reg = 1e-4;  % ==== To-Do: You might want to tune the regularization factor
model.reg=reg;
dataset_path = '/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/TeleopData/';

%% Load Demonstrations
for n = 1:model.nbSamples
    fileName = [dataset_path 'follower_' num2str(n) '.txt'];
    pos = loadDemos(fileName)';
    pos = spline(1:size(pos,2), pos, linspace(1, size(pos,2), model.nbD));
    vel = [diff(pos,1,2), zeros(3,1)]./model.dt;

    demos(n).pos = pos;
    demos(n).vel = vel;
    demos(n).Data1 = [pos; vel];
end

%% Frames (Red is start, Green is end)
red_frame = [0.41  0.15 0.40 0 0  1.5708];  % start
green_frame   = [0.67 -0.32 0.40 0 0 -0.8708];  % end

% Demonstrations
% start
demos(1).p(1).A = eul2rotm(red_frame(4:6), 'XYZ');
demos(1).p(1).b = red_frame(1:3)';
demos(1).p(1).color = [1.0, 0.0, 0.0];

% end
demos(1).p(2).A = eul2rotm(green_frame(4:6), 'XYZ');
demos(1).p(2).b = green_frame(1:3)';
demos(1).p(2).color = [0.0, 1.0, 0.0];



%% ==== To-Do: Define the frames for additional demonstrations====
%yellow_frame   = [0.0 -0.32 0.40 0 0 -0.8708];  % change accordingly
custom_frame   = [0.55 0.25 0.40 0 0 0.5];

% start
demos(2).p(1).A = eul2rotm(red_frame(4:6), 'XYZ');
demos(2).p(1).b = red_frame(1:3)';
demos(2).p(1).color = [1.0, 0.0, 0.0];

% end
demos(2).p(2).A = eul2rotm(custom_frame(4:6), 'XYZ');
demos(2).p(2).b = custom_frame(1:3)';
demos(2).p(2).color = [0.0, 1.0, 0.0];

%% ==== To-Do: Define the frames for novel test points====
blue_frame =   [0.5, -0.37, 0.40, 0, 0, -pi/3];  % change accordingly 

% Novel test
% start 
p_new(1).A = eul2rotm(red_frame(4:6), 'XYZ');
p_new(1).b = red_frame(1:3)';
p_new(1).color = [1.0, 1.0, 0.0];

% end
p_new(2).A = eul2rotm(blue_frame(4:6), 'XYZ');
p_new(2).b = blue_frame(1:3)';
p_new(2).color = [0.0, 0.0, 1.0];


%% Plot demonstrations
figure;
hold on; grid on;
for n = 1:model.nbSamples
    plot3(demos(n).pos(1,:), demos(n).pos(2,:), demos(n).pos(3,:), 'LineWidth', 2.0);
    h_frames = plotFrames3D(demos(n).p);
end
xlabel('x'); ylabel('y'); zlabel('z');


%% ==== To-Do: Transform Demonstration Data to Local Frame Coordinates ====
model.nbVar = 6; %%[x, v]
model.nbFrames = 2;
TPGMM_Data = zeros(model.nbVar, model.nbFrames, model.nbSamples * model.nbD);
for n = 1:model.nbSamples
    for m = 1:model.nbFrames
        for t = 1:model.nbD
            x_global = demos(n).Data1(:,t);
            pos_g = x_global(1:3);
            vel_g = x_global(4:6);
            % Frame rotation and translation
            A = demos(n).p(m).A;  % 3x3 rotation matrix
            b = demos(n).p(m).b;  % 3x1 position vector
            % ==== To-Do: Transform global position to local
            x_local = A' * (pos_g - b);

            % ==== To-Do: Transform global velocity to local
            v_local = A' * vel_g;
            
            TPGMM_Data(:,m,(n-1)*model.nbD + t) = [x_local; v_local];
        end
    end
end

%% Train TP-GMM
model = trainTPGMM_tensor(TPGMM_Data, model);

%% Plot demonstrations observerd from frames and the TP-GMM states
for m = 1:model.nbFrames

figure
hold on; grid on;

for n = 1:model.nbSamples
    x_local = TPGMM_Data(:,m,(n-1)*model.nbD+1:n*model.nbD);
    h_demo(n) = plot3(x_local(1,:), x_local(2,:), x_local(3,:), 'LineWidth', 2.0);
end

% Plot Gaussian states
GMM3D_plot(squeeze(model.Mu(1:3,m,:)), squeeze(model.Sigma(1:3,1:3,m,:)),1);
  
end
% Set labels and view
xlabel('x'); ylabel('y'); zlabel('z');
view(135, 30); % Top view


%% TP-GMR Rollout (v|x)
DataIn = zeros(3, model.nbD);
DataOut = zeros(3, model.nbD);
DataIn(:,1) =  demos(1).pos(:,1);


for t = 2:model.nbD
    h = zeros(model.nbStates,1);
    mu_v_all = zeros(3, model.nbStates);
    sigma_v_all = zeros(3,3, model.nbStates);

    x = DataIn(:,t-1);  % Current position

    for i = 1:model.nbStates
        Sigma_sum = zeros(6);
        Mu_sum = zeros(6,1);

        for m = 1:model.nbFrames
            A_aug = blkdiag(p_new(m).A, p_new(m).A);
            b_aug = [p_new(m).b; zeros(3,1)];

            Mu_local = model.Mu(:,m,i);
            Sigma_local = model.Sigma(:,:,m,i);

            Mu_global = A_aug * Mu_local + b_aug;
            Sigma_global = A_aug * Sigma_local * A_aug';

            Sigma_sum = Sigma_sum + inv(Sigma_global + reg*eye(6));
            Mu_sum = Mu_sum + inv(Sigma_global + reg*eye(6)) * Mu_global;
        end

        Sigma_joint = inv(Sigma_sum + reg*eye(6));
        Mu_joint = Sigma_joint * Mu_sum;

        % GMR: condition v | x
        mu_x = Mu_joint(1:3);
        mu_v = Mu_joint(4:6);
        sigma_xx = Sigma_joint(1:3,1:3);
        sigma_vx = Sigma_joint(4:6,1:3);
        sigma_vv = Sigma_joint(4:6,4:6);

        % Regularize sigma_xx before using it
        sigma_xx = (sigma_xx + sigma_xx') / 2;                    % Force symmetry
        sigma_xx = sigma_xx + eye(size(sigma_xx)) * reg;          % Add small regularization

        mu_vx = mu_v + sigma_vx / sigma_xx * (x - mu_x);
        sigma_vx_cond = sigma_vv - sigma_vx / sigma_xx * sigma_vx';

        h(i) = model.Priors(i) * mvnpdf(x, mu_x, sigma_xx);
        mu_v_all(:,i) = mu_vx;
        sigma_v_all(:,:,i) = sigma_vx_cond;
    end

    % % Normalize weights
    if sum(h) < 1e-8 || any(isnan(h))
        h = ones(model.nbStates,1);
    end
    h = h / sum(h);

    % Weighted fusion
    mu_out = sum(mu_v_all .* h', 2);
    Sigma_out = zeros(3);
    for i = 1:model.nbStates
        Sigma_out = Sigma_out + h(i) * (sigma_v_all(:,:,i) + mu_v_all(:,i)*mu_v_all(:,i)');
    end
    Sigma_out = Sigma_out - mu_out * mu_out';

    DataOut(:,t) = mu_out;
    DataIn(:,t) = DataIn(:,t-1) + model.dt * mu_out;
end

% Trajectory to be reproduced is the learned position
x_repro = DataIn;

%% Plot result
figure;
hold on; grid on;
for n = 1:model.nbSamples
    plot3(demos(n).pos(1,:), demos(n).pos(2,:), demos(n).pos(3,:), 'LineWidth', 2.0);
    h_frames = plotFrames3D(demos(n).p);
end
plot3(DataIn(1,:), DataIn(2,:), DataIn(3,:), 'k--', 'LineWidth', 2.0);
h_frames_new = plotFrames3D(p_new);
xlabel('x'); ylabel('y'); zlabel('z');
title('TP-GMR Reproduction v|x in New Frame');
legend('Demo 1', 'Demo 2', 'TP-GMR Reproduction');

%% Save Learned Trajectory
x_interp = spline(1:size(x_repro,2), x_repro, linspace(1,size(x_repro,2),model.nbD*12));
csvwrite([dataset_path 'learned.csv'], x_interp);
