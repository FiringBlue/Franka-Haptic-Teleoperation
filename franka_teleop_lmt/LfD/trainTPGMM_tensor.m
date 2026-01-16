function model = trainTPGMM_tensor(Data_tensor, model)
% Train a Task-Parameterized GMM using Expectation-Maximization
% Data_tensor: D x M x T tensor (D = dimension, M = number of frames, T = total timepoints)
% model: structure with fields:
%        nbStates - number of mixture components
%        nbFrames - number of frames
%        nbVar    - data dimension D
%        [optional] params_diagRegFact, params_nbMaxSteps, etc.

% Set defaults if missing
if ~isfield(model, 'params_diagRegFact'), model.params_diagRegFact = model.reg; end
if ~isfield(model, 'params_nbMaxSteps'), model.params_nbMaxSteps = 100; end
if ~isfield(model, 'params_nbMinSteps'), model.params_nbMinSteps = 5; end
if ~isfield(model, 'params_maxDiffLL'), model.params_maxDiffLL = 1e-4; end

[D, M, T] = size(Data_tensor);
K = model.nbStates;

% === K-means initialization ===
Data_concat = reshape(permute(Data_tensor, [1 3 2]), D*T, M);
Data_kmeans = reshape(Data_concat, D*M, T)';
[idx, C] = kmeans(Data_kmeans, K);
model.Priors = zeros(1,K);
model.Mu = zeros(D,M,K);
model.Sigma = zeros(D,D,M,K);

for k = 1:K
    model.Priors(k) = sum(idx==k)/T;
    for m = 1:M
        Data_m = squeeze(Data_tensor(:,m,idx==k));
        model.Mu(:,m,k) = mean(Data_m,2);
        model.Sigma(:,:,m,k) = cov(Data_m') + eye(D)*model.params_diagRegFact;
    end
end

% === EM Loop ===
loglik_old = -inf;
for iter = 1:model.params_nbMaxSteps
    %% E-step
    Lik = zeros(K, T);
    for k = 1:K
        p_k = ones(1,T);
        for m = 1:M
            mu = model.Mu(:,m,k);
            sigma = model.Sigma(:,:,m,k);
            for t = 1:T
                x = Data_tensor(:,m,t);
                p_k(t) = p_k(t) * mvnpdf(x', mu', sigma);
            end
        end
        Lik(k,:) = model.Priors(k) * p_k;
    end
    gamma = Lik ./ (sum(Lik,1) + realmin);

    %% M-step
    for k = 1:K
        Nk = sum(gamma(k,:));
        model.Priors(k) = Nk / T;
        for m = 1:M
            mu_km = zeros(D,1);
            sigma_km = zeros(D,D);
            for t = 1:T
                x = Data_tensor(:,m,t);
                mu_km = mu_km + gamma(k,t) * x;
            end
            mu_km = mu_km / Nk;
            model.Mu(:,m,k) = mu_km;
            for t = 1:T
                x = Data_tensor(:,m,t);
                sigma_km = sigma_km + gamma(k,t) * (x - mu_km)*(x - mu_km)';
            end
            model.Sigma(:,:,m,k) = sigma_km / Nk + eye(D)*model.params_diagRegFact;
        end
    end

    %% Check log-likelihood
    loglik = sum(log(sum(Lik,1) + realmin));
    if iter > model.params_nbMinSteps && abs(loglik - loglik_old) < model.params_maxDiffLL
        fprintf('EM converged at iteration %d\n', iter);
        break;
    end
    loglik_old = loglik;
end
end
