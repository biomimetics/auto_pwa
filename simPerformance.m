function SP = simPerformance(T,trial,models,sim_length,n_sample)

if nargin < 3
    models = 1:5;
end

if nargin < 4
    sim_length = [5 10 20 50 100 200];
end

if nargin < 5
    n_sample = 100;
end

SP.models = models;
SP.sim_length = sim_length;

n_t = size(T.D.X_dot,2);
n_m = length(models);
n_sl = length(sim_length);

sl_max = max(sim_length);

start_times = datasample(1:(n_t-sl_max),n_sample,'Replace',false);

scores = zeros(n_sample, n_sl, n_m);
trans = zeros(n_sample, n_sl, n_m);
match = zeros(n_sample, n_sl, n_m);

X = xsmoothToFullState(trial);
Ex_inv = cov(X',1)^-1;


for st = 1:n_sample
    tic;
    t_range =  (1:sl_max) + start_times(st);
    Sn = simulateK50Model(T,trial,t_range,models);
    Xs = X(:,t_range(sim_length));
    
    for m = 1:n_m
        err = Sn.X(:,sim_length,m) - Xs;    
        scores(st,:,m) = sqrt(sum(err .* (Ex_inv * err),1));
        
        m_trans = [0; (diff(Sn.idx(:,m)) ~= 0)];
        m_match = Sn.idx(:,m) == T.idx{m}(t_range)';
        for sl = 1:n_sl
            sl_range = 1:sim_length(sl);
            trans(st,sl,m) = sum(m_trans(sl_range));
            match(st,sl,m) = sum(m_match(sl_range));
        end
    end
    
    proc_time = toc;
    if st == 1
        sprintf('Estimated Completion: %f',proc_time*n_sample)
    end
end

SP.scores = squeeze(mean(scores,1));
SP.stds = squeeze(std(scores,1,1));
% ldivide produces average time spent in a region, rdivide is probability
% of transition on a given timestep
SP.trans = bsxfun(@ldivide,squeeze(mean(trans,1)),sim_length');
SP.match = bsxfun(@ldivide,squeeze(mean(match,1)),sim_length');