function [T,S] = crossvalSim(trial,t_range,models)

n_t = size(trial.fuse.xsmooth,2);
model_range = [1:(t_range(1)-1) (t_range(end)+1):n_t];

D = extractModelFeatures(trial,model_range);

T = testAllModels(D);

S = simulateK50Model(T,trial,t_range,models);

