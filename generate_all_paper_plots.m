%% Plot phase as a function of speed, heatmap
% this plot is used to justify the region we choose to look at forward
% simulation

load clipped_data/full_stats.mat

%%
figure(1)
clf
plotPhaseTrends(gains,statData);

%%
clear

%% Load data for trajectory information of 0.25 trial
load_good_trial

%% Plot dynamics changes with phase
diff_drive_err = mahalMetric(trial.metrics.vw_legs-trial.metrics.vw,...
    trial.metrics.vw);
[coords, counts, means, stds] = binCorrelation(diff_drive_err,...
    T.D.X(10:11,:), 75);

%% 
figure(2)
clf
pcolor(coords(:,1), coords(:,2), means(:,:,1))
xlabel('\alpha_l (rad)')
ylabel('\alpha_r (rad)')
shading flat
axis equal
axis tight
cm = colormap('gray');
cm = cm(64:-1:1,:);
colormap(cm)
cb = colorbar('peer',gca);
set(get(cb,'ylabel'),'String','Diff-Drive Score');
%% Plot transition heatmap
% Shows how model transitions are correlated with physical nonlinearities
figure(3)
clf
plotPhaseHeatmap(T,trial);


%% calculate region error information
R = calcRegionError(T);

%% Plot model performance trends
figure(4)
clf
plotPredErrVsDist(R);

%% Simulation plots
t_range = (1:200)+23970;
% t_range = (1:200)+38900;
models = [2 4 5];
sim_axes = [9 6];

%%
S = simulateK50Model(T,trial,t_range,models);
figure(5)
clf
plotSimReal(S,trial,sim_axes)

%% Crossvalidated simulation
if ~exist('Tcv','var')
    [Tcv,Scv] = crossvalSim(trial,t_range,models);
else
    Scv = simulateK50Model(Tcv,trial,t_range,models);
end

figure(6)
clf
plotSimReal(Scv,trial,sim_axes)

%% Simulation Performance
sim_models = 1:5;
sim_samples = 100;
min_time = 10;
max_time = 100;
time_step = 200;
sim_length = floor(logspace(log10(min_time),log10(max_time),time_step));

%%
SP = simPerformance(T,trial,sim_models,sim_length,sim_samples);

%% Simulation performance plots
figure(7)
clf
plotSimPerformance(T,SP)