function trial = calcVtMetrics(trial)

t_clip = trial.params.t_clip;

lp = extractTrialState(trial,{'legs_phase'},t_clip);
stats.phase = collectStats(abs(lp));

pos = extractTrialState(trial,{'X_i'},t_clip);
stats.error.pos = collectStats(abs(pos(1:2,:)));

angle = extractTrialState(trial,{'euler_i'},t_clip);
stats.error.angle = collectStats(abs(angle(1,:)));

energy = extractTrialState(trial,{'energy'},t_clip);
stats.energy.pos = collectStats(energy(1:3,:));
stats.energy.angle = collectStats(energy(4:6,:));

trial.stats = stats;