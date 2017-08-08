function trial = calcControlMetrics(trial)

trial.params.Kcrank = 0.0102; %(m/rad) effective leg_r: leg_v*Kcrank = ground_v
trial.params.Kl = 10.43; % (rad/ticks) leg_command = leg_v*Kl
trial.params.mass = 34; % (g)
trial.params.I = [9.29 -0.12 -0.22; -0.12 21.52 0.007; -0.22 0.007 21.97] *1e-6; %(kg mm^2)

legs = trial.fuse.xsmooth(trial.fuse.params.legs_i,:);

% legs_r - legs_l to be consistent with omega calculation. Also L + phase =
% R. Add pi to left legs to account for crank offset (alternating tripod is
% phase 0)
trial.metrics.legs_phase = mod(legs(2,:)-legs(1,:)+pi,2*pi);
shift_i = trial.metrics.legs_phase>pi;
trial.metrics.legs_phase(shift_i) = trial.metrics.legs_phase(shift_i) - 2*pi;

trial.metrics.legs_stride = floor(legs/pi);

trial.metrics.legs_v = [[0 diff(legs(1,:))];[0 diff(legs(2,:))]]/trial.fuse.params.dt;
trial.metrics.legs_v_command = trial.interp.Control.command(:,[1 5])'/trial.params.Kl;

trial.metrics.vw = worldToRobotV(trial);
trial.metrics.vw_command = expectedControl(trial, trial.metrics.legs_v_command);
trial.metrics.vw_legs = expectedControl(trial, trial.metrics.legs_v);

trial.metrics.legs_v_effective = expectedLegV(trial, trial.metrics.vw);

angle_v = extractTrialState(trial, {'gyro_i'});
robot_v_world = extractTrialState(trial, {'X_dot_i'});

% robot frame energy (x,y,z,yaw,pitch,roll) in mJ
trial.metrics.energy = 1/2 * [trial.params.mass * robot_v_world.^2;...
  1000 * bsxfun(@times,diag(trial.params.I), angle_v.^2)];
