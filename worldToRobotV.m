function vw_robot = worldToRobotV(trial)

X = trial.fuse.xsmooth;
idx = trial.fuse.params;
Vt = trial.interp.Control.treadSpeed(1,1);

% Euler is yaw, pitch, roll
yaw = X(idx.euler_i(1),:);

v_hat_world_unit = [cos(yaw); sin(yaw)];

vw_robot(1,:) = sum(v_hat_world_unit .* [X(idx.X_dot_i(1),:) + Vt(:,1)'; X(idx.X_dot_i(2),:)],1);
vw_robot(2,:) = [0 diff(yaw)/trial.fuse.params.dt];