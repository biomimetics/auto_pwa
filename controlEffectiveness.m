function [v_error, omega_error] = controlEffectiveness(trial, legSource)

if nargin < 2
  legSource = 'robot';
end

if strcmp(legSource, 'robot')
  legs = trial.fuse.xsmooth(trial.fuse.params.legs_i,:);
  legs_v = [[0 diff(legs(1,:))];[0 diff(legs(2,:))]]/trial.fuse.params.dt;
else
  Kl = 10.43; % conversion from rad/s to leg command
  legs_v = trial.interp.Control.command(:,[1 5])'/Kl;
end

[v_hat_robot, omega_hat_robot] = expectedControl(legs_v,trial.params.Kcrank,trial.params.d);

[v_robot, omega_robot] = worldToRobotV(trial);

% Find the average of the next filtN values
filt_N = 10;
avg_filt = ones(1,filt_N)/filt_N;
avg_v_robot = conv(v_robot, avg_filt);
avg_omega = conv(omega_robot, avg_filt);
avg_v_robot = avg_v_robot(filt_N:end);
avg_omega = avg_omega(filt_N:end);

v_error =  avg_v_robot - v_hat_robot;

omega_error = avg_omega - omega_hat_robot;