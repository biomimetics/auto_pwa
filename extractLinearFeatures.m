function [full_state, sample_periods] = extractLinearFeatures(trial, t_range)

if nargin < 2
  t_range = 1:length(trial.interp.t);
end

sample_periods = cumsum(any([diff(trial.metrics.legs_stride(1,t_range)) == 1; ...
  diff(trial.metrics.legs_stride(2,t_range))==1],1));

lpos = mod(extractTrialState(trial,'legs_i',t_range),2*pi);
lvel = extractTrialState(trial,'legs_v',t_range);
bemf = trial.interp.Robot.BEMF(t_range,:)';
gyro = extractTrialState(trial, 'gyro_i',t_range);
accel = extractTrialState(trial, 'X_ddot_i',t_range);

pos = extractTrialState(trial, 'X_i',t_range);
vel = extractTrialState(trial, 'X_dot_i',t_range);
angle = extractTrialState(trial, 'euler_i',t_range);

cmd = extractTrialState(trial, 'legs_v_command',t_range);

full_state = [lpos; lvel; bemf; gyro; accel; pos; vel; angle; cmd];

