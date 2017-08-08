function [X,u,X_dot,X_dot_names] = extractDiffdriveFeatures(trial)

if nargin < 2
  t_range = 1:length(trial.interp.t);
end

% Only z
pos = extractTrialState(trial, 'X_i',t_range);
pos = pos(3,:);

% Only body specific pitch and roll
angle = extractTrialState(trial, 'euler_i',t_range);
angle = angle(2:3,:);

gyro = extractTrialState(trial, 'gyro_i',t_range);
accel = extractTrialState(trial, 'X_ddot_i',t_range);

lpos = mod(extractTrialState(trial,'legs_i',t_range),2*pi);
vw_legs = extractTrialState('vw_legs',t_range);

vw = extractTrialState(trial,'vw',t_range);
vw_dot = [[0;0] diff(vw,1,2)]/trial.fuse.params.dt;

X = [pos; angle; accel; gyro; lpos; vw_legs];
u = extractTrialState(trial, 'vw_command',t_range);

X_dot_names = {'v_hat','omega_hat','v_hat''','omega_hat'''};
X_dot = [vw; vw_dot];
