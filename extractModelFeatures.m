function D = extractModelFeatures(trial,t_range)

if nargin < 2
  t_range = 1:length(trial.interp.t);
end

pos = extractTrialState(trial, 'X_i',t_range);
pos = pos(3,:);

vel_world = extractTrialState(trial, 'X_dot_i',t_range);
vel = zeros(size(vel_world));
acc = extractTrialState(trial, 'X_ddot_i',t_range);

[b,a] = butter(3,0.1);

angle = extractTrialState(trial, 'euler_i',t_range);

R = angle2dcm(angle(1,:),angle(2,:),angle(3,:));
for r = 1:size(R,3)
  vel(:,r) = R(:,:,r)*vel_world(:,r);
end

angle = angle(2:3,:);

gyro = filtfilt(b,a,extractTrialState(trial, 'gyro_i',t_range)')';

w_cross = omegaCrossTerms(gyro, trial.params.I);
gyro_dot = [[0;0;0] diff(gyro,1,2)]/trial.fuse.params.dt-w_cross;

lpos = mod(extractTrialState(trial,'legs_i',t_range),2*pi);
% lpos = [cos(2*extractTrialState(trial,'legs_i',t_range));...
%   sin(2*extractTrialState(trial,'legs_i',t_range))];
lvel = filtfilt(b,a,extractTrialState(trial,'legs_v',t_range)')';
lacc = [[0;0] diff(lvel,1,2)]/trial.fuse.params.dt;

D.X = [pos; angle; vel; gyro; lpos; lvel];
D.u = extractTrialState(trial, 'vw_command',t_range);
D.X_dot_names = {'x"', 'y"', 'z"', 'psi"', 'theta"','phi"','alpha_l"','alhpa_r"'};
D.X_dot = [acc; gyro_dot; lacc];
