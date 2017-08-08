function vw = expectedControl(trial, legs_v)

% Kcrank maps crank velocity in rad/s to average foot velocity in m/s
% d is robot foot width

vw(1,:) = trial.params.Kcrank*(legs_v(1,:) + legs_v(2,:))/2;
vw(2,:) = trial.params.Kcrank*(legs_v(2,:) - legs_v(1,:))/trial.params.d;