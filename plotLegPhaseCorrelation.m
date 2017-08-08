function [coords, counts, means, stds] = plotLegPhaseCorrelation(trial, t_span, n_bins)

if nargin < 2
  t_span = 1:size(trial.fuse.xsmooth,2);
end

if nargin < 3
  n_bins = 100;
end

leg_error_robot = legEffectiveness(trial, 'robot');

leg_error_robot = leg_error_robot(:,t_span);
leg_angle = mod(trial.fuse.xsmooth(trial.fuse.params.legs_i,t_span),2*pi);
leg_phase = leg_angle - [leg_angle(2,:); leg_angle(1,:)];

%[coords, counts, means, stds] = plotCorrelation(leg_error_robot(:)', [leg_angle(:) mod(leg_phase(:),2*pi)]', n_bins);
[coords, counts, means, stds] = plotCorrelation(leg_error_robot(:)', sin(leg_phase(:))', n_bins);