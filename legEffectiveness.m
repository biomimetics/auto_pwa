function leg_error = legEffectiveness(trial, legSource)

% Get effective leg action in m/s
effectiveLegs = expectedLegV(trial,trial.metrics.legs_vw);

if strcmp(legSource, 'robot')
  legs = trial.fuse.xsmooth(trial.fuse.params.legs_i,:);
  legs_v = [[0 diff(legs(1,:))];[0 diff(legs(2,:))]]/trial.fuse.params.dt;
else
  Kl = 10.43; % conversion from rad/s to leg command
  legs_v = trial.interp.Control.command(:,[1 5])'/Kl;
end

leg_error = legs_v*trial.params.Kcrank - effectiveLegs;
