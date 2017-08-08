function legs = expectedLegV(trial, vw)

legs(1,:) = (vw(1,:) - vw(2,:) * trial.params.d/2)/trial.params.Kcrank;
legs(2,:) = (vw(1,:) + vw(2,:) * trial.params.d/2)/trial.params.Kcrank;