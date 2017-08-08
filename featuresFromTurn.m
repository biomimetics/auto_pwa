function [full_state, sample_periods] = featuresFromTurn(trial, turn)

peak_idx = turn.maxYaw(2);

[full_state, sample_periods] = extractLinearFeatures(trial,peak_idx + (-300:300));