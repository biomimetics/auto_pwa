function stride_average = strideAverage(trial, data)

% find average of each input signal with respect to each of the legs
stride_average = zeros([2 size(data)]);
stride = trial.metrics.leg_stride;

for s = 1:max(max(stride))
  stride_average(1,:,stride(1,:) == s) = mean(data(:,stride(1,:)==s),2);
  stride_average(2,:,stride(2,:) == s) = mean(data(:,stride(2,:)==s),2);
end
