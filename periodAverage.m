function avg = periodAverage(full_state,sample_periods)

n_period = max(sample_periods);
avg = zeros(size(full_state,1),n_period);

for p = 1:n_period
  avg(:,p) = mean(full_state(:,sample_periods==p),2);
end