function [widths, signs] = find_widths(time, signal)

der = diff(signal);

widths = zeros(1,sum(der ~= 0)-1);
signs = zeros(size(widths));

events = der ~= 0;
last_idx = find(events,1);

out_idx = 1;
while last_idx < length(der) && out_idx <= length(widths)
  next_idx = last_idx + 1;
  while der(next_idx) == 0
    next_idx = next_idx + 1;
  end
  widths(out_idx) = time(next_idx) - time(last_idx);
  signs(out_idx) = signal(last_idx+1);
  last_idx = next_idx;
  out_idx = out_idx + 1;
end
