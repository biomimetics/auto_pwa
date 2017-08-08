function [gains, data] = collateStats(directory)

folders = dir(directory);

data = [];
t = 1;
for f = 1:length(folders)
  foldername = folders(f).name;
  if strncmp(foldername,'.',1) || ~strcmp(foldername((end-8):(end-4)),'stats')
    continue
  end
  
  statsname = sprintf('./%s/%s',directory, foldername);
  load(statsname)
  
  gains(t,:) = [stats.vt stats.gains'];
  t = t+1;
  data = [data [stats.vt*ones(1,size(stats.data,2)); stats.data]];
  clear stats
end