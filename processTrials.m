function yaw = processTrials(directory)

folders = dir(directory);

t = 1;
for f = 1:length(folders)
  foldername = folders(f).name;
  if strncmp(foldername,'.',1) || ~strcmp(foldername((end-8):(end-4)),'_clip')
    continue
  end
  trialname = foldername(1:29)
  filename = sprintf('./%s/%s', directory, foldername);
%  statsname = sprintf('./%s/%s_stats.mat',directory, trialname);
%   trialname = sprintf('Trial%s',foldername)
%   filename = ;
%   statsname = sprintf('./%s/%s.mat',directory, foldername);
  load(filename)
%   metrics_calculated = 0;
%   if eval(['~isfield(' trialname ',''metrics'');'])
    eval(['angle = extractTrialState(' trialname ',''euler_i'');']);
    yaw{t} = angle(1,:);
 %   save(modelsname,'M')
%    clear M
%     metrics_calculated = 1;
%   end
  
%   % treadmill speed
%   eval(['stats.vt = ' trialname '.interp.Control.treadSpeed(floor(length('...
%     trialname '.interp.t)/2),1);']);
%   stats.vt = round(stats.vt*100)/100;
%   
%   % control gains
%   eval(sprintf('stats.gains = [%s.params.Kx; %s.params.Ky; %s.params.Ktheta];',...
%      trialname,trialname,trialname));
% 
%   eval(['stats.data = extractTrialState(' trialname ...
%     ',{''legs_phase'',''X_i'',''euler_i'',''energy''});']);
%   
%   save(statsname, 'stats')
%   clear stats
  
%   if metrics_calculated
%     save(filename, trialname)
%   end
  clear(trialname)
  t = t+1;
end