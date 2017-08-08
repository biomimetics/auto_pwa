function X = xsmoothToFullState(trial,t_range)

if nargin < 2
    t_range = 1:size(trial.fuse.xsmooth,2);
end

% xsmooth to full state indices
% xsmooth
% [x_w,y_w,z_w,x_w',y_w',z_w',x_r",y_r",z_r",...
%  yw_w,p_w,r_w,r_r',p_r',yw_r',a_l,a_r]
%
%full state
% [x_w,y_w,z_w, r_w,p_w,yw_w, a_l,a_r, ...
%  x_r',y_r',z_r', r_r',p_r',yw_r', a_l',a_r']

xsm2f_idx = [1 2 3 12 11 10 16 17 4 5 6 13 14 15];

[b,a] = butter(3,0.1);
lvel = filtfilt(b,a,trial.metrics.legs_v')';
%lvel = trial.metrics.legs_v(:,t_range);

X = [trial.fuse.xsmooth(xsm2f_idx,t_range); lvel(:,t_range)];

R = angle2dcm(X(6,:),X(5,:),X(4,:));
for i = 1:length(t_range)
    X(9:11,i) = R(:,:,i)*X(9:11,i);
end

X(7:8,:) = mod(X(7:8,:),2*pi);

