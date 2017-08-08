function Xr = fullToRegionState(X)

%full state (input X)
% [x_w,y_w,z_w, r_w,p_w,yw_w, a_l,a_r, ...
%  x_r',y_r',z_r', r_r',p_r',yw_r', a_l',a_r']

% model region state (Xr, used for segmentation and derivative calculation)
% [z_w,p_w,r_w, x_r',y_r',z_r', r_r',p_r',yw_r', a_l,a_r,a_l',a_r']
Xr = X([3 5 4 9:14 7 8 15 16],:);
Xr(10:11,:) = mod(Xr(10:11,:),2*pi);