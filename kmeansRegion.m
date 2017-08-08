function [r_idx,r_zdist] = kmeansRegion(Xr,K)

% If input is full state, convert to region state
if size(Xr,1) == 16
    Xr = fullToRegionState(Xr);
end

%Zscore normalize vector to compare to means in region space 
Xrz = bsxfun(@minus, Xr, K.xr_means);
Xrz = bsxfun(@rdivide, Xrz, K.xr_stds);

r_idx = zeros(1,size(Xrz,2));
r_zdist = zeros(size(r_idx));

for i = 1:length(r_idx)
    zerr = bsxfun(@minus, K.z_centers, Xrz(:,i));
    [d, r] = min(sqrt(sum(zerr.^2,1)));
    r_idx(i) = r;
    r_zdist(i) = d;
end