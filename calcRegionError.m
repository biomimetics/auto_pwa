function R = calcRegionError(T)
% output error vectors in original space

Exr = cov(T.D.X',1);
%R.Exr = Exr;

for m = 1:length(T.F)
    R.K{m} = kmeansFromTrial(T,m);
    [r_idx, r_zdist] = kmeansRegion(T.D.X,R.K{m});
    R.idx{m} = r_idx;
    %R.xz_dist{m} = r_zdist;
    
    % Calculate region centers in feature space
    Cr = bsxfun(@times, R.K{m}.z_centers, R.K{m}.xr_stds);
    Cr = bsxfun(@plus, Cr, R.K{m}.xr_means);
    xr_err = Cr(:,r_idx) - T.D.X;
    
    % Calculate Mahalanobis distance of the error
    R.xr_dist{m} = sqrt(sum(xr_err.* (Exr^-1 * xr_err),1));
end

R.pred = znormPredictionError(T);