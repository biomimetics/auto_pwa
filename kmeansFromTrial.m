function K = kmeansFromTrial(T,m_idx)

if nargin < 2
    n_idx = 4;
end

n_r = max(T.idx{m_idx});

%use normalization by N since we did that for segmentation
[Xrz,means,stds] = zscore(T.D.X',1);
Xrz = Xrz';
K.xr_means = means';
K.xr_stds = stds';
n_x = size(Xrz,1);

K.z_centers = zeros(n_x,n_r);
for r = 1:n_r
    K.z_centers(:,r) = sum(Xrz(:,T.idx{m_idx}==r),2)/sum(T.idx{m_idx}==r);
end