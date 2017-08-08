function [X_dot,r] = kmeansDerivative(X,u,F,K,r)

if nargin < 5
    r = kmeansRegion(X,K);
end

Xr = fullToRegionState(X);
X_ddot = F.A{r}*Xr + F.B{r}*u + F.f{r};

% Rotate robot frame velocities to world frame
R = angle2dcm(X(6),X(5),X(4));
X_dot = [R'*X(9:11); R'*X(12:14); X(15:16); X_ddot];