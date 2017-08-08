function [xFilt,varargout] = kalmanFilterSmoother(params, robotData, y, B, C, d, u, Q, R, init_x, init_V)
%
% function [xfilt, xpred, Vfilt, loglik, xsmooth, Vsmooth, Q, R] = 
%           kf_smooth(y, A, B, C, d, u, Q, R, init_x, init_V)
%
%
% Kalman filter
% [xfilt, xpred, Vfilt] = ekf_smooth(y_all, A, B, C, d, Q, R, init_x, init_V);
%
% Kalman filter with Smoother
% [xfilt, xpred, Vfilt, loglik, xsmooth, Vsmooth] = ekf_smooth(y_all, A, B, C, d, Q, R, init_x, init_V);
%
% Kalman filter with Smoother and EM algorithm
% [xfilt, xpred, Vfilt, loglik, xsmooth, Vsmooth, Q, R] = ekf_smooth(y_all, A, B, C, d, Q, R, init_x, init_V);
%
%
% INPUTS:
% y - observations
% A, B, C, d:  x(:,t+1) = A x(:,t) + B u(:,t) + w(:,t) 
%              y(:,t)   = C x(:,t) + d        + v(:,t)
% Q - covariance matrix of system x(t+1)=A*x(t)+w(t) , w(t)~N(0,Q)
% R - covariance matrix of output y(t)=C*x(t)+v(t) , v(t)~N(0,R)
% init_x -
% init_V -
%
%
% OUTPUTS:
% xfilt = E[X_t|t]
% varargout(1) = xpred - the filtered values at time t before measurement
% at time t has been accounted for
% varargout(2) = Vfilt - Cov[X_t|0:t]
% varargout(3) = loglik - loglikelihood
% varargout(4) = xsmooth - E[X_t|0:T]
% varargout(5) = Vsmooth - Cov[X_t|0:T]
% varargout(6) = Q - estimated system covariance according to 1 M step (of EM)
% varargout(7) = R - estimated output covariance according to 1 M step (of EM)

n_var_out = max(nargout,1)-1; % number of variable number of outputs

T = size(y,2);
ss = size(Q,1); % size of state space

%% Forward pass (Filter)
% Kalman filter
% function [xfilt, xpred, Vfilt] = kf_smooth(y, A, B, C, d, u, Q, R, init_x, init_V)
%           

startTime = 1;
if ~isempty(find(startTime == robotData.interp.Optitrak.RealIdx))
    optUpdate = 1;
else
    optUpdate = 2;
end

xFilt = zeros(ss,T);
vFilt = zeros(ss,ss,T);

%Initial Measurement Update
K = init_V*C(:,:,optUpdate)'*(C(:,:,optUpdate)*init_V*C(:,:,optUpdate)'+R)^-1;
xFilt(:,startTime) = init_x + K*(y(:,startTime)-(C(:,:,optUpdate)*init_x+d));
vFilt(:,:,startTime) = (eye(ss) - K*C(:,:,optUpdate))*init_V;

for t = startTime:T-1
    % Dynamics Update
    if ~isempty(find(t == robotData.interp.Optitrak.RealIdx))
        optUpdate = 1;
    else
        optUpdate = 2;
    end
    currentXFilt = xFilt(:,t);
    currentVFilt = vFilt(:,:,t);
    A = numerical_derivative(@(x) dynamicsUpdate_int(x, params), currentXFilt);
    As(:,:,t) = A;
    currentXPred = A*currentXFilt; % + B*u(:,t);
    currentVPred = A*(currentVFilt)*A' + Q;
    xpred(:,t+1) = currentXPred; VPred(:,:,t) = currentVPred; 

    % Measurement Update 
    K = currentVPred*C(:,:,optUpdate)'*(C(:,:,optUpdate)*currentVPred*C(:,:,optUpdate)'+R)^-1;
    xFilt(:,t+1) = currentXPred + K*(y(:,t+1)-(C(:,:,optUpdate))*currentXPred+d);
    vFilt(:,:,t+1) = (eye(ss) - K*C(:,:,optUpdate))*currentVPred;
    % xFilt(10:13,t+1) = xFilt(10:13,t+1)/norm(xFilt(10:13,t+1));
end

if(n_var_out >= 1), varargout(1) = {xpred}; end
if(n_var_out >= 2), varargout(2) = {vFilt}; end
% if(n_var_out >= 3), varargout(3) = {loglik}; end


%% Backward pass (RTS Smoother and EM algorithm)

% Kalman filter with Smoother
%[xfilt, xpred, Vfilt, loglik, xsmooth, Vsmooth, Q, R] = kf_smooth(y, A, B, C, d, u, Q, R, init_x, init_V); % YOURS to implement

if(n_var_out >= 4)
    xsmooth(:,T) = xFilt(:,T);
    Vsmooth(:,:,T) = vFilt(:,:,T);
    preQ=[];    preR=[]; 
    
    for t = (T-1):-1:startTime
        %E step      
        if ~isempty(find(t == robotData.interp.Optitrak.RealIdx))
            optUpdate = 1;
        else
            optUpdate = 2;
        end
        L(:,:,t) = vFilt(:,:,t) * As(:,:,t)' * VPred(:,:,t)^-1;            
        xsmooth(:,t) = xFilt(:,t) + L(:,:,t) * (xsmooth(:,t+1) - xpred(:,t+1));
        Vsmooth(:,:,t) = vFilt(:,:,t) + L(:,:,t)*(Vsmooth(:,:,t+1) - VPred(:,:,t))*L(:,:,t)';
        % xsmooth(10:13,t+1) = xsmooth(10:13,t+1)/norm(xsmooth(10:13,t+1));

        %M Step
        preQ(:,:,t) = (xsmooth(:,t+1) - As(:,:,t) * xsmooth(:,t))*(xsmooth(:,t+1) - As(:,:,t) * xsmooth(:,t))'...
            +As(:,:,t)*Vsmooth(:,:,t)*As(:,:,t)'+Vsmooth(:,:,t+1)-Vsmooth(:,:,t+1)*L(:,:,t)'*As(:,:,t)'-As(:,:,t)*L(:,:,t)*Vsmooth(:,:,t+1);
        preR(:,:,t) = (y(:,t)-C(:,:,optUpdate)*xsmooth(:,t))*(y(:,t)-C(:,:,optUpdate)*xsmooth(:,t))'+C(:,:,optUpdate)*Vsmooth(:,:,t)*C(:,:,optUpdate)';
        
            %Log-Likelihood Calculation
            ubar = C(:,:,optUpdate)*xpred(:,t) + d;      
            Vbar = C(:,:,optUpdate)*VPred(:,:,t)*C(:,:,optUpdate)' + R;
            logsum(t) = log( (2*pi^(2/2)*det(Vbar)^0.5)^-1 * exp( -0.5*(y(:,t)-ubar)'*Vbar^-1*(y(:,t)-ubar)) );

            
    end
    %Take care of end cases
    preR(:,:,T) = (y(:,T)-C(:,:,1)*xsmooth(:,T))*(y(:,T)-C(:,:,1)*xsmooth(:,T))'+C(:,:,1)*Vsmooth(:,:,T)*C(:,:,1)';
    preR(:,:,T+1) = R;
    preQ(:,:,T) = Q;    %Initial value
  
    Q = 1/T*sum(preQ,3);
    R = 1/(T+1)*sum(preR,3);
    loglik = sum(logsum);
    
   if(n_var_out >= 4), varargout(3) = {loglik}; end
   varargout(4) = {xsmooth};
   if(n_var_out >= 5), varargout(5) = {Vsmooth}; end
   if(n_var_out >= 6), varargout(6) = {Q}; end
   if(n_var_out == 7), varargout(7) = {R}; end
end


















