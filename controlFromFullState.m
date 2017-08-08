function u = controlFromFullState(X,trial)

x = X(1,:);
y = X(2,:);
theta = X(6,:);
Vt = median(trial.Control.treadSpeed(:,1));

Kx = trial.params.Kx;
Ky = trial.params.Ky;
Kt = trial.params.Ktheta;
Ks = trial.params.Ks;
Ka = trial.params.Ka;
d = trial.params.d;

u = [(-Kx*x + Ks*Vt)./(100*cos(Ka*theta)); -d/pi*(Kt*theta + Ky*y)];