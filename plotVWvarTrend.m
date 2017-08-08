function plotVWvarTrend(T,yaw)

trial.params.Kcrank = 0.0102;
trial.params.d = 0.065;

n_t = length(T);
vw_err = [];

vt = 0.1:0.05:0.5;

for t = 1:n_t
  legs_v = T{t}.D.X(12:13,:);
  legs_vw = expectedControl(trial,legs_v);
  vw = T{t}.D.X([4 9],:);
  vxt = [cos(yaw{t})*vt(t);zeros(1,size(vw,2))];
  vw_err = [vw_err [vw + vxt - legs_vw; vt(t)*ones(1,size(vw,2))]];
end

figure
boxplot(vw_err(1,:),vw_err(3,:),'symbol','r.','whisker',1.8);
title('Velocity Error vs Treadmill Speed')
ylabel('V_{robot} - V_{legs} (m/s)')
xlabel('Treadmill Velocity (m/s)')

figure
boxplot(vw_err(2,:),vw_err(3,:),'symbol','r.','whisker',1.8);
title('Angular Rate Error vs Treadmill Speed')
ylabel('\omega_{robot} - \omega_{legs} (rad/s)')
xlabel('Treadmill Velocity (m/s)')

% plot(vt,vw(:,1),'b--.',vt,legs(:,1),'b-+')
% hold on
% plot(vt,vw(:,2),'g--.',vt,legs(:,2),'g-+')
% l = legend('$\dot{x}$', '$\dot{x}_{legs}$','$\dot{\psi}$','$\dot{\psi}_{legs}$');
% set(l,'interpreter','latex');
% 
% axis([0.1 0.5 0 1])
% 
% title('Control Normed Variance vs. Treadmill Speed')
% xlabel('Treadmill Speed (m/s)')
% ylabel('Variance Ratio')