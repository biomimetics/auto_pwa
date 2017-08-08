function plotVWSpeedTrend(vDot,wDot)

vt = 0.1:0.05:0.5;

plot(vt,vDot(1,:),'b--*',vt,vDot(2,:),'b-*')
hold on
plot(vt,wDot(1,:),'g--o',vt,wDot(2,:),'g-o')
l = legend('K-10 $\ddot{x}$','K-50 $\ddot{x}$','K-10 $\ddot{\psi}$',...
  'K-50 $\ddot{\psi}$');
set(l,'interpreter','latex');

axis([0.1 0.5 0 1])


title('Model Variance Reduction vs. Treadmill Speed')
xlabel('Treadmill Speed (m/s)')
ylabel('Variance Ratio')