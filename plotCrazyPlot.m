function plotCrazyPlot(M)

k10idx = M.idx{4};
n_c10 = max(k10idx);
k50idx = M.idx{5};
n_c50 = max(k50idx);

X = M.D.X;
u = M.D.u;
Xdot = M.D.X_dot;

xc = mean(X,2);
n_x = size(X,1);

i_phase = 10:11;
i_angle = 2:3;

k10c = NaN*ones(n_x,n_c10);
k50c = NaN*ones(n_x,n_c50);

for c = 1:n_c10
  k10c(:,c) = mean(X(:,k10idx==c),2);
end

for c = 1:n_c50
  k50c(:,c) = mean(X(:,k50idx==c),2);
end

turnIdx = (735-100):(885-50);
tripodIdx = 39210:39390;

% figure(1)
% hold on
% 
% colors = hsv(n_c10);
% for i = 1:n_c10
%   plot(X(i_phase(1),k10idx==i),X(i_phase(2),k10idx==i),...
%     'Color', colors(i,:),'MarkerSize',4,...
%     'Marker', '.', 'LineStyle', 'none')
% end
% 
% zH = plot(xc(i_phase(1)),xc(i_phase(2)),'k*','MarkerSize',10,'LineWidth',2);
% k10H = plot(k10c(i_phase(1),:),k10c(i_phase(2),:),'k+',...
%   'MarkerSize',10,'LineWidth',2);
% k50H = plot(k50c(i_phase(1),:),k50c(i_phase(2),:),'ko',...
%   'MarkerSize',6,'LineWidth',2);
% 
% tuH = plot(X(i_phase(1),turnIdx),X(i_phase(2),turnIdx),'k','LineWidth',2);
% trH = plot(X(i_phase(1),tripodIdx),X(i_phase(2),tripodIdx),'k--',...
%   'LineWidth',2);
% 
% axis tight
% xlabel('alpha_l')
% ylabel('alpha_r')
% legend([zH,k10H,k50H,tuH,trH],{'Z-1.5','K-10','K-50','turn','tripod'})
% 
% figure(2)
% hold on
% colors = hsv(n_c10);
% for i = 1:n_c10
%   plot3(X(i_angle(1),k10idx==i),X(i_angle(2),k10idx==i),X(1,k10idx==i),...
%     'Color', colors(i,:),...
%     'Marker', '.', 'LineStyle', 'none')
% end
% plot(xc(i_angle(1)),xc(i_angle(2)),'k+')
% 
% plot(k10c(i_angle(1),:),k10c(i_angle(2),:),'b*')
% plot(k50c(i_angle(1),:),k50c(i_angle(2),:),'g.')
% plot(X(i_angle(1),turnIdx),X(i_angle(2),turnIdx),'r.-')
% plot(X(i_angle(1),tripodIdx),X(i_angle(2),tripodIdx),'m.-')
% axis tight
% xlabel('theta')
% ylabel('phi')
% legend('mean','K-10','K-50','turn','tripod')
% 
% figure(3)
% plot(Xdot(3,tripodIdx),'g')
% hold on
% 
notTripodIdx = [1:(tripodIdx(1)-1) (tripodIdx(end)+1):size(X,2)];
tripodDtrain.X = X(:,notTripodIdx);
tripodDtrain.u = u(:,notTripodIdx);
tripodDtrain.X_dot = Xdot(:,notTripodIdx);
tripodDtest.X = X(:,tripodIdx);
tripodDtest.u = u(:,tripodIdx);

Mk10 = buildModel(tripodDtrain, ...
  @(D) cloneSegmenter(D,M.F{4}.P,k10idx(notTripodIdx)));

% Xdot10 = evaluateModel(Mk10,tripodDtest,k10idx(tripodIdx));
k10switch = find([0 diff(k10idx(tripodIdx))~=0]);
% plot(Xdot10(3,:),'b')
% plot(k10switch,Xdot10(3,k10switch),'b*')

Mk50 = buildModel(tripodDtrain, ...
  @(D) cloneSegmenter(D,M.F{5}.P,k50idx(notTripodIdx)));
% Xdot50 = evaluateModel(Mk50,tripodDtest,k50idx(tripodIdx));
k50switch = find([0 diff(k50idx(tripodIdx))~=0]);
% plot(Xdot50(3,:),'r')
% plot(k50switch,Xdot50(3,k50switch),'r*')
% 
% xlim([0 length(tripodIdx)])
% legend('Measured','K-10','K-10 Switch','K-50','K-50 Switch')
% xlabel('Time (ms)')
% yl = ylabel('$\ddot{z} \; (m/s^2)$');
% set(yl,'interpreter','latex')
% tt = title('$\ddot{z}$ Estimate and Hybrid Transitions, Alternating Tripod');
% set(tt,'interpreter','latex')

figure(5)
X0 = simpleToFullState(tripodDtest.X(:,1));
sim_time = length(tripodIdx);
odeU = tripodDtest.u;

plot(1:sim_time,tripodDtest.X(6,1:sim_time),'g')
hold on

odeModel = Mk10;

odeIdx = k10idx(tripodIdx);
% t_span = 1:length(tripodIdx);
% [tout,xout] = ode45(@(t,x) modelODE(t,x,odeIdx,odeModel,odeU),t_span,X0);
xout = zeros(size(X0,1),sim_time);
xout(:,1) = X0;
for t = 2:sim_time
  xout(:,t) = xout(:,t-1) +  ...
    modelODE(t,xout(:,t-1),odeIdx,odeModel,odeU) * 0.001;
  xout(7:8,t) = mod(xout(7:8,t),2*pi);
  %xout(:,t)
end
plot(1:sim_time,xout(11,1:sim_time),'b')
plot(k10switch,xout(11,k10switch),'b*')

odeModel = Mk50;
odeIdx = k50idx(tripodIdx);
% t_span = 1:length(tripodIdx);
% [tout,xout] = ode45(@(t,x) modelODE(t,x,odeIdx,odeModel,odeU),t_span,X0);
xout = zeros(size(X0,1),sim_time);
xout(:,1) = X0;
for t = 2:sim_time
  xout(:,t) = xout(:,t-1) +  ...
    modelODE(t,xout(:,t-1),odeIdx,odeModel,odeU) * 0.001;
  xout(7:8,t) = mod(xout(7:8,t),2*pi);
  %xout(:,t)
end
plot(1:sim_time,xout(11,1:sim_time),'r')
plot(k50switch,xout(11,k50switch),'r*')

legend('Measured','K-10','K-10 Switch','K-50','K-50 Switch')
xlabel('Time (ms)')
ylabel('z'' (m/s)');
title('Forward simulation of Z velocity')

% figure(4)
% plot(Xdot(6,turnIdx),'g')
% hold on
% 
notTurnIdx = [1:(turnIdx(1)-1) (turnIdx(end)+1):size(X,2)];
turnDtrain.X = X(:,notTurnIdx);
turnDtrain.u = u(:,notTurnIdx);
turnDtrain.X_dot = Xdot(:,notTurnIdx);
turnDtest.X = X(:,turnIdx);
turnDtest.u = u(:,turnIdx);
% 
% Mk10 = buildModel(turnDtrain, ...
%   @(D) cloneSegmenter(D,M.F{4}.P,k10idx(notTurnIdx)));
% Xdot10 = evaluateModel(Mk10,turnDtest,k10idx(turnIdx));
% k10switch = find([0 diff(k10idx(turnIdx))~=0]);
% plot(Xdot10(6,:),'b')
% plot(k10switch,Xdot10(6,k10switch),'b*')
% 
% Mk50 = buildModel(turnDtrain, ...
%   @(D) cloneSegmenter(D,M.F{5}.P,k50idx(notTurnIdx)));
% Xdot50 = evaluateModel(Mk50,turnDtest,k50idx(turnIdx));
% k50switch = find([0 diff(k50idx(turnIdx))~=0]);
% plot(Xdot50(6,:),'r')
% plot(k50switch,Xdot50(6,k50switch),'r*')
% 
% legend('Measured','K-10','K-10 Switch','K-50','K-50 Switch')
% xlabel('Time (ms)')
% 
% xlim([0 length(turnIdx)])
% yl = ylabel('$\ddot{z} \; (m/s^2)$');
% set(yl,'interpreter','latex')
% tt = title('$\ddot{\psi}$ Estimate and Hybrid Transitions, Aggresive Turn');
% set(tt,'interpreter','latex')

figure(6)
X0 = simpleToFullState(turnDtest.X(:,1));
sim_time = length(turnIdx);
odeU = turnDtest.u;

plot(1:sim_time,turnDtest.X(9,1:sim_time),'g')
hold on

odeModel = Mk10;

odeIdx = k10idx(turnIdx);
xout = zeros(size(X0,1),sim_time);
xout(:,1) = X0;
for t = 2:sim_time
  xout(:,t) = xout(:,t-1) +  ...
    modelODE(t,xout(:,t-1),odeIdx,odeModel,odeU) * 0.001;
  xout(7:8,t) = mod(xout(7:8,t),2*pi);
end
plot(1:sim_time,xout(14,1:sim_time),'b')
plot(k10switch,xout(14,k10switch),'b*')

odeModel = Mk50;
odeIdx = k50idx(turnIdx);
xout = zeros(size(X0,1),sim_time);
xout(:,1) = X0;
for t = 2:sim_time
  xout(:,t) = xout(:,t-1) +  ...
    modelODE(t,xout(:,t-1),odeIdx,odeModel,odeU) * 0.001;
  xout(7:8,t) = mod(xout(7:8,t),2*pi);
  %xout(:,t)
end
plot(1:sim_time,xout(14,1:sim_time),'r')
plot(k50switch,xout(14,k50switch),'r*')

legend('Measured','K-10','K-10 Switch','K-50','K-50 Switch')
xlabel('Time (ms)')
ylabel('yaw'' (rad/s)');
title('Forward simulation of yaw velocity')

end

function [P,idx] = cloneSegmenter(D,P,idx)
end

function Xsimple = fullToSimpleState(Xfull)
  Xsimple = Xfull([3 5 6 9:14 7 8 15 16]);
end

function Xfull = simpleToFullState(Xsimple)
  Xfull = zeros(16,1);
  Xfull([3 5 6 9:14 7 8 15 16]) = Xsimple;
end

function X_dot = modelODE(t,X,idx,F,u)
  i = idx(floor(t));
  A = F.A{i};
  B = F.B{i};
  f = F.f{i};
  q_dot = X(9:end);
  q_ddot = A*fullToSimpleState(X) + f + B*u(:,floor(t));
  X_dot = [q_dot; q_ddot];
end