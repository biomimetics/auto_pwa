function [X,X_dot,A,B,f,u,idx] = hybridDiffDrive(u)

if nargin < 1
  t = 1:0.01:10;
  u = [0.5+sin(t); -0.2+cos(2*t)];
end

T = size(u,2);

J = 3;
m = 4;
k_x = 0.5;
k_phi = 0.6;
mu_x = 0.7;
mu_phi = 0.8;

%x, phi, x_dot, phi_dot
x_0 = [0;0;0;0];

X = zeros(4,size(u,2));
idx = zeros(1,size(u,2));
X_dot = X;
X(1:4,1) = x_0;

A{1} = [0 0 1 0; 0 0 0 1; 0 0 -mu_x/m 0; 0 0 0 -mu_phi/J];
A{2} = A{1};
f{1} = [0; 0; 0; 0];
f{2} = [-1; 1; 0; 0];
B{1} = [0 0; 0 0; k_x 0; 0 k_phi];
B{2} = B{1};

for t = 1:T
  i = (X(3,t)>0)+1;
  idx(t) = i;
  X_dot(:,t) = A{i}*X(:,t) + f{i} + B{i}*u(:,t);
  if t < T
    X(:,t+1) = X(:,t) + X_dot(:,t);
  end
end

idx = (idx == 2);