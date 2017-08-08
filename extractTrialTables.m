function [varTable, methodTable, vDot, wDot, varTex, methodTex] = extractTrialTables(T)

n_t = length(T);
varTable = zeros(4,8);
varTable(2,:) = Inf*ones(1,8);
varTable(4,:) = -Inf*ones(1,8);
varRows = {'$\sigma$', 'Min Ratio', 'Avg Ratio', 'Max Ratio'};
varCols = {'$\ddot{x}$','$\ddot{y}$','$\ddot{z}$',...
  '$\ddot{\psi}$','$\ddot{\theta}$','$\ddot{\phi}$',...
  '$\ddot{\alpha_l}$','$\ddot{\alpha_r}$'};
  
methodTable = zeros(6,4);
methodTable([1 4],:) = Inf*ones(2,4);
methodTable([3 6],:) = -Inf*ones(2,4);
methodCols = {'Avg','Z-1.5','K-10', 'K-50'};
methodRows = {'$\ddot{x}$ Min Ratio', '$\ddot{x}$ Avg Ratio',...
  '$\ddot{x}$ Max Ratio', '$\ddot{\psi}$ Min Ratio', '$\ddot{\psi}$ Avg Ratio',...
  '$\ddot{\psi}$ Max Ratio'};

%k-10,k-50; n
vDot = zeros(2,n_t);
wDot = zeros(2,n_t);

for t = 1:n_t
  % Average performance of methods and normalize against variable variance
  n_m = length(T{t}.vars);
  n_x = size(T{t}.vars{1},1);
  data = zeros(n_x,n_m);
  data(:,1) = var(T{t}.D.X_dot,0,2);
  for m = 2:n_m
    validModels = all(~isnan(T{t}.vars{m}),1);
    data(:,m) = 1-(mean(T{t}.vars{m}(:,validModels),2)./data(:,1));
  end
  
  % Average performance of k-10 and k-50 at current speed on predicting
  % v and omega
  vDot(:,t) = data(1,4:5)';
  wDot(:,t) = data(4,4:5)';
  
  % Performance of k-50 across variables. Also collect average sigma
  varTable(1,:) = varTable(1,:) + sqrt(data(:,1))'/n_t;
  k50Ratio = data(:,5)';
  varTable(2,:) = min(varTable(2,:), k50Ratio);
  varTable(3,:) = varTable(3,:) + k50Ratio/n_t;
  varTable(4,:) = max(varTable(4,:), k50Ratio);
  
  segXddot = data(1,2:end);
  methodTable(1,:) = min(methodTable(1,:), segXddot);
  methodTable(2,:) = methodTable(2,:) + segXddot/n_t;
  methodTable(3,:) = max(methodTable(3,:), segXddot);
  
  segPsiddot = data(4,2:end);
  methodTable(4,:) = min(methodTable(4,:), segPsiddot);
  methodTable(5,:) = methodTable(5,:) + segPsiddot/n_t;
  methodTable(6,:) = max(methodTable(6,:), segPsiddot);
end

varTex = mat2tex(varTable,varRows,varCols);
methodTex = mat2tex(methodTable,methodRows,methodCols);

function l = mat2tex(m,rows,cols)

tableStr = ['\\begin{table}\n'...
'  \\begin{tabular}{ %s } \\hline\n'...
'    %s \\\\ \\hline\n'...
'%s \n'...
'    \\hline\n'...
'  \\end{tabular}\n'...
'\\end{table}\n'];

colTypes = '| r |';
colHeader = '';
for c = 1:length(cols)
  colTypes = [colTypes ' c |'];
  colHeader = [colHeader ' & ' cols{c}];
end

body = '';
for r = 1:length(rows)
  body = [body '    ' rows{r}];
  for c = 1:length(cols)
    body = [body ' & ' sprintf('%.2f',m(r,c))];
  end
  if r ~= length(rows)
    body = [body sprintf(' \\\\ \n')];
  end
end

l = sprintf(tableStr,colTypes,colHeader,body);
  
  