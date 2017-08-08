function D = numerical_derivative(fun, state)

delta = 1e-6;
n_states = size(state,1);

Y = fun(state);
Y = Y(:);
D = zeros(size(Y,1), n_states);

for i = 1:n_states 
    Y0 = fun(state-delta*(1:n_states==i)');
    Y1 = fun(state+delta*(1:n_states==i)');
    D(:,i) = (Y1(:)-Y0(:))/(2*delta);
end