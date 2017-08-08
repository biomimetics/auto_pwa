function A = linearModel(state)

next_state = state(:,2:end);
state = state(:,1:(end-1));

A = next_state * state' * (state * state')^-1;