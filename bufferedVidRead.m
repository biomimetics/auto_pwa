function [frame,state] = bufferedVidRead(state,time,vobj)

n_frame = vobj.vid.NumberOfFrames;

f_idx = ceil((time-vobj.offset)*300);
f_idx = max(f_idx, 1);
f_idx = min(f_idx, n_frame);

% Store desired frame
state.frame_idx = f_idx;

% If a new buffer is ready, swap
if state.update
    state.buffer = state.new_buffer;
    state.buf_idx = state.new_buf_idx;
    state.update = 0;
end

% Determine buffer relative index
f_idx = f_idx-state.buf_idx+1;
f_idx = max(f_idx,1);
f_idx = min(f_idx,state.buf_size);

frame = state.buffer(:,:,:,f_idx);