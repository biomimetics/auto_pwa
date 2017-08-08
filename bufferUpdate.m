function state = bufferUpdate(state,vobj)

if ~isfield(state,'frame_idx')
    state.buf_size = 30;
    state.update_size = 10;
    
    state.frame_idx = 1;
    state.buf_idx = 1;
    state.buffer = read(vobj.vid,[1 state.buf_size]);
    state.update = 0;
end

if ~state.update && state.frame_idx < state.buf_idx || ...
        state.frame_idx - state.buf_idx > state.update_size
    
    read_range = [state.frame_idx state.frame_idx+state.buf_size];
    read_range(2) = min(read_range(2),vobj.vid.NumberOfFrames);
    state.new_buffer = read(vobj.vid,read_range);
    state.new_buf_idx = state.frame_idx;
    state.update = 1;
end