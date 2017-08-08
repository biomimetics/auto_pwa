function animation = plotFrontSideVid(animation, vobj, time)

[fimg, animation.vid.front.buf] = ...
    bufferedVidRead(animation.vid.front.buf, time, vobj.front);
[simg, animation.vid.side.buf] = ...
    bufferedVidRead(animation.vid.side.buf, time, vobj.side);

if ~isfield(animation.vid,'fig')
    animation.vid.fig = figure(2);
    
    animation.vid.front.ax = subplot(2,1,1);    
    animation.vid.front.img = imshow(fimg,'Parent',animation.vid.front.ax);
    
    animation.vid.side.ax = subplot(2,1,2);
    animation.vid.side.img = imshow(simg,'Parent',animation.vid.side.ax);
else
    set(animation.vid.front.img,'CData',fimg);
    set(animation.vid.side.img,'CData',simg);
end

title(animation.vid.front.ax, sprintf('Frame %d',...
    animation.vid.front.buf.frame_idx))
title(animation.vid.side.ax, sprintf('Frame %d',...
    animation.vid.side.buf.frame_idx))