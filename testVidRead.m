function testVidRead(vobj)

timer_count = 0;

function increment(obj,event)
    timer_count = timer_count + 1;
end

update_timer = timer('TimerFcn',{@increment},'Period',0.01,...
    'ExecutionMode','fixedRate','BusyMode','drop');

tic;

start(update_timer)

while(toc < 1)
    pause(0.1)
    read(vobj.vid,[1 30]);
    continue;
end

stop(update_timer)

timer_count
end