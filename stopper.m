function stop=stopper(y,optimValues,state,time_per_it)
if state=='init'
    tic
    stop=0;
else
    time=toc;
    if time>=time_per_it
        stop=1;
    else
        stop=0;
    end
end
end