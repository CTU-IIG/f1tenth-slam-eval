function cartographer_callback(src,event)
    % callback that sorts cartographer odometry data to one cell array
    
    global cartographer;
    s = length(cartographer);
    cartographer{s+1} = event;
end