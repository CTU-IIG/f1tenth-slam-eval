function vicon_acceleration_callback(src,event)
% callback that sorts vicon acceleration stamped data to one cell array

    global vicon_acceleration;
    s = length(vicon_acceleration);
    vicon_acceleration{s+1} = event;
end