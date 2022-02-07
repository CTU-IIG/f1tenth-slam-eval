function vicon_twist_callback(src,event)
% callback that sorts vicon acceleration stamped data to one cell array

    global vicon_twist;
    s = length(vicon_twist);
    vicon_twist{s+1} = event;
end