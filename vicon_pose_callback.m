function vicon_pose_callback(src,event)
    % callback that sorts vicon pose stamped data to one cell array

    global vicon_pose;
    s = length(vicon_pose);
    vicon_pose{s+1} = event;
end