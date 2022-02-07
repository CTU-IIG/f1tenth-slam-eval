function T = calculate_translation(cartographer_positions,vicon_positions)
% function that calculated the translation of the vicon data to the
% cartographer frame

cart_mean = mean(cartographer_positions);
vicon_mean = mean(vicon_positions);
T = cart_mean - vicon_mean;
end