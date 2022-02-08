% Main m file for evaluating the SLAM accuracy. This file is divided into
% several sections, run one after the other.


%%
% initialization of the ROS environment

rosshutdown;
ros_master_ip = 'http://localhost:11311';
rosinit(ros_master_ip);

%%
% declaration of subscribers

global cartographer;
cartographer = {};
global vicon_pose;
vicon_pose = {};
global vicon_acceleration;
vicon_acceleration = {};
global vicon_twist;
vicon_twist = {};

cart_sub = rossubscriber('/tf/converted','nav_msgs/Odometry',@(src,event)cartographer_callback(src,event),'DataFormat','struct');
vicon_pose_sub = rossubscriber('/vrpn_client_node/mpo_sensor2/pose','geometry_msgs/PoseStamped',@(src,event)vicon_pose_callback(src,event),'DataFormat','struct');
vicon_acc_sub = rossubscriber('/vrpn_client_node/mpo_sensor2/accel','geometry_msgs/TwistStamped',@(src,event)vicon_acceleration_callback(src,event),'DataFormat','struct');
vicon_twist_sub = rossubscriber('/vrpn_client_node/mpo_sensor2/twist','geometry_msgs/TwistStamped',@(src,event)vicon_twist_callback(src,event),'DataFormat','struct');

pause(1);

%%
% remapping cell array data (received from callbacks or mat files) to matrices used in later code 

cartographer_positions = zeros(length(cartographer),2);
cartographer_orientations = zeros(length(cartographer),4);

for index = 1:length(cartographer)
    cartographer_positions(index,1) = cartographer{index}.Pose.Pose.Position.X;
    cartographer_positions(index,2) = cartographer{index}.Pose.Pose.Position.Y;

    cartographer_orientations(index,1) = cartographer{index}.Pose.Pose.Orientation.X;
    cartographer_orientations(index,2) = cartographer{index}.Pose.Pose.Orientation.Y;
    cartographer_orientations(index,3) = cartographer{index}.Pose.Pose.Orientation.Z;
    cartographer_orientations(index,4) = cartographer{index}.Pose.Pose.Orientation.W;
end

vicon_positions = zeros(length(vicon_pose),2);
vicon_orientations = zeros(length(vicon_pose),4);

for index = 1:length(vicon_pose)
    vicon_positions(index,1) = vicon_pose{index}.Pose.Position.X;
    vicon_positions(index,2) = vicon_pose{index}.Pose.Position.Y;

    vicon_orientations(index,1) = vicon_pose{index}.Pose.Orientation.X;
    vicon_orientations(index,2) = vicon_pose{index}.Pose.Orientation.Y;
    vicon_orientations(index,3) = vicon_pose{index}.Pose.Orientation.Z;
    vicon_orientations(index,4) = vicon_pose{index}.Pose.Orientation.W;
end

%%
% calculating rotation matrix from line experiment

R = calculate_rotation(cartographer_positions,vicon_positions);

%%
% calculates translation for standing experiment
V = R*vicon_positions.';
T = calculate_translation(cartographer_positions,V.');

%%

% Visualization for positions

V = R*vicon_positions.';
V = V+T.';

scatter(cartographer_positions(:,1),cartographer_positions(:,2));
hold on;
scatter(V(1,:),V(2,:));
plot(cartographer_positions(1,1),cartographer_positions(1,2),'xg','MarkerSize', 15,'LineWidth',3)
lgd = legend('Cartographer','ViCON','Beginning');
lgd.FontSize = 14;
xl = xlabel('X [m]');
xl.FontSize = 14;
yl = ylabel('Y [m]');
yl.FontSize = 14;
ax = gca;
ax.FontSize = 14;
hold off

%%

% Data analysis for stationary experiment with visualization

V = R*vicon_positions.';
V = V+T.';
vicon_mean = mean(V.');

cartographer_distances = zeros(1,length(cartographer_positions));
RMSE = 0;

for index = 1:length(cartographer_positions)
    % finds the distance squared
    sq = (cartographer_positions(index,1)-vicon_mean(1))^2+(cartographer_positions(index,2)-vicon_mean(2))^2;
    % calculates distance and stores it inside matrix
    cartographer_distances(index) = sqrt(sq);
    % add to RMSE sum
    RMSE = RMSE + sq/length(cartographer_positions);
end
% calculates final RMSE
RMSE = sqrt(RMSE);

% visualization

fprintf('Stationary (calibration) data RMSE: %f\n',RMSE);

plot(cartographer_distances)
hold on
xlim([0 length(cartographer_positions)]);
plot([0 length(cartographer_positions)],[RMSE RMSE],'--r')
lgd = legend('Cartographer Distances','RMSE');
lgd.FontSize = 14;
xl = xlabel('Sample');
xl.FontSize = 14;
yl = ylabel('Distance [m]');
yl.FontSize = 14;
ax = gca;
ax.FontSize = 14;
hold off

%%

% other experiments with calculating the RMSE using closest value

V = R*vicon_positions.';
V = V+T.';

cartographer_distances = zeros(1,length(cartographer_positions));
RMSE = 0;

for index = 1:length(cartographer_positions)
    cp = cartographer_positions(index,:);
    tmp_vicon = V.' - cp;
    
    % creates the distance matrix and finds the closes point (index of it)
    dist_matrix = (tmp_vicon(:,1).^2+tmp_vicon(:,2).^2).^0.5;
    [min_dist,min_index] = min(dist_matrix);
    
    % finds the distance squared
    sq = tmp_vicon(min_index,1).^2 + tmp_vicon(min_index,2).^2;
    % calculates distance and stores it inside matrix
    cartographer_distances(index) = sqrt(sq);
    % add to RMSE sum
    RMSE = RMSE + sq/length(cartographer_positions);
end

% calculates final RMSE
RMSE = sqrt(RMSE);
% visualization

fprintf('Average RMSE: %f, Maximal Distance is %f m\n',RMSE,max(cartographer_distances));

plot(cartographer_distances)
hold on
xlim([0 length(cartographer_positions)]);
plot([0 length(cartographer_positions)],[RMSE RMSE],'--r')
lgd = legend('Cartographer Distances','RMSE');
lgd.FontSize = 14;
xl = xlabel('Sample');
xl.FontSize = 14;
yl = ylabel('Distance [m]');
yl.FontSize = 14;
ax = gca;
ax.FontSize = 14;
hold off