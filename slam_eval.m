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

display('Init complete');

pause(1);

%%
% remapping cell array data to matrix

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
c1 = cartographer_positions(1,:);
c2 = cartographer_positions(end,:);

v1 = vicon_positions(1,:);
v2 = vicon_positions(end,:);

psi = atan2(c2(2)-c1(2),c2(1)-c1(1));
theta = atan2(v2(2)-v1(2),v2(1)-v1(1));
phi = psi-theta;

R = [cos(phi) -sin(phi);sin(phi) cos(phi)];
v = v2-v1;
f = R*v.';
f = f + v1;

plot([c1(1) c2(1)],[c1(2) c2(2)],'o-b')
hold on
plot([v1(1) v2(1)],[v1(2) v2(2)],'o-r')
plot([v1(1) f(1)],[v1(2) f(2)],'o-m')

%%
% for standing (finds translation)
V = R*vicon_positions.';
T = calculate_translation(cartographer_positions,V.');
V = V+T.';

scatter(cartographer_positions(:,1),cartographer_positions(:,2));
hold on;
scatter(V(1,:),V(2,:));
scatter(vicon_positions(:,1)+T(1),vicon_positions(:,2)+T(2));
legend('cartographer','vicon','unrotated')
xlabel('X');
ylabel('Y');
hold off

%%

V = R*vicon_positions.';
V = V+T.';

scatter(cartographer_positions(:,1),cartographer_positions(:,2));
hold on;
scatter(V(1,:),V(2,:));
legend('cartographer','vicon')
xlabel('X');
ylabel('Y');
hold off

%%

% Stationary experiment (calibration data analysis)

V = R*vicon_positions.';
V = V+T.';
vicon_mean = mean(V.');

vicon_distances = zeros(1,length(V));
cartographer_distances = zeros(1,length(cartographer_positions));
RMSE = 0;

for index = 1:length(V)
    vicon_distances(index) = sqrt((V(1,index)-vicon_mean(1))^2+(V(2,index)-vicon_mean(2))^2);
end

for index = 1:length(cartographer_positions)
    sq = (cartographer_positions(index,1)-vicon_mean(1))^2+(cartographer_positions(index,2)-vicon_mean(2))^2;
    cartographer_distances(index) = sqrt(sq);
    RMSE = RMSE + sq/length(cartographer_positions);
end
RMSE = sqrt(RMSE);

fprintf('Stationary (calibration) data RMSE: %f\n',RMSE);

plot(cartographer_distances)
hold on
xlim([0 length(cartographer_positions)]);
plot([0 length(cartographer_positions)],[RMSE RMSE],'--r')
legend('cartographer distances','RMSE');
xlabel('samples');
ylabel('distance [m]')

%%

% other experiments with calculating the RMSE using closest value

V = R*vicon_positions.';
V = V+T.';

cartographer_distances = zeros(1,length(cartographer_positions));
RMSE = 0;

for index = 1:length(cartographer_positions)
    cp = cartographer_positions(index,:);
    tmp_vicon = V.' - cp;

    dist_matrix = (tmp_vicon(:,1).^2+tmp_vicon(:,2).^2).^0.5;

    [min_dist,min_index] = min(dist_matrix);

    sq = tmp_vicon(min_index,1).^2 + tmp_vicon(min_index,2).^2;
    cartographer_distances(index) = sqrt(sq);
    RMSE = RMSE + sq/length(cartographer_positions);
end

RMSE = sqrt(RMSE);

fprintf('Average RMSE: %f\n',RMSE);

plot(cartographer_distances)
hold on
xlim([0 length(cartographer_positions)]);
plot([0 length(cartographer_positions)],[RMSE RMSE],'--r')
legend('cartographer distances','RMSE');
xlabel('samples');
ylabel('distance [m]')