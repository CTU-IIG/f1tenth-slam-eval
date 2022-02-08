function R = calculate_rotation(cartographer_positions,vicon_positions)

    % calculates rotation from first and second point
    
    c1 = cartographer_positions(1,:);
    c2 = cartographer_positions(end,:);
    
    v1 = vicon_positions(1,:);
    v2 = vicon_positions(end,:);
    
    psi = atan2(c2(2)-c1(2),c2(1)-c1(1));
    theta = atan2(v2(2)-v1(2),v2(1)-v1(1));
    phi = psi-theta;
    
    R = [cos(phi) -sin(phi);sin(phi) cos(phi)];
end