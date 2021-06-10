function [T] = lidarMapping(T, lidar_data, imu_data, gps_data, gps_prev, numPts)
    %% Transform 1
    transform_data = lidar_data;

    alpha = pi/4;

    transform_data(:,2) = transform_data(:,3)*sin(alpha);
    transform_data(:,5) = transform_data(:,6)*sin(alpha);

    transform_data(:,3) = -transform_data(:,3)*cos(alpha);
    transform_data(:,6) = -transform_data(:,6)*cos(alpha);

    %% Transform 2
    transform_data(:,1) = transform_data(:,1) + 0.9;
    transform_data(:,4) = transform_data(:,4) - 0.9;

    %% Transform 3
    rpyt = imu_data;
    
    r = rpyt(1);
    p = rpyt(2);
    y = rpyt(3);
    
    B = [1 0 0; 0 cos(p) sin(p); 0 -sin(p) cos(p)]; % rotation about x = pitch
    C = [cos(y) 0 -sin(y); 0 1 0; sin(y) 0 cos(y)]; % rotation about y = yaw
    D = [cos(r) sin(r) 0; -sin(r) cos(r) 0; 0 0 1]; % rotation about z = roll
    
    R = B*C*D;
    Rinv = inv(R);
    
    transform_data(:,1:3) = (Rinv*transform_data(:,1:3)')';
    transform_data(:,4:6) = (Rinv*transform_data(:,4:6)')';

    %% Transform 4
    xyzt = gps_data;

    xyz = xyzt(1:3);
    
    for jj=1:numPts
        transform_data(jj,1:3) = transform_data(jj,1:3) + xyz;
        transform_data(jj,4:6) = transform_data(jj,4:6) + xyz;
    end

    %z_offset = (2.22-0.76);
    %transform_data(:,3) = transform_data(:,3) + z_offset;
    %transform_data(:,6) = transform_data(:,6) + z_offset;
    
    %% Update Terrain Map
    T(1:length(T)-numPts,:) = T(numPts+1:length(T),:);
    T(length(T)-numPts+1:length(T),:) = transform_data;
    
%     for jj=1:length(T)
%         T(jj,1:3) = T(jj,1:3) - xyz + gps_prev(1:3);
%         T(jj,4:6) = T(jj,4:6) - xyz + gps_prev(1:3);
%     end
end
























