function [T] = lidarMapping(T, lidar_data, imu_data, gps_data, numPts)
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
    
    R = [cos(p)*cos(y), cos(p)*sin(y), -sin(p)
        sin(r)*sin(p)*cos(y)-cos(r)*sin(y), sin(r)*sin(p)*sin(y)+cos(r)*cos(y), cos(p)*sin(r)
        cos(r)*sin(p)*cos(y)+sin(r)*sin(y), cos(r)*sin(p)*sin(y)-sin(r)*cos(y), cos(p)*cos(r)];
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

    z_offset = (2.22-0.76);
    transform_data(:,3) = transform_data(:,3) + z_offset;
    transform_data(:,6) = transform_data(:,6) + z_offset;
    
    %% Update Terrain Map
    T(1:length(T)-numPts,:) = T(numPts+1:length(T),:);
    T(length(T)-numPts+1:length(T),:) = transform_data;
    
end
























