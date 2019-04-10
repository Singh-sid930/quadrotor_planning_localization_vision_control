function [pos, q] = estimate_pose(sensor, map_lay)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include:
%          - is_ready: logical, indicates whether sensor data is valid
%          - rpy, omg, acc: imu readings (you should not use these in this phase)
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags, if no tags are present return empty
%                arrays for pos, q
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);
%   pos - 3x1 position of the quadrotor in world frame
%   q   - 4x1 quaternion of the quadrotor [w, x, y, z] where q = w + x*i + y*j + z*k



p1 = sensor.p1;
p2 = sensor.p2;
p3 = sensor.p3;
p4 = sensor.p4;
p0 = sensor.p0;
id = sensor.id;
time = sensor.t;




len = length(id);


if len == 0
    
    
    T_IC = [];
    quat = [];
    
else 
    
    map = get_positions(id,map_lay);

    p1_g = map.p1;
    p2_g = map.p2;
    p3_g = map.p3;
    p4_g = map.p4;
    p0_g = map.p0;

    
    
    A = [];
    i = 1;

    while(i<=len)

        A1 = [p0_g(1,i),p0_g(2,i),1,0,0,0,-p0(1,i)*p0_g(1,i),-p0(1,i)*p0_g(2,i),-p0(1,i);
              0,0,0,p0_g(1,i),p0_g(2,i),1, -p0(2,i)*p0_g(1,i),-p0(2,i)*p0_g(2,i),-p0(2,i);
              p1_g(1,i),p1_g(2,i),1,0,0,0,-p1(1,i)*p1_g(1,i),-p1(1,i)*p1_g(2,i),-p1(1,i);
              0,0,0,p1_g(1,i),p1_g(2,i),1, -p1(2,i)*p1_g(1,i),-p1(2,i)*p1_g(2,i),-p1(2,i);
              p2_g(1,i),p2_g(2,i),1,0,0,0,-p2(1,i)*p2_g(1,i),-p2(1,i)*p2_g(2,i),-p2(1,i);
              0,0,0,p2_g(1,i),p2_g(2,i),1, -p2(2,i)*p2_g(1,i),-p2(2,i)*p2_g(2,i),-p2(2,i);
              p3_g(1,i),p3_g(2,i),1,0,0,0,-p3(1,i)*p3_g(1,i),-p3(1,i)*p3_g(2,i),-p3(1,i);
              0,0,0,p3_g(1,i),p3_g(2,i),1, -p3(2,i)*p3_g(1,i),-p3(2,i)*p3_g(2,i),-p3(2,i);
              p4_g(1,i),p4_g(2,i),1,0,0,0,-p4(1,i)*p4_g(1,i),-p4(1,i)*p4_g(2,i),-p4(1,i);
              0,0,0,p4_g(1,i),p4_g(2,i),1, -p4(2,i)*p4_g(1,i),-p4(2,i)*p4_g(2,i),-p4(2,i)];

        A = [A;A1];
        i = i +1;
    end


    [U,S,V] = svd(A);
    h_c = V(:,length(V));
    h_c = transpose(h_c);
    k_mat = map_lay.k_mat;
    h = [h_c(1:3);h_c(4:6);h_c(7:9)];
    
    h = h/h(3,3);
    RRT = k_mat\ h;
    div = norm(RRT(:,1)) +norm(RRT(:,2));
    div = div/2;
    T = RRT(:,length(RRT))/div;
    R_hat = [RRT(:,1),RRT(:,2),cross(RRT(:,1),RRT(:,2))];
    [U1,S1,V1] = svd(R_hat);
    R = U1*[1,0,0;0,1,0;0,0,det(U1*transpose(V1))]*transpose(V1);
    %R_IC = [1,0,0,0;0,cos(pi),-sin(pi),0;0,sin(pi),cos(pi),0;0,0,0,1]*[cos(pi/4),-sin(pi/4),0,-0.04;sin(pi/4),cos(pi/4),0,0;0,0,1,-0.03;0,0,0,1];
    R_IC = [0.7071,-0.7071,0;
           -0.7071,-0.7071,0;
             0,0,-1];
    R_f = R_IC * R;
    R_f = transpose(R_f);
    T_IC = R\([-0.04;0.0;-0.03]-T);
    quat = rotm2quat(R_f);
end


pos = T_IC;
q = quat;


end
