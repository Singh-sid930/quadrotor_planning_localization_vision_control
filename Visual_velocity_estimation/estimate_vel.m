function [vel, omg] = estimate_vel(sensor, map)
%ESTIMATE_VEL 6DOF velocity estimator
%   sensor - struct stored in provided dataset, fields include:
%          - is_ready: logical, indicates whether sensor data is valid; if the
%                      sensor data is invalid, return empty arrays for vel, omg
%          - t: timestamp
%          - rpy, omg, acc: imu readings (you should not use these in this phase)
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags; if no tags are present return empty
%                arrays for vel, omg
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables youwish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doin6
%              estimate_vel_handle = ...
%                  @(sensor) estimate_vel(sensor, your personal input arguments);
%   vel - 3x1 velocity of the quadrotor in world frame
%   omg - 3x1 angular velocity of the quadrotor



persistent flag PointTracker t_old delta_t_old img_old Vel_old Omg_old %sensor_old p0 p1 p2 p3 p4

if isempty(sensor.id)
vel = [];
omg = [];
return;
end
%Corners1 = detectFASTFeatures(sensor.img);
%corners = Corners1.selectStrongest(50);
%imshow(sensor.img);
%hold on;
%plot(corners);

%**********************************************************************
%initialization for the very first image where no data is returned; 
if isempty(flag)
    disp("first initialization");
    %Corners = detectFASTFeatures(sensor.img);
    %PointTracker = vision.PointTracker;
    %track_corners = Corners.selectStrongest(100).Location;
    %p0 = abs(sensor.p0');
    %p1 = abs(sensor.p1');
    %p2 = abs(sensor.p2');
    %p3 = abs(sensor.p3');
    %p4 = abs(sensor.p4');
    %track_corners  = [track_corners;p0;p1;p2;p3;p4];
    %track_corners  = [p0;p1;p2;p3;p4];
    %initialize(PointTracker,track_corners,sensor.img);   
    t_old = sensor.t;
    vel = [0;0;0]';
    omg = [0;0;0]';
    delta_t_old = 0;
    img_old = sensor.img;
    Vel_old = [0,0,0];
    Omg_old = [0,0,0];
    flag = 1;
    return;
end
%***********************************************************************%%



% getting information for calculating depth and transformation parameters. 

[T,R,h,H] = estimate_pose(sensor,map);

%disp(T);
%disp(R);
%disp(H_C2I);
%disp(H);


%%*******************************************************************%%%

%detecting the corners of the old image and creating the point tracker
%object on the old image.
if flag == 2
release(PointTracker);
end
Corners_old= detectFASTFeatures(img_old);
PointTracker= vision.PointTracker;
C_points_old = Corners_old.selectStrongest(100).Location;
%C_points_old  = [C_points_old;p0;p1;p2;p3;p4];
%C_points_old  = [p0;p1;p2;p3;p4];
initialize(PointTracker,C_points_old,img_old);


%finding the new coordinates of the moved points through the tracker object
%and the current image. 

[C_points_new,~,~] = step(PointTracker,sensor.img);
%[C_points_new,point_validity,scores] = PointTracker(sensor.img);

%disp("getting points");
%imshow(img_old);
%hold on;
%plot(C_points_old);
%hold on;
%plot(C_points_new);

%%*********************************************************************%%
%initializing the low pass filter coefficient for time. The first time it
%would be one and then on it would be 0.1;

if flag ==1 
    alpha = 1;
    flag =2;
else
    alpha = 0.1;
end


%finding the difference in time through the low pass filter. 
delta_t = alpha*(sensor.t-t_old) + (1-alpha) * delta_t_old;

%%***********************************************************************%


%transforming the old and new corner points into homogenous coordinates.
C_points_old = [C_points_old,ones(size(C_points_old,1),1)];
C_points_new = [C_points_new,ones(size(C_points_new,1),1)];

%disp(C_points_new');
%disp(inv(H));
%finding ground positions for the homoegnous coordinates.
C_points_new_ground = H\C_points_new';


for i = 1:length(C_points_new_ground)
    C_points_new_ground(:,i) = C_points_new_ground(:,i)/C_points_new_ground(3,i);
end




%% ****************************************************************%%%%%

%Converting ground positions into hogenous coordinates. 
C_points_new_ground = [C_points_new_ground(1,:);C_points_new_ground(2,:);zeros(1,size(C_points_old,1));ones(1,size(C_points_old,1))];

%disp(C_points_new_ground);
%converting ground positions to camera coordinates. 
Cam_points = [R(:,1),R(:,2),R(:,3),T]*C_points_new_ground;



%finding the depth of each point. 
Z = Cam_points(3,:);

%disp(Z);


P_cur = [];
P_pre = [];

K = [311.0520 , 0 , 201.8724;
     0 , 311.3885, 113.6210;
     0 , 0 , 1];

%converting pixel coordinates to camera coordinates.  
for i=1:size(C_points_new,1)
    P_in = ((K)\C_points_new(i,:)')';
    P_cur = [P_cur;P_in];
    P_in_pre = ((K)\C_points_old(i,:)')';
    P_pre = [P_pre;P_in_pre]; 
end

%Adding point correspondences of corner points in the points

%Z=[];
%for i=1: size(P_cur,1)
%Z=[Z;-T(3)/(R(3,1)*P_cur(i,1)+R(3,2)*P_cur(i,2)+R(3,3))];
%end

%finding the 2d point flow 
point_flow = (P_cur - P_pre)/delta_t;
    

opt_flow = [];
F_mat = [];

%finding the flow matrix and total optical flow
for i= 1:size(P_cur,1)
    F_mat =  [F_mat;-1/Z(i) , 0 , P_cur(i,1)/Z(i) , P_cur(i,1)*P_cur(i,2) , -(1+P_cur(i,1)^2) , P_cur(i,2);
              0  , -1/Z(i) , P_cur(i,2)/Z(i) , (1+P_cur(i,2)^2) , -P_cur(i,1)*P_cur(i,2) , -P_cur(i,1)];
    opt_flow = [opt_flow;point_flow(i,1);point_flow(i,2)];
end




%%%% ************* Implement RANSAC here ***************************%%%

%velocity = RANSAC(F_mat,opt_flow);

%%*****************RANSAC ends here ******************************%%%%%

%finding the velocities
velocity = F_mat\opt_flow;

if isempty(velocity) 
    velocity = [0,0,0,0,0,0];
end

%initialize a constant if velocity also needs to be passed through a low
%pass filter
beta = 0.2;
betao = 0.8;

%for j=1:6
%velocity(j)=beta*velocity(j)+(1-beta)*vel_old(j);
%end




img_old= sensor.img;
delta_t_old = delta_t;
t_old = sensor.t;
%sensor_old = sensor;
%p0 = abs(sensor.p0');
%p1 = abs(sensor.p1');
%p2 = abs(sensor.p2');
%p3 = abs(sensor.p3');
%p4 = abs(sensor.p4');
%transforming velocities into the world frame. 

 R_IC = [0.7071,-0.7071,0;
           -0.7071,-0.7071,0;
             0,0,-1];
 Tci = [-0.04;0.0;-0.03];
 %R = R_IC * R_IW;
 R_IW = R_IC*R;
 

VEL = R*[velocity(1) velocity(2) velocity(3) ]'+R_IW*cross(velocity(4:6),Tci);
%VEL = R*[velocity(1) velocity(2) velocity(3) ]'+cross(velocity(4:6),H_C2I(1:3,4));
OMG =R* [velocity(4) velocity(5) velocity(6) ]';

beta = [0.85 0.85 0.8];
betao = [0.05 0.05 0.1];

VEL_now = (1-beta).*Vel_old + (beta).*VEL;
OMG_now = betao.*Omg_old + (1-betao).*OMG;

if(any(abs(VEL_now-Vel_old)>0.8))
    VEL_now = Vel_old;
end

if(any(abs(OMG_now-Omg_old)>0.8))
    OMG_now = Omg_old;
end

Vel_old = VEL_now;
Omg_old = OMG_now;
vel=[VEL_now(1) VEL_now(2) VEL_now(3)]';
omg=[OMG_now(1) OMG_now(2) OMG_now(3)]';
%publishing velocity
%vel=[VEL(1) VEL(2) VEL(3)]';
%omg=[OMG(1) OMG(2) OMG(3)]';
%}



end
