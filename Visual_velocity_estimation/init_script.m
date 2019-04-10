% add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_pose_handle = @(sensor) estimate_pose(sensor, your_input);
% We will only call estimate_pose_handle in the test function.
% Note that unlike project 1 phase 3, this will only create a function
% handle, but not run the function at all.
map(1).tag_id =    [0, 12, 24, 36, 48, 60, 72, 84, 96;
                1, 13, 25, 37, 49, 61, 73, 85,  97;
                2, 14, 26, 38, 50, 62, 74, 86,  98;
                3, 15, 27, 39, 51, 63, 75, 87,  99;
                4, 16, 28   , 40, 52, 64, 76, 88, 100;
                5, 17, 29, 41, 53, 65, 77, 89, 101;
                6, 18, 30, 42, 54, 66, 78, 90, 102;
                7, 19, 31, 43, 55, 67, 79, 91, 103;
                8, 20, 32, 44, 56, 68, 80, 92, 104;
                9, 21, 33, 45, 57, 69, 81, 93, 105;
                10, 22, 34, 46, 58, 70, 82, 94, 106;
                11, 23, 35, 47, 59, 71, 83, 95, 107];
            
 
 map(1).p0x = 0.076:0.304:3.42;          
 map(1).p1x = 0.152:0.304:3.496;
 map(1).p2x = 0.152:0.304:3.496;
 map(1).p3x = 0:0.304:3.3440;
 map(1).p4x = 0:0.304:3.3440;
 
 
 
    
 map(1).p0y = [0,0.304,0.608,0.938,1.242,1.546,1.876,2.18,2.484]+0.076;
 map(1).p1y = [0,0.304,0.608,0.938,1.242,1.546,1.876,2.18,2.484];
 map(1).p2y = [0,0.304,0.608,0.938,1.242,1.546,1.876,2.18,2.484]+0.152;
 map(1).p3y = [0,0.304,0.608,0.938,1.242,1.546,1.876,2.18,2.484]+0.152;
 map(1).p4y = [0,0.304,0.608,0.938,1.242,1.546,1.876,2.18,2.484];
 
 disp(size(map.p4x));
 disp(size(map.p4y));
 
 map(1).k_mat = [311.0520 0        201.8724;
              0        311.3885 113.6210;
              0        0        1];
map(1).xyz = [-0.04, 0.0, -0.03];

map(1).yaw = pi/4;


estimate_pose_handle = @(sensor)estimate_pose(sensor,map);

estimate_vel_handle = @(sensor) estimate_vel(sensor,map);

%******* Portion to remove after the thing is working *********%%

velx = [];
vely = [];
velz = [];
velwx = [];
velwy = [];
velwz = [];

data_len = length(data);

dat_t = [data.t];

figure 
subplot(2,3,1)
plot(time,vicon(7,:));

subplot(2,3,2)
plot(time,vicon(8,:));


subplot(2,3,3)
plot(time,vicon(9,:));


subplot(2,3,4)
plot(time,vicon(10,:));


subplot(2,3,5)
plot(time,vicon(11,:));


subplot(2,3,6)
plot(time,vicon(12,:));

for i = 1:data_len
   % disp(i);
   [vel,omega] = estimate_vel_handle(data(i));
   
   if(isempty(vel))
   velx = [velx,0];
   vely = [vely,0];
   velz = [velz,0];
   
   velwx = [velwx,0];
   velwy = [velwy,0];
   velwz = [velwz,0];
   else
   
   velx = [velx,vel(1)];
   vely = [vely,vel(2)];
   velz = [velz,vel(3)];
   
   velwx = [velwx,omega(1)];
   velwy = [velwy,omega(2)];
   velwz = [velwz,omega(3)];
   end
   
end
figure 
subplot(2,3,1)
plot(dat_t(150:length(dat_t)),velx(150:length(velx)));
hold on;
plot(time,vicon(7,:));
title("velocity in x direction");

subplot(2,3,2)
plot(dat_t(150:length(dat_t)),vely(150:length(vely)));
hold on;
plot(time,vicon(8,:));
title("velocity in y direction");


subplot(2,3,3)
plot(dat_t(150:length(dat_t)),velz(150:length(velz)));
hold on;
plot(time,vicon(9,:));
title("velocity in z direction");


subplot(2,3,4)
plot(dat_t,velwx);
hold on;
plot(time,vicon(10,:));
title("roll angular velocity");


subplot(2,3,5)
plot(dat_t,velwy);
hold on;
plot(time,vicon(11,:));
title("pitch angular velocity");


subplot(2,3,6)
plot(dat_t,velwz);
hold on;
plot(time,vicon(12,:));
title("yaw angular velocity");


%}
