
function [X, Z] = ekf1(sensor, vic, map)
% EKF1 Extended Kalman Filter with Vicon velocity as inputs
%
% INPUTS:
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: sensor timestamp
%          - rpy, omg, acc: imu readings
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   vic    - struct for storing vicon linear velocity in world frame and
%            angular velocity in body frame, fields include
%          - t: vicon timestamp
%          - vel = [vx; vy; vz; wx; wy; wz]
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor, vic) ekf1(sensor, vic, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 6
%     the state should be in the following order
%     [x; y; z; qw; qx; qy; qz; other states you use]
%     we will only take the first 7 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 7
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement

persistent mu_r mu_p c_r c_p t_old; 

bg = 0*[1;1;1];
Nv = 0*[1;1;1];
Ng = 0*[1;1;1];

Q = 1; 
R = 0.01;

if isempty(mu_r)

	 if ~isempty(sensor.id)
		[p,q] = estimate_pose1(sensor,map);
        q = q';
        disp(p);
        disp(q);
		X = [p;q];
		Z = [p;q];
		[yaw,roll,pitch] = quat2angle(q','ZXY');
		mu_r = [p;roll;pitch;yaw];
		c_r = 0;
		t_old = sensor.t;

	 else
		X = zeros(7,1);
		Z = zeros(7,1);
		mu_r = zeros(6,1);
		c_r = 0;
		t_old = vic.t;
     end

else

%*******************Process Model ****************************%

G = [cos(mu_r(5)) 0 -cos(mu_r(4))*sin(mu_r(5)); 0 1 sin(mu_r(4)); sin(mu_r(5)) 0 cos(mu_r(4))*cos(mu_r(5))]; 
f = [vic.vel(1:3)-Nv;...
     G\(vic.vel(4:6)-bg-Ng)]; 


%*******************Measurement Model ************************%

C = [eye(3) zeros(3,3);zeros(3,3) eye(3)];
% Z = C*ur

%*******************Prediction ********************************%

t_delta = vic.t - t_old;
t_old = vic.t;
mu_p = mu_r + f*t_delta; 

A = findjacobian1(vic.vel,Nv,Ng,bg,mu_r);
F = eye(6) + t_delta*A;
U = eye(6);
V = U*t_delta;
c_p = F*c_r*F' + V*Q*V';


%********************Update **********************************%

if ~isempty(sensor.id)

	[p,q] = estimate_pose1(sensor,map);
    q = q';
	Z = [p;q];
	[yaw,roll,pitch] = quat2angle(q','ZXY');
	z = [p;roll;pitch;yaw];
	W = eye(6);
	K = c_p*C'/(C*c_p*C' + W*R*W');
	mu_r = mu_p + K*(z-C*mu_p);
	c_r = c_p -K*C*c_p;

else 
	mu_r = mu_p;
	c_r = c_p;
	Z = zeros(7,1);

end
quat = angle2quat(mu_r(6),mu_r(4),mu_r(5),'ZXY');
X = [mu_r(1:3);quat'];
	
end

end










