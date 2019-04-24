function [X, Z] = ekf2(sensor, map)
% EKF2 Extended Kalman Filter with IMU as inputs
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
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor) ekf2(sensor, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 9
%     the state should be in the following order
%     [x; y; z; vx; vy; vz; qw; qx; qy; qz; other states you use]
%     we will only take the first 10 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement


persistent mu_r mu_p c_r c_p t_old;

bg = 0*[1;1;1]; % gryo bias
ba = 0*[1;1;1]; % accel bias
Na = 0*[1;1;1]; 
Ng = 0*[1;1;1];
g = [0;0;-9.81]; % gravity
% til now best  good 1:1 1:10 bad 1:100 1:0.01
Q = 1; % IMU noise
R = 10; % VISION noise


%***************************** Firt Step ***********************************************%

if isempty(mu_r)

	if ~isempty(sensor.id)

		[vel,~] = estimate_vel(sensor,map);
        [p,q] = estimate_pose1(sensor,map);
        q = q';
        disp(q);
        disp(vel);
        disp(p);
		X = [p;vel;q];
		Z = [p;q;vel];
		[yaw,roll,pitch] = quat2angle(q','ZXY');
		mu_r = [p;roll;pitch;yaw;vel];
		c_r = 0;
		t_old = sensor.t;

	else 
		X = zeros(10,1);
		Z = zeros(10,1);

	end

else

%**************** Process Model  ******************************%

G = [cos(mu_r(5)) 0 -cos(mu_r(4))*sin(mu_r(5)); 0 1 sin(mu_r(4)); sin(mu_r(5)) 0 cos(mu_r(4))*cos(mu_r(5))];

Rq = [cos(mu_r(6))*cos(mu_r(5))-sin(mu_r(4))*sin(mu_r(5))*sin(mu_r(6)) -cos(mu_r(4))*sin(mu_r(6)) cos(mu_r(6))*sin(mu_r(5))+cos(mu_r(5))*sin(mu_r(4))*sin(mu_r(6));...
     sin(mu_r(6))*cos(mu_r(5))+sin(mu_r(4))*sin(mu_r(5))*cos(mu_r(6))  cos(mu_r(4))*cos(mu_r(6)) sin(mu_r(6))*sin(mu_r(5))-cos(mu_r(5))*sin(mu_r(4))*cos(mu_r(6));...
     -cos(mu_r(4))*sin(mu_r(5))                                  sin(mu_r(4))            cos(mu_r(4))*cos(mu_r(5))];
f = [mu_r(7:9);...
     G\(sensor.omg-bg-Ng);...
     g+Rq*(sensor.acc-ba-Na)]; 

%*************** Measurement Model **************************%

C = [eye(3) zeros(3,3) zeros(3,3);zeros(3,3) eye(3) zeros(3,3); zeros(3,3) zeros(3,3) eye(3)];
% Z = C*ur

% *************** Prediction Step ***************************%

t_step = sensor.t - t_old;
mu_p = mu_r + t_step*f;
t_old = sensor.t;

A = findjacobian2(sensor.img, sensor.acc, g, Na, Ng, ba, bg, mu_r);
F = double(eye(9)) + double(t_step*A);
U = eye(9);
V = U*t_step;
c_p = F*c_r*F' + V*Q*V';

%*************** Update step ****************************%

if ~isempty(sensor.id)

	[p,q] = estimate_pose1(sensor,map);
    q = q';
	v = (p-mu_r(1:3))/t_step;
 	Z = [p;q;v];
	[yaw,roll,pitch] = quat2angle(q','ZXY');
	z = [p;roll;pitch;yaw;v];
	W = eye(9);
	K = c_p*C'/(C*c_p*C'+W*R*W'); 
	mu_r = mu_p + K*(z-C*mu_p);
	c_r = c_p - K*C*c_p;
else 

	mu_r = mu_p;
	c_r = c_p;
	Z = zeros(10,1);

end

quat = angle2quat(mu_r(6),mu_r(4), mu_r(5), 'ZXY');
X = [mu_r(1:3);mu_r(7:9);quat'];

end

end
