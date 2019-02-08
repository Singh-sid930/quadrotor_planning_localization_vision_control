function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
%% Inputs:
%
% qd{qn}: state and desired state information for quadrotor #qn (qn
%         will be = 1 since we are only flying a single robot)
%
%  qd{qn}.pos, qd{qn}.vel   position and velocity
%  qd{qn}.euler = [roll;pitch;yaw]
%  qd{qn}.omega     angular velocity in body frame
% 
%  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des  desired position, velocity, accel
%  qd{qn}.yaw_des, qd{qn}.yawdot_des
%
% t: current time
%    
% qn: quadrotor number, should always be 1
%    
% params: various parameters
%  params.I     moment of inertia
%  params.grav  gravitational constant g (9.8...m/s^2)
%  params.mass  mass of robot
%
%% Outputs:
%
% F: total thrust commanded (sum of forces from all rotors)
% M: total torque commanded
% trpy: thrust, roll, pitch, yaw (attitude you want to command!)
% drpy: time derivative of trpy
%
% Using these current and desired states, you have to compute the desired
% controls u, and from there F and M
%

% =================== Your code goes here ===================
% ...
% ==============================

% Desired roll, pitch and yaw (in rad). In the simulator, those will be *ignored*.
% When you are flying in the lab, they *will* be used (because the platform
% has a built-in attitude controller). Best to fill them in already
% during simulation.

phi_des   = 0;
theta_des = 0;
psi_des   = 0;
a_com    = zeros(3:1);

pos_des = qd{qn}.pos_des; % 3X1 vector
vel_des = qd{qn}.vel_des; % 3X1 vector 
acc_des = qd{qn}.acc_des; %3X1Vector
yaw_des = qd{qn}.yaw_des; % one value 
yawdot_des = qd{qn}.yawdot_des; % one value 

pos_cur = qd{qn}.pos; % 3X1 vector
vel_cur = qd{qn}.vel; %3X1 vector
orient_cur = qd{qn}.euler; %3X1 vector
omega_cur = qd{qn}.omega;  %3X1 vector

I = params.I;
grav =  params.grav;
mass =   params.mass;

kp        = [15;15;15]
kd        = [15;15;13];


a_com     = acc_des - kd.*(vel_cur-vel_des) - kp.*(pos_cur-pos_des);
u1        = (a_com(3)+grav)*(mass);


theta_des = 1/grav * (a_com(1)*cos(yaw_des) + a_com(2)*sin(yaw_des));
phi_des = 1/grav * (a_com(1)*sin(yaw_des) - a_com(2)*cos(yaw_des));

orient_vec = [orient_cur(1)-phi_des;orient_cur(2)-theta_des;orient_cur(3)-yaw_des];
omega_vec  = [omega_cur(1);omega_cur(2);omega_cur(3)];

kp1 = -1.*[500;500;500];
kd1 = -1.*[30;50;50];
u2 = I * (kp1.*(orient_vec)+kd1.*(omega_vec));

%
%
%
%u    = zeros(4,1); % control input u, you should fill this in
                  
% Thrust
F    = u1;       % This should be F = u(1) from the project handout

% Moment
M    = u2;     % note: params.I has the moment of inertia


% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end

%
% ------------------------------------------------------------
%    should you decide to write a geometric controller,
%    the following functions should come in handy
%

function m = eulzxy2rotmat(ang)
    phi   = ang(1);
    theta = ang(2);
    psi   = ang(3);
    
    m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
          cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
         [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
          sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
         [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
end

function eul = rotmat2eulzxy(R)
    if R(3,2) < 1
        if R(3,2) > -1
            thetaX = asin(R(3,2));
            thetaZ = atan2(-R(1,2), R(2,2));
            thetaY = atan2(-R(3,1), R(3,3));
        else % R(3,2) == -1
            thetaX = -pi/2;
            thetaZ = -atan2(R(1,3),R(1,1));
            thetaY = 0;
        end
    else % R(3,2) == +1
        thetaX = pi/2;
        thetaZ = atan2(R(1,3),R(1,1));
        thetaY = 0;
    end
    eul = [thetaX, thetaY, thetaZ];
end

function w = veemap(R)
    w = [-R(2,3), R(1,3), -R(1,2)];
end
