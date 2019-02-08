function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0
if t == 0
    pos = [0;0;0];
    vel = [0;0;0];
    acc = [0;0;0];
elseif t < 3 && t>0
    tf = t;
    tfunc = ((10/27)*tf^3 + (-5/27)*tf^4  + (2/81)*tf^5);
    tv = ((10/27)*3*tf^2 + (-5/27)*4*tf^3  + (2/81)*5*tf^4);
    ta = ((10/27)*6*tf + (-5/27)*12*tf^2  + (2/81)*20*tf^3);
    pos = [0+((1/4)-0)*tfunc; 0+(1.414-0)*tfunc ; 0+(1.414-0)*tfunc];
    vel = [((1/4)-0)*tv; (1.414-0)*tv ; (1.414-0)*tv];
    acc = [((1/4)-0)*ta; (1.414-0)*ta ; (1.414-0)*ta];
elseif t < 6 && t>=3
    tf = t-3;
    tfunc = ((10/27)*tf^3 + (-5/27)*tf^4  + (2/81)*tf^5);
    tv = ((10/27)*3*tf^2 + (-5/27)*4*tf^3  + (2/81)*5*tf^4);
    ta = ((10/27)*6*tf + (-5/27)*12*tf^2  + (2/81)*20*tf^3);
    pos = [1/4+((1/2)-(1/4))*tfunc; 1.414+(0-1.414)*tfunc ; 1.414+(2.828-1.414)*tfunc];
    vel = [((1/2)-(1/4))*tv; (0-1.414)*tv ; (2.828-1.414)*tv];
    acc = [((1/2)-(1/4))*ta; (0-1.414)*ta ; (2.828-1.414)*ta];
elseif t < 9 && t>=6
     tf = t-6;
     tfunc = ((10/27)*tf^3 + (-5/27)*tf^4  + (2/81)*tf^5);
     tv = ((10/27)*3*tf^2 + (-5/27)*4*tf^3  + (2/81)*5*tf^4);
     ta = ((10/27)*6*tf + (-5/27)*12*tf^2  + (2/81)*20*tf^3);    
     pos = [1/2+((3/4)-(1/2))*tfunc; 0+(-1.414-0)*tfunc ; 2.828+(1.414-2.828)*tfunc];
     vel = [((3/4)-(1/2))*tv; (-1.414-0)*tv ; (1.414-2.828)*tv];
     acc = [((3/4)-(1/2))*ta; (-1.414-0)*ta ; (1.414-2.828)*ta];
elseif t < 12 && t>=9
     tf = t-9;
     tfunc = ((10/27)*tf^3 + (-5/27)*tf^4  + (2/81)*tf^5);
     tv = ((10/27)*3*tf^2 + (-5/27)*4*tf^3  + (2/81)*5*tf^4);
     ta = ((10/27)*6*tf + (-5/27)*12*tf^2  + (2/81)*20*tf^3);
     pos = [3/4+((1)-(3/4))*tfunc; -1.414+(0-(-1.414))*tfunc ; 1.414+(0-1.414)*tfunc];
     vel = [((1)-(3/4))*tv; (0-(-1.414))*tv ; (0-1.414)*tv];
     acc = [((1)-(3/4))*ta; (0-(-1.414))*ta ; (0-1.414)*ta];
else 
     pos = [1; 0 ; 0];
     vel = [0; 0 ; 0];
     acc = [0; 0; 0];
    
end

%pos = [0; 0; 0];
%vel = [0; 0; 0];
%acc = [0; 0; 0];
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
