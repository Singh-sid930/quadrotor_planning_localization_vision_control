function A = findjacobian2(omg,acc,g,Na,Ng,ba,bg,ur)

wm1=omg(1);
wm2=omg(2);
wm3=omg(3);
am1=acc(1);
am2=acc(2);
am3=acc(3);
g1 = g(1);
g2 = g(2);
g3 = g(3);
na1=Na(1);
na2=Na(2);
na3=Na(3);
ng1=Ng(1);
ng2=Ng(2);
ng3=Ng(3);
ba1=ba(1);
ba2=ba(2);
ba3=ba(3);
bg1=bg(1);
bg2=bg(2);
bg3=bg(3);
x1 = ur(1);
x2 = ur(2);
x3 = ur(3);
x4 = ur(4);
x5 = ur(5);
x6 = ur(6);
x7 = ur(7);
x8 = ur(8);
x9 = ur(9);

% syms x1 x2 x3 x4 x5 x6 x7 x8 x9 wm1 wm2 wm3 am1 am2 am3 g1 g2 g3 na1 na2 na3 ng1 ng2 ng3 ba1 ba2 ba3 bg1 bg2 bg3
% 
% jacobian([x7;x8;x9;...
%          [cos(x5) 0 -cos(x4)*sin(x5); 0 1 sin(x4); sin(x5) 0 cos(x4)*cos(x5)] \ [wm1-bg1-ng1;wm2-bg2-ng2;wm3-bg3-ng3];...
%          [g1;g2;g3]+ [cos(x6)*cos(x5)-sin(x4)*sin(x5)*sin(x6) -cos(x4)*sin(x6) cos(x6)*sin(x5)+cos(x5)*sin(x4)*sin(x6);...
%           sin(x6)*cos(x5)+sin(x4)*sin(x5)*cos(x6)  cos(x4)*cos(x6) sin(x6)*sin(x5)-cos(x5)*sin(x4)*cos(x6);...
%           -cos(x4)*sin(x5)   sin(x4)   cos(x4)*cos(x5)]*[am1-ba1-na1;am2-ba2-na2;am3-ba3-na3] ],[x1,x2,x3,x4,x5,x6,x7,x8,x9]);

A = [ 0, 0, 0,                                                                                                                         0,                                                                                                                             0,                                                                                                                                                                 0, 1, 0, 0;...
 0, 0, 0,                                                                                                                         0,                                                                                                                             0,                                                                                                                                                                 0, 0, 1, 0;...
 0, 0, 0,                                                                                                                         0,                                                                                                                             0,                                                                                                                                                                 0, 0, 0, 1;...
 0, 0, 0,                                                                                                                         0,                                             bg1*sin(x5) - ng3*cos(x5) - bg3*cos(x5) + wm3*cos(x5) + ng1*sin(x5) - wm1*sin(x5),                                                                                                                                                                 0, 0, 0, 0;...
 0, 0, 0,                             (bg3*cos(x5) + ng3*cos(x5) - bg1*sin(x5) - wm3*cos(x5) - ng1*sin(x5) + wm1*sin(x5))/cos(x4)^2,                        -(sin(x4)*(bg1*cos(x5) + ng1*cos(x5) + bg3*sin(x5) - wm1*cos(x5) + ng3*sin(x5) - wm3*sin(x5)))/cos(x4),                                                                                                                                                                 0, 0, 0, 0;...
 0, 0, 0,                  -(sin(x4)*(bg3*cos(x5) + ng3*cos(x5) - bg1*sin(x5) - wm3*cos(x5) - ng1*sin(x5) + wm1*sin(x5)))/cos(x4)^2,                                   (bg1*cos(x5) + ng1*cos(x5) + bg3*sin(x5) - wm1*cos(x5) + ng3*sin(x5) - wm3*sin(x5))/cos(x4),                                                                                                                                                                 0, 0, 0, 0;...
 0, 0, 0, cos(x4)*sin(x5)*sin(x6)*(ba1 - am1 + na1) - sin(x4)*sin(x6)*(ba2 - am2 + na2) - cos(x4)*cos(x5)*sin(x6)*(ba3 - am3 + na3), (cos(x6)*sin(x5) + cos(x5)*sin(x4)*sin(x6))*(ba1 - am1 + na1) - (cos(x5)*cos(x6) - sin(x4)*sin(x5)*sin(x6))*(ba3 - am3 + na3), (cos(x5)*sin(x6) + cos(x6)*sin(x4)*sin(x5))*(ba1 - am1 + na1) + (sin(x5)*sin(x6) - cos(x5)*cos(x6)*sin(x4))*(ba3 - am3 + na3) + cos(x4)*cos(x6)*(ba2 - am2 + na2), 0, 0, 0;...
 0, 0, 0, cos(x6)*sin(x4)*(ba2 - am2 + na2) + cos(x4)*cos(x5)*cos(x6)*(ba3 - am3 + na3) - cos(x4)*cos(x6)*sin(x5)*(ba1 - am1 + na1), (sin(x5)*sin(x6) - cos(x5)*cos(x6)*sin(x4))*(ba1 - am1 + na1) - (cos(x5)*sin(x6) + cos(x6)*sin(x4)*sin(x5))*(ba3 - am3 + na3), cos(x4)*sin(x6)*(ba2 - am2 + na2) - (cos(x6)*sin(x5) + cos(x5)*sin(x4)*sin(x6))*(ba3 - am3 + na3) - (cos(x5)*cos(x6) - sin(x4)*sin(x5)*sin(x6))*(ba1 - am1 + na1), 0, 0, 0;...
 0, 0, 0,                         cos(x5)*sin(x4)*(ba3 - am3 + na3) - cos(x4)*(ba2 - am2 + na2) - sin(x4)*sin(x5)*(ba1 - am1 + na1),                                                         cos(x4)*cos(x5)*(ba1 - am1 + na1) + cos(x4)*sin(x5)*(ba3 - am3 + na3),                                                                                                                                                                 0, 0, 0, 0];
 

end