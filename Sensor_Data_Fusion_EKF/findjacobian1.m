function A = findjacobian1(vel,Nv,Ng,bg,ur)
vm1=vel(1);
vm2=vel(2);
vm3=vel(3);
wm1=vel(4);
wm2=vel(5);
wm3=vel(6);
nv1=Nv(1);
nv2=Nv(2);
nv3=Nv(3);
ng1=Ng(1);
ng2=Ng(2);
ng3=Ng(3);
bg1=bg(1);
bg2=bg(2);
bg3=bg(3);
x1 = ur(1);
x2 = ur(2);
x3 = ur(3);
x4 = ur(4);
x5 = ur(5);
x6 = ur(6);

% syms x1 x2 x3 x4 x5 x6 vm1 vm2 vm3 wm1 wm2 wm3 nv1 nv2 nv3 ng1 ng2 ng3 bg1 bg2 bg3
% 
% jacobian([vm1-nv1;vm2-nv2;vm3-nv3;...
%          [cos(x5) 0 -cos(x4)*sin(x5); 0 1 sin(x4); sin(x5) 0 cos(x4)*cos(x5)] \ [wm1-bg1-ng1;wm2-bg2-ng2;wm3-bg3-ng3]],[x1,x2,x3,x4,x5,x6])

A = [ 0, 0, 0,                                                                                                        0,                                                                                                      0, 0;...
  0, 0, 0,                                                                                                        0,                                                                                                      0, 0;...
  0, 0, 0,                                                                                                        0,                                                                                                      0, 0;...
  0, 0, 0,                                                                                                        0,                      bg1*sin(x5) - ng3*cos(x5) - bg3*cos(x5) + wm3*cos(x5) + ng1*sin(x5) - wm1*sin(x5), 0;...
  0, 0, 0,            (bg3*cos(x5) + ng3*cos(x5) - bg1*sin(x5) - wm3*cos(x5) - ng1*sin(x5) + wm1*sin(x5))/cos(x4)^2, -(sin(x4)*(bg1*cos(x5) + ng1*cos(x5) + bg3*sin(x5) - wm1*cos(x5) + ng3*sin(x5) - wm3*sin(x5)))/cos(x4), 0;...
  0, 0, 0, -(sin(x4)*(bg3*cos(x5) + ng3*cos(x5) - bg1*sin(x5) - wm3*cos(x5) - ng1*sin(x5) + wm1*sin(x5)))/cos(x4)^2,            (bg1*cos(x5) + ng1*cos(x5) + bg3*sin(x5) - wm1*cos(x5) + ng3*sin(x5) - wm3*sin(x5))/cos(x4), 0];
 

end