tf = 11;
syms x y z
eqn1 = x*tf^3 + y*tf^4  + z*tf^5 == 1;
eqn2 = x*3*tf^2 + y*4*tf^3 + z*5*tf^4 == 0.0;
eqn3 = x*6*tf + y*12*tf^2 + z*20*tf^3.0== 0.0;
sol = solve([eqn1, eqn2, eqn3], [x, y, z]);
xSol = sol.x
ySol = sol.y
zSol = sol.z

disp(xSol*tf^3 + ySol*tf^4  + zSol*tf^5)