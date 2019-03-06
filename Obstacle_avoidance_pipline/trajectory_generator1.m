function [desired_state] = trajectory_generator(t, qn, map, path)
%function path1 = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. At first, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% parameters:
%   map: The map structure returned by the load_map function
%   path: This is the path returned by your planner (dijkstra function)
%   desired_state: Contains all the information that is passed to the
%                  controller, as in Phase 1
%
% NOTE: It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;


    persistent path0 map0;
    persistent new_path;
    persistent time_path q ts coeff;
    
    
pos = [0;0;0];
vel = [0;0;0];
acc = [0;0;0];
    

yaw = 0;
yawdot = 0;


tn = 2;
i = 3;
p = 2;
tt=1;

    
    
if nargin >2
    path0 = path;
    map0 = map;
    q = 1;
    p1 = path0(1,:);
    p2 = path0(2,:);
    p3 = path0(3,:);
    new_path(1,:)= p1;
    time_path(1) = 0;
    new_path1(1,:)= p1;
    time_path1(1) = 0;
end

len = size(path0,1);


%Total_time = 20;
%Tt = Total_time;
%tstep = Total_time/len;
   




if nargin > 2
    total_dist = 0;
    total_time = 0;
disp("4 arguments");
    while true

        if (i==len)
            break;
        end

        if(Collinear(p1,p2,p3))
            p2 = p3;
            p3 = path(i,:);
            i= i+1;
            tt=tt+1;
        else 
            dist = norm(p1-p2);
            time = dist/1.8;
            total_dist = total_dist + dist;
            total_time = total_time + time;
            new_path1(tn,:) = p2;
            time_path1(p) = total_time;
            %disp("##");
            %disp(norm(p2-p1));
            %disp(time_path1(p)-time_path1(p-1))
            %disp("##");
            p1 = p2;
            p2 = p3;
            p3 = path(i+1,:);
            i = i+1;
            tn=tn+1;
            p = p+1;
            tt=tt+1;
        end

    end

end

if nargin>2
    new_path1(size(new_path1,1)+1,:) = path0(len,:);
    time_path1(length(time_path1)+1) = total_time+3; 
    disp(new_path1);
end


if nargin>2
   [new_path,time_path] = refine_path(new_path1,time_path1,map0);
    new_path(size(new_path,1)+1,:) = path0(len,:);
    p_last = transpose(new_path(size(new_path,1),:));
    p_slast = transpose(new_path(size(new_path,1)-1,:));
    time_path(length(time_path)+1) = time_path(length(time_path)) + norm(p_last-p_slast)/1.8;
   
end

disp(new_path);
disp(time_path);



if nargin<=2

   if t == 0
        pos = [0;0;0];
        vel = [0;0;0];
        acc = [0;0;0];        
        %ts = time_path(q);
   else
       

    
       if t>=time_path(q) && t<time_path(q+1)
           T = time_path(q+1)-time_path(q);
           if T < 0.5
               T = 0.5;
           end
           A = [T^3,T^4,T^5;3*T^2,4*T^3,5*T^4;6*T,12*T^2,20*T^3];
           A = inv(A);
           coeff = A*[1;0;0];
           ts = time_path(q);
           %disp("from point to point");
           %disp(new_path(q,:));
           %disp(new_path(q+1,:));
           %disp("time slot");
           %disp(T);
           q=q+1;
       end

       tf       =   t-ts;
       tfunc    =   coeff(1,1)*tf^3     + coeff(2,1)*tf^4       + coeff(3,1)*tf^5;
       tv       =   coeff(1,1)*3*tf^2   + coeff(2,1)*4*tf^3     + coeff(3,1)*5*tf^4;
       ta       =   coeff(1,1)*6*tf     + coeff(2,1)*12*tf^2    + coeff(3,1)*20*tf^3;
       pos      =   [new_path(q-1,1)+(new_path(q,1)-new_path(q-1,1))*tfunc; new_path(q-1,2)+(new_path(q,2)-new_path(q-1,2))*tfunc ; new_path(q-1,3)+(new_path(q,3)-new_path(q-1,3))*tfunc];
       vel      =   [(new_path(q,1)-new_path(q-1,1))*tv; (new_path(q,2)-new_path(q-1,2))*tv ; (new_path(q,3)-new_path(q-1,3))*tv];
       acc      =   [(new_path(q,1)-new_path(q-1,1))*ta; (new_path(q,2)-new_path(q-1,2))*ta ; (new_path(q,3)-new_path(q-1,3))*ta];

        if t>=time_path(length(time_path))
           disp("go to final position");
           pos = new_path(size(new_path,1),:);
           vel = [0;0;0];
           acc = [0;0;0]; 
        end
   end
   
end


%path1 = new_path;


%disp(pos);
%disp(vel);
%disp(acc);

%disp(vel);

%}

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;



end



