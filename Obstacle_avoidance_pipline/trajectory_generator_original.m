function [desired_state] = trajectory_generator(t, qn, map, path)
%function path1 = trajectory_generator(t, qn, map, path)
% TRAJCTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory

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



    
    
if nargin >2
    path0 = path;
    map0 = map;
    q = 1;
end



if nargin > 2
    len = size(path0,1);
    new_path1 = collinear_path(path0);
    new_path1(size(new_path1,1)+1,:) = path0(len,:);
    %disp(new_path1);
end

%path1 = new_path1;



if nargin>2
    [new_path2,time_path2] = refine_path(new_path1,map0);
    
    new_path2(size(new_path2,1)+1,:) = (new_path1(length(new_path1)-1,:));
    new_path2(size(new_path2,1)+1,:) = path0(len,:);
    
    p_last = transpose(new_path2(size(new_path2,1),:));
    p_slast = transpose(new_path2(size(new_path2,1)-1,:));
    p_tlast = transpose(new_path2(size(new_path2,1)-2,:));
    time_path2(length(time_path2)+1) = time_path2(length(time_path2)) + (norm(p_slast-p_tlast)/1.7)+0.5;
    time_path2(length(time_path2)+1) = time_path2(length(time_path2)) + (norm(p_last-p_slast)/1.7)+0.5;
    %disp(new_path2);
    %disp(time_path2);
end

%path1 = new_path2;

%disp(new_path2);
%disp(time_path2);

if nargin>2
    new_path(1,:)=new_path2(1,:);
    time_path(1)=time_path2(1);
    i=1;
    x = 2;
    while ( i <= length(time_path2)-1)
        
        if time_path2(i+1)-time_path2(i)>0.6

            new_path(x,:)  = new_path2(i+1,:);
            time_path(x)=time_path2(i+1);
            x=x+1;

        end
        i=i+1;
    end
end


%path1 = new_path;
%disp(new_path);
%disp(time_path);

if nargin>2
    coeff = create_coeff(time_path);
    %coeff = create_coeff_mj(new_path,time_path);
end


if nargin<=2

   if t == 0
        pos = [0;0;0];
        vel = [0;0;0];
        acc = [0;0;0];        
        
   else
        if t>=time_path(length(time_path))+2 
               disp("go to final position");
               pos = new_path(size(new_path,1),:);
               vel = [0;0;0];
               acc = [0;0;0];
       
        else
            
               if t>time_path(q)
                   ts = time_path(q);
                   q=q+1;
               end
               
                   tf       =   t-ts;
                   %disp("starting time")
                   %disp(tf);
                   tfunc    =   coeff(1,q-1)*tf^3     + coeff(2,q-1)*tf^4       + coeff(3,q-1)*tf^5;
                   tv       =   coeff(1,q-1)*3*tf^2   + coeff(2,q-1)*4*tf^3     + coeff(3,q-1)*5*tf^4;
                   ta       =   coeff(1,q-1)*6*tf     + coeff(2,q-1)*12*tf^2    + coeff(3,q-1)*20*tf^3;
                   pos      =   [new_path(q-1,1)+(new_path(q,1)-new_path(q-1,1))*tfunc; new_path(q-1,2)+(new_path(q,2)-new_path(q-1,2))*tfunc ; new_path(q-1,3)+(new_path(q,3)-new_path(q-1,3))*tfunc];
                   vel      =   [(new_path(q,1)-new_path(q-1,1))*tv; (new_path(q,2)-new_path(q-1,2))*tv ; (new_path(q,3)-new_path(q-1,3))*tv];
                   acc      =   [(new_path(q,1)-new_path(q-1,1))*ta; (new_path(q,2)-new_path(q-1,2))*ta ; (new_path(q,3)-new_path(q-1,3))*ta];       
       end

        
   end
   
end

%}

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;



end



