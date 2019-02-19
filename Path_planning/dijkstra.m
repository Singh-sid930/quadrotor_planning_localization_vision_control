function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an mx3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path. The first
%   row is start and the last row is goal. If no path is found, PATH is a
%   0x3 matrix. Consecutive points in PATH should not be farther apart than
%   neighboring voxels in the map (e.g. if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of nodes that were expanded while performing the search.
%   
% paramaters:
%   map     - the map object to plan in
%   start   - 1x3 vector of the starting coordinates [x,y,z]
%   goal:   - 1x3 vector of the goal coordinates [x,y,z]
%   astar   - boolean use astar or dijkstra
size_map1   =   size(map.occgrid);
disp(size_map1);
size_map    =   [size(map.occgrid,1),size(map.occgrid,2),size(map.occgrid,3)];
disp(size_map);
cost_mat    = 50000*ones(size_map(1),size_map(2),size_map(3));
parent_path = ones(size_map(1),size_map(2),size_map(3));
visit_mat    = zeros(size_map(1),size_map(2),size_map(3));

goal_sub    =   pos2sub(map,goal);
goal_ind    =   sub2ind(size_map,goal_sub(1),goal_sub(2),goal_sub(3));
start_sub   =   pos2sub(map,start);
start_ind   =   sub2ind(size_map,start_sub(1),start_sub(2),start_sub(3));
start_val   =   map.occgrid(start_sub(1),start_sub(2),start_sub(3));
goal_val    =   map.occgrid(goal_sub(1),goal_sub(2),goal_sub(3));

expanded_nodes = 0;

if start_val ==1
    disp("the start position is in obstacle, assign new start position");
    return
end

if goal_val ==1
    disp("the goal position is in obstacle, assign new goal position");
    return
end

cost_mat(start_sub(1),start_sub(2),start_sub(3)) = 0;

%iter = 1;

while true
    
    
    [v,loc]     = min(cost_mat(:));
    [ii,jj,k]   = ind2sub(size(cost_mat),loc);
    curr_I      = loc;
    curr_sub    = [ii,jj,k];
    
    if curr_I == goal_ind
        %X = ["goal has been found at " , curr_I];
        %disp(X);
        break;
    end
    
    nebr_list = Conn_4(map,curr_I);
    for i = 1:6
        
        nebr_I = sub2ind(size_map,nebr_list(i,1),nebr_list(i,2),nebr_list(i,3));
        
        if visit_mat(nebr_list(i,1),nebr_list(i,2),nebr_list(i,3)) ==1
            %disp("already visited this node");
            continue;
        end 
        
    
        if nebr_I == curr_I
            %disp("skipped");
            continue;
        end
        
        
        if map.occgrid(nebr_list(i,1),nebr_list(i,2),nebr_list(i,3))==0
            
            expanded_nodes = expanded_nodes + 1;
            
            if astar == true
                cost = v + norm(sub2pos(map,nebr_list(i,:)) - sub2pos(map,curr_sub)) + norm(goal - sub2pos(map,nebr_list(i,:)));
            end
            
            if astar == false
                cost = v + norm(sub2pos(map,nebr_list(i,:)) - sub2pos(map,curr_sub));
            end
            
            if cost < cost_mat(nebr_list(i,1),nebr_list(i,2),nebr_list(i,3))
                
                cost_mat(nebr_list(i,1),nebr_list(i,2),nebr_list(i,3)) = cost;
                parent_path(nebr_list(i,1),nebr_list(i,2),nebr_list(i,3))=curr_I;
                %X = ["parent of" nebr_I, "is" , curr_I, "with cost" , cost];
                %disp(X);
                
            end 
            
        end
        
    end 
    cost_mat(curr_sub(1),curr_sub(2),curr_sub(3)) = 50000;
    visit_mat(curr_sub(1),curr_sub(2),curr_sub(3)) = 1;   
    
end




n = curr_I;
i =1;
while true 
    
    if n == start_ind
        break;
    end
    
    path(i,:) = ind2pos(map,n);
    [p,q,r] = ind2sub(size_map,n);
    n = parent_path(p,q,r);
    %disp(n);
    i=i+1; 
end 

if nargin < 4
    astar = false;
end
[m,n] = size(path);
path(1,:) = goal;
path(m+1,:) = start;
path = flipud(path);
%disp(path);


num_expanded = expanded_nodes;



if nargin < 4
    astar = false;
end

path = path;
num_expanded = 0;
end
