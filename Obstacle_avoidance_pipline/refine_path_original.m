function [new_path,time_path] = refine_path(new_path1,map)

t = 50;
i = 2;
r = 2;

p1 = transpose(new_path1(1,:));
p3 = transpose(new_path1(2,:));

col_chk = 0;
new_path(1,:) = p1;
time_path(1) = 0;
dist_path(1) = 0;
total_dist = 0;
total_time = 0;
m = 1;

while (true)
    
    col_chk=0;
    
    if (i-1)==length(new_path1)
        break;
    end
    
    array_pos = transpose(p1+(p3-p1)*linspace(0,1,t));
    array_sub = pos2sub(map,array_pos);
    
    
    for q = 1:t
        
        col_chk = col_chk + map.occgrid(array_sub(q,1),array_sub(q,2),array_sub(q,3));
        
    end
    
    if col_chk ==0 
        
        i=i+1;
        p3 = transpose(new_path1(i,:));
        
    else
        
        %i=i-1;
        p2 = transpose(new_path1(i-1,:));
        
        new_path(r,:) = p2;
        
        
        dist = norm(p2-p1);
        total_dist = total_dist +dist;
        dist_path(r) = total_dist;
        total_time = total_time + dist/1.7;
        time_path(r) = total_time;
        
        r=r+1;
        
        
        p1 = p2;
        if i ==length(new_path1)
            break
        end
        p3 = transpose(new_path1(i+1,:));
        i=i+1;
        
    end 
end



end