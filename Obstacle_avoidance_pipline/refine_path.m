function [new_path,time_path] = refine_path(new_path1,map)

t = 70;
i = 2;
r = 2;

cur_p = transpose(new_path1(1,:));



path_check = zeros(1,length(new_path1));

path_check(1) = 1;


while (true)
    
   if i == length(new_path1)
        break;
   end 
   
   col_chk = 0;
   
   
   i = i+1;
   chk_p = transpose(new_path1(i,:));
   
   array_pos = transpose(cur_p+(chk_p-cur_p)*linspace(0,1,t));
   array_ind = pos2ind(map,array_pos);
   
   col_chk = sum(map.occgrid(array_ind));
   
   if col_chk>0
       disp("found a collision");
       path_check(i-1) = 1;
       cur_p = transpose(new_path1(i-1,:));
       continue;
       
   end
   
  
     
end

path_check(length(path_check)) = 1;


path = transpose(path_check).*new_path1;
path1 = path(find(any(path,2)),:);

time_path(1) = 0;
total_dist = 0;
total_time = 0;
for i = 1:length(path1)-1
    p1 = path1(i,:);
    p2 = path(i+1,:);
    
    dist = norm(p2-p1);
    total_dist = total_dist +dist;
    total_time = total_time + sqrt(dist/1.5);
    time_path(i+1) = total_time;
    
end


new_path = path1;


disp(path1);

end