function is_it = check_coll(p2,p1,map)

t = 50;
col_chk=0;
array_pos = transpose(p1+(p2-p1)*linspace(0,1,t));
array_sub = pos2sub(map,array_pos);
%disp(array_sub);

  for q = 1:t
        
        col_chk = col_chk + map.occgrid(array_sub(q,1),array_sub(q,2),array_sub(q,3));
        
  end
  
  if col_chk==0
      is_it = false;
  else 
      is_it = true;
  end

end