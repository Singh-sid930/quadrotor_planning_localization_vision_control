function nbr_list = Conn_4(map,I)

size_map    =   [size(map.occgrid,1),size(map.occgrid,2),size(map.occgrid,3)];
[i,j,k] = ind2sub(size_map,I);
sub_pos = [i,j,k];




n1 = [sub_pos(1)-1,sub_pos(2),sub_pos(3)];
if (ismember(0,n1))
    n1 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end


n2 = [sub_pos(1),sub_pos(2)-1,sub_pos(3)];
if (ismember(0,n2))
    n2 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n3 = [sub_pos(1),sub_pos(2),sub_pos(3)-1];
if (ismember(0,n3))
    n3 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n4 = [sub_pos(1)+1,sub_pos(2),sub_pos(3)];
if (ismember(0,n4-size_map))
    n4 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n5 = [sub_pos(1),sub_pos(2)+1,sub_pos(3)];
if (ismember(0,n5-size_map))
    n5 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n6 = [sub_pos(1),sub_pos(2),sub_pos(3)+1];
if (ismember(0,n6-size_map))
    n6 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end 

n7 = [sub_pos(1)+1,sub_pos(2)+1,sub_pos(3)];
if (ismember(0,n7-size_map))
    n7 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n8 = [sub_pos(1)-1,sub_pos(2)-1,sub_pos(3)];
if (ismember(0,n8))
    n8 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n9 = [sub_pos(1)+1,sub_pos(2)-1,sub_pos(3)];
if (ismember(0,n9) ||ismember(0,n9-size_map))
    n9 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end


n10 = [sub_pos(1)-1,sub_pos(2)+1,sub_pos(3)];
if (ismember(0,n10) ||ismember(0,n10-size_map))
    n10 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end



nbr_list = [n1;n2;n3;n4;n5;n6;n7;n8;n9;n10];



end
