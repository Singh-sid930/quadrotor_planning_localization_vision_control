function nbr_list = Conn_4(map,I)

size_map    =   size(map.occgrid);
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
if (ismember(size_map(1)+1,n4))
    n4 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n5 = [sub_pos(1),sub_pos(2)+1,sub_pos(3)];
if (ismember(size_map(2)+1,n5))
    n5 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n6 = [sub_pos(1),sub_pos(2),sub_pos(3)+1];
if (ismember(size_map(3)+1,n6))
    n6 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

nbr_list = [n1;n2;n3;n4;n5;n6];



end
