function nbr_list = Conn_26(map,I)

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


n11 = [sub_pos(1),sub_pos(2)+1,sub_pos(3)+1];
if (ismember(0,n11-size_map))
    n11 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n12 = [sub_pos(1),sub_pos(2)-1,sub_pos(3)-1];
if (ismember(0,n12))
    n12 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n13 = [sub_pos(1),sub_pos(2)-1,sub_pos(3)+1];
if (ismember(0,n13) ||ismember(0,n13-size_map))
    n13 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end


n14 = [sub_pos(1),sub_pos(2)+1,sub_pos(3)-1];
if (ismember(0,n14) ||ismember(0,n14-size_map))
    n14 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end


n15 = [sub_pos(1)+1,sub_pos(2),sub_pos(3)+1];
if (ismember(0,n15-size_map))
    n15 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n16 = [sub_pos(1)-1,sub_pos(2),sub_pos(3)-1];
if (ismember(0,n16))
    n16 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n17 = [sub_pos(1)-1,sub_pos(2),sub_pos(3)+1];
if (ismember(0,n17) ||ismember(0,n17-size_map))
    n17 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end


n18 = [sub_pos(1)+1,sub_pos(2),sub_pos(3)-1];
if (ismember(0,n18) ||ismember(0,n18-size_map))
    n18 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n19 = [sub_pos(1)+1,sub_pos(2)+1,sub_pos(3)+1];
if (ismember(0,n19-size_map))
    n19 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n20 = [sub_pos(1)-1,sub_pos(2)+1,sub_pos(3)-1];
if (ismember(0,n20)||ismember(0,n20-size_map))
    n20 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n21 = [sub_pos(1)-1,sub_pos(2)+1,sub_pos(3)+1];
if (ismember(0,n21) ||ismember(0,n21-size_map))
    n21 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end


n22 = [sub_pos(1)+1,sub_pos(2)+1,sub_pos(3)-1];
if (ismember(0,n22) ||ismember(0,n22-size_map))
    n22 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end


n23 = [sub_pos(1)+1,sub_pos(2)-1,sub_pos(3)+1];
if (ismember(0,n23-size_map)||ismember(0,n23))
    n23 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n24 = [sub_pos(1)-1,sub_pos(2)-1,sub_pos(3)-1];
if (ismember(0,n24))
    n24 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end

n25 = [sub_pos(1)-1,sub_pos(2)-1,sub_pos(3)+1];
if (ismember(0,n25) ||ismember(0,n25-size_map))
    n25 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end


n26 = [sub_pos(1)+1,sub_pos(2)-1,sub_pos(3)-1];
if (ismember(0,n26) ||ismember(0,n26-size_map))
    n26 = [sub_pos(1),sub_pos(2),sub_pos(3)];
end


nbr_list = [n1;n2;n3;n4;n5;n6;n7;n8;n9;n10;n11;n12;n13;n14;n15;n16;n17;n18;n19;n20;n21;n22;n23;n24;n25;n26];



end
