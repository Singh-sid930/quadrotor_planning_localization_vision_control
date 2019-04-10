function map = get_positions(id, lay)

len = length(id);


p1 = zeros(2,len);
p2 = zeros(2,len);
p3 = zeros(2,len);
p4 = zeros(2,len);
p0 = zeros(2,len);

id_layout = lay.tag_id;

p0x = lay.p0x;
p1x = lay.p1x;
p2x = lay.p2x;
p3x = lay.p3x;
p4x = lay.p4x;

p0y = lay.p0y;
p1y = lay.p1y;
p2y = lay.p2y;
p3y = lay.p3y;
p4y = lay.p4y;


for i=1:len
    
    [r,c] = find(id_layout==id(i));
    
    p1(1,i) = p1x(r);
    p1(2,i) = p1y(c);
    
    p2(1,i) = p2x(r);
    p2(2,i) = p2y(c);
    
    p3(1,i) = p3x(r);
    p3(2,i) = p3y(c);
    
    
    p4(1,i) = p4x(r);
    p4(2,i) = p4y(c);
    
    p0(1,i) = p0x(r);
    p0(2,i) = p0y(c);
    
    
end

map.p1 = p1;
map.p2 = p2;
map.p3 = p3;
map.p4 = p4;
map.p0 = p0;


end