function is_it = Collinear(point1,point2,point3)

if (norm(point2-point1)+norm(point3-point2)==norm(point3-point1))    
    is_it = true;
else 
    is_it = false;
end

end