function [new_path] = collinear_path(path,map)

    p1 = path(1,:);
    p2 = path(2,:);
    p3 = path(3,:);
    new_path(1,:)= p1;
    
    tn = 2;
    i = 3;
    len = size(path,1);
    
    
    while true

        if (i==len)
            break;
        end

        if(Collinear(p1,p2,p3))
            i= i+1;
            p2 = p3;
            p3 = path(i,:);
            
            
        else 
            i = i+1;
            new_path(tn,:) = p2;
            p1 = p2;
            p2 = p3;
            p3 = path(i,:);
            
            tn=tn+1;
            
        end

    end


end