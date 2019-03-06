function coeff_list = create_coeff(time_path)

len = length(time_path);
 
for q = 1:len-1
           T = time_path(q+1)-time_path(q);
           if T < 0.5
               T = 1.5;
           end
           A = [T^3,T^4,T^5;3*T^2,4*T^3,5*T^4;6*T,12*T^2,20*T^3];
           A = inv(A);
           coeff = A*[1;0;0];
           coeff_list(:,q) = coeff; 

end

end