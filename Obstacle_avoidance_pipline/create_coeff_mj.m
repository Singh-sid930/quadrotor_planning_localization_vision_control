function coeff_list = create_coeff_mj(new_path,time_path)

len = length(time_path);
num_trajs = len-1;
p =len-1;

dim = num_trajs * 6;
disp(dim);
R1 = [time_path(1)^5,time_path(1)^4,time_path(1)^3,time_path(1)^2,time_path(1),1,zeros(1,6*(num_trajs-1))];
R2 = [time_path(2)^5,time_path(2)^4,time_path(2)^3,time_path(2)^2,time_path(2),1,zeros(1,6*(num_trajs-1))];
R3 = [5*time_path(1)^4,4*time_path(1)^3,3*time_path(1)^2,2*time_path(1),1,0,zeros(1,6*(num_trajs-1))];
R4 = [5*time_path(1)^4,4*time_path(1)^3,3*time_path(1)^2,2*time_path(1),1,0,-5*time_path(2)^4,-4*time_path(2)^3,-3*time_path(2)^2,-2*time_path(2),-1,0,zeros(1,6*(num_trajs-2))];
R5 = [20*time_path(1)^3,12*time_path(1)^2,6*time_path(1),2,0,0,zeros(1,6*(num_trajs-1))];
R6 = [20*time_path(1)^3,12*time_path(1)^2,6*time_path(1),2,0,0,-20*time_path(2)^3,-12*time_path(2)^2,-6*time_path(2),-2,0,0,zeros(1,6*(num_trajs-2))];
A = [R1;R2;R3;R4;R5;R6];
n = 0;
q=1;
%disp(time_path);
while(q<num_trajs-1)
    R1 = [];
    R2 = [];
    R3 = [];
    R4 = [];
    R5 = [];
    R6 = [];
    
    
       R1((q*6)+1:dim) = [time_path(q+1)^5,time_path(q+1)^4,time_path(q+1)^3,time_path(q+1)^2,time_path(q+1),1,zeros(1,6*(num_trajs-(q+1)))];
       R2((q*6)+1:dim) = [time_path(q+2)^5,time_path(q+2)^4,time_path(q+2)^3,time_path(q+2)^2,time_path(q+2),1,zeros(1,6*(num_trajs-(q+1)))]; 
       R3((q*6)+1:dim) = [5*time_path(q)^4,4*time_path(q)^3,3*time_path(q)^2,2*time_path(q),1,0,-5*time_path(q+1)^4,-4*time_path(q+1)^3,-3*time_path(q+1)^2,-2*time_path(q+1),-1,0,zeros(1,6*(num_trajs-(q+2)))];
       R4((q*6)+1:dim) = [5*time_path(q+1)^4,4*time_path(q+1)^3,3*time_path(q+1)^2,2*time_path(q+1),1,0,-5*time_path(q+2)^4,-4*time_path(q+2)^3,-3*time_path(q+2)^2,-2*time_path(q+2),-1,0,zeros(1,6*(num_trajs-(q+2)))];
       R5((q*6)+1:dim) = [20*time_path(q)^3,12*time_path(q)^2,6*time_path(q),2,0,0,-20*time_path(q+1)^3,-12*time_path(q+1)^2,-6*time_path(q+1),-2,0,0,zeros(1,6*(num_trajs-(q+2)))];
       R6((q*6)+1:dim) = [20*time_path(q+1)^3,12*time_path(q+1)^2,6*time_path(q+1),2,0,0,-20*time_path(q+2)^3,-12*time_path(q+2)^2,-6*time_path(q+2),-2,0,0,zeros(1,6*(num_trajs-(q+2)))];
       A = [A;R1;R2;R3;R4;R5;R6];
           
       q = q+1;    
          
           
end

    R1 = [];
    R2 = [];
    R3 = [];
    R4 = [];
    R5 = [];
    R6 = [];
    
R1(dim-12+1:dim) = [time_path(p)^5,time_path(p)^4,time_path(p)^3,time_path(p)^2,time_path(p),1,zeros(1,6)];
R2(dim-6+1:dim) = [time_path(p+1)^5,time_path(p+1)^4,time_path(p+1)^3,time_path(p+1)^2,time_path(p+1),1];
R3(dim-12+1:dim) = [5*time_path(p)^4,4*time_path(p)^3,3*time_path(p)^2,2*time_path(p),1,0,-5*time_path(p+1)^4,-4*time_path(p+1)^3,-3*time_path(p+1)^2,-2*time_path(p+1),-1,0];
R4(dim-6+1:dim) = [5*time_path(p+1)^4,4*time_path(p+1)^3,3*time_path(p+1)^2,2*time_path(p+1),1,0];
R5(dim-12+1:dim) = [20*time_path(p)^3,12*time_path(p)^2,6*time_path(p),2,0,0,-20*time_path(p+1)^3,-12*time_path(p+1)^2,-6*time_path(p+1),-2,0,0];
R6(dim-6+1:dim) = [20*time_path(p+1)^3,12*time_path(p+1)^2,6*time_path(p+1),2,0,0];
A = [A;R1;R2;R3;R4;R5;R6];

coeff_list = A;
          

end