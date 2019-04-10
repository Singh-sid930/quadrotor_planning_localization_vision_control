function velocity = RANSAC(F_mat,u_mat)


sample_points = 3;
len = size(F_mat,1);

perc_inlier_max = 0;

num_iter = 100;
F_mat_opt = F_mat;
u_mat_opt = u_mat;

for iter = 1:50
    
    
sample_index  = randsample(len,sample_points);



for i = 1:3
    if(mod(sample_index(i),2)==0 && sample_index(i)~=1)
        sample_index(i) = sample_index(i)-1;
    end
end



sample_index = [sample_index(1),sample_index(1)+1,sample_index(2),sample_index(2)+1,sample_index(3),sample_index(3)+1];
F_sample = F_mat(sample_index,:);
u_sample = u_mat(sample_index,:);


vel_sample = F_sample\u_sample;

error_mat = sqrt((u_mat.^2 - (F_mat*vel_sample).^2));



inlier_index = error_mat<0.005;

F_inlier = F_mat(inlier_index,:);
u_inlier = u_mat(inlier_index,:);



num_inlier = size(F_inlier,1);
num_total = size(F_mat,1);

perc_inlier = num_inlier/num_total;

%disp(perc_inlier);

if(perc_inlier>0.8)
    F_mat_opt = F_inlier;
    u_mat_opt = u_inlier;
    break;
end

if(perc_inlier>perc_inlier_max)
    perc_inlier_max = perc_inlier;
    F_mat_opt = F_inlier;
    u_mat_opt = u_inlier;
end
%disp(size(u_mat_opt));

end


velocity = F_mat_opt\u_mat_opt;


end 
