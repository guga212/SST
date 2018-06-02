function [c,ceq, gradc, gradceq]=ConstraintFunction(a,X,D_min,d_min,a_0,v_0,x_0,t,V_max,a_max)
    
    dt=t/size(a,3);
    
    %Calculate all velocity of trajectory
    for i=1:size(a,1)
        v(i,1,1)=v_0(i,1)+a(i,1,1)*dt;
        v(i,2,1)=v_0(i,2)+a(i,2,1)*dt;
        for j=2:size(a,3)
            v(i,1,j)=v(i,1,j-1)+a(i,1,j)*dt;
            v(i,2,j)=v(i,2,j-1)+a(i,2,j)*dt;
        end
    end
    
    %Calculate all positions of trajectory
    for i=1:size(a,1)        
        x(i,1,1)=x_0(i,1)+0.5*(v_0(i,1)+v(i,1,1))*dt;
        x(i,2,1)=x_0(i,2)+0.5*(v_0(i,2)+v(i,2,1))*dt;
        for j=2:size(v,3)
            x(i,1,j)=x(i,1,j-1)+0.5*(v(i,1,j-1)+v(i,1,j))*dt;
            x(i,2,j)=x(i,2,j-1)+0.5*(v(i,2,j-1)+v(i,2,j))*dt;
        end
    end


    cur_ind=0; 
    ceq=[];
    c=[];
    
    %Minimal distance from main target in every position
    for i=1:size(x,1)
        for j=1:1
            cur_ind=cur_ind+1; 
            c(cur_ind)=D_min^2-(X(1,1)-x(i,1,j))^2-(X(1,2)-x(i,2,j))^2;
        end
    end
     
    %Minimal distance from other vehicles in every position
    for k=1:size(x,3)
    
        start_compare=2;    
        for i=1:size(x,1)        
  
            for j=start_compare:size(x,1)            
                cur_ind=cur_ind+1; 
                c(cur_ind)=d_min^2-(x(j,1,k)-x(i,1,k))^2-(x(j,2,k)-x(i,2,k))^2;
            end
       
            start_compare=start_compare+1;      
        end

    end

end