function [out,grad]=TargetFunction(a,X,D,d,v_0,x_0,t)
    
    out=0;
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
    
    %Calculate final position
    for i=1:size(a,1)        
      
        x(i,1)=x_0(i,1)+(v_0(i,1)+v(i,1,1))*0.5*dt;
        x(i,2)=x_0(i,2)+(v_0(i,2)+v(i,2,1))*0.5*dt;
        
        for j=2:size(a,3)            
                x(i,1)=x(i,1)+(v(i,1,j-1)+v(i,1,j))*0.5*dt;
                x(i,2)=x(i,2)+(v(i,2,j-1)+v(i,2,j))*0.5*dt;
        end
        
    end
    
    %Minimize distance from main target in the final point
    for i=1:size(a,1)        
        out=out+(D(i)^2-(X(1,1)-x(i,1))^2-(X(1,2)-x(i,2))^2)^2;
    end
    
    %Minimize formation error in the final point
    for i=1:size(a,1)       
        for j=1:size(a,1)            
            weight_ferr=1.0;         
            out=out+weight_ferr*(d(i,j)^2-(x(i,1)-x(j,1))^2-(x(i,2)-x(j,2))^2)^2;                    
        end
    end
    
    %Minimize velocity square(loss function)
    for i=1:size(v,1)        
        for j=1:size(v,3)
            weight_vel=0.01;
            out=out+weight_vel*(v(i,1,j)^2+v(i,2,j)^2);
        end
    end

    %Gradient required
    if nargout > 1 
        for i=1:size(v,1)
           for j=1:size(v,3)
               
               % Gradient minimize distance from main trgt
               grad1(i,1,j)=0;
               grad1(i,2,j)=0;
               grad1(i,1,j)=0.5*dt^2;
               grad1(i,2,j)=0.5*dt^2;
               for k=1:size(v,3)-j
                   grad1(i,1,j)=grad1(i,1,j)+dt^2;
                   grad1(i,2,j)=grad1(i,2,j)+dt^2;
               end
               grad1(i,1,j)=2*(D(i)^2-(X(1,1)-x(i,1))^2-(X(1,2)-x(i,2))^2)*-2*(X(1,1)-x(i,1))*-grad1(i,1,j);
               grad1(i,2,j)=2*(D(i)^2-(X(1,1)-x(i,1))^2-(X(1,2)-x(i,2))^2)*-2*(X(1,2)-x(i,2))*-grad1(i,2,j);
               
               
               % Gradient minimize formation error
               grad2(i,1,j)=0;
               grad2(i,2,j)=0;                                                                        
               grad2_comp(i,1,j)=0.5*dt^2;
               grad2_comp(i,2,j)=0.5*dt^2;
               for k=1:size(v,3)-j
                    grad2_comp(i,1,j)=grad2_comp(i,1,j)+dt^2;
                    grad2_comp(i,2,j)=grad2_comp(i,2,j)+dt^2;
               end
               for n=1:size(a,1) 
                   grad2(i,1,j)= grad2(i,1,j)+2*weight_ferr*(d(i,n)^2-(x(i,1)-x(n,1))^2-(x(i,2)-x(n,2))^2)*-2*(x(i,1)-x(n,1))*grad2_comp(i,1,j);
                   grad2(i,2,j)= grad2(i,2,j)+2*weight_ferr*(d(i,n)^2-(x(i,1)-x(n,1))^2-(x(i,2)-x(n,2))^2)*-2*(x(i,2)-x(n,2))*grad2_comp(i,2,j);
               end
               grad2(i,1,j)=grad2(i,1,j)*2; %%multiply by two cause target function do same error calculation twice
               grad2(i,2,j)=grad2(i,2,j)*2; %%multiply by two cause target function do same error calculation twice
               
               %Gradient minimize velocity square(loss function)
               grad3(i,1,j)=0;
               grad3(i,2,j)=0;
               for k=j:size(v,3)
                    grad3(i,1,j)=grad3(i,1,j)+2*weight_vel*v(i,1,k)*dt;
                    grad3(i,2,j)=grad3(i,2,j)+2*weight_vel*v(i,2,k)*dt;
               end
               
               
               %%Summarize gradients
               grad(i,1,j)=grad1(i,1,j)+grad2(i,1,j)+grad3(i,1,j);
               grad(i,2,j)=grad1(i,2,j)+grad2(i,2,j)+grad3(i,2,j);
           end           
        end
    end
    
end