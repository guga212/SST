function [a_x,a_y]=OptimizeNextMove(X,D,d,a_0,v_0,x_0,t,D_min,d_min,V_max,a_max)
    tfun = @(a)TargetFunction(a,X,D,d,v_0,x_0,t);
    cfun=@(a)ConstraintFunction(a,X,D_min,d_min,a_0,v_0,x_0,t,V_max,a_max);
    
    %options = optimoptions('fmincon','Display','off');
    %options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',24000,'MaxIterations', 3200);
    options = optimoptions('fmincon','Algorithm','sqp', 'SpecifyObjectiveGradient', true);
    
    A=zeros(2*size(a_0,1)*size(a_0,2)*size(a_0,3),size(a_0,1)*size(a_0,2)*size(a_0,3));
    b=zeros(2*size(a_0,1)*size(a_0,2)*size(a_0,3),1);
    
    global_indx=0;
    
    for n=1:2
        if n>1
            mult=-1;
        else
            mult=1;
        end
        var_indx=0;
        for z=1:size(a_0,3)
            for j=1:size(a_0,2)
                for i=1:size(a_0,1)
                    var_indx= var_indx+1;
                    global_indx= global_indx+1;
                    b(global_indx)=V_max-mult*v_0(i,j);
                    for k=0:z-1
                        A(global_indx,var_indx-k)=mult*t/size(a_0,3);
                    end
                end
            end
        end
    end
    
    
    a_ub=(a_0.*0)+a_max;
    a_lb=(a_0.*0)-a_max;
    
    %%Solver
    [res, minval,exitflag,output,lambda,grad]=fmincon(tfun,a_0,A,b,[],[],a_lb,a_ub,cfun,options);
        
    %Global minimum
%     rng default 
%     gs = GlobalSearch;
%     problem = createOptimProblem('fmincon','x0',a_0,'objective',tfun,'Aineq',A,'bineq',b,'lb',a_lb,'ub',a_ub,'nonlcon',cfun);
%     res = run(gs,problem);
    
    a_x=squeeze(res(:,1,:));
    a_y=squeeze(res(:,2,:));
end