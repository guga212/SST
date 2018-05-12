
function simpleTest()
    clear();
     disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');

        % enable the synchronous mode on the client:
        vrep.simxSynchronous(clientID,true);

        % start the simulation:
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);

        % Now step a few times:
        N=4;
        sim_dt=0.05;
        sim_t=0;
        idx=1;
        t=0.05;
        a_0=zeros(N,2,t/sim_dt);   
        v_0=zeros(N,2);   
        
        while(1)
            
            if(sim_t<=0.1*sim_dt)
                [main_trgt_posd, main_trgt_pos]=GetDisturbedMainTrgtPosition(vrep,clientID);
                [xd_0, x_0]=GetDisturbedQuadPosition(vrep,clientID,N);
             
                [trgt_alg_x, trgt_alg_y]=OptimizeNextMove( main_trgt_pos(1:2), [1.41;1.41;1.41;1.41 ].*1.0, [0 2 2.828 2; 2 0 2 2.828; 2.828 2 0 2; 2 2.828 2 0].*1.0,a_0,v_0,x_0(:,1:2),t,1.2, 1.2, 1, 6);
                %[trgt_alg_x, trgt_alg_y]=OptimizeNextMove( main_trgt_pos(1:2), [1.41;1;1.41;1;1.41;1;1.41;1].*1.8, [0 1 2 2.236 2.828 2.236 2 1; 1 0 1 1.41 2.236 2 2.236 1.41;  2 1 0 1 2 2.236 2.828 2.236; 2.236 1.41 1 0 1 1.41 2.236 2; 2.828 2.236 2 1 0 1 2 2.236; 2.828 2 2.236 1.41 1 0 1 1.41; 2 2.236 2.828 2.236 2 1 0 1; 1 1.41 2.236 2 2.236 1.41 1 0; ].*1.8, v_0,x_0(:,1:2),t,1.4, 1.6, 1, 6);
                a_0=[trgt_alg_x(:,end), trgt_alg_y(:,end)];               
                sim_t=t;
                idx=1;
                posX=squeeze(x_0(:,1));
                posY=squeeze(x_0(:,2));
            end
            
            for i=0:N-1
                posX(i+1)=posX(i+1)+sim_dt*v_0(i+1,1)+0.5*trgt_alg_x(i+1,idx)*sim_dt^2;
                posY(i+1)=posY(i+1)+sim_dt*v_0(i+1,2)+0.5*trgt_alg_y(i+1,idx)*sim_dt^2;
                SetQuadTrgtPos(vrep, clientID, i, [posX(i+1) posY(i+1) x_0(i+1,3)]);                                
            end
            
            vrep.simxSynchronousTrigger(clientID);
            sim_t=sim_t-sim_dt;
            v_0(:,1)=v_0(:,1)+trgt_alg_x(:,idx)*sim_dt;
            v_0(:,2)=v_0(:,2)+trgt_alg_y(:,idx)*sim_dt;
            idx=idx+1;
            
        end

        % stop the simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);

        % Now close the connection to V-REP:    
        vrep.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!
    
    disp('Program ended');
end

function [dpos, pos]=GetDisturbedMainTrgtPosition(VrepAPI, ClientID)
    dpos=zeros(1,3);
    pos=GetMainTrgtPos(VrepAPI, ClientID);
    dpos(1,:)=pos+(rand(1,3)-ones(1,3)).*pos*0.05;
end

function [dpos, pos]=GetDisturbedQuadPosition(VrepAPI, ClientID, size)
    dpos=zeros(size,3);
    pos=zeros(size,3);
    for i=1:size
        pos(i,:)=GetQuadTrgtPos(VrepAPI, ClientID,i-1);
        dpos(i,:)=pos(i,:)+(rand(1,3)-ones(1,3)).*pos(i,:)*0.05;        
    end
end


 
%X zmienna celu
%D zadana odleglosc do celu 
%d zadana macierz odleglosc do sasiada
%v_0 predkosc poczatkowa
%x_0 pozycja poczatkowa
%t horyzont predykcji
%D_min minimalna odleglosc do celu
%d_min minimalna odleglosc do sasiada
%V_max maksymalna predkosc
%a_max maksymalne przyspieszenie
function [Vx,Vy]=OptimizeNextMove(X,D,d,a_0,v_0,x_0,t,D_min,d_min,V_max,a_max)
    tfun = @(a)TargetFunction(a,X,D,d,v_0,x_0,t);
   %[res, minval]=fminunc(tfun, a_0);

    
    cfun=@(a)ConstraintFunction(a,X,D_min,d_min,a_0,v_0,x_0,t,V_max,a_max);
    %options = optimoptions('fmincon','Display','off');
    %options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',24000,'MaxIterations', 3200);
    options = optimoptions('fmincon','Algorithm','sqp', 'SpecifyObjectiveGradient',false);
    [res, minval,exitflag,output,lambda,grad]=fmincon(tfun,a_0,[],[],[],[],[],[],cfun,options);

    Vx=squeeze(res(:,1,:));
    Vy=squeeze(res(:,2,:));
end

%a zmienne przysp 
%X zmienna celu
%D zadana odleglosc do celu 
%d zadana macierz odleglosc do sasiada
%v_0 predkosc poczatkowa
%x_0 pozycja poczatkowa
%t horyzont predykcji
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
            weight=1.0;         
            out=out+weight*(d(i,j)^2-(x(i,1)-x(j,1))^2-(x(i,2)-x(j,2))^2)^2;                    
        end
    end
    
    %Minimize velocity square(loss function)
    for i=1:size(v,1)        
        for j=1:size(v,3)
            weight=0.01;
            out=out+weight*(v(i,1,j)^2+v(i,2,j)^2);
        end
    end
    
    %Gradient required
    if nargout > 1 
        for i=1:size(v,1)
           for j=1:size(v,3)
               
               % Gradient minimize distance from main trgt
               grad1(i,1,j)=0.5*dt^2;
               grad1(i,2,j)=0.5*dt^2;
               for k=1:size(v,3)-1
                   grad1(i,1,j)=grad1(i,1,j)+dt;
                   grad1(i,2,j)=grad1(i,2,j)+dt;
               end
               grad1(i,1,j)=2*(D(i)^2-(X(1,1)-x(i,1))^2)*2*(X(1,1)-x(i,1))*-grad1(i,1,j);
               grad1(i,2,j)=2*(D(i)^2-(X(1,2)-x(i,2))^2)*2*(X(1,2)-x(i,2))*-grad1(i,2,j);
               
               
               % Gradient minimize formation error
               grad2(i,1,j)=0;
               grad2(i,2,j)=0;
               
               for n=1:size(a,1)               
                   grad2(i,1,j)=grad2(i,1,j)+0.5*dt^2;
                   grad2(i,2,j)=grad2(i,2,j)+0.5*dt^2;
                   for k=1:size(v,3)-1
                       grad2(i,1,j)=grad2(i,1,j)+dt;
                       grad2(i,2,j)=grad2(i,2,j)+dt;
                   end               
                   grad2(i,1,j)=2*weight*(d(i,n)^2-(x(i,1)-x(n,1))^2)*2*(x(i,1)-x(n,1))*-grad2(i,1,j);
                   grad2(i,2,j)=2*weight*(d(i,n)^2-(x(i,2)-x(n,2))^2)*2*(x(i,2)-x(n,2))*-grad2(i,1,j);
               end
               
               
               
               % Gradient minimize formation error
               grad3(i,1,j)=0;
               grad3(i,2,j)=0;
                 for k=1:size(v,3)-1
                   grad3(i,1,j)=grad3(i,1,j)+dt;
                   grad3(i,2,j)=grad3(i,2,j)+dt;
                 end
               grad3(i,1,j)=2*weight*v(i,1,j)*grad3(i,1,j);
               grad3(i,2,j)=2*weight*v(i,2,j)*grad3(i,2,j);
               
               grad(i,1,j)=grad1(i,1,j)+grad2(i,1,j)+grad3(i,1,j);
               grad(i,2,j)=grad1(i,2,j)+grad2(i,2,j)+grad3(i,2,j);
           end           
        end
    end
    
end


%a zmienne przysp
%X zmienna celu
%D_min minimalna odleglosc do celu
%d_min minimalna odleglosc do sasiada
%x_0 pozycja poczatkowa
%t horyzont predykcji
%V_max maksymalna predkosc
%a_max maksymalne przyspieszenie
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
    
    %Maximal velocity constraint
    for k=1:size(v,3)    
        for i=1:size(v,1)                               
            cur_ind=cur_ind+1; 
            c(cur_ind)=v(i,1,k)^2+v(i,2,k)^2-V_max^2;
        end
    end
     
    %Maximal acceleration
    for i=1:size(a,1)
        for j=1:size(a,3)
            cur_ind=cur_ind+1;          
            c(cur_ind)=a(i,1,j)^2+a(i,2,j)^2-a_max^2;
        end
    end
    
    
   % Gradient of constraints
   % if nargout>2
%         v_symb=sym(v);
%         c_symb=sym(c);
%         gradceq=[];
%         gradc=[];
   % end
   
     %Minimal distance from main target in every position
%     for i=1:size(x,1)
%         for j=1:1
%             cur_ind=cur_ind+1; 
%             dx2=(X(1,1)-x(i,1,j))^2;
%             dy2=(X(1,2)-x(i,2,j))^2;
%             sum=dx2+dy2;
%             c(cur_ind)=D_min^2-sum;
%             %c(cur_ind)=A^2-(X(1,1)-x(i,1,j))^2-(X(1,2)-x(i,2,j))^2;
%         end
%     end
    
end

function Position = GetMainTrgtPos(VrepAPI, ClientID)
    MainTrgtName='Bill_base';
    persistent MainTrgtHandle;
    if isempty(MainTrgtHandle)
        [return_code, MainTrgtHandle]=VrepAPI.simxGetObjectHandle(ClientID, MainTrgtName, VrepAPI.simx_opmode_blocking);
    end
    [return_code, Position]=VrepAPI.simxGetObjectPosition (ClientID, MainTrgtHandle, -1, VrepAPI.simx_opmode_blocking);
    Position=double(Position);
end

function SetQuadTrgtPos(VrepAPI, ClientID, QuadNumber, Position)
    persistent QuadTrgtHandle;
    persistent Number;
    
    StdQuadTrgtName='Quadricopter_target#';
    QuadNumber=num2str(QuadNumber);
    QuadTrgtName=strcat(StdQuadTrgtName, QuadNumber);

    if isempty(QuadTrgtHandle) || Number~=QuadNumber
        [return_code, QuadTrgtHandle]=VrepAPI.simxGetObjectHandle(ClientID, QuadTrgtName, VrepAPI.simx_opmode_blocking);
    end
    return_code=VrepAPI.simxSetObjectPosition(ClientID, QuadTrgtHandle, -1, Position, VrepAPI.simx_opmode_blocking);
    
    Number=QuadNumber;
end

function Position = GetQuadTrgtPos(VrepAPI, ClientID, QuadNumber)
    persistent QuadTrgtHandle;
    persistent Number;

    StdQuadTrgtName='Quadricopter_target#';
    %StdQuadTrgtName='Quadricopter_base#';
    QuadNumber=num2str(QuadNumber);
    QuadTrgtName=strcat(StdQuadTrgtName, QuadNumber);
    
    if isempty(QuadTrgtHandle) || Number~=QuadNumber
    [return_code, QuadTrgtHandle]=VrepAPI.simxGetObjectHandle(ClientID, QuadTrgtName, VrepAPI.simx_opmode_blocking);
    end
    [return_code, Position]=VrepAPI.simxGetObjectPosition (ClientID, QuadTrgtHandle, -1, VrepAPI.simx_opmode_blocking);
    Position=double(Position);
    Number=QuadNumber;
end