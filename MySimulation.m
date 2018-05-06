
function simpleTest()
    clear();
    disp('Program started');

    vrep=remApi('remoteApi');
    vrep.simxFinish(-1);
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');

        
        figure(1);
        H = uicontrol('Style', 'PushButton','String', 'Break', 'Callback', 'delete(gcbf)');   
        while (ishandle(H))
            
            tic;
            
            main_trgt_pos=GetMainTrgtPos(vrep,clientID);
           
            x_0=zeros(4,2);
            
            x_0f(1,:)=GetQuadTrgtPos(vrep, clientID, 0)';
            x_0(1,:)=x_0f(1,1:2);
            
            x_0f(2,:)=GetQuadTrgtPos(vrep, clientID, 1)';
            x_0(2,:)=x_0f(2,1:2);
            
            x_0f(3,:)=GetQuadTrgtPos(vrep, clientID, 2)';
            x_0(3,:)=x_0f(3,1:2);
            
            x_0f(4,:)=GetQuadTrgtPos(vrep, clientID, 3)';
            x_0(4,:)=x_0f(4,1:2);
            
            [trgt_alg_x, trgt_alg_y]=OptimizeNextMove([double(main_trgt_pos(1)) double(main_trgt_pos(2))], [1.41;1.41;1.41;1.41 ].*0.9, [2;2;2;2].*0.9, x_0, 1.0, 1.2, 0.7);
            
            for i=0:3
                
                SetQuadTrgtPos(vrep, clientID, i, [trgt_alg_x(i+1) trgt_alg_y(i+1) x_0f(i+1,3)]);                
                
            end
            
            elapsed_time=toc;
        end
        

        % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(clientID);

        % Now close the connection to V-REP:    
        vrep.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!
    
    disp('Program ended');
end


function [x,y]=OptimizeNextMove(X,D,d,x_0,A,a,dx)
    tfun = @(x)TargetFunction(x,X,D,d);
    cfun=@(x)ConstraintFunction(x,X,A,a,x_0,dx);
    options = optimoptions('fmincon', 'MaxFunctionEvaluations', 4000);
    res=fmincon(tfun,x_0,[],[],[],[],[],[],cfun,options);
    x=res(:,1);
    y=res(:,2);
end

%x zmienne pozycji 
%X zmienna celu
%D zadana odleglosc do celu 
%d zadana odleglosc do sasiada
function out=TargetFunction(x,X,D,d)
    out=0;
    
    for i=1:size(x,1)
        out=out+(D(i)^2-(X(1,1)-x(i,1))^2-(X(1,2)-x(i,2))^2)^2;
    end
    
    for i=1:size(x,1)
        
        index_neighb_1=size(x,1);
        index_neighb_2=i+1;
            
        if i==1
            index_neighb_1=size(x,1);
        end
        
        if i==size(x,1)
            index_neighb_2=1;
        end
        
        weight=1.0;
        
        out=out+weight*(d(i)^2-(x(i,1)-x(index_neighb_1,1))^2-(x(i,2)-x(index_neighb_1,2))^2)^2;
        out=out+weight*(d(i)^2-(x(i,1)-x(index_neighb_2,1))^2-(x(i,2)-x(index_neighb_2,2))^2)^2;
        
    end
    
end


%x zmienne pozycji 
%X zmienna celu
%A minimalna odleglosc do celu
%d minimalna odleglosc do sasiada
function [c,ceq]=ConstraintFunction(x,X,A,a,x_0,dx)
    
     cur_ind=0; 
     %c=zeros(2*size(x,1)+(0.5+0.5*size(x,1))*size(x,1),1);
     ceq=[];
     
    for i=1:size(x,1)
         cur_ind=cur_ind+1; 
         c(cur_ind)=A^2-(X(1,1)-x(i,1))^2-(X(1,2)-x(i,2))^2;
    end
    
    start_compare=2;    
   for i=1:size(x,1)        
  
       for j=start_compare:size(x,1)
         cur_ind=cur_ind+1; 
         c(cur_ind)=a^2-(x(j,1)-x(i,1))^2-(x(j,2)-x(i,2))^2;
       end
       
       start_compare=start_compare+1;      
    end
    
    for i=1:size(x,1)
        cur_ind=cur_ind+1; 
        c(cur_ind)=(x_0(i,1)-x(i,1))^2+(x_0(i,2)-x(i,2))^2-dx^2;
    end
    
end

function Position = GetMainTrgtPos(VrepAPI, ClientID)
    MainTrgtName='Bill_base';
    persistent MainTrgtHandle;
    %if isempty(MainTrgtHandle)
        [return_code, MainTrgtHandle]=VrepAPI.simxGetObjectHandle(ClientID, MainTrgtName, VrepAPI.simx_opmode_blocking);
    %end
    [return_code, Position]=VrepAPI.simxGetObjectPosition (ClientID, MainTrgtHandle, -1, VrepAPI.simx_opmode_blocking);
end

function SetQuadTrgtPos(VrepAPI, ClientID, QuadNumber, Position)
    StdQuadTrgtName='Quadricopter_target#';
    QuadNumber=num2str(QuadNumber);
    QuadTrgtName=strcat(StdQuadTrgtName, QuadNumber);
    persistent QuadTrgtHandle;
   % if isempty(QuadTrgtHandle)
        [return_code, QuadTrgtHandle]=VrepAPI.simxGetObjectHandle(ClientID, QuadTrgtName, VrepAPI.simx_opmode_blocking);
   % end
    return_code=VrepAPI.simxSetObjectPosition(ClientID, QuadTrgtHandle, -1, Position, VrepAPI.simx_opmode_blocking);
end

function Position = GetQuadTrgtPos(VrepAPI, ClientID, QuadNumber)
    %StdQuadTrgtName='Quadricopter_target#';
    StdQuadTrgtName='Quadricopter_base#';
    QuadNumber=num2str(QuadNumber);
    QuadTrgtName=strcat(StdQuadTrgtName, QuadNumber);
    persistent QuadTrgtHandle;
    %if isempty(QuadTrgtHandle)
    [return_code, QuadTrgtHandle]=VrepAPI.simxGetObjectHandle(ClientID, QuadTrgtName, VrepAPI.simx_opmode_blocking);
   % end
    [return_code, Position]=VrepAPI.simxGetObjectPosition (ClientID, QuadTrgtHandle, -1, VrepAPI.simx_opmode_blocking);
end