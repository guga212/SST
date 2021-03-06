% This small example illustrates how to use the remote API
% synchronous mode. The synchronous mode needs to be
% pre-enabled on the server side. You would do this by
% starting the server (e.g. in a child script) with:
%
% simRemoteApi.start(19999,1300,false,true)
%
% But in this example we try to connect on port
% 19997 where there should be a continuous remote API
% server service already running and pre-enabled for
% synchronous mode.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

function simpleSynchronousTest()
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
        while(1)
            
             for i=0:3     
                
                main_trgt_pos=GetDisturbedMainTrgtPosition(vrep,clientID);
                [xd_0, x_0]=GetDisturbedQuadPosition(vrep,clientID,4);
                xd_0(i+1,:)=x_0(i+1,:);
                [trgt_alg_x, trgt_alg_y]=OptimizeNextMove([double(main_trgt_pos(1)) double(main_trgt_pos(2))], [1.41;1.41;1.41;1.41 ].*0.9, [0 2 2.828 2; 2 0 2 2.828; 2.828 2 0 2; 2 2.828 2 0].*0.9, xd_0(:,1:2), 1.0, 1.2, 0.4);
                
                SetQuadTrgtPos(vrep, clientID, i, [trgt_alg_x(i+1) trgt_alg_y(i+1) x_0(i+1,3)]);                                
            end
            
            vrep.simxSynchronousTrigger(clientID);
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
%A minimalna odleglosc do celu
%a minimalna odleglosc do sasiada
function [x,y]=OptimizeNextMove(X,D,d,x_0,A,a,dx)
    tfun = @(x)TargetFunction(x,X,D,d);
    cfun=@(x)ConstraintFunction(x,X,A,a,x_0,dx);
    options = optimoptions('fmincon','Display','off');
    [res minval]=fmincon(tfun,x_0,[],[],[],[],[],[],cfun,options);
    x=res(:,1);
    y=res(:,2);
end

%x zmienne pozycji 
%X zmienna celu
%D zadana odleglosc do celu 
%d zadana macierz odleglosc do sasiada
function out=TargetFunction(x,X,D,d)

    out=0;
    
    for i=1:size(x,1)
        out=out+(D(i)^2-(X(1,1)-x(i,1))^2-(X(1,2)-x(i,2))^2)^2;
    end
    
    for i=1:size(x,1)       
        for j=1:size(x,1)            
            weight=1.0;         
            out=out+weight*(d(i,j)^2-(x(i,1)-x(j,1))^2-(x(i,2)-x(j,2))^2)^2;                    
        end
    end
    
end


%x zmienne pozycji 
%X zmienna celu
%A minimalna odleglosc do celu
%a minimalna odleglosc do sasiada
%x_0 pozycja poczatkowa
%dx maksymalne przesuniecie
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
    if isempty(MainTrgtHandle)
        [return_code, MainTrgtHandle]=VrepAPI.simxGetObjectHandle(ClientID, MainTrgtName, VrepAPI.simx_opmode_blocking);
    end
    [return_code, Position]=VrepAPI.simxGetObjectPosition (ClientID, MainTrgtHandle, -1, VrepAPI.simx_opmode_blocking);
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

    %StdQuadTrgtName='Quadricopter_target#';
    StdQuadTrgtName='Quadricopter_base#';
    QuadNumber=num2str(QuadNumber);
    QuadTrgtName=strcat(StdQuadTrgtName, QuadNumber);
    
    if isempty(QuadTrgtHandle) || Number~=QuadNumber
    [return_code, QuadTrgtHandle]=VrepAPI.simxGetObjectHandle(ClientID, QuadTrgtName, VrepAPI.simx_opmode_blocking);
    end
    [return_code, Position]=VrepAPI.simxGetObjectPosition (ClientID, QuadTrgtHandle, -1, VrepAPI.simx_opmode_blocking);
    
     Number=QuadNumber;
end
