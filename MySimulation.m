% Make sure to have the server side running in V-REP: 
% in a child script of a V-REP scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

function simpleTest()
    disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
        [return_code, quad_0]=vrep.simxGetObjectHandle(clientID, 'Quadricopter#0', vrep.simx_opmode_blocking);
        [return_code, quad_trgt_0]=vrep.simxGetObjectHandle(clientID, 'Quadricopter_target#0', vrep.simx_opmode_blocking);
        
        max_val=0.6;
        
        for i=1:1:10
            [return_code, quad_trgt_0_position]=vrep.simxGetObjectPosition (clientID, quad_trgt_0, -1, vrep.simx_opmode_blocking);
            quad_trgt_0_position=quad_trgt_0_position + max_val*rand(1,3)-0.5*[max_val,max_val,max_val];
            [return_code]=vrep.simxSetObjectPosition (clientID, quad_trgt_0, -1, quad_trgt_0_position, vrep.simx_opmode_blocking);
            pause(3.6);
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
