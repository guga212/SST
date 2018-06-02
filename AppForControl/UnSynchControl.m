function UnSynchControl()
    clear();
    disp('Program started');

    vrep=remApi('remoteApi');
    vrep.simxFinish(-1);
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');

% Parameters of simulation:
        N=4;
        sim_dt=0.3;
        sim_t=0;
        idx=1;
        t=1*sim_dt;
        a_0=zeros(N,2,t/sim_dt);   
        v_0=zeros(N,2);   
        
        while(1)
            
            if(sim_t<=0.1*sim_dt)
                [main_trgt_posd, main_trgt_pos]=GetDisturbedMainTrgtPosition(vrep,clientID);
                [xd_0, x_0]=GetDisturbedQuadPosition(vrep,clientID,N);
                                    
                 [trgt_alg_x, trgt_alg_y]=OptimizeNextMove( main_trgt_pos(1:2), [1.41;1.41;1.41;1.41 ].*1.0, [0 2 2.828 2; 2 0 2 2.828; 2.828 2 0 2; 2 2.828 2 0].*1.0,a_0,v_0,x_0(:,1:2),t,1.2, 1.2, 0.6, 6);
                %[trgt_alg_x, trgt_alg_y]=OptimizeNextMove( main_trgt_pos(1:2), [1.41;1;1.41;1;1.41;1;1.41;1].*1.8, [0 1 2 2.236 2.828 2.236 2 1; 1 0 1 1.41 2.236 2 2.236 1.41;  2 1 0 1 2 2.236 2.828 2.236; 2.236 1.41 1 0 1 1.41 2.236 2; 2.828 2.236 2 1 0 1 2 2.236; 2.828 2 2.236 1.41 1 0 1 1.41; 2 2.236 2.828 2.236 2 1 0 1; 1 1.41 2.236 2 2.236 1.41 1 0; ].*1.8, a_0,v_0,x_0(:,1:2),t, 1.6, 1.6, 0.4, 5);
                for i=1:size(a_0,3)
                    a_0(:,1,i)=trgt_alg_x(:,i);
                    a_0(:,2,i)=trgt_alg_y(:,i);
                end
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
            
            sim_t=sim_t-sim_dt;
            v_0(:,1)=v_0(:,1)+trgt_alg_x(:,idx)*sim_dt;
            v_0(:,2)=v_0(:,2)+trgt_alg_y(:,idx)*sim_dt;
            idx=idx+1;
            
        end


        % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(clientID);

        % Now close the connection to V-REP:    
        vrep.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    
    %the destructor call 
    vrep.delete(); 
    
    disp('Program ended');
end