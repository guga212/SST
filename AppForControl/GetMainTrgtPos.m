function Position = GetMainTrgtPos(VrepAPI, ClientID,name)
    %MainTrgtName='Bill_base';
    MainTrgtName=name;        
    [return_code, MainTrgtHandle]=VrepAPI.simxGetObjectHandle(ClientID, MainTrgtName, VrepAPI.simx_opmode_blocking);   
    [return_code, Position]=VrepAPI.simxGetObjectPosition (ClientID, MainTrgtHandle, -1, VrepAPI.simx_opmode_blocking);
    Position=double(Position);
end