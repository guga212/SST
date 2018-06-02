function Position = GetMainTrgtPos(VrepAPI, ClientID)
    MainTrgtName='Bill_base';
    persistent MainTrgtHandle;
    if isempty(MainTrgtHandle)
        [return_code, MainTrgtHandle]=VrepAPI.simxGetObjectHandle(ClientID, MainTrgtName, VrepAPI.simx_opmode_blocking);
    end
    [return_code, Position]=VrepAPI.simxGetObjectPosition (ClientID, MainTrgtHandle, -1, VrepAPI.simx_opmode_blocking);
    Position=double(Position);
end