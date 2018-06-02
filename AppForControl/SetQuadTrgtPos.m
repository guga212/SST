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