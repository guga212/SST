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
    Position=double(Position);
    Number=QuadNumber;
end