function [dpos, pos]=GetDisturbedMainTrgtPosition(VrepAPI, ClientID, scale, name)
    dpos=zeros(1,3);
    pos=GetMainTrgtPos(VrepAPI, ClientID,name);
    dpos(1,:)=pos+(rand(1,3)-ones(1,3)).*pos*(scale/100);
end