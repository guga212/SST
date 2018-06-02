function [dpos, pos]=GetDisturbedQuadPosition(VrepAPI, ClientID, size, scale)
    dpos=zeros(size,3);
    pos=zeros(size,3);
    for i=1:size
        pos(i,:)=GetQuadTrgtPos(VrepAPI, ClientID,i-1);
        dpos(i,:)=pos(i,:)+(rand(1,3)-ones(1,3)).*pos(i,:)*(scale/100);        
    end
end