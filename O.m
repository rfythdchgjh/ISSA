function a=O(L)
P=zeros(1,L);
    for i=1:L
        if i==1
            P(i)=rand;
        else
        P(i)=4*P(i-1)*(1-P(i-1));
        end
    end
a=P;
% plot(P,'Color','g')
end