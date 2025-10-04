function a=T(l)
t=zeros(1,l);
t(1)=0.6;
% if l>1
    for i=2:l
        if t(i-1)<0.7
            t(i)=t(i-1)/0.7;
        else
            t(i)=(1-t(i-1))/0.3;
        end
       
    end
a=t;


% plot(t);

% for i=1:l
%     t(i)=30*(1-atan(1+(i/l)^3));                  %t(i)=rand*(1-(i^2)/(l^2));
% end
% plot(t)
% xlabel('Iter');
% ylabel('Value')



% function [a]=T(TMAX)
% t=zeros(1,TMAX);
% 
%     for i=1:TMAX
%         t(i)=1-cos((pi*i)/(2*TMAX));
%         
%     end
% 
% % plot(t,'Color','g')
% a=t;
% 




end