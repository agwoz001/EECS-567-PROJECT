PhysicalParameters;
% FinalBodyAngle=linspace(0,pi/2);
% u= [1 0 0]'; %point on rigid body, local coord's
% ua= [-2*L1+L0 0 0];
% for i=1:length(FinalBodyAngle
%     R=[cos(FinalBodyAngle(i)),-sin(FinalBodyAngle(i)),0;sin(FinalBodyAngle(i)),cos(FinalBodyAngle(i)),0;0,0,1];
%     c=R*[2*L1+L0 0 0]; %TODO - check: vector to CM of rigid body 
%     x=R*u+c; %global position of point on rigid body
% end
figure(1)
hold on
for n=linspace(0,1,10)
    a=0.157; %m/s^2
    h=220/100; %m
    theta=linspace(10*pi/180,pi/2,100);
    %n=.6;  %choose same weight as paper
    vbefore=a*(2*h/a)^.5;
    %vbefore=.8;
    r1=L1;
    r2=r1+L1+L2;
    r3=r2+L2+L3;
    w=(r1*sin(theta)*M1*vbefore+r2*sin(theta)*M2*vbefore+r3*sin(theta)*M3*vbefore)/(I1+M1*r1^2+I2+M2*r2^2+I3+M3*r3^2);
    Tbefore=1/2*(M1+M2+M3)*vbefore^2;
    Tafter=1/2*(I1+M1*r1^2)*w.^2+1/2*(I2+M2*r2^2)*w.^2+1/2*(I3+M3*r3^2)*w.^2;
    troll=1./(w.*tan(theta));
    metric=n*(Tbefore-Tafter)-(1-n)*troll;  %eq. 15
    plot(theta*180/pi,metric);
    hold on
    %plot(theta,Tbefore-Tafter)
    %figure
    %plot(troll)
    %axis([0,pi/2,0,1000])
end
