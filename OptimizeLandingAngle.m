PhysicalParameters;
FinalBodyAngle=linspace(0,pi/2);
u= [1 0 0]'; %point on rigid body, local coord's
ua= [-2*L1+L0 0 0];
for i=1:length(FinalBodyAngle
    R=[cos(FinalBodyAngle(i)),-sin(FinalBodyAngle(i)),0;sin(FinalBodyAngle(i)),cos(FinalBodyAngle(i)),0;0,0,1];
    c=R*[2*L1+L0 0 0]; %TODO - check: vector to CM of rigid body 
    x=R*u+c; %global position of point on rigid body
end


