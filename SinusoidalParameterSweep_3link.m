clear;
q0_init = 45*pi/180;


% k = 0.7;
%omega = 4;                      %determines frequency of sinusoidal driving
%phaseangle=pi/2;

k=linspace(0,pi/4,15);
omega = linspace(0,10,30);
phaseangle=linspace(0,pi/2,5);

for i=1:length(k)
    for j=1:length(omega)
        for l=1:length(phaseangle)
            phaseshift=SinusoidCyclePhaseChange(k(i),q0_init,omega(j),phaseangle(l));
            phaseshiftdeg(i,j,l)=phaseshift*180/pi;
            Paramsk(i,j,l,:)=[k(i) omega(j) phaseangle(l)];
        end
    end
end

plot3(Paramsk(:,:,end,1),Paramsk(:,:,end,2),phaseshiftdeg(:,:,end))
xlabel('k')
ylabel('omega')
