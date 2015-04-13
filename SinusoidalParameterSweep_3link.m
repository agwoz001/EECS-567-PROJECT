clear;
close all;
q0_init = 45*pi/180;

% k = 0.7;
%omega = 4;                      %determines frequency of sinusoidal driving
%phaseangle=pi/2;

k=linspace(0,pi/4,15);
omega = linspace(.1,1,4);
phaseangle=linspace(0,pi,15);
for j=1:length(omega)
    for i=1:length(k)
        for l=1:length(phaseangle)
            phaseshift=SinusoidCyclePhaseChange(k(i),q0_init,omega(j),phaseangle(l));
            phaseshiftdeg(i,l)=phaseshift*180/pi;
            Paramsk(i,l,:)=[k(i) omega(j) phaseangle(l)];
        end
    end
    figure;
    plot3(Paramsk(:,:,1),Paramsk(:,:,3),phaseshiftdeg(:,:))
    xlabel('k')
    ylabel('phase angle')
    zlabel('Phase change (deg)')
    title(strcat('Phase angle vs. k - omega=',num2str(omega(j))))
    clear Paramsk
end



% plot3(Paramsk(:,:,end,1),Paramsk(:,:,end,2),phaseshiftdeg(:,:,end))
% xlabel('k')
% ylabel('omega')
