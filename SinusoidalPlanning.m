close all;
clear all;

deltat = 0.01;
phaseangle = pi/2;
q1_cycle = -pi/2;
q1_init = 0;
q2_init = 0;
q0_init = 45*pi/180;
q0_end = 75*pi/180;
precision = 1.5;        %how close to an integer number of periods is acceptable (unitless)


k_test = linspace(.00001,2.5,50);%linspace(0,pi/3,50);
w_test = linspace(1,5,10);
phaseshift = zeros(length(k_test),10);

figure(1)
hold on
for i = 1:length(k_test)
    for w = 1:length(w_test)    
        if ((k_test(i)/w_test(w))>(pi/4))
            phaseshift(i,w) = 0;
        else
            phaseshift(i,w) = SinusoidCyclePhaseChange(k_test(i),q0_init,w_test(w),phaseangle); 
        end
        temp(i,w)=k_test(i)/w_test(w);
    end
end

% for i = 1:length(k_test)
%     w_test(i)=k_test(i)/(pi/2);
%     phaseshift(i)=SinusoidCyclePhaseChange(k_test(i),q0_init,w_test(i),phaseangle);
% end

%find where phase shift is approximately an integer
quotient_map = (q0_end-q0_init)./phaseshift;             %find desired phase shift divided by phase of one cycle
quotient_map(~isfinite(quotient_map)) = 0;               %remove infinities
integer_map = mod(quotient_map, 1) < precision           %find values that are approximately divide the phase by even cycles
allowable_phases = integer_map .* phaseshift;

max_phase = max(max(allowable_phases))
[maxloc_row, maxloc_col] = find(allowable_phases == max(max(allowable_phases)))
k = k_test(maxloc_row);
omega = w_test(maxloc_col)
num_cycles = (q0_end-q0_init)/max_phase

%plot(k_test,phaseshift*180/pi)
%xlabel('k'); ylabel('phase shift (deg)');
%plot3(k_test, w_test, phaseshift(:,:))

q2_cycle = pi/2 - k/omega; 
T = 2*pi/omega;                             %time of 1 oscillation

t = 0:deltat:1.2*num_cycles*T-deltat;       %time of fall
t1 = t(1:(1/12)*length(t)); 
t2 = t1(end)+deltat:deltat:num_cycles*T+t1(end);
t3 = t2(end)+2*deltat:deltat:1.2*num_cycles*T;
tend = 1.2*num_cycles*T;

%SINUSOIDAL PLANNING
%entiring q cycle for 1 second
q1_1 = q1_init:(q1_cycle-q1_init)/(length(t1)-1):q1_cycle;
q2_1 = q2_init:(q2_cycle-q2_init)/(length(t1)-1):q2_cycle;
q0_1 = q0(t1, q0_init, q1_1, q2_1);

%entering sinusoidal cycle
u1 = k*sin(t);                  %driving signal to inputs
u2 = -k*cos(t);   
%q1_2 = -k/omega*cos(omega*t)+q1_cycle+k/omega;
q1_2 = -k/omega*cos(omega*t)+q1_cycle+k/omega;
q2_2 = -k/omega*sin(omega*t)+q2_cycle;
q0_2 = q0(t,q0_1(end),q1_2,q2_2);
% 
% q1_2 = q1_2(length(t1)+1:length(t1)+length(t2));
% q2_2 = q2_2(length(t1)+1:length(t1)+length(t2));
% q0_2 = q0_2(length(t1)+1:length(t1)+length(t2));

q1_2 = q1_2(1:length(t2));
q2_2 = q2_2(1:length(t2));
q0_2 = q0_2(1:length(t2));


%entering reverse q cycle for 1 second
q1_3 = q1_2(end):(q1_init-q1_2(end))/(length(t3)-1):q1_init;
q2_3 = q2_2(end):(q2_init-q2_2(end))/(length(t3)-1):q2_init;
q0_3 = q0(t3, q0_2(end), q1_3, q2_3);

% %find time steps remaining and wait for collision
q0_final = [q0_1, q0_2, q0_3];
q1_final = [q1_1, q1_2, q1_3];
q2_final = [q2_1, q2_2, q2_3];

figure(1)
plot(t,q1_final*180/pi,'r',t,q2_final*180/pi,'b',...
    t, 90*ones(size(t)), 'k', t, -90*ones(size(t)),'k')
legend('q1','q2')
title('Path of Joint Angles');
grid on
figure(2)
plot(t,q0_final*180/pi,t, 75*ones(size(q0_final)),'r')
title('Path of Body Angle')
grid on

q1_signal = [t; q1_final]';
q2_signal = [t; q2_final]';
q0_signal = [t; q0_final]';
save qvecs q1_signal  q2_signal  q0_signal

%final error = 0.6178%
