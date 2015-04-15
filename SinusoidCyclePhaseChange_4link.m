function phaseshift=SinusoidCyclePhaseChange(k,q0_init,omega,phaseangle)

% q1_init=0; q2_init=0;
q1_cycle = -pi/2;
q2_cycle = pi/2 - k/omega;    
q3_cycle = 0;

deltat=2*pi/omega/100; %100 points per cycle, regardless of length.
t = 0:deltat:2*pi/omega;

%entering sinusoidal cycle
% u1 = k*sin(t);                  %driving signal to inputs
% u2 = -k*cos(t);   
q1_2 = -k/omega*sin(omega*t+phaseangle)+q1_cycle+k;
q2_2 = -k/omega*sin(omega*t)+q2_cycle;
q3_2 = -k/omega*sin(omega*t+2*phaseangle)-k;
q0_2 = q0_4(t,q0_init,q1_2,q2_2,q3_2);

phaseshift=q0_2(end)-q0_init;

%plot(t,q1_2*180/pi,t,q2_2*180/pi,t,q0_2*180/pi)
%legend('q1','q2','q0')





