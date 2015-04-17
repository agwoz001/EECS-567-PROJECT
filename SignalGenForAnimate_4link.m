clear;
%Enter these variables according to desired falling trajectory
t=0:.05:40;
q0init=pi()/3;


k = .7;
omega = 1;                      %determines frequency of sinusoidal driving
phaseangle=pi/3;


q1=(-k/omega*sin(omega*t)-pi/2+k/omega);
q2=(k/omega*sin(omega*t+phaseangle)+pi/2-k/omega);%+q2_cycle;
%q3=zeros(length(q2),1)';
q3=(k/omega*sin(omega*t+phaseangle*2)+pi/2-k/omega);


%%%%%%%%%%%%% Use q0 script to calc appropriate q0 angles
q0=q0_4(t,q0init,q1,q2,q3);

%%%%%%%%%%%put variables in workspace in appropriate variables
q1_signal = [t; q1]';
q2_signal = [t; q2]';
q3_signal = [t; q3]';
q0_signal = [t; q0]';

plot(q0_signal(:,1),q0_signal(:,2),t,q1,t,q2,t,q3)
legend('q0','q1','q2','q3')

save qvecs4 q1_signal q2_signal q3_signal q0_signal