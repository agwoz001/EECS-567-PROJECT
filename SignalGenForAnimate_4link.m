clear;
%Enter these variables according to desired falling trajectory
t=0:.05:40;
q0init=pi()/2;
q2=-sin(t)+pi/2;
q1=cos(t)+pi/2-1;
q3=sin(t+pi/8);
%%%%%%%%%%%%% Use q0 script to calc appropriate q0 angles
q0=q0_4(t,q0init,q1,q2,q3);

%%%%%%%%%%%put variables in workspace in appropriate variables
q1_signal = [t; q1]';
q2_signal = [t; q2]';
q3_signal = [t; q3]';
q0_signal = [t; q0]';

%plot(q0_signal(:,1),q0_signal(:,2))
save qvecs4 q1_signal q2_signal q3_signal q0_signal