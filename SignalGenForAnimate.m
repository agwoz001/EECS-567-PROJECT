clear;
%Enter these variables according to desired falling trajectory
t=0:.01:10;
q0init=pi()/2;
q2=5*t;
q1=2*ones(1,length(t));
%%%%%%%%%%%%% Use q0 script to calc appropriate q0 angles
q0=q0(t,q0init,q1,q2);

%%%%%%%%%%%put variables in workspace in appropriate variables
q1_signal = [t; q1]';
q2_signal = [t; q2]';
q0_signal = [t; q0]';

save qvecs q1_signal  q2_signal  q0_signal