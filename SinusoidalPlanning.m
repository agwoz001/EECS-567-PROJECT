clear all;
deltat = 0.05;
k = 0.55;
q0_init = 45*pi/180;
q0_end = 75*pi/180;

q1_init = 0;
q2_init = 0;
q1_cycle = -pi/2;
q2_cycle = pi/2 - k;

%entiring q cycle
t1 = 0:0.05:3;
q1_1 = q1_init:(q1_cycle-q1_init)/(length(t1)-1):q1_cycle;
q2_1 = q2_init:(q2_cycle-q2_init)/(length(t1)-1):q2_cycle;
q0_1 = q0(t1, q0_init, q1_1, q2_1);

%entering sinusoidal driving
t2 = 0:deltat:25;   
u1 = k*sin(t2);
u2 = -k*cos(t2);          

%q1_2 = k*cos(t2);
q1_2 = -k*cos(t2)+q1_cycle+k;
q2_2 = -k*sin(t2)+q2_cycle;

q0_2=q0(t2,q0_1(end),q1_2,q2_2);
t2 = t2 + 3;
t = [t1,t2];
q1 = [q1_1, q1_2];
q2 = [q2_1, q2_2];
q0 = [q0_1, q0_2];

q1_signal = [t; q1]';
q2_signal = [t; q2]';
q0_signal = [t; q0]';
save qvecs q1_signal  q2_signal  q0_signal

plot(t,q1,'r',t,q2,'b')