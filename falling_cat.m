%Falling cat code goes here

%frame g - ground
%frame 0 - middle of center link
%frame c - center of mass
%frame c to frame g: translation only
%frame 3 - frame that collides with ground

%from table 1
syms L0 L1 L2;    %for symbolic subsitution
syms M0 M1 M2;    
syms I0 I1 I2;    
% L0 = 5.60;      %cm
% L1 = 5.44;      %cm
% L2 = L1;        %cm
% M0 = 210.2;     %g
% M1 = 120.2;     %g
% M2 = M1;        %g
% I0 = 1499.7;    %g-cm^2
% I1 = 895.1;     %g-cm^2
% I2 = I1;        %g-cm^2

%joint angles
syms q0 q1 q2
syms u0 u1 u2       %q dots

%matrix in Hc left term to be multiplied by [q1' q2' q3']'
R01 = rotz(q1);
R02 = rotz(q2);
A1 = [0 0 0; 0 0 0; I0+I1+I2, I1, I2];
r00 = [0 0 0].';                                                            %rij are vectors between centers of mass of links, not frames
r10 = [-L0 0 0].' + R01*[-L1 0 0].';
r20 = [L0 0 0].' + R02*[L2 0 0].';
rc0 = (r00*M0 + r10*M1 + r20*M2)/(M0+M1+M2);                                %R matrices are dependent on choice of frames
                                                                            %r vectors are not
v00 = [0 0 0]';
v10 = cross([0 0 u1].',R01*[-L1,0,0].'); 
v20 = cross([0 0 u2].',R02*[L2,0,0].');
vc0 = (v00*M0+ v10*M1 + v20*M2)/(M0+M1+M2)