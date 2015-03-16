%returns rate of change of q0 as a function of the configuration and the rate of change of q1 and q2
function u0 = falling_cat(q0,q1,q2,u1,u2)



% Falling cat code goes here

% frame g - ground
% frame 0 - middle of center link
% frame c - center of mass
% frame c to frame g: translation only
% frame 3 - frame that collides with ground

 
PhysicalParameters;


% Rotation Matrices
Rc0=[cos(q0),-sin(q0),0;sin(q0),cos(q0),0;0,0,1];
R01=[cos(q1),-sin(q1),0;sin(q1),cos(q1),0;0,0,1];
R02=[cos(q2),-sin(q2),0;sin(q2),cos(q2),0;0,0,1];

% position Vectors (All in the c frame!!!)
% point "a" is at the joint between links 0 and 1, and "b" between
% links 0 and 2.
z=[0 0 1]';
r1a=Rc0 * R01 * [-L1, 0, 0].';
ra0=Rc0 * [-L0, 0, 0].'; 
r2b=Rc0 * R02 * [L2, 0, 0].';
rb0=Rc0 * [L0, 0, 0].';
r00 = [0,0,0].'; 
r10 = r1a+ra0;
r20 = r2b+rb0;
rc0 = (r00*M0 + r10*M1 + r20*M2)/(M0+M1+M2);  
r0c=-rc0;
r1c=r10+r0c;
r2c=r20+r0c;


% Really long formula for the velocity (or differential change) in u0
% as a function of u1 and u2. I hope it works!!!  8[
u0=(-u2*((z.' * X(r2c) * X(z) * r2b-z.' * X(r2c) * ((X(z) * r2b)*M2)/(M2+M1+M0))*M2-(z.' * X(r1c) * ((X(z) * r2b)*M2)/(M2+M1+M0))*M1-(z.' * X(r0c) * ((X(z) * r2b)*M2)/(M2+M1+M0))*M0+I2)-u1*(-(z.' * X(r2c) * ((X(z) * r1a)*M1)/(M2+M1+M0))*M2+(z.' * X(r1c) * X(z) * r1a-z.' * X(r1c) * ((X(z) * r1a)*M1)/(M2+M1+M0))*M1-(z.' * X(r0c) * ((X(z) * r1a)*M1)/(M2+M1+M0))*M0+I1))/((-z.' * X(r2c) * ((X(z) * rb0+X(z) * r2b)*M2+(X(z) * ra0+X(z) * r1a)*M1)/(M2+M1+M0)+z.' * X(r2c) * X(z) * rb0+z.' * X(r2c) * X(z) * r2b)*M2+(-z.' * X(r1c) * ((X(z) * rb0+X(z) * r2b)*M2+(X(z) * ra0+X(z) * r1a)*M1)/(M2+M1+M0)+z.' * X(r1c) * X(z) * ra0+z.' * X(r1c) * X(z) * r1a)*M1-(z.' * X(r0c) * ((X(z) * rb0+X(z) * r2b)*M2+(X(z) * ra0+X(z) * r1a)*M1)/(M2+M1+M0))*M0+I2+I1+I0);


end

