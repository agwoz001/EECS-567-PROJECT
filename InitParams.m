clear all;
clc;
close all;  

% from table 1
% L0 = 5.60;      %cm
% L1 = 5.44;      %cm
% L2 = L1;        %cm
% M0 = 210.2;     %g
% M1 = 120.2;     %g
% M2 = M1;        %g
% I0 = 1499.7;    %g-cm^2
% I1 = 895.1;     %g-cm^2
% I2 = I1;        %g-cm^2
PhysicalParameters
load qvecs
  
bgain=76;   
%system parameters
    

    
    tend=q0_signal(end,1);%10.0;
    ts=1.0*10^(-4);
    tsc=1.0/1000.0;
    tslow=1.0/50.0;
    tsdata=1.0/1000.0;
  
bs9=6*bgain;
bs10=6*bgain;
bs11=6*bgain;
w9=0.85*2*pi;
w10=6.772;
w11=9.0;
phi9=0;
phi10=0;
phi11=0;



    
