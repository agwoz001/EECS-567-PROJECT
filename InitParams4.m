clear all;
clc;
close all;  

PhysicalParameters_4
load qvecs4;
  
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



    
