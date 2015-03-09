function [sys,x0,str,ts,simStateCompliance] = FallingRobotAnimate_3link(t,x,u,flag,tsdef)
%SFUNTMPL General MATLAB S-Function Template
%   With MATLAB S-functions, you can define you own ordinary differential
%   equations (ODEs), discrete system equations, and/or just about
%   any type of algorithm to be used within a Simulink block diagram.
%
%   The general form of an MATLAB S-function syntax is:
%       [SYS,X0,STR,TS,SIMSTATECOMPLIANCE] = SFUNC(T,X,U,FLAG,P1,...,Pn)
%
%   What is returned by SFUNC at a given point in time, T, depends on the
%   value of the FLAG, the current state vector, X, and the current
%   input vector, U.
%
%   FLAG   RESULT             DESCRIPTION
%   -----  ------             --------------------------------------------
%   0      [SIZES,X0,STR,TS]  Initialization, return system sizes in SYS,
%                             initial state in X0, state ordering strings
%                             in STR, and sample times in TS.
%   1      DX                 Return continuous state derivatives in SYS.
%   2      DS                 Update discrete states SYS = X(n+1)
%   3      Y                  Return outputs in SYS.
%   4      TNEXT              Return next time hit for variable step sample
%                             time in SYS.
%   5                         Reserved for future (root finding).
%   9      []                 Termination, perform any cleanup SYS=[].
%
%
%   The state vectors, X and X0 consists of continuous states followed
%   by discrete states.
%
%   Optional parameters, P1,...,Pn can be provided to the S-function and
%   used during any FLAG operation.
%
%   When SFUNC is called with FLAG = 0, the following information
%   should be returned:
%
%      SYS(1) = Number of continuous states.
%      SYS(2) = Number of discrete states.
%      SYS(3) = Number of outputs.
%      SYS(4) = Number of inputs.
%               Any of the first four elements in SYS can be specified
%               as -1 indicating that they are dynamically sized. The
%               actual length for all other flags will be equal to the
%               length of the input, U.
%      SYS(5) = Reserved for root finding. Must be zero.
%      SYS(6) = Direct feedthrough flag (1=yes, 0=no). The s-function
%               has direct feedthrough if U is used during the FLAG=3
%               call. Setting this to 0 is akin to making a promise that
%               U will not be used during FLAG=3. If you break the promise
%               then unpredictable results will occur.
%      SYS(7) = Number of sample times. This is the number of rows in TS.
%
%
%      X0     = Initial state conditions or [] if no states.
%
%      STR    = State ordering strings which is generally specified as [].
%
%      TS     = An m-by-2 matrix containing the sample time
%               (period, offset) information. Where m = number of sample
%               times. The ordering of the sample times must be:
%
%               TS = [0      0,      : Continuous sample time.
%                     0      1,      : Continuous, but fixed in minor step
%                                      sample time.
%                     PERIOD OFFSET, : Discrete sample time where
%                                      PERIOD > 0 & OFFSET < PERIOD.
%                     -2     0];     : Variable step discrete sample time
%                                      where FLAG=4 is used to get time of
%                                      next hit.
%
%               There can be more than one sample time providing
%               they are ordered such that they are monotonically
%               increasing. Only the needed sample times should be
%               specified in TS. When specifying more than one
%               sample time, you must check for sample hits explicitly by
%               seeing if
%                  abs(round((T-OFFSET)/PERIOD) - (T-OFFSET)/PERIOD)
%               is within a specified tolerance, generally 1e-8. This
%               tolerance is dependent upon your model's sampling times
%               and simulation time.
%
%               You can also specify that the sample time of the S-function
%               is inherited from the driving block. For functions which
%               change during minor steps, this is done by
%               specifying SYS(7) = 1 and TS = [-1 0]. For functions which
%               are held during minor steps, this is done by specifying
%               SYS(7) = 1 and TS = [-1 1].
%
%      SIMSTATECOMPLIANCE = Specifices how to handle this block when saving and
%                           restoring the complete simulation state of the
%                           model. The allowed values are: 'DefaultSimState',
%                           'HasNoSimState' or 'DisallowSimState'. If this value
%                           is not speficified, then the block's compliance with
%                           simState feature is set to 'UknownSimState'.

%   Copyright 1990-2010 The MathWorks, Inc.
%   $Revision: 1.18.2.5 $

global hlen vlen roundx roundy a b i refvector size;
green = [0 .8 .3];
previewon = 1;
    m0=210.2; %g
    m1=120.2; %g
    m2=m1;
    l0=5.6; %cm
    l2=5.44; %cm
if flag==0, % initialize the animation
    
    h1=figure(1);	
    fullscreen = get(0,'ScreenSize');
    hlen=fullscreen(3);
    vlen=fullscreen(4);
    set(h1,'Position',[0 -100 fullscreen(3) fullscreen(4)])
    set(h1,'paperposition',[.25 .25 10.5 8])
    
    axis([-hlen/2 hlen/2 -vlen/2 vlen/2]);
    axis off;
%     a=vlen/10;
%     b=vlen/10*0.617;
%     thetaround=0:pi/2:2*pi;
%     roundx=b*cos(thetaround);
%     roundy=b*sin(thetaround);
    
    sys = [0 0 0 5 0 0 1]; %5 inputs
    x0  = [];
    str = [];
    ts  = [tsdef, 0];
    % specify that the simState for this s-function is same as the default
    simStateCompliance = 'DefaultSimState';
    i=1;
    refvector=zeros(2,13);
    for n=1:length(refvector)
        refvector(1,n)=10*n-10;
        size(n)=length(refvector)-n+1;
    end
elseif flag ==2;
    clf;
    axis([-hlen/2 hlen/2 -vlen/2 vlen/2]);	%set axis scaling to screen size 1:1
    axis off;
    
    q1=u(3);
    q2=u(4);
    q0=u(5); %TODO: solve for q0

    
    R01=rotz(q1);
    R02=rotz(q2);
    Rc0=rotz(q0);
    T01=[R01 [-l0; 0; 0]; 0 0 0 1];
    T02=[R02 [l0; 0; 0]; 0 0 0 1];
    
    r0_00=[0 0 0]'; %in frame 0: vector from CM of link 0 to CM of link 0
    r0_20=[ l0+l2*cos(q2);  l2*sin(q2); 0]; %in frame 0: vector from CM of link 0 to CM of link 2
    r0_10=[-l0-l2*cos(q1); -l2*sin(q1); 0]; %in frame 0: vector from CM of link 0 to CM of link 1
    
    r0_c0=(r0_00*m0+r0_10*m1+r0_20*m2)/(m0+m1+m2);%in frame 0: vector from CM of link 0 to CG of robot
    
    rc_c0=Rc0*r0_c0; %in CG frame: vector from CM of link 0 to CG of robot
    Oc0=-1*rc_c0; %origin of frame 0 expressed in frame c
    
    Tc0=[Rc0 Oc0; 0 0 0 1];
    Tc1=Tc0*T01;
    Tc2=Tc0*T02;
    
    joint1=Tc1(1:3,4);
    joint2=Tc2(1:3,4);
    
    center2=joint2+l2*Tc2(1:3,1);
    center1=joint1-l2*Tc1(1:3,1);
    
    %plot(0,0,'x',Tc0(1,4),Tc0(2,4),'x',joint1(1),joint1(2),'o',joint2(1),joint2(2),'o')
    line([joint1(1) joint2(1)],[joint1(2) joint2(2)])
    line([joint1(1) center1(1)],[joint1(2) center1(2)])
    line([joint2(1) center2(1)],[joint2(2) center2(2)])
    xlim([-10 10])
    ylim([-10 10])
    
    %grid on;
    
    drawnow;
    
    sys=[];
    
end;

