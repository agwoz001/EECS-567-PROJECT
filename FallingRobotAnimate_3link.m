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
PhysicalParameters;

if flag==0, % initialize the animation
    
    h1=figure(1);
    fullscreen = get(0,'ScreenSize');
    hlen=fullscreen(3);
    vlen=fullscreen(4);
    set(h1,'Position',[0 -100 fullscreen(3) fullscreen(4)])
    set(h1,'paperposition',[.25 .25 10.5 8])
    
    axis([-hlen/20 hlen/20 -vlen/20 vlen/20]);
 
    axis off;
    
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
    axis([-hlen/20 hlen/20 -vlen/20 vlen/20]);	%set axis scaling to screen size 1:1
    axis off;
    
    q1=u(3);
    q2=u(4);
    q0=u(5); %TODO: solve for q0
    
    
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
    
    T01=[R01 [-L0; 0; 0]; 0 0 0 1];
    T02=[R02 [L0; 0; 0]; 0 0 0 1];
    Tc0=[Rc0 r0c; 0 0 0 1];%changed from rc0
    Tc1=Tc0*T01;
    Tc2=Tc0*T02;
  
    center0=r0c;
    joint1=center0-Rc0*[L0; 0; 0];
    joint2=center0+Rc0*[L0; 0; 0];
    center1=joint1-Rc0*R01*[L1; 0 ;0];
    center2=joint2+Rc0*R02*[L2; 0 ;0];
    end1=joint1-Rc0*R01*[2*L1; 0 ;0];
    end2=joint2+Rc0*R02*[2*L2; 0 ;0];
%     joint1=Tc1(1:3,4);
%     joint2=Tc2(1:3,4);
%     center2=joint2+Tc2(1:3,1:3)*[L2 0 0]';
%     center1=joint1+Tc1(1:3,1:3)*[-L1 0 0]';
%     end2=joint2+Tc2(1:3,1:3)*[2*L2 0 0]';
%     end1=joint1+Tc1(1:3,1:3)*[-2*L1 0 0]';    
    
sqrt((joint2(1)-center2(1))^2+(joint2(2)-center2(2))^2)
%%%%% draw robot. Graph is expressed in frame c for now. Eventually need to
%%%%% add y offset to everything as robot falls.
    line([joint1(1) joint2(1)],[joint1(2) joint2(2)])
    line([joint1(1) end1(1)],[joint1(2) end1(2)])
    line([joint2(1) end2(1)],[joint2(2) end2(2)])
    jointsize=0.2;
    rectangle('Position',[joint1(1)-jointsize/2,joint1(2)-jointsize/2,jointsize,jointsize],'Curvature',[1 1])
    rectangle('Position',[joint2(1)-jointsize/2,joint2(2)-jointsize/2,jointsize,jointsize],'Curvature',[1 1])
    rectangle('Position',[center1(1)-jointsize/2,center1(2)-jointsize/2,jointsize,jointsize],'EdgeColor','r')
    rectangle('Position',[center2(1)-jointsize/2,center2(2)-jointsize/2,jointsize,jointsize],'EdgeColor','r')
    rectangle('Position',[center0(1)-jointsize/2, center0(2)-jointsize/2,jointsize,jointsize],'EdgeColor','r')
    rectangle('Position',[-2*jointsize/2,-2*jointsize/2,2*jointsize,2*jointsize],'EdgeColor','g')
   % xlim([-10 10])
   % ylim([-10 10])
    drawnow;
    sys=[];
    
end;

