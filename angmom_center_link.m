function H = angmom_center_link(t,q0traj,q1traj,q2traj)

    %input validation
    if or(or(length(q1traj)~=length(t),length(q2traj)~=length(t)),length(q0traj)~=length(t))
        error='time and trajectories must all have the same length'
    end


    PhysicalParameters;
    
    for i=1:length(t)
        
        if(i==length(t))
            break;
            q0=q0traj(i);
            q1=q1traj(i);
            q2=q2traj(i);
            u0=(q0traj(i)-q0traj(i-1))/(t(i)-t(i-1));
            u1=(q1traj(i)-q1traj(i-1))/(t(i)-t(i-1));
            u2=(q2traj(i)-q2traj(i-1))/(t(i)-t(i-1));
        else
            q0=q0traj(i);
            q1=q1traj(i);
            q2=q2traj(i);
            u0=(q0traj(i+1)-q0traj(i))/(t(i+1)-t(i));
            u1=(q1traj(i+1)-q1traj(i))/(t(i+1)-t(i));
            u2=(q2traj(i+1)-q2traj(i))/(t(i+1)-t(i));
        end
        
        
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



        %not so really long formula for angular momentum
        H(i)=(z.' * X(r20) * (u0*(X(z) * rb0)+(u2+u0)*(X(z) * r2b)))*M2+(z.' * X(r10) * (u0*(X(z) * ra0)+(u1+u0)*(X(z) * r1a)))*M1+u0*(I2+I1+I0)+u2*I2+u1*I1
    end
    
    plot(t(1:end-1),H)
    xlabel('time [s]')
    ylabel('Angular Momenum [g-cm^2/s]')

end

