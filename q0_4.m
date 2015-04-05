% q0(t,q0init,q1traj,q2traj)
% t         time vector
% q0init    inital q0 position
% q1        vector of q1 corresponding to time vector
% q2        vector of q2 corresponding to time vector
function q0traj = q0_4(t,q0init,q1,q2,q3)


%input validation
if or(or(length(q1)~=length(t),length(q2)~=length(t)),length(q3)~=length(t))
    error='time and trajectories must all have the same length'
end

%just using simple approximation rather than trying to see if we can get ode45 working.
for i=1:length(t)
    if i==1
        q0(i)=q0init;
    else
        if t(i)==t(i-1)
            error='time cannot stop'
        end
        dt=t(i)-t(i-1);
        u1=(q1(i)-q1(i-1))/dt;
        u2=(q2(i)-q2(i-1))/dt;
        u3=(q3(i)-q3(i-1))/dt;
        u0=falling_cat_4(q0(i-1),q1(i-1),q2(i-1),q3(i-1),u1,u2,u3);
        q0(i)=q0(i-1)+u0*dt; %Euler's method for approximating ODE's?
    end
end


q0traj=q0;

end
