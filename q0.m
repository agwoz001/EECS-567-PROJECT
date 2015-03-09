function q0traj = q0(t,q1,q2)


q0traj=ode45(falling_cat,q1,q2,t)


end
