function ur_hat = online_terminal_optim_QP(ur,u0,H,g)


H_obj=eye(2);
f_obj=-ur;
A_constr=H;
b_constr=g-H*u0;


options = optimoptions(@quadprog,'Algorithm','interior-point-convex','Display','off');


usol=quadprog(H_obj,f_obj,A_constr,b_constr,[],[],[],[],[],options);
ur_hat=usol;

end

