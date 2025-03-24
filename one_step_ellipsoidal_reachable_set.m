function Qnext = one_step_ellipsoidal_reachable_set(A,Bu,Bd,Qcurr,Qu,Qd)



if A'*Qcurr*A+Bu'*Qu*Bu-Bd'*Qd*Bd<=zeros(2,2)
    Qnext=zeros(2,2);
    error('The one-step reachable set is void ')
    
end


Qnext= (sqrt(Qcurr)+sqrt(-Bu'*Qu*-Bu)-sqrt(-Bd'*Qd*-Bd))^2;



end

