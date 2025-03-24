function uk = online_phase(x_curr,Qpre,A,B,H,g,Rd,ur,u_pre)



%Set theoretic MPC online optimization
obj_fun=@(u)(A*x_curr+B*(u+ur)-B*ur)'*(A*x_curr+B*(u+ur)-B*ur)+(u-u_pre)'*Rd*(u-u_pre);


    function [c,ceq]=quad_constr(u,H,g,Qpre,ur,A,B,x_curr)
        c1=H*(u+ur)-g;
        c2=(A*x_curr+B*(u+ur)-B*ur)'*inv(Qpre)*(A*x_curr+B*(u+ur)-B*ur)-1;
        c=[c1;c2];
        ceq=[];
    end



options=optimoptions('fmincon','Display','off');
usol = fmincon(obj_fun,[0;0],[],[],[],[],[],[],@(u)quad_constr(u,H,g,Qpre,ur,A,B,x_curr),options);
uk=usol;


test_g=H*(uk+ur)-g;

test1=test_g(1);
test2=test_g(2);
test3=test_g(3);
test4=test_g(4);

test5=(A*x_curr+B*(uk))'*inv(Qpre)*(A*x_curr+B*(uk))-1;


test=test1<0 & test2<0 & test3<0 &test4<0 & test5<0;


obj_fun=(A*x_curr+B*(uk+ur)-B*ur)'*(A*x_curr+B*(uk+ur)-B*ur)+(uk-u_pre)'*Rd*(uk-u_pre);




if test~=1
    
    error('ATTENZIONE')
    
end



end

