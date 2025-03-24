clear;
close all
Ts=0.1;
eight_traj_generation


load('Simulation_data')

pose_seq = pose_seq(:,1:end-1);


figure
grid
hold on
p1=plot(pose_seq(1,:),pose_seq(2,:));


p2=plot(xr,yr,'k--');
p3=plot(pose_seq(1,1),pose_seq(2,1),'b-p','MarkerIndices',[1 1],'MarkerFaceColor','yellow','MarkerSize',15);

p1.LineWidth=1.5;
p2.LineWidth=1.5;
p3.LineWidth=1.5;


axis([-3.5 3.5 -3.5 3.5])

l=legend([p1,p2,p3],'Car Trajectory','Reference','Initial point');
l.FontSize=27;
xlabel('x[m]')
ylabel('y[m]')
title('Eight-Shaped Trajectory')



figure
title('Control inputs')
subplot(4,1,1)
grid
hold on
p1=plot(t,v12seq(1,:));

p2=plot(t,v1max*ones(1,length(t)),'r--');
p3=plot(t,-v1max*ones(1,length(t)),'r--')

p1.LineWidth=1.5;
p2.LineWidth=1.5;
p3.LineWidth=1.5;

axis([0 150.7 0 1.1])
xlabel('t[sec]')
ylabel('v[m/sec]')


l=legend([p1,p2],'Linear velocity','v_{MAX}');
l.FontSize=15;


subplot(4,1,2)
grid
hold on
p1=plot(t,v12seq(2,:));
p2=plot(t,v2max*ones(1,length(t)),'r--');
p3=plot(t,-v2max*ones(1,length(t)),'r--')
axis([0 150.7 -1.1 1.1])
xlabel('t[sec]')
ylabel('\omega[RAD/sec]')


p1.LineWidth=1.5;
p2.LineWidth=1.5;
p3.LineWidth=1.5;


p8.LineWidth=1.5;

l=legend([p1,p2],'Angular steering velocity','\omega_{MAX}');
l.FontSize=15;

subplot(4,1,3)
grid
hold on

title('Orientation \theta(k)')

p1=plot(t,pose_seq(3,:));

p2=plot(t,thetar,'k--');
axis([0 t(end) 0 6])

xlabel('t[sec]')
ylabel('\theta[RAD]')

p1.LineWidth=1.5;
p2.LineWidth=1.5;


l=legend([p1,p2],'Heading angle','\theta_{ref}');
l.FontSize=15;


subplot(4,1,4)
grid
hold on

title('Steering angle \varphi(k)')

p1=plot(t,pose_seq(4,:));

p2=plot(t,phir,'k--');
axis([0 t(end) -pi/2 pi/2])

xlabel('t[sec]')
ylabel('\varphi[RAD]')

p1.LineWidth=1.5;
p2.LineWidth=1.5;


l=legend([p1,p2],'Steering angle','\theta_{ref}');
l.FontSize=15;




figure
title('Set-membership index - i(t)')
p=plot(t,index_seq);
hold on
grid
p.LineWidth=2;

l=legend("Set-memebership");





figure

Q0=Q_seq(1:2,1:2);
ell0=ellipsoid(Q0);


p9=plot(z_tilde_seq(1,1),z_tilde_seq(2,1),'r-p','MarkerIndices',[1 1],'MarkerFaceColor','yellow','MarkerSize',20);
hold on
grid


ell_seq=[];



for j=length(Q_seq):-40:1
    Q_k=Q_seq(1:2,j-1:j);
    ell_k=ellipsoid(Q_k);
    ell_seq=[ell_seq ell_k];
end

p=plot(z_tilde_seq(1,:),z_tilde_seq(2,:));
p.LineWidth=3;
p.Color=[0.4660 0.6740 0.1880];
for i=8:length(ell_seq)
    if i<length(ell_seq)
        plot(ell_seq(i),'b');
    else
        plot(ell_seq(i));
    end
    
end


return
figure

p_ctrl=plot(200,100,'b o');

hold on
p_inv=plot(200,100,'r o');
p_star=plot(z_tilde_seq(1,1),z_tilde_seq(2,1),'b-p','MarkerIndices',[1 1],'MarkerFaceColor','yellow','MarkerSize',20);
legend([p_star,p_inv(1),p_ctrl(1)],{'Current Tracking Error ','Robust Control Invariant set','Robust One-Step Controllable sets'})


writerObj = VideoWriter('sim1.avi');
writerObj.Quality=100;
writerObj.FrameRate = 6.66667;
open(writerObj);



plot(ell0);
hold on
grid
plot(z_tilde_seq(1,1),z_tilde_seq(2,1),'b-p','MarkerIndices',[1 1],'MarkerFaceColor','yellow','MarkerSize',20);
axis([-0.5 0.5 -0.5 0.5])


xl=0.5;
yl=0.5;


for i=1:length(z_tilde_seq)
    
    
    index=9;
    
    clf
    
    p_ctrl=plot(200,100,'b o');
    hold on
    p_inv=plot(200,100,'r o');

    
    plot(ell0)
    hold on
    grid
    for j=length(ell_seq)-1:-1:9
        
        if isinternal(ell_seq(j),[z_tilde_seq(1,i);z_tilde_seq(2,i)])
            plot(ell_seq(j),'b')
        end
        
    end
    
    p_star=plot(z_tilde_seq(1,i),z_tilde_seq(2,i),'b-p','MarkerIndices',[1 1],'MarkerFaceColor','yellow','MarkerSize',20);
    
    
    if(i>15)
        xl=xl-0.05;
        yl=yl-0.05;
        if xl<0.08
            xl=0.08 ;
        end
        if yl<0.08
            yl=0.08;
        end
        
        
    end
    axis([-xl xl -yl yl])
    
    legend([p_star,p_inv(1),p_ctrl(1)],{'Current Tracking Error ','Robust Control Invariant set','Robust One-Step Controllable sets'})
    
    
    frame = getframe;
    writeVideo(writerObj,frame);
    
    
    pause(0.001)
    
end

close(writerObj);



