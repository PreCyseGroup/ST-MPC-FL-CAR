clear;
close all;


%%  System, Constraints and Disrturbances
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%if these are changed, then all the offline
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%parts must be re-computed

%% Robot modeling
Ts=0.1; %Sampling time

l=1;
A = eye(2);
Bu = eye(2)*Ts;
Bd = eye(2)*Ts;
C = eye(2);
D = [0; 0];

%scaling factor of the trajectory
eta=3;
alpha=12;
k=0:Ts:2*pi*alpha*2;

xr=eta*sin(k/alpha); %trajectory position along x axis
yr=eta*sin(k/(2*alpha)); %trajetcory position along y axis


%Velocity trajectory
xdr=eta*cos(k/alpha)*(1/alpha); %trajectory velocity along x axis
ydr=eta*cos(k/(2*alpha))*(1/(2*alpha));%trajectory velocity along y axis

%Acceleration trajectory
xddr=-eta*sin(k/alpha)*(1/alpha)*(1/alpha); %trajetcory acceleration along x axis
yddr=-eta*sin(k/(2*alpha))*(1/(2*alpha))*(1/(2*alpha));%trajetcory acceleration y axis


xdddr=-eta*cos(k/alpha)*(1/alpha)*(1/alpha)*(1/alpha);%trajetcory jerk along x axis
ydddr=-eta*cos(k/(2*alpha))*(1/(2*alpha))*(1/(2*alpha))*(1/(2*alpha));%trajetcory jerk y axis

%Driving velocity reference
v1r=sqrt(xdr.^2+ydr.^2); %Linear velocity of the trajectory


% Orientation reference
thetar=atan2(ydr./v1r,xdr./v1r); %Trajectory Orientation


%Adjusting Orientation ( a part of tetha(k) is out of phase
thetar_diff=diff(thetar);
for i=1:length(thetar_diff)
    if thetar_diff(i)<-6
        i1=i+1;
    elseif thetar_diff(i)>6
        i2=i;
    end
end
thetar(i1:i2)=thetar(i1:i2)+2*pi; %Ts=0.15


%Vehicle Angular velocity reference
thetadr=(yddr.*xdr-xddr.*ydr)./v1r.^2; %Angular velocity velocity of the trajectory

%Steering angle reference
phir=atan(l*(yddr.*xdr-xddr.*ydr)./v1r.^3);

%Steering velocity reference
v2r=l*v1r.*((ydddr.*xdr-xdddr.*ydr).*v1r.^2-3*(yddr.*xdr-xddr.*ydr).*(xdr.*xddr+ydr.*yddr))./(v1r.^6+l^2*(yddr.*xdr-xddr.*ydr).^2);


%Nonlinear model reference state
qr=[xr;yr;thetar;phir];

x0=-1.2; y0=0; theta0=0; phi0=0;


%Index for scanning the trajectory
K=1;
Kf=length(xr);

Delta=0.35;

%Feedback linearization matrix
T_FL=@(phi,theta,Delta,l)([cos(theta)-tan(phi)*(sin(theta)+Delta*sin(theta+phi)/l) -Delta*sin(theta+phi); sin(theta)+tan(phi)*(cos(theta)+Delta*cos(theta+phi)/l) Delta*cos(theta+phi)]);


%Reference position feedback linearized system
zr=[xr+l*cos(thetar)+Delta*cos(phir+thetar);yr+l*sin(thetar)+Delta*sin(phir+thetar)];
zdr=zeros(2,length(xr));
for i=1:length(xr)
    
    vr_i=[v1r(i);v2r(i)];
    
    zdr(:,i)=T_FL(phir(i),thetar(i),Delta,l)*vr_i;
end


%Constraints
v1max=1; %Driving velocity limit (m/s)
v2max=1; %Steering velocity limit (rad/s)

u_max=[v1max;v2max];
phi_max=0.5;

T=[-1 0; 0 -1; 1 0; 0 1];
g=[v1max;v2max;v1max;v2max];


%Worst case input constraint set for the feedback-linearized model
ru=min(v1max,Delta*l*v2max/(sqrt(Delta^2+l^2))); %radius of the worst case circle 


%%Bound on the disturbance
disturbance=zeros(2,length(xr));
disturbance_norm=zeros(1,length(xr));
for i=1:length(xr) 
    vr_i=[v1r(i);v2r(i)];
    disturbance(1:2,i)= T_FL(phir(i),thetar(i),Delta,l)*vr_i;
    disturbance_norm(i)=disturbance(:,i)'*disturbance(:,i);
end

rd=sqrt((max(disturbance_norm)));
Qd=rd^2*eye(2);% disturbance Shaping matrix

figure
plot(disturbance(1,:),disturbance(2,:))
hold on
ell_d=ellipsoid(Qd);
plot(ell_d)
hold off

Qu=ru^2*eye(2);% disturbance Shaping matrix

Nsets=450;
Q0=Bd'*Qd*Bd;

plot_sets=false;
if plot_sets
    ell_0=ellipsoid(Q0);
    figure
    plot(ell_0)
    grid
    hold on
end

Qcurr=Q0;
Q_k=Q0;

for i=1:Nsets
    Qi=one_step_ellipsoidal_reachable_set(A,Bu,Bd,Qcurr,Qu,Qd);
    if plot_sets
        ell_i=ellipsoid(Qi);
        plot(ell_i,'b')
    end

    pause(0.00001)
    Q_k=[Q_k Qi];
    Qcurr=Qi;
    
end

%% Variables to store and plot the results
q0=[x0;y0;theta0;phi0];
z0=[x0+l*cos(theta0)+Delta*cos(phi0+theta0);y0+l*sin(theta0)+Delta*sin(phi0+theta0)];


tt=[];
qseq=q0;
v12seq=[];

k=1;
N_steps_simulation=length(xr);

%control effort matrix
R=0*eye(2);
R_d=0.05*eye(2);
u_pre=[0;0];



index_seq=[];

uk_seq=[];


%% Plots
figure
grid
hold on
axis([-3.5 3.5 -3.5 3.5])
plot(q0(1,1),q0(2,1),'b-p','MarkerIndices',[1 1],'MarkerFaceColor','yellow','MarkerSize',15)

plot(xr,yr,'r--')


avg_comp_time1=0;
optim1_steps=0;
optim2_steps=0;

avg_comp_time2=0;
z_tilde_seq=[];

for i=0:Ts:10000000
    %
    tt=[tt i];
    
    %extracting current state of differential-drive
    x=qseq(1,end);
    y=qseq(2,end);
    theta=qseq(3,end);
    phi=qseq(4,end);
    
    
    z_curr=[x+l*cos(theta)+Delta*cos(phi+theta);y+l*sin(theta)+Delta*sin(phi+theta)]-zr(:,k);
    z_tilde_seq=[z_tilde_seq z_curr]; 

 
    
    %Compute current input constraint set
    %     U_curr=T_FL_inv*U_uni;
    
    H_curr=T*inv(T_FL(phi,theta,Delta,l));
    U_curr=Polyhedron(H_curr,g);
    
    isintern=0;
    for j=1:2:length(Q_k)
        Q_curr=Q_k(1:2,j:j+1);
        if  z_curr'*inv(Q_curr)*z_curr<=1
            
            index=(j-1)/2+1;
            index_seq=[index_seq index];
            isintern=1;
            if index>1
                Qpre=Q_k(1:2,j-2:j-1);
            end
            break
        end
    end
 
    
    
    if isintern==0
        error('The current point is outside the domain of attraction of the algorithm')
    end
    
    
    ur=T_FL(phir(k),thetar(k),Delta,l)*[v1r(k);v2r(k)];
    
    
    
    if index>1
        
        
        tic
        ukhat = online_phase(z_curr,Qpre,A,Bu,H_curr,g,R_d,ur,u_pre);
        curr_comp_tim1=toc;
        avg_comp_time1=avg_comp_time1+curr_comp_tim1;
        optim1_steps=optim1_steps+1;   
    else
        
        tic
        ukhat=-inv(Bd)*z_curr;
        ur=online_terminal_optim_QP(ur,ukhat,H_curr,g);
        %ur=online_terminal_optim(ur,ukhat,H_curr,g);
        
        curr_comp_tim2=toc;
        avg_comp_time2=avg_comp_time2+curr_comp_tim2;
        optim2_steps=optim2_steps+1;
    end
    
    
    uk=ukhat+ur;
    u_pre=ukhat;

    uk_seq=[uk_seq uk];
   
    v12=inv(T_FL(phi,theta,Delta,l))*uk;
    
    v12seq=[v12seq v12];
    
    %Apply control law to the nonlinear car model
    v1=v12(1); v2=v12(2);
    t=0:0.00001:Ts;
    [t,q]= ode45(@(t,q)car_model(t,q,v1,v2,l),t,qseq(:,end));
    
    %update the state and input sequence
    qseq=[qseq q(end,:)'];
    
    %plotting robot trajectory
    plot(qseq(1,end),qseq(2,end),'b--x')
    pause(0.00001)
    
    if k>=N_steps_simulation
        break
    else
        k=k+1;
    end
end

avg_comp_time1=avg_comp_time1/optim1_steps;
avg_comp_time2=avg_comp_time2/optim2_steps;

avg_comp_time1*1000
avg_comp_time2*1000

figure
subplot(3,1,1);
hold on;
p1=plot(tt,v12seq(1,:));
p2=plot(tt,ones(1,length(tt))*v1max);
p2.LineStyle='--';
p2.LineWidth=2;
p2.Color='red';
p2=plot(tt,ones(1,length(tt))*-v1max);
p1.LineWidth=2;
p2.LineStyle='--';
p2.LineWidth=2;
p2.Color='red';
l1=legend(p2,'\omega_{r,max}','Interpreter','latex');
l1.set('FontSize',10)
grid;
xlbl1=xlabel('Time[sec]','Interpreter','latex');
ylbl1=ylabel('$\omega_r(t)[RAD/sec]$','Interpreter','latex');
ylbl1.FontSize=13;
xlbl1.FontSize=13;

subplot(3,1,2);
hold on;
p3=plot(tt,v12seq(2,:));
p3.LineWidth=2;

p4=plot(tt,ones(1,length(tt))*v2max,'r');
p4.LineStyle='--';
p4.LineWidth=2;

p4=plot(tt,ones(1,length(tt))*-v2max,'r');

p4.LineStyle='--';
p4.LineWidth=2;
p4.Color='red';
l2=legend(p4,'\omega_{l,max}','Interpreter','latex');
l2.FontSize=10;
grid;
xlbl2=xlabel('Time[sec]','Interpreter','latex');
ylbl2=ylabel('$\omega_l(t)[RAD/sec]$','Interpreter','latex');
ylbl2.FontSize=13;
xlbl2.FontSize=13;



subplot(3,1,3)
pl=plot(tt,qseq(3,1:end-1));
pl.LineWidth=2;
grid
% axis([0 tt(length(tt)) 0 6])
xlbl2=xlabel('Time [sec]','Interpreter','latex');
ylbl2=ylabel('$\theta(t)[RAD]$','Interpreter','latex');

figure
p1=plot(tt,index_seq);
grid
p1.LineWidth=2;

eps=0;
for k=1:N_steps_simulation
    x_tilde=[xr(k);yr(k)]-qseq(1:2,k);
    eps=eps+x_tilde'*x_tilde;
    
end

eps=eps/N_steps_simulation

pose_seq=qseq;
Q_seq=Q_k;
t=tt;

%save all simulation data
save('Simulation_data','index_seq','pose_seq','Q_seq','t','thetar','phir','v12seq','xr','yr','v1max','v2max','z_tilde_seq')