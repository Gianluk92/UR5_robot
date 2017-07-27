%% parametri DH [theta d a alpha sigma(0 rotoidale 1 prismatico)]
close all 
clear all
syms th1 th2 th3 th4 th5 th6

%%
d1=0.089459;    d2=0;           d3=0;           d4=0.10915;     d5=0.09465;     d6=0.0823;
a1=0;           a2=-0.42500;    a3=-0.39225;    a4=0;           a5=0;           a6=0;
alpha1=pi/2;    alpha2= 0;      alpha3=0;       alpha4=pi/2;    alpha5=-pi/2;   alpha6=0;

%[THETA D A ALPHA SIGMA]
linkSym(1) = Link([0 d1 a1 alpha1 0]); %dimensioni in metri
linkSym(1).m=3.7;
linkSym(1).r=[0,-0.02561, 0.00193];
linkSym(1).G=1;
linkSym(1).I=[0.0084 0 0; 0 0.0064 0; 0 0 0.0084];

linkSym(2) = Link([0 d2 a2 alpha2 0]);
linkSym(2).m=8.3930;
linkSym(2).r=[0.2125, 0, 0.11336];
linkSym(2).G=1;
linkSym(2).I=[0.0078 0 0; 0 0.2100 0; 0 0 0.2100];

linkSym(3) = Link([0 d3 a3 alpha3 0]);
linkSym(3).m=2.33;
linkSym(3).r=[0.15, 0, 0.0265];
linkSym(3).G=1;
linkSym(3).I=[0.0016 0 0; 0 0.0462 0; 0 0 0.0462];

linkSym(4) = Link([0 d4 a4 alpha4 0]);
linkSym(4).m=1.219;
linkSym(4).r=[0, -0.0018, 0.01634];
linkSym(4).G=1;
linkSym(4).I=[0.0016 0 0; 0 0.0016 0; 0 0 0.0009];

linkSym(5) = Link([0 d5 a5 alpha5 0]);
linkSym(5).m= 1.219;
linkSym(5).r=[0, -0.0018, 0.01634];
linkSym(5).G=1;
linkSym(5).I= [0.0016 0 0; 0 0.0016 0; 0 0 0.0009];

linkSym(6) = Link([0 d6 a6 alpha6 0]);
linkSym(6).m= 0.1897;
linkSym(6).r=[0, 0, -0.001159];
linkSym(6).G=1;
linkSym(6).I= [0.0001 0 0; 0 0.0001 0; 0 0 0.0001];


%robotSym = SerialLink(linkSym,'name','Gimbal');
UR5 = SerialLink(linkSym,'name','UR5','qlim',[0 pi;0 pi;0 pi;0 pi;0 pi;0 pi]);

UR5.payload(5,[0,0,0.1]) % carico massimo (tutte le possibili telecamere)
%A=zeros(1,length(linkSym));
base=0;
UR5.base=transl([0 0 base]);
%robotSym.tool=troty(pi/2)
UR5.plot([0 0 0 0 0 0]),axis equal

%J=robotSym.jacobn([th1,th2]);

A=UR5.base*linkSym(1).A(th1)*linkSym(2).A(th2)*linkSym(3).A(th3)...
    *linkSym(4).A(th4)*linkSym(5).A(th5)*linkSym(6).A(th6)*UR5.tool;

T=UR5.fkine([0,0,0,0,0,0]); %for initial position [100 35 100]

UR5.dyn(); %mass, centre of mass, inertia, gear ratio, motor inertia and motor friction

return
UR5.gravload([0,0,0,0,0,0])

return
%%
q=UR5.ikine(T,[(pi*100/180),(pi*35/180)],[0 0 0 1 1 0])

zmin=base;
zmax=base+a2;
xmin=a2+d2; xmax=-a1;
ymin=a1;    ymax=a2+d2;

%%
% mass center at base configuration
mUR5=[3.7000, 8.3930, 2.33, 1.2190, 1.2190, 0.1897];
cdmUR5 = [0 -0.02561 0.00193; 0.2125  0 0.11336; 0.15 0 0.0265;0 -0.0018 0.01634;0 -0.0018 0.01634;...
    0 0 -0.001159]; 

cmdLink{1}=[cdmUR5(1,1)             -(cdmUR5(1,3)+d2)           d1+cdmUR5(1,2)]
cmdLink{2}=[cdmUR5(2,1)-a2          -(cdmUR5(2,3)+d2+d3)        d1+cdmUR5(2,2)]
cmdLink{3}=[cdmUR5(3,1)-a2-a3       -(cdmUR5(3,3)+d2+d3+d4)     d1+cdmUR5(3,2)]
cmdLink{4}=[cdmUR5(4,1)-a2-a3-a4    -(cdmUR5(4,2)+d2+d3+d4)     d1-cdmUR5(4,3)]
cmdLink{5}=[cdmUR5(5,1)-a2-a3-a4    -(cdmUR5(5,3)+d2+d3+d4)     d1-d5+cdmUR5(5,2)]
cmdLink{6}=[cdmUR5(6,1)-a2-a3-a4+a6 -(cdmUR5(6,3)+d2+d3+d4+d6)  d1-d5+cdmUR5(6,2)]

xm=0;ym=0;zm=0;
for i=1:1:6       
    xm=xm+cmdLink{i}(1)*mUR5(i);
    ym=ym+cmdLink{i}(2)*mUR5(i);
    zm=zm+cmdLink{i}(3)*mUR5(i);    
end
xmass{1}=xm/sum(mUR5);
ymass{1}=ym/sum(mUR5);
zmass{1}=zm/sum(mUR5);

%mass center at max extension with variable payload
mUR5=[3.7000, 8.3930, 2.33, 1.2190, 1.2190, 0.1897];
cdmUR5 = [0 -0.02561 0.00193; 0.2125  0 0.11336; 0.15 0 0.0265;0 -0.0018 0.01634;0 -0.0018 0.01634;...
    0 0 -0.001159]; 

cmdLink{1}=[cdmUR5(1,1)             -(cdmUR5(1,3)+d2)           d1+cdmUR5(1,2)]
cmdLink{2}=[cdmUR5(2,1)-a2          -(cdmUR5(2,3)+d2+d3)        d1+cdmUR5(2,2)]
cmdLink{3}=[cdmUR5(3,1)-a2-a3       -(cdmUR5(3,3)+d2+d3+d4)     d1+cdmUR5(3,2)]
cmdLink{4}=[cdmUR5(4,3)-a2-a3-a4    -(cdmUR5(4,2)+d2+d3+d4)     d1+cdmUR5(4,1)]
cmdLink{5}=[d5-cdmUR5(5,2)-a2-a3-a4 -(cdmUR5(5,3)+d2+d3+d4)     d1+cdmUR5(5,1)]
cmdLink{6}=[d5-cdmUR5(6,2)-a2-a3-a4 -(cdmUR5(6,3)+d2+d3+d4+d6)  d1+a6+cdmUR5(6,1)]

xm=0;ym=0;zm=0;
endeffMass=[0 0.5 1 1.5 2 3 4 5];
for m=1:1:8
    for i=1:1:6
        if i==6
            xm=xm+cmdLink{i}(1)*(mUR5(i)+endeffMass(m));
            ym=ym+cmdLink{i}(2)*(mUR5(i)+endeffMass(m));
            zm=zm+cmdLink{i}(3)*(mUR5(i)+endeffMass(m)); 
        else
            xm=xm+cmdLink{i}(1)*(mUR5(i));
            ym=ym+cmdLink{i}(2)*(mUR5(i));
            zm=zm+cmdLink{i}(3)*(mUR5(i));     
        end
    end
xmass{1+m}=xm/(sum(mUR5)+endeffMass(m));
ymass{1+m}=ym/(sum(mUR5)+endeffMass(m));
zmass{1+m}=zm/(sum(mUR5)+endeffMass(m));
xm=0;ym=0;zm=0;
end

xm=0; ym=0; zm=0;
for i=1:1:9
    xm(i)=xmass{i};
    ym(i)=ymass{i};
    zm(i)=zmass{i}; 
    plot3(xm(i),ym(i),zm(i),'.','MarkerSize',i*7)
    if ((i==4)||(i==5)||(i==6)||(i==7))
        text(xm(i),ym(i),zm(i),num2str([xm(i); ym(i); zm(i)]),'FontSize',12)
    end
    hold on
end
legend('no load','no load','0.5 Kg','1 Kg','1.5','2 Kg','3 Kg','4 Kg','5 Kg')
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
grid on
return
%% parametri DH [theta d a alpha sigma(0 rotoidale 1 prismatico)]
close all 
clear all
syms th1 th2 th3

%%
a1=0.002; %off1
d2=0.005; %off2 sbraccio
a2=0.06;  %dlink

% lunghezza del vettore del triangolo
ltri=sqrt( ((d2+a2)^2)+(a1^2) );

linkSym(1) = Link([0 0 a1 pi/2 0]); %dimensioni in metri
linkSym(2) = Link([0 d2 a2 pi/2 0]);


%robotSym = SerialLink(linkSym,'name','Gimbal');
robotSym = SerialLink(linkSym,'name','Gimbal','qlim',[0 pi;0 pi]);
%A=zeros(1,length(linkSym));
base=0.045;
robotSym.base=transl([0 0 base]);
%robotSym.tool=troty(pi/2)
robotSym.plot([pi/4 pi/4]),axis equal
%[theta1 theta2 0 pi/2 theta3]
J=robotSym.jacobn([th1,th2]);

A=robotSym.base*linkSym(1).A(th1)*linkSym(2).A(th2)*robotSym.tool

T=robotSym.fkine([(pi*0/180),(pi*0/180)]) %for initial position [100 35 100]
q=robotSym.ikine(T,[(pi*100/180),(pi*35/180)],[0 0 0 1 1 0])

zmin=base;
zmax=base+a2;
xmin=a2+d2; xmax=-a1;
ymin=a1;    ymax=a2+d2;
return
%% symbolic parametri DH [theta d a alpha sigma(0 rotoidale 1 prismatico)]
close all 
clear all
% syms th1 th2 al1 al2 a1 a2 d1 d2
% linkSym(1) = Link([th1 d1 a1 al1 0]); %dimensioni in metri
% linkSym(2) = Link([th2 d2 a2 al2 0]);
syms th1 th2 a1 a2 d1 d2
linkSym(1) = Link([th1 d1 a1 pi/2 0]); %dimensioni in metri
linkSym(2) = Link([th2 d2 a2 0 0]);

robotSym = SerialLink(linkSym,'name','Gimbal','qlim',[0 pi;0 pi]);
robotSym.base=transl([0 0 0.045]);
robotSym.tool=troty(0.5)

A=robotSym.base*linkSym(1).A(th1)*linkSym(2).A(th2)*robotSym.tool
return
%% parametri DH [theta d a alpha sigma(0 rotoidale 1 prismatico)]
close all 
clear all
syms th1 th2 th3
linkSym(1) = Link([0 0.06 0 pi/2 0]); %dimensioni in metri
linkSym(2) = Link([0 0 0.012 pi/2 0]); %dimensioni in metri
linkSym(3) = Link([0 -0.01 0.065 0 0]);
linkSym(4) = Link([0 0 0.01 pi/2 0]);
linkSym(5) = Link([0 0 0 0 0]);

%robotSym = SerialLink(linkSym,'name','Gimbal');
robotSym = SerialLink(linkSym,'name','Gimbal','qlim',[0 pi;0 pi;0 0;pi/2 pi/2;0 pi]);
%A=zeros(1,length(linkSym));
robotSym.plot([0 0 0 pi/2 0]),axis equal
%[theta1 theta2 0 pi/2 theta3]
J=robotSym.jacobn([th1,th2,0,pi/2,th3]);

T=robotSym.fkine([(pi*100/180),(pi*35/180),0,pi/2,(pi*100/180)]) %for initial position [100 35 100]
q=robotSym.ikine(T,[(pi*100/180),(pi*35/180),0,pi/2,(pi*100/180)],[1 1 1 0 0 0])
return
%%
%T=robotSym.fkine([th1,th2,0,pi/2,th3])
%q=robotSym.ikine(T,[th1,th2,0,pi/2,th3],[1 1 1 0 0 0])


T=robotSym.fkine([0,0,0,pi/2,0])
q=robotSym.ikine(T,[0 0 0 pi/2 0],[1 1 1 0 0 0])

T1=robotSym.fkine([pi/2,0,0,pi/2,0])



return
%% parametri DH [theta d a alpha sigma(0 rotoidale 1 prismatico)]
close all 
clear all

linkSym(1) = Link([0 0 0.012 pi/2 0]); %dimensioni in metri
linkSym(2) = Link([0 -0.01 0.065 0 0]);
linkSym(3) = Link([0 0 0.01 pi/2 0]);
%linkSym(4) = Link([0 0 0 0 0]);

robotSym = SerialLink(linkSym,'name','Gimbal');
%A=zeros(1,length(linkSym));
robotSym.plot([0 0 pi/2]),axis equal

return

%% prova DH solo con 3 giunti
% parametri DH [theta d a alpha sigma(0 rotoidale 1 prismatico)]
close all 
clear all

linkSym(1) = Link([0 0 0.012 -pi/2 0]); %dimensioni in metri
linkSym(2) = Link([0 -0.01 0.065 pi/2 0]);
linkSym(3) = Link([0 0 0.01 pi/2 0]);

%linkSym(4) = Link([0 0 0 0 0]);

robotSym = SerialLink(linkSym,'name','Gimbal');
%A=zeros(1,length(linkSym));
robotSym.plot([0 -pi/4 pi/4]),axis equal