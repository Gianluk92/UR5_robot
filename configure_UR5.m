%% initialization 
%clear all
close all
startup_rvc
global robot

%% construction

% the mass centre and the inertia tensor are computed on the data furnished
% by the manual.
%[THETA D A ALPHA SIGMA] DH matrix
link(1) = Link ('theta',0,'a',0,'alpha',pi/2);
 link(1).m = 1; %[kg]
 link(1).r = [0,-0.02561,0.00193]; % mass centre position [m]
 link(1).I = [0.0067 0 0; 0 0.0064 0;0 0 0.0067]; % Inertia

 link(2) = Link ('theta',0,'a',0,'alpha',0);
 link(2).m = 1; %[kg]
 link(2).r = [0,-0.02561,0.00193]; % mass centre position [m]
 link(2).I = [0.0067 0 0; 0 0.0064 0;0 0 0.0067]; % Inertia

 
link(3) = Link ('d',0.08916,'a',0,'alpha',pi/2);
link(3).m = 3.7; %[kg]
link(3).r = [0,-0.02561,0.00193]; % mass centre position [m]
% assuming constant mass density ro = M/(pi*r^2*h)
% Ixx = 1/2*M*(3*r^2+h^2) Iyy = Izz = 1/12*M*(3*r^2+h^2)
link(3).I = [0.0067 0 0; 0 0.0064 0;0 0 0.0067]; % Inertia

link(4) = Link ('d',0,'a',-0.425,'alpha',0);
link(4).m = 8.393; %[kg]
link(4).r = [0.2125,0,0.11336]; %mass centre position [m]
link(4).I = [0.0149 0 0; 0 0.3564 0;0 0 0.3553]; % Inertia

link(5) = Link ('d',0,'a',-0.39225,'alpha',0);
link(5).m = 2.275; %[kg]
link(5).r = [0.11993,0,0.0265]; %mass centre position [m]
link(5).I = [0.0025 0 0.0034; 0 0.0551 0;0.0034 0 0.0546]; % Inertia

link(6) = Link ('d',0.10915,'a',0,'alpha',pi/2);
link(6).m = 1.219; %[kg]
link(6).r = [0,-0.0018,0.01634]; %mass centre position [m]
link(6).I = [0.0012 0 0; 0 0.0012 0;0 0 0.0009]; % Inertia

link(7) = Link ('d',0.09465,'a',0,'alpha',-pi/2);
link(7).m = 1.219; %[kg]
link(7).r = [0,0.0018,0.01634]; %mass centre position [m]
link(7).I = [0.0012 0 0; 0 0.0012 0;0 0 0.0009]; % Inertia

link(8) = Link ('d',0.0823,'a',0,'alpha',0);
link(8).m = 0.1879; %[kg]
link(8).r = [0,0,-0.001159]; %mass centre position [m]
link(8).I = [0.0001 0 0; 0 0.0001 0;0 0 0.0001]; % Inertia


%% PPRRRRRR
W = [-5 5 -5 5 -5 5];
B = [rpy2r(0,pi/2,0) [0,0,0.2]';
    0 0 0 1];
%       
Qlim = [-2 2;-2 2;pi 2*pi;pi 2*pi;-pi/2 pi/2;0 2*pi;-2*pi 2*pi;0 2*pi];
robot = SerialLink(link,'name','UR5','qlim',Qlim,'base',B);
robot.plotopt = {'workspace' W};

