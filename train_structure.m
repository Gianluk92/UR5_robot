%% Generate scenario
function [obstacle] = train_structure()
%close all
% asse centrale
dim_asse = [0.1,0.1,2.8];
pos_asse = [0,0.6,1];
angl_asse = rpy2r(0,pi/2,0);
Tasse = [angl_asse pos_asse';
         0 0 0 1];
% disco freno 1     
dim_disco1 = [0.3,0.3,0.1];
pos_disco1 = [0.4,0.6,1];
angl_disco1 = rpy2r(0,pi/2,0);
Tdisco1 = [angl_disco1 pos_disco1';
          0 0 0 1;];
% disco freno 2      
dim_disco2 = [0.3,0.3,0.1];
pos_disco2 = [1.3,0.6,1];
angl_disco2 = rpy2r(0,pi/2,0);
Tdisco2 = [angl_disco2 pos_disco2';
          0 0 0 1;];
% disco freno 3       
dim_disco3 = [0.3,0.3,0.1];
pos_disco3 = [2.2,0.6,1];
angl_disco3 = rpy2r(0,pi/2,0);
Tdisco3 = [angl_disco3 pos_disco3';
          0 0 0 1;];
% pasticca freno 1
dim_pasticca1 = [0.025,0.22,0.12];
pos_pasticca11 = [0.525,0.85,1];
angl_pasticca1 = rpy2r(pi/2,0,0);
Tpasticca11 = [angl_pasticca1 pos_pasticca11';
            0 0 0 1;];
pos_pasticca12 = [0.375,0.85,1];
Tpasticca12 = [angl_pasticca1 pos_pasticca12';
            0 0 0 1;];
% pasticca freno 2
dim_pasticca2 = [0.025,0.22,0.12];
pos_pasticca21 = [1.425,0.85,1];
angl_pasticca2 = rpy2r(pi/2,0,0);
Tpasticca21 = [angl_pasticca2 pos_pasticca21';
            0 0 0 1;];
pos_pasticca22 = [1.275,0.85,1];
Tpasticca22 = [angl_pasticca2 pos_pasticca22';
            0 0 0 1;];
% pasticca freno 3
dim_pasticca3 = [0.025,0.22,0.12];
pos_pasticca31 = [2.325,0.85,1];
angl_pasticca3 = rpy2r(pi/2,0,0);
Tpasticca31 = [angl_pasticca3 pos_pasticca31';
            0 0 0 1;];
pos_pasticca32 = [2.175,0.85,1];
Tpasticca32 = [angl_pasticca3 pos_pasticca32';
            0 0 0 1;];

figure = {'cylinder','cylinder','cylinder','cylinder','box','box','box','box','box','box'};        
dimension = {dim_asse,dim_disco1,dim_disco2,dim_disco3,dim_pasticca1,dim_pasticca1,dim_pasticca2,dim_pasticca2,dim_pasticca3,dim_pasticca3};
position = {Tasse,Tdisco1,Tdisco2,Tdisco3,Tpasticca11,Tpasticca12,Tpasticca21,Tpasticca22,Tpasticca32,Tpasticca31};
obstacle = createObstacle(10,figure,dimension,position);

obstacle.plot();
end