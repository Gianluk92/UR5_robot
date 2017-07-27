close all
%clear all
%configure_UR5
global robot

nstep = 100;
init_cart = [-0.18,-1.8,0.1054];
init_angle = [0.1,-1.5,-1.5];
final_cart = [0.6,0.4,1];
final_angle = [0,1.5,0.1];
control = [0.5,0.3,1];

% generiamo le matrici di trasformazione omogenea
T0 = [rpy2r(init_angle) init_cart';
      0 0 0 1];
T1 = [rpy2r(final_angle) final_cart';
      0 0 0 1];

obstacle = train_structure();
obstacle.plot();

% %% path avoid the singularity
% [traj,p_traj] = generatePathNoOpt(T0,T1,nstep);
% %robot.plot(traj,'scale',0.3);
% 
% hold on
% for i = 1:1:nstep-2
%     if(isColliding(robot,obstacle,traj(i,:),2))
%         disp('collision detect');
%         disp(i);
%         break;
%     end
%     if(i == 1)
%         plot3(p_traj(1+4*i,4),p_traj(2+4*i,4),p_traj(3+4*i,4),'g*')
%     elseif(i == nstep-2)
%         plot3(p_traj(1+4*i,4),p_traj(2+4*i,4),p_traj(3+4*i,4),'r*')
%     else
%         plot3(p_traj(1+4*i,4),p_traj(2+4*i,4),p_traj(3+4*i,4),'b-')
%     end
% end

%% Path avoid singularity and optimize the inter-joint distance  
[traj_op,p_traj_op] = generatePathOpt(T0,T1,nstep,control);
%robot.plot(traj_op,'scale',0.3);

hold on
plot3(control(1),control(2),control(3),'ro')
for i = 1:1:nstep-2
    if(isColliding(robot,obstacle,traj_op(i,:),2))
        disp('collision detect');
        disp(i);
        break;
    end
    if(i == 1)
        plot3(p_traj_op(1+4*i,4),p_traj_op(2+4*i,4),p_traj_op(3+4*i,4),'b*')
    elseif(i == nstep-2)
        plot3(p_traj_op(1+4*i,4),p_traj_op(2+4*i,4),p_traj_op(3+4*i,4),'r*')
    else
        plot3(p_traj_op(1+4*i,4),p_traj_op(2+4*i,4),p_traj_op(3+4*i,4),'bo')
    end
end
