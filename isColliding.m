
function [isColliding,j] = isColliding(robot,obstacle,q,axes)
    debug = false;
    isColliding = false;
    joint_pos = q;
    Eucli_cord = robot.base*robot.links(1).A(joint_pos(1))*robot.links(2).A(joint_pos(2));
    hold on
    for j=3:1:robot.n
    % this is the trasformation matrix at the end-effector
    % return the position of the end effector wrt the world-coordinate
    %%    
        Eucli_cord = Eucli_cord*robot.links(j).A(joint_pos(j));
        % check if the joint is inside an obstacle during its path    
        point = linkCollision(robot,Eucli_cord,j);
        if size(point,2)
         if(obstacle.collision(point))
         disp('Link collision')
         end
        end
        if((j<robot.n && obstacle.collision(Eucli_cord(1:3,4)')) || isColliding)  
            X = ['Joint crash: ',num2str(j)];
            disp(X);
            isColliding = true;
            return;
        end
    end
    if(axes == 3)
        s_p1 =(Eucli_cord(1:3,4)+Eucli_cord(1:3,1:3)*[0;0;0.12]);
        s_p2 =(Eucli_cord(1:3,4)+Eucli_cord(1:3,1:3)*[0.06;0;0.12]);
        s_p3 =(Eucli_cord(1:3,4)+Eucli_cord(1:3,1:3)*[-0.06;0;0.12]);
    elseif(axes == 2)
        s_p1 =(Eucli_cord(1:3,4)+Eucli_cord(1:3,1:3)*[0;0.12;0]);
        s_p2 =(Eucli_cord(1:3,4)+Eucli_cord(1:3,1:3)*[0;0.12;0.06]);
        s_p3 =(Eucli_cord(1:3,4)+Eucli_cord(1:3,1:3)*[0;0.12;-0.06]);
    else 
        error('error choose y or z axes');
    end
    if(debug)
        % plotta la traiettoria del controllo effettuato
        plot3(s_p2(1),s_p2(2),s_p2(3),'r*')
        plot3(s_p3(1),s_p3(2),s_p3(3),'r*')
        plot3(s_p1(1),s_p1(2),s_p1(3),'go')
        % plotta la traiettoria dell'EE
        % plot3(Eucli_cord(1,4),Eucli_cord(2,4),Eucli_cord(3,4),'go')
        % plotta il cono di controllo
        %c = Cone([Eucli_cord(1:3,1:3) s_p1;0 0 0 1],0.06);
        %c.plot();
    end
    
    if(obstacle.collision(s_p1') || obstacle.collision(s_p2') || obstacle.collision(s_p3'))
        isColliding = true;
        disp('hand effector crash');
        return;
    end
end