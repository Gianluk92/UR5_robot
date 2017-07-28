function [w] = functional(q,control)
   global robot
   Pos = cell(8,1);
   dist = [];
   Eucli_cord = robot.base;
   T1 = transl(control);
   
   for j=1:1:robot.n
   % this is the trasformation matrix at the end-effector
   % return the position of the end effector wrt the world coordinate
      Eucli_cord =Eucli_cord*robot.links(j).A(q(j));
      %joint_pos = [joint_pos; Eucli_cord];
   % this compute the distance between the joint J to the point more the
   % joint is near more it value will be changed
      dist =[dist; norm(Eucli_cord(1:3,4)'-T1(1:3,4)')];
   end
   dist';
   %w = sqrt(sum((dist.^2)));
   w = min((dist.^2));
end
