% this function control if some link collide with the obstacle
function [point] = linkCollision(robot,Eucli_cord,j)
    debug = false;
    link_control = {[0;0.1],[0;-0.1],[0.1;0],[-0.1;0]};
    center = Eucli_cord(1:3,4);
    rotation = Eucli_cord(1:3,1:3);
    control_point = [];
    if robot.links(j).a ~= 0
        for i=1:1:4
            control_point =[control_point, (center + rotation*[-robot.links(j).a/4;link_control{i}])'];    
            control_point =[control_point, (center + rotation*[-robot.links(j).a/2;link_control{i}])'];    
            control_point =[control_point, (center + rotation*[-robot.links(j).a*3/4;link_control{i}])'];    
            control_point =[control_point, (center + rotation*[-robot.links(j).a;link_control{i}])'];    
        end
    end
    point = control_point;

    if(debug)
        hold on
        for i=1:3:size(point,2)
            plot3(point(i),point(i+1),point(i+2),'g*');
        end
        next_joint = center + rotation*[-robot.links(j).a;0;0];
        link = Cylinder({Eucli_cord(1:3,4)',next_joint'},0.1);
        link.plot()
    end
end