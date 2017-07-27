function [obstacle] = createObstacle(n,string_form,dim,pos)
    object = cell(n);
    for i=1:1:n
        switch(string_form{i})
            case 'box'
                object{i} = Box(pos{i},1);
                object{i}.scale = dim{i};
            case 'cone'
                object{i} = Cone(pos{i},1);
                object{i}.scale = dim{i};
            case 'sphere'
                object{i} = Sphere(pos{i},1);
                object{i}.scale = dim{i};
            case 'cylinder'
                object{i} = Cylinder(pos{i},1,1);
                object{i}.scale = dim{i};
        end
    end
    obstacle = CollisionModel(object{1:n}); 
end