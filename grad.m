function [q0_dot] = grad(q,control)
    global robot 
    debug = false;
    pas = eye(size(q)*[0;1])*0.00001;
    q0_dot = [];
    for i=1:1:robot.n
        if((q(i)+0.00001 > robot.qlim(i,2)))
            next_q = q;
            X = ['out',num2str(i)];
            disp(X);
        else
            next_q = q + pas(i,:);
        end
        if(debug)
            X = [num2str(functional(next_q,control)),'   |   ',num2str(functional(q,control)),'  |  ',num2str(i)];
            disp('next_q   |    q      | joint ')
            disp(X)
        end
        %a = functional(q,control);
        q0_dot = [q0_dot;(functional(next_q,control)-functional(q,control))/0.00001];
    end 
    q0_dot = 0.1 * q0_dot;
end