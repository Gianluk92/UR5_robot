function [path,plot_path] = generatePathOpt(T_init,T_end,N_step,control)
    global robot a b;
    b = [];
    path = [];
    plot_path = [];
    K = 0.05;
    T = ctraj(T_init,T_end,N_step);
    q = [0,1,3/2*pi,3/2*pi,0,pi,0,0];
    q_test = q;
    %q = robot.ikine(T_init,'alpha',0.05);
    path = [path; q];
    a = [];
    for i=2:1:N_step
        ve(1:3) = transl(T(:,:,i))-transl(T(:,:,i-1));
        ve(4:6) = tr2rpy(T(:,:,i))-tr2rpy(T(:,:,i-1));
        
        J = robot.jacob0(q);
        J_star = J'*(J*J'+ (K^2)*eye(size(J*J')))^-1;
        % calcolo di q0_dot
        %% traiettoria originale
%         tmp_1 = robot.fkine(q_test+J_star*ve');
%         hold on
%         plot3(tmp_1(1,4),tmp_1(2,4),tmp_1(3,4),'ro')
%         plot3(0.1,0.1,-0.1,'bo');
        %%
        q0_dot = grad(q,control);
        q_dot = (J_star*ve'+(eye(size(J_star*J))-J_star*J)*q0_dot)';
        a = [a;q0_dot'];
        q = q + q_dot;
        b = [b;q]; 
        %hold on
        path = [path; q];
        tmp = robot.fkine(q);
        plot_path = [plot_path; tmp];
    end
end