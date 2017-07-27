function [path,plot_path,orig] = generatePathNoOpt(T_init,T_end,N_step)
    global robot;
    path = [];
    K = 0.005;
    plot_path = [];
    T = ctraj(T_init,T_end,N_step);
    orig = T;
    q = [0,1,3/2*pi,3/2*pi,0,pi,0,0];
    %q = robot.ikine(T(:,:,1),'alpha',0.05);
    path = [path; q];
    for i=2:1:N_step
        ve(1:3) = transl(T(:,:,i))-transl(T(:,:,i-1));
        ve(4:6) = tr2rpy(T(:,:,i))-tr2rpy(T(:,:,i-1));
        J = robot.jacob0(q);
        J_c = J'*((J*J'+K*eye(size(J*J')))^-1);
        q_dot = J_c * ve';
        q_next = q + q_dot';
        q = q_next;
        path = [path; q];
        tmp = robot.fkine(q);
        plot_path = [plot_path; tmp];
    end
end