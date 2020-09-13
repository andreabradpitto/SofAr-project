figure,plot(t,xg(1,:),t,xg(2,:),t,xg(3,:));legend('x','y','z'); % plot position ref signal

if dont_have_good_omega
    [~,~,TOe,~,~] = robot_control(Baxter, dt, tmax, xg, wg, 0, qinit,...
        ROg0, 1, 0, 1,nolims,params,0,draw_robot,nmv,zeros(n,tn));
    ROg_ref = TOe(1:3,1:3,:); ROg_new_init = TOe(1:3,1:3,1);
else, ROg_ref = eye(3); % not used in simulation, pass some arbitrary matrix
end
if exec_opt(1)
    clear joint_constr reg_pinv;
    robot_control(Baxter, dt, tmax, xg, wg, ROg_ref, qinit, ROg0, 3, 1,... 
    ~dont_have_good_omega,nolims,params,1,draw_robot,nmv,noise);
    input('Press enter to continue.');
end
if exec_opt(2)
    clear joint_constr reg_pinv;
    robot_control(Baxter, dt, tmax, xg, wg, ROg_ref, qinit, ROg0, 3,...
        2, ~dont_have_good_omega,nolims,params,1,draw_robot,nmv,noise);
    input('Press enter to continue.');
end
if exec_opt(3)
    clear joint_constr reg_pinv;
    robot_control(Baxter, dt, tmax, xg, wg, ROg_ref, qinit, ROg0,...
    2, 1, ~dont_have_good_omega,nolims,params,1,draw_robot,nmv,noise);
    input('Press enter to continue.');
end
if exec_opt(4)
    clear joint_constr reg_pinv;
    robot_control(Baxter, dt, tmax, xg, wg, ROg_ref, qinit, ROg0,...
        2, 2, ~dont_have_good_omega,nolims,params,1,draw_robot,nmv,noise);
    clear joint_constr reg_pinv;
    input('Press enter to continue.');
end