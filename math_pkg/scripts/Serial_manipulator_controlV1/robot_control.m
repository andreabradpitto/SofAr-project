% Desc here

function [q,qdot,TOe,eta,rho] = robot_control(robot_struct, dt, tmax, xg, wg, ROg_v,...
    qinit, ROg_init, ctrl_type, opt_type, rotrefw, limitsoff, params, mustplot, draw_robot,...
    not_move_feat,qnoise)

    % close all;

    % Parameters of the simulation.
    Q_HARD_MARGIN = 0.02;NJOINTS = robot_struct.NJOINTS;NTASKS = 2; %TODO
    QMIN = robot_struct.QMIN + Q_HARD_MARGIN;
    QMAX = robot_struct.QMAX - Q_HARD_MARGIN;
    QDOTMAX = robot_struct.QDOTMAX; % max joint velocities
    QTYPES = robot_struct.QTYPES;QFAV = zeros(7,1);
    gamma = 0.1;eta1 = 2;x = 0;
    REGOPT1 = 0.00001;REGOPT2 = 0.001;
    REG = [params.reg1,params.reg2];
    QMARGIN = 0.1; QDOTMARGIN = 0.1;
    Kvel = 40; Kpos = 20*20;KposP = params.KposP;Krot = params.KrotP;
    sat = params.sat; diff_prec = params.diff_prec;
    stay_still_thr = params.stay_still_thr;

    % Def time
    t = 0:dt:tmax; tn = length(t);
    
    % Get linear velocities and accelerations.
    [vg,ag] = get_lin_vels_and_accs(xg,dt,tn);

    % Initialize some variables.
    [eta,rho,q,qdot,J,Jlims,rdot] = init_stuff(NJOINTS,tn);
    etadot = zeros(3,tn);
    TOe = zeros(4,4,tn);
   
    if ~rotrefw, wg = zeros(3,tn); end
        
    % What to track? Will need to look at TASKS... TODO
    if ctrl_type == 0, i1 = 1;i2 = 3; %posture tracking
    elseif ctrl_type == 1, i1 = 4;i2 = 6; %position tracking
    elseif ctrl_type > 1, i1 = 1;i2 = 6; %full tracking
    end
    
    A_track_sz = i2 - i1 + 1; % size of A matrix for tracking task
    A = cell(NTASKS,1);
    
    diff_opt = ones(A_track_sz,tn);
    
    for k = 1:tn % for each time step
        if k==1 % at first step initialize ROg, etadot, JLdot.
            ROg = ROg_init;
            etadot(:,k) = 0;JLdot = zeros(3,NJOINTS);
            q(:,k) = qinit;
        end
        
        % Compute T matrices.
        [Tvec,TOe(:,:,k)] = gm2(gm1(q(:,k), robot_struct.DH, QTYPES,...
            robot_struct.Tne));
        
        % Compute J for tracking task.
        J(:,:,k) = km(TOe(:,:,k),Tvec,QTYPES);
        
        % Compute A matrix for joint limits task.
        
        [Jlims(:,:,k),A{1},rdot(:,k)] =...
            joint_constr(q(:,k),qdot(:,k),QMIN,QMAX,QDOTMAX,...
            QMARGIN,QDOTMARGIN,dt);
        
        Jlim = Jlims(:,:,k); % for convenient access
        
        % Compute eta = linear error.
        xe = TOe(1:3,4,k);
        eta(:,k) = xg(:,k) - xe;
       
        
        % Compute rho = angle between ee & goal.
        if rotrefw
            ROg = strap_down(wg(:,k),ROg,dt);
        else, ROg = ROg_v(:,:,k);
        end
        rho(:,k) = p_a(ROg,TOe(:,:,k));
        A{NTASKS} = eye(A_track_sz);
        if not_move_feat
            if (all(abs([eta(:,k);rho(:,k)]) < stay_still_thr))
                eta(:,k) = zeros(3,1);
                rho(:,k) = zeros(3,1);
            end
        end
        if k > 1
            % Compute derivative of the linear error and JLdot.
            if k == 2 || diff_prec == 0
                etadot(:,k) = (eta(:,k) - eta(:,k-1)) / dt;
                JLdot = (J(4:6,:,k) - J(4:6,:,k-1)) / dt;
            elseif diff_prec == 1
                etadot(:,k) = (eta(:,k) - eta(:,k-2)) / dt;
                JLdot = (J(4:6,:,k) - J(4:6,:,k-2)) / dt;
                
            end
        end
        
        % Compute ee velocity.
        a_tmp = ag(:,k) - JLdot * qdot(:,k) + Kvel * etadot(:,k) + Kpos * eta(:,k);
        ve = dt * a_tmp + J(4:6,:,k) * qdot(:,k);
        if ctrl_type == 0, xedot = wg(:,k) + Krot * rho(:,k); % only posture tracking
        elseif ctrl_type == 1, xedot = ve + 0 * eta(:,k); % only linear tracking
        elseif ctrl_type == 2, xedot = [wg(:,k); ve] + Krot * [rho(:,k); zeros(3,1)]; %full tracking CLIK2
        elseif ctrl_type == 3, xedot = [wg(:,k); vg(:,k)] + [Krot*rho(:,k); KposP*eta(:,k)]; %full tracking CLIK1
        end
        
        Jac = {Jlim, J(i1:i2,:,k)};
        XD = {rdot(:,k), xedot};
        if limitsoff, A{1}=A{1}*0; end
        [qdot(:,k+1),Qlast] = compute_qdot(NJOINTS,Jac,XD,A,REG);
        if opt_type == 1 % qdot minimization opt
            IdMinusQlast = eye(NJOINTS) - Qlast;
            topinv = Qlast' * Qlast + eta1*(IdMinusQlast' * IdMinusQlast);
            [rp,x] = reg_pinv(topinv,REGOPT1,eye(NJOINTS),eye(NJOINTS));
            temp1 = -rp * Qlast';
            diff_opt(:,k) = Jac{NTASKS} * (Qlast * temp1 * qdot(:,k+1)); % should be around zero
          %  if abs(diff_opt(:,k)) < 1e-4
                qdot(:,k+1) = qdot(:,k+1) + Qlast * temp1 * qdot(:,k+1);
          %  else, diff_opt(:,k) = 0;
          %  end
        elseif opt_type == 2 % favorite config opt
            qdotstar = gamma * (q(:,k) - QFAV);
            temp1 = (qdotstar - qdot(:,k+1));
            [rp,x] = reg_pinv(Qlast,REGOPT2,eye(NJOINTS),eye(NJOINTS));
            diff_opt(:,k) = Jac{NTASKS} * (Qlast * rp * temp1); % should be around zero
            %if abs(diff_opt(:,k)) < 1e-4
                qdot(:,k+1) = qdot(:,k+1) + Qlast * rp * temp1 ;
           % else, diff_opt(:,k) = 0;
           % end
        end
        if k == tn, break; end
        if sat, qdot(:,k+1) = saturate(qdot(:,k+1),-QDOTMAX,QDOTMAX); end
        
        %dq = dt * qdot(:,k);q(:,k+1) = q(:,k) + dq; % rectangular integration
        for s = 1: NJOINTS
           q(s,k+1) = qinit(s) + simps(t(1:k+1),qdot(s,1:k+1)); % Simpson integration
        end
        q(:,k+1) = wrapToPi(q(:,k+1) + qnoise(:,k));
        
        if isstruct(draw_robot) && ~mod(k,10) && t(k) > draw_robot.tminplot && t(k) < draw_robot.tmaxplot
            figure; 
            robot_struct.robot.teach(q(:,k+1)' + [0,pi/2,zeros(1,5)]);
        end
    end
    
    if mustplot
        plot_stuff;
        disp('~~~~~~~~~ START LOG ~~~~~~~~~');
        disp(['Max  diff between unopt and opt ',num2str(max(max(abs(diff_opt))))]);
        disp(['Mean diff between unopt and opt ',num2str(mean2(diff_opt))]);
        disp(['and the std is: ',num2str(std2(diff_opt))]);
        disp(['SVO regularization used ' num2str(x) ' times']);
        disp(['Max abs x lin error ' num2str(max(abs(eta(1,:))))]);
        disp(['Max abs y lin error ' num2str(max(abs(eta(2,:))))]);
        disp(['Max abs z lin error ' num2str(max(abs(eta(3,:))))]);
        disp(['Mean x lin error ' num2str(mean(eta(1,:)))]);
        disp(['Mean y lin error ' num2str(mean(eta(2,:)))]);
        disp(['Mean z lin error ' num2str(mean(eta(3,:)))]);
        t1 = find(t == tmax - 1);
        disp(['Mean x lin error over last sec: ' num2str(mean(eta(1,t1:end)))]);
        disp(['Mean y lin error over last sec: ' num2str(mean(eta(2,t1:end)))]);
        disp(['Mean z lin error over last sec: ' num2str(mean(eta(3,t1:end)))]);
        disp(['Final x lin error ' num2str(mean(eta(1,end)))]);
        disp(['Final y lin error ' num2str(mean(eta(2,end)))]);
        disp(['Final z lin error ' num2str(mean(eta(3,end)))]);
        disp('~~~~~~~~~ END LOG ~~~~~~~~~');
    end
    qdot = qdot(:,1:end-1);
    q = q(:,1:end-1);
end