% Function that computes qdot according to iterative algorithm.
% INPUTS:
% NJOINTS = num of joints;
% J = cell array of Jacobian matrices sorted by decreasing priority;
% XD = cell array of xdot vectors, sorted according to J;
% REG = array of regularization thresholds, sorted according to J.

function [qdot,QLAST] = compute_qdot(NJOINTS,J,XD,A,REG)
    NTASKS = length(J);

    % No actual need to store all iterations of the algorithm, it is just
    % for debug / inspection purposes.
    W = cell(NTASKS,1);
    Q = cell(NTASKS+1,1);
    qdot_tmp = cell(NTASKS+1,1);
    
    % Initial values.
    qdot_tmp{1}=zeros(NJOINTS,1);
    Q{1}=eye(NJOINTS);
    
    for ii = 1:NTASKS
        L = size(Q{ii},1);
        
        pinv_aux = reg_pinv(J{ii}*Q{ii},REG(ii),A{ii},Q{ii});
        pinvJQ = reg_pinv(J{ii}*Q{ii},REG(ii),A{ii},Q{1});
        
        % Update matrices W, Q.
        W{ii} = J{ii} * Q{ii} * pinv_aux;
        Q{ii+1} = Q{ii} * (eye(L) - pinvJQ * J{ii} * Q{ii});
        
        % Update joint velocities.
        qdot_tmp{ii+1} = qdot_tmp{ii} + Q{ii} * pinvJQ * W{ii} * ...
            (XD{ii} - J{ii}*qdot_tmp{ii});
    end
    qdot = qdot_tmp{NTASKS + 1}; % final joint velocity vector
    QLAST = Q{3};
end

% Author: Luca Tarasi.