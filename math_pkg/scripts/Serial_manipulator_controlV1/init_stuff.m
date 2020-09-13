% Function that initializes variables.
% INPUTS: n = num of joints, tn = num of time steps, sn = num of
% simulations to be run.
% OUTPUTS: as the names say..

function [eta,rho,q,qdot,J,Jlims,rdot] = init_stuff(n,tn)
    eta = zeros(3,tn);rho = zeros(3,tn);q = zeros(n,tn);
    qdot = zeros(n,tn);
    J = zeros(6,n,tn);Jlims = zeros(n,n,tn);
    rdot = zeros(n,tn);
    q(:,1,:) = zeros(n,1);
end