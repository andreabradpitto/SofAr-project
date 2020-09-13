% Function that computes trajectory velocity & acceleration using simple
% Euler differentation.
% INPUTS:
% xg = goal position;
% dt = simulation timestep;
% tn = number of simulation steps.
%
% OUTPTUTS:
% vg = linear goal velocity;
% ag = linear goal acceleration.

function [vg,ag] = get_lin_vels_and_accs(xg,dt,tn)
    vx=diff(xg(1,:))/dt;
    vy=diff(xg(2,:))/dt;
    vz=diff(xg(3,:))/dt; 
    vg=[vx;vy;vz];vg(:,tn)=zeros(3,1); % linear vel

    ax=diff(vg(1,:))/dt;
    ay=diff(vg(2,:))/dt;
    az=diff(vg(3,:))/dt; 
    ag=[ax;ay;az];ag(:,tn)=zeros(3,1); % linear acc
end

% Author: Luca Tarasi