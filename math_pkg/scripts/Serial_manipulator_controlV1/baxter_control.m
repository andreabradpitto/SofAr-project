Baxter = read_robot('baxter.txt');

dt = 0.01; tmax = 20; t = 0:dt:tmax;
tn = length(t);
r = 1.17594;
Z = .194;

xg = 0.1*[r*exp(-0.1*t);r*exp(-0.1*t);Z + 0*t]; % target position
xg = 0.1*[r*exp(-0.1*t);r*exp(-0.1*t);Z + 0.1*t.*sin(t)];
xg = xg * 0 + [0.4*sin(0.1*t);0.5 + 0*t;0.3 + 0*t];

wg = [0.1+0*t; 0*t; 0*t]; % target angular velocity
% See Baxter PDF, page 12.
f = @(q)wrapToPi(deg2rad(q)); % convert from deg to rad & wrap to pi
qinit = [0,f(-31),0,f(43),0,f(72),0]';

for eta = 5:5:5
    for sat = 1:1
        clear joint_constr;
        close all;
        out = robot_control(Baxter, dt, tmax, xg, wg, zeros(1,7), 0);
        ;
    end
end