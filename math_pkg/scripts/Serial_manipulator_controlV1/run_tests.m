close all;
clear;
execute_test = [0 1 0 0 0 0 0 0];
plotqdot = false;
plotcurves = true;
 %#ok<*NASGU>
 %#ok<*PREALL>

Baxter = read_robot('baxter.txt');
%f = @(q)wrapToPi(deg2rad(q)); % convert from deg to rad & wrap to pi
%qinit = [0,f(-31),0,f(43),0,f(72),0]';
params.sat = 1;params.reg1 = .0001;params.reg2 = .0001;
params.corrpole_joint = 70; params.corrpole_vel = 1;
params.eta = 5; params.KposP = 10; params.KrotP = 10;
params.stay_still_thr = 5* 1e-4; % seems reasonable for Baxter
n = Baxter.NJOINTS;

% Radius and height of ee when arm is stretched.
r = 1.176;
Z = .1910;

% Test 1 % valley
qinit = zeros(n,1);nmv = false;
params.diff_prec = 0;
ROg0 = roty(pi/2); plotq = false;
dt = 0.01; tmax = 20; t = 0:dt:tmax;tn = length(t);
xg = [r*cos(0.1*t);r*sin(0.1*t);Z + 0*t]; % target position
tnhalf = floor(tn/2);
xg(:,tnhalf+3:end) = flip(xg(:,1:tn-tnhalf-3+1)')'; % at t = 10 non differentiable point
wg = [0.1+0*t; 0*t; 0*t]; % target angular velocity
dont_have_good_omega = true;draw_robot = 0;
exec_opt = [1 0 0 0];
nolims = false;plotq = false;
disp(params);
noise = zeros(n,tn);
if execute_test(1), baxter_control_test; end
clear draw_robot;

% Test 2 % XY circle
qinit = zeros(Baxter.NJOINTS,1);nmv = false;
params.diff_prec = 0;
ROg0 = roty(pi/2); plotq = false;
dt = 0.01; tmax = 8; t = 0:dt:tmax;tn = length(t);
xg = [r*cos(0.1*t);r*sin(0.1*t);Z + 0*t]; % target position
wg = [0*t; 0*t; 0*t]; % target angular velocity
dont_have_good_omega = true;draw_robot = 0;
nolims = false;plotq = true;
disp(params);
exec_opt = [1 1 1 1];
noise = zeros(n,tn);
%noise = randn(n,tn) * 10^(-5) / 3; % average 0, stdev = (max err) / 3
if execute_test(2), baxter_control_test; end
clear draw_robot;

% Test 3 % XZ circle
%params.corrpole = 5; params.eta = 5; params.KposP = 3; params.KrotP = 3;
qinit = zeros(Baxter.NJOINTS,1);nmv = false;
params.diff_prec = 0;
ROg0 = roty(pi/2); plotq = false;
dt = 0.01; tmax = 8; t = 0:dt:tmax;tn = length(t);
xg = [r*cos(0.1*t);0.2+0*t;r*sin(0.1*t)]; % target position
wg = [0*t; 0*t; 0*t]; % target angular velocity
dont_have_good_omega = true;draw_robot = 0;
nolims = false;
exec_opt = [1 0 0 0];
plotq=true;
disp(params);
noise = zeros(n,tn);
if execute_test(3), baxter_control_test; end
clear draw_robot;

% Test 4 % straight lines between 2 points
qinit = zeros(Baxter.NJOINTS,1);nmv = false;
params.diff_prec = 0;
ROg0 = roty(pi/2); plotq = false;
dt = 0.01; tmax = 20; t = 0:dt:tmax;tn = length(t);
coeffX = polyfit([0, tmax], [r, 0], 1);aX = coeffX(1);bX = coeffX(2);
coeffY = polyfit([0, tmax], [0, r], 1);aY = coeffY(1);bY = coeffY(2);
xg = [aX*t + bX;aY*t + bY;0*t]; % target position
wg = [0*t; 0*t; 0*t]; % target angular velocity
tmax = 15;last_t_idx = find(t == tmax);
t = t(1,1:last_t_idx); tn = length(t);
xg = xg(:,1:last_t_idx);
wg = wg(:,1:last_t_idx);
dont_have_good_omega = true;%draw_robot.tminplot = 3;draw_robot.tmaxplot = 3.6;
draw_robot = 0;
nolims = false;
exec_opt = [1 0 0 0 ];
disp(params);
plotq=true;
noise = zeros(n,tn);
if execute_test(4), baxter_control_test; end % after 3 sec cannot follow with joint limits
%params.diff_prec = 1;
%if execute_test(5), baxter_control_test; end % after 3 sec cannot follow with joint limits
clear draw_robot;

% Test 6 % Constant target that is not the initial pose
qinit = zeros(Baxter.NJOINTS,1);nmv = true;
params=params;
params.diff_prec = 0; plotq = true;
params.eta =5;
ROg0 = rpy2r(-116.227,5.898,-125.858,'deg','xyz');
dt = 0.01; tmax = 4; t = 0:dt:tmax;tn = length(t);
xg = [0.672+0*t;0.306+0*t;-0.329 + 0*t]; % target position
wg = [0*t; 0*t; 0*t]; % target angular velocity
dont_have_good_omega = false;nolims = false;exec_opt = [1 1 1 0];draw_robot = 0;
disp(params);
noise = zeros(n,tn);
%noise = randn(n,tn) * 10^(-3) / 3; % average 0, stdev = (max err) / 3
if execute_test(6), baxter_control_test; end % after 3 sec cannot follow with joint limits
clear draw_robot;
% In 3-1, eta2: initial chattering due to joint limits. See pics folder
params.eta = 5; % restore

% Test 7 % Target = initial pose with initial q4 close to lower limit
qinit = zeros(Baxter.NJOINTS,1); nmv = true;
params.diff_prec = 0;
ROg0 = roty(pi/2); plotq = true;
dt = 0.01; tmax = 4; t = 0:dt:tmax;tn = length(t);
xg = [r*cos(0.1*0*t);r*sin(0.1*0*t);Z + 0*t]; % target position
wg = [0*t; 0*t; 0*t]; % target angular velocity
dont_have_good_omega = false;nolims = false;exec_opt = [1 1 0 1];draw_robot = 0;
disp(params);
noise = zeros(n,tn);
if execute_test(7), baxter_control_test; end
clear draw_robot;

% Test 8 % Target = initial pose with all q far from limits
qinit = zeros(Baxter.NJOINTS,1);nmv = true;
qinit(4) = 0; % pi/2 safe, 0 is close to unsafe
params.diff_prec = 0;
ROg0 = rpy2r(-180,0,180,'deg','xyz'); plotq = true;
dt = 0.01; tmax = 4; t = 0:dt:tmax;tn = length(t);
xg = [0.4230 + 0*t;0*t;-0.541 + 0*t]; % target position
wg = [0*t; 0*t; 0*t]; % target angular velocity
dont_have_good_omega = false;nolims = false;exec_opt = [1 1 1 1];draw_robot = 0;
disp(params);
noise = zeros(n,tn);
if execute_test(8), baxter_control_test; end % after 3 sec cannot follow with joint limits