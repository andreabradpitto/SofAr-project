% Function that reads input configuration file of a serial link and saves
% the read parameters into an output data structure.

% Accepted format for the configuration file (see Manual):
% Row 1: num of joints (NJOINTS)
% Row 2: joint types (0 revolute, 1 prismatic)
% Rows 3 to 2+NJOINTS: DH parameters for joint i on row i, ordered as: [theta,d,a,alpha]
% Row 3+NJOINTS: DH parameters for ee, ordered as prev field
% Row 4+NJOINTS: lower joint limits
% Row 5+NJOINTS: upper joint limits
% Row 6+NJOINTS: joint maximum speed
% Row 7+NJOINTS: number of tasks to execute (NTASKS)
% Rows 8+NJOINTS to 7+NJOINTS+NTASKS: task name for task i on row i
% Row 8+NJOINTS+NTASKS: mode (simulate, real, all)
% Row 9+NJOINTS+NTASKS: robot name (optional)

% INPUTS: filename = local name of configuration file for the robot
% OUTPUT: robot_s = structure containing robot data

function robot_struct = read_robot(filename)
    %#ok<*ST2NM> % warning suppression for use of str2num instead of str2double

    fd = fopen(filename); % open file
    
    % Go to beginning of file, needed if file was not closed in previous
    % execution. Can happen in debug, don't remove...
    frewind(fd);
    
    NJOINTS = str2double(fgets(fd));
    
    % DH: matrix of the DH parameters
    % Row i --> joint i; each row will have [theta,d,a,alpha]
    DH = zeros(NJOINTS, 4);
    
    QTYPES = str2num(fgets(fd));
    for ii = 1 : NJOINTS, DH(ii,:) = str2num(fgets(fd)); end
    
    DHne = str2num(fgets(fd)); % DH parameters for ee
    
    % Obtain transformation matrix from base frame to ee frame from the ee
    % DH parameters.
    Tne = transl(DHne(3), 0, 0) * r2t(rotx(DHne(4))) ...
        * r2t(rotz(DHne(1))) * transl(0, 0, DHne(2));
    
    % Convert DH parameters into Corke DH parameters.
    c_DH = DH;
    c_DH(:,3) = [c_DH(2:NJOINTS,3); DHne(3)];
    c_DH(:,4) = [c_DH(2:NJOINTS,4); DHne(4)];
    c_DH(7,2) = .368;
    
    QMIN = str2num(fgets(fd)); % lower joint limits
    QMAX = str2num(fgets(fd)); % upper joint limits
    QDOTMAX = str2num(fgets(fd)); % joints max speed
    
    NTASKS = str2num(fgets(fd)); % number of tasks
    TASKS = cell(NTASKS,1); % will contain task names
    
    for ii = 1 : NTASKS, TASKS{ii} = strip(fgets(fd)); end
    
    MODE = strip(fgets(fd));
    NAME = strip(fgets(fd));
    
    fclose(fd);
    
    % Create serial link using Corke's library.
    robot = SerialLink(c_DH);
    robot.name = NAME;
    
    % Fill fields of output structure.
    robot_struct.DH = DH;
    robot_struct.MODE = MODE;
    robot_struct.NAME = NAME;
    robot_struct.NJOINTS = NJOINTS;
    robot_struct.QDOTMAX = QDOTMAX;
    robot_struct.QMAX = QMAX;
    robot_struct.QMIN = QMIN;
    robot_struct.QTYPES = QTYPES;
    robot_struct.robot = robot;
    robot_struct.TASKS = TASKS;
    robot_struct.Tne = Tne;
    
end