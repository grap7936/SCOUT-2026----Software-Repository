% Code to Estimate the Trajetory of an object in camera video

%% Setup

% extract data from files
filename = "debrisLog.txt";
objects = readtable(filename);
numObjs = height(objects);
filename = "motorLog.txt";
motor = readtable(filename);
numMtrs = height(motor);

% Camera parameters
frameWidth = 2464; % pixels
frameWFOV = 8.1 * pi/180.0; % radians
radPerPx = frameWFOV / frameWidth; % radians per pixel
%%
frameRate = 79.0; % fps
dt = 1/frameRate; % s

% Values for data recorded stationary on earth's surface at a defined latitude
lat = 40; % degrees latitude
stationary_v = cosd(lat) * 465.1; % equator_v = 465.1 m/s
stationary_theta = 0;
stationary_phi = 0;
earth_r = 6376000; % meters

% manually defined as no sensor is saving data'
P = [];
for k = 1:2000
    temp = [k, (stationary_v/frameRate)*(k-1), 0, earth_r, 0, 0 ];
    P = [P; temp];
end
positions = array2table(P, 'VariableNames', ["fid", "x", "y", "z", "theta", "phi"]);
%%

% process/extract object data

pgap = objects.frame_num(1) - positions.fid(1);
mgap = 1;
objsSorted = [];
idMap = table('Size', [0, 2], 'VariableNames', "index", "id");

for i = 1:numObjs

    mFrameID = objects.frame_num + mgap;
    debrisID = objects.id(i);

    pFrameID = objects.frame_num + pgap;
    CamPos = positions(positions.frame_num == pFrameID, :);

    

    T = idMap;
    found = T(T.id == debrisID, :);
    if isempty(found)
        objs = objects(objects.id == i, :);
        idMap = [idMap; {height(idMap)+1, newObj.debrisID}]; % Update idMap with new object

         % extract and reorganize data
        [~, thetas] = motor(ismember(motor.frame_num, mFrameID));
        
        newObj = struct("frameIDs", objs.frame_num, "debrisID", debrisID, ...
                        "x", objs.x, "y", objs.y, "kx", objs.kx, "ky", objs.ky, ...
                        "scores", objs.score, "mFrameIDs", mFrameID, "thetas", thetas, ...
                        "pFrameIDs", pFrameID, "CamXYZ", [CamPos.x, CamPos.y, CamPos.z], ...
                        "CamRad", [CamPos.theta, CamPos.phi]);
        
        objsSorted = [objsSorted, newObj];

    end
end

%% Main Calculation

% Solving for debris position <x0,y0,z0> and velocity <vx,vy,vz>
% Assuming constant v, p1 = <x1,y1,z1> = <x0,y0,z0> + v*dt
%       or vice versa, p0 = <x1,y1,z1> - v*dt

trajectories = [];

for object = objsSorted

    numInstances = max(size(object.frameIDs));
    
    if numInstances < 3
        continue
    end
    
    % define vectors
    
    camPositionVecs = object.CamXYZ;
    camHeadingAngles = object.CamRad;
    
    gammaVecs = computeGammaVectors(camHeadingAngles, object.thetas, [object.x-frameWidth/2,object.y-frameHeight/2], radPerPx);
    
    %%%%%%%%
    traj = solveTrajectory(camPositionVecs, gammaVecs, dt, "const_v");
    
    trajectories = [trajectories; traj]; % Store the computed trajectory for the current object

end


%% Helper Functions
function v = computeGammaVectors(heading, motorAngles, frameCoords, radPerPx)
    
    if height(heading) ~= height(frameCoords)
        fprintf("Error: Mismatched matrix sizes for camera heading and frame data. %d | %d", height(heading), height(frameCoords));
    end

    theta = heading(1);
    phi = heading(2);

    motor_theta = motorAngles;

    frame_theta = frameCoords(1) .* radPerPx;
    frame_phi = frameCoords(2) .* radPerPx;

    theta = theta + motor_theta + frame_theta;
    phi = phi + frame_phi;


    x = sin(phi)*cos(theta);
    y = sin(phi)*sin(theta);
    z = cos(phi);

    v = [x, y, z];

end

function traj = solveTrajectory(camPositionVecs, gammaVecs, dt, model)
% camPositionVecs : N x 3  camera positions c_k (common frame)
% gammaVecs       : N x 3  bearing vectors gamma_k (same frame)
% dt              : scalar seconds per frame
% model           : "const_v"  -> p_k = p0 + v*t          (6 unknowns)
%                   "const_a"  -> p_k = p0 + v*t + a*t^2/2 (9 unknowns)
%                   (defaults to "const_v" if omitted)
%
% Returns struct traj with:
%   p0        1x3   object start position
%   v         1x3   object velocity (at t=0)
%   a         1x3   object acceleration ([0 0 0] in const_v mode)
%   positions Nx3   object position at every frame (p_k for all k)
%   times     Nx1   elapsed time t_k for each frame
%   condA     scalar  condition number (watch for weak geometry)
%   resid     scalar  norm(A*u - b), the fit residual
%   model     string  which model was used

    if nargin < 4 || isempty(model)
        model = "const_v";
    end
    accel = (model == "const_a");     % logical flag
    ncols = 6 + 3*accel;              % 6 for const_v, 9 for const_a

    N = size(gammaVecs,1);
    A = zeros(3*N, ncols);
    b = zeros(3*N, 1);

    for k = 1:N
        g = gammaVecs(k,:).';  g = g/norm(g);   % unit bearing
        c = camPositionVecs(k,:).';             % camera position
        t = (k-1)*dt;                           % elapsed time

        S = [   0   -g(3)  g(2);
              g(3)    0   -g(1);
             -g(2)  g(1)    0  ];

        rows = (3*k-2):(3*k);
        A(rows,1:3) = S;                 % p0 columns  (both models)
        A(rows,4:6) = t*S;               % v  columns  (both models)
        if accel
            A(rows,7:9) = 0.5*t^2*S;     % a  columns  (const_a only)
        end
        b(rows) = S*c;
    end

    u  = A\b;                   % least-squares solve
    p0 = u(1:3).';
    v  = u(4:6).';
    if accel
        a = u(7:9).';
    else
        a = [0 0 0];            % no acceleration term in this model
    end

    % --- list every frame's position ---
    times = ((1:N).' - 1) * dt;                      % N x 1
    positions = p0 + times*v + 0.5*(times.^2)*a;     % N x 3
    % (a is [0 0 0] in const_v, so the last term vanishes automatically)

    traj.p0        = p0;
    traj.v         = v;
    traj.a         = a;
    traj.positions = positions;
    traj.times     = times;
    traj.condA     = cond(A);
    traj.resid     = norm(A*u - b);
    traj.model     = model;
end