% Try to fit a BezierPartTrajectory when specifiying start, end (with
% orientations), safe regions and maximum curvature

%% Setup
figure(1);
clf; hold on; grid on;

% Configuration
arrow_length=0.5;
start = [1;2];
start_ori = -3*pi/6;

% goal = [5;11];
% goal = [12;2];
goal = [11;2];
goal_ori = -pi/2-pi/4; %pi/3;

V{1} = [0 0; 0 3; 7 -1; 7 -4];
% V{2} = [4 -1; 6 12; 10 12; 9 -2];
V{2} = [6.1 -1; 6 12; 10 12; 9 -2];
V{3} = [4 11; 5 12; 13 4; 12 0];

% Generate safe regions, and plot stuff
for i=1:length(V)
    Ps{i} = Polyhedron(V{i});
    hull_idx = convhull(V{i});
    plot(V{i}(hull_idx, 1), V{i}(hull_idx, 2), '--b');
end

% Plot start, goal and orientations
plot(start(1), start(2), 'or', 'MarkerSize', 15)
plot(goal(1), goal(2), 'og', 'MarkerSize', 15)
start_vec = start + arrow_length * [cos(start_ori);sin(start_ori)];
goal_vec = goal + arrow_length * [cos(goal_ori);sin(goal_ori)];
plot([start(1) start_vec(1)], [start(2) start_vec(2)], 'k')
plot([goal(1) goal_vec(1)], [goal(2) goal_vec(2)], 'k')


%% Fit Bezier Curve
constraints = [];
% Curve 1 degree 4, 5 control points
P{1}{1} = start';



    % Start orientation
    c0 = sdpvar(1,1);
    constraints = [constraints c0 > 1e-3];
    P{1}{2} = P{1}{1} + c0 * [cos(start_ori) sin(start_ori)];

for i=3:5
    P{1}{i} = sdpvar(1,2);
end

% Curve 2 deg 5, 6 control points
for i=1:6
    P{2}{i} = sdpvar(1,2);
end
    % C2 continuity: curve 1 with curve 2
    constraints = [constraints P{1}{5} == P{2}{1}];
    constraints = [constraints 4 * (P{1}{5} - P{1}{4}) == 5 * (P{2}{2} - P{2}{1})];
    constraints = [constraints 4 * 3 * (P{1}{5} - 2 * P{1}{4} + P{1}{3}) == 5 * 4 * (P{2}{3} - 2 * P{2}{2} + P{2}{1})];

% Curve 3 deg 4, 5 control points
P{3}{5} = goal';
for i=1:3
    P{3}{i} = sdpvar(1,2);
end

    % C2 continuity: curve 2 with curve 3
    constraints = [constraints P{2}{6} == P{3}{1}];
    constraints = [constraints 5 * (P{2}{6} - P{2}{5}) == 4 * (P{3}{2} - P{3}{1})];
    constraints = [constraints 5 * 4 * (P{2}{6} - 2 * P{2}{5} + P{2}{4}) == 4 * 3 * (P{3}{3} - 2 * P{3}{2} + P{3}{1})];
    
    % End orientation
    cf = sdpvar(1,1);
    constraints = [constraints cf > 1e-3];
    P{3}{4} = P{3}{5} - cf * [cos(goal_ori) sin(goal_ori)];

 
 % Control points in their safe regions
 
    % Curve 1
    for i=2:5        
        constraints = [constraints Ps{1}.A * P{1}{i}' <= Ps{1}.b];
    end
    % Curve 2
    for i=1:6        
        constraints = [constraints Ps{2}.A * P{2}{i}' <= Ps{2}.b];
    end
    % Curve 3
    for i=1:4        
        constraints = [constraints Ps{3}.A * P{3}{i}' <= Ps{3}.b];
    end



    
    
%% Arc length objective
% http://pomax.github.io/bezierinfo/#arclength
[x, w] = lgwt(20,0,1)
obj = 0;
P_joint = {}
for i = 1:3
    Pi_obj = [];
    for j=1:length(P{i})
        Pi_obj = [Pi_obj;  P{i}{j}];    
    end
    P_joint{i} = Pi_obj;
    bb = BezierTrajectory(Pi_obj);
    deriv = bb.getDerivative(1, x);
    obj = obj +  w' * sum(deriv .* deriv, 2); % Aprox method, quadratic
%     obj = obj +  w' * sqrtm(sum(deriv .* deriv, 2)); % 'Exact', nonlinear
end

%% Mixed curvature - arc length objective

[x, w] = lgwt(20,0,1);
k_arclength = 1;
k_curvature = 0.001;
obj = 0;
P_joint = {};
for i = 1:3
    Pi_obj = [];
    for j=1:length(P{i})
        Pi_obj = [Pi_obj;  P{i}{j}];    
    end
    P_joint{i} = Pi_obj;
    bb = BezierTrajectory(Pi_obj);
    deriv = bb.getDerivative(1, x);
    obj = obj +  k_arclength * w' * sum(deriv .* deriv, 2); % Aprox method, quadratic
%     obj = obj +  w' * sqrtm(sum(deriv .* deriv, 2)); % 'Exact', nonlinear
    xd = bb.getDerivative(1, x);
    xdd = bb.getDerivative(2, x);
    curv_prop = xd(:,1) .* xdd(:, 2) - xd(:, 2) .* xdd(:, 1);
%     obj = obj + k_curvature * w' * (curv_prop .* curv_prop);
    obj = obj + k_curvature * w' * (curv_prop);

%     curv_prop = xd(:,1) .* xdd(:, 2) - xd(:, 2) .* xdd(:, 1);
end

%% Minimum snap objective


obj = 0;
P_joint = {};
for i = 1:3
    Pi_obj = [];
    for j=1:length(P{i})
        Pi_obj = [Pi_obj;  P{i}{j}];    
    end
    P_joint{i} = Pi_obj;
    bb = BezierTrajectory(Pi_obj);
    obj = obj + bb.snapIntegral();
end

%% Curvature objective
% obj = 0;
% P_joint = {}
% t_curv=0:0.05:1;
% for i = 1:3
%     Pi_obj = [];
%     for j=1:length(P{i})
%         Pi_obj = [Pi_obj;  P{i}{j}];    
%     end
%     P_joint{i} = Pi_obj;
%     bb = BezierTrajectory(Pi_obj);
%     xd = bb.getDerivative(1, t_curv);
%     xdd = bb.getDerivative(2, t_curv);
%     curv_prop = xd(:,1) .* xdd(:, 2) - xd(:, 2) .* xdd(:, 1);
%     obj = obj +  curv_prop' * curv_prop;
% end
%% Curvature constraint
% bez_part = BezierPartTrajectory(P_joint);
% t_curv=0:0.15:1;
% curv = bez_part.curvature(t_curv);
% constraints = [constraints curv < 2];
% obj = sum(curv);

P_joint = {}
t_curv=0:0.1:1;
% t_curv = [0 1];
for i = 1:3
    Pi_obj = [];
    for j=1:length(P{i})
        Pi_obj = [Pi_obj;  P{i}{j}];    
    end
    P_joint{i} = Pi_obj;
    bb = BezierTrajectory(Pi_obj);
    xd = bb.getDerivative(1, t_curv);
    xdd = bb.getDerivative(2, t_curv);
    curv_prop = xd(:,1) .* xdd(:, 2) - xd(:, 2) .* xdd(:, 1);
    curv_den = (sum(xd .* xd,2)) .^ (3/2);
    
    constraints = [constraints curv_prop < 10 * curv_den];  
    constraints = [constraints curv_prop > -10 * curv_den];
end

%% Solve

options = sdpsettings('verbose',1);
% options = sdpsettings('verbose',1, 'solver','snopt');
% options = sdpsettings('verbose',1, 'solver','gurobi');
% options = sdpsettings('verbose',1, 'solver','mosek');
% obj = -c0 - cf;
sol = optimize(constraints, obj, options)

%% Convert solution and plot
Psol = {}
hold on;
for i=1:3
    li = length(P{i});
    Psol{i} = zeros(li,2);
    for j=1:li
        Psol{i}(j,:) = value(P{i}{j});
    end
    % Plot
    plot(Psol{i}(:,1), Psol{i}(:,2), '+r')
    bez = BezierTrajectory(Psol{i});
    t=0:0.0001:1;
    bez_t = bez.evaluate(t);
    plot(bez_t(:,1), bez_t(:,2),'m')
end

%% Curvature
bezierPart = BezierPartTrajectory(Psol);
figure(2);hold on; grid on; title('Curvature');
plot(t, bezierPart.curvature(t))

