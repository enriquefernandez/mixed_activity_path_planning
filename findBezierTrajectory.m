function [bezierPart]= findLinearTrajectory(safe_regions_seq, start, goal, vmax)

% Start and end can either be 2x1 vectors (contained in the last and end
% safe regions), or Polyhedrons (areas that need to be reached).
% safe_regions_seq ordered list of intersecting regions from start to goal

% 
% n_b_bounds = 4;
% n_b_inner = 5;

n_b_bounds = 4;
n_b_inner = 4;


P = {}

num_curves = length(safe_regions_seq);

% Create optimization variables (control points)
for i=1:num_curves
   if i==1 || i==num_curves
    num_i_cpoints = n_b_bounds + 1;
   else
       num_i_cpoints = n_b_inner + 1;
   end
   for j=1:num_i_cpoints
        P{i}{j} = sdpvar(1,2);
   end
end

% Impose constraints
constraints = [];

% Start and end conditions
constraints = addBoundaryConstraint(constraints, P{1}{1}, start');
constraints = addBoundaryConstraint(constraints, P{num_curves}{n_b_bounds+1}, goal');

% Impose continuity constraints
if num_curves > 1
    for i=2:num_curves
        if i==2
            n_previ = n_b_bounds;
        else
            n_previ = n_b_inner;
        end
        if i==num_curves
            n_i = n_b_bounds;
        else
            n_i = n_b_inner;
        end
        % C2 between i-1 and i
        constraints = [constraints P{i-1}{n_previ+1} == P{i}{1}];
        constraints = [constraints n_previ * (P{i-1}{n_previ+1} - P{i-1}{n_previ}) == n_i * (P{i}{2} - P{i}{1})];
        constraints = [constraints n_previ * (n_previ -1) * (P{i-1}{n_previ+1} - 2 * P{i-1}{n_previ} + P{i-1}{n_previ-1}) == n_i * (n_i-1) * (P{i}{3} - 2 * P{i}{2} + P{i}{1})];
    end
    
else
   error('Not sure this would work with only one safe region...'); 
end


% Control points inside assigned safe regions
for i=1:num_curves
    if i==1 || i==num_curves
        num_i_cpoints = n_b_bounds + 1;
    else
       num_i_cpoints = n_b_inner + 1;
    end
    for j=1:num_i_cpoints
       constraints = [constraints safe_regions_seq(i).A * P{i}{j}' <= safe_regions_seq(i).b];
    end
end


%% Minimum snap objective
obj = 0;
P_joint = {};
for i = 1:length(P)
    Pi_obj = [];
    for j=1:length(P{i})
        Pi_obj = [Pi_obj;  P{i}{j}];    
    end
    P_joint{i} = Pi_obj;
    bb = BezierTrajectory(Pi_obj);
    obj = obj + bb.snapIntegral();
end

%% Arc length objective
% http://pomax.github.io/bezierinfo/#arclength
[x, w] = lgwt(20,0,1)
obj = 20 * obj;
% obj = 0;
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

%% Solve

options = sdpsettings('verbose',1);
% options = sdpsettings('verbose',1, 'solver','snopt');
% options = sdpsettings('verbose',1, 'solver','gurobi');
% options = sdpsettings('verbose',1, 'solver','mosek');
% obj = -c0 - cf;
sol = optimize(constraints, obj, options);




%% Convert solution and plot
% figure(1);
Psol = {}
hold on;
for i=1:length(P)
    li = length(P{i});
    Psol{i} = zeros(li,2);
    for j=1:li
        Psol{i}(j,:) = value(P{i}{j});
    end
    % Plot
%     plot(Psol{i}(:,1), Psol{i}(:,2), '+r')
%     bez = BezierTrajectory(Psol{i});
%     t=0:0.0001:1;
%     bez_t = bez.evaluate(t);
%     plot(bez_t(:,1), bez_t(:,2),'m')
end

%% Curvature
bezierPart = BezierPartTrajectory(Psol);
% figure(2);hold on; grid on; title('Curvature');
% plot(t, bezierPart.curvature(t))

%% Aux function
function C = addBoundaryConstraint(C, opt_var, bound_condition)
        if isa(bound_condition, 'Polyhedron')
            C = [C, bound_condition.A * opt_var' <= bound_condition.b];
        elseif isa(bound_condition, 'double')
            C = [C, opt_var == bound_condition];
        else
            error('Start and end can only be either arrays or Polyhedrons');
        end
end

end