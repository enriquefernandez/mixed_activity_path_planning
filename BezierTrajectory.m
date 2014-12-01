classdef BezierTrajectory
    properties
        P = [0 0; 2 2; 4 2; 6 1; 8 2]; % n+1 x m matrix of n+1 control points of dimension m.       
    end
    methods
        function obj = BezierTrajectory(P)
            if  nargin > 0
                obj.P = P;
            end
        end
        function traj = evaluate(obj, t)
            n = size(obj.P,1) - 1;
            num_p = length(t);
%             traj = zeros(num_p, m);
            B = zeros(num_p, n+1);
            for i=0:n                
                Bi = nchoosek(n, i) * (1-t).^(n-i) .* t.^i;
                B(:, i+1) = Bi';                
            end
            traj = B * obj.P;
        end
        
        function plot(obj)
            t = 0:0.01:1;            
            traj = obj.evaluate(t);
            figure(1); hold on;
            plot(obj.P(:,1), obj.P(:,2), '+-r')
            plot(traj(:,1), traj(:,2), 'b')
        end
        
        function traj = getDerivative(obj, d, t)
            n = size(obj.P,1) - 1;
            m = size(obj.P, 2);
            
            if d==0
                traj = obj.evaluate(t);
            else
%                 Pd = zeros(n,m);
                Pd = [];
                for i=1:n
%                     Pd(i,:) = n * (obj.P(i+1,:) - obj.P(i,:));
                    Pd = [Pd; n * (obj.P(i+1,:) - obj.P(i,:))];
                end
                bezd = BezierTrajectory(Pd);
                traj = bezd.getDerivative(d-1, t);
            end
        end
        
        function y = curvature(obj, t)
            xd = obj.getDerivative(1,t);
            xdd = obj.getDerivative(2,t);
            
            y = (xd(:,1) .* xdd(:, 2) - xd(:, 2) .* xdd(:, 1)) ./ (sum(xd .* xd, 2).^(3/2));
        end
        
    end
    
    
end
