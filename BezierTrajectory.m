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
        
        
    end
    
    
end
