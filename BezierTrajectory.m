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
            figure(2); hold on;
            plot(obj.P(:,1), obj.P(:,2), '+r', 'MarkerSize', 20)
            plot(traj(:,1), traj(:,2), 'b')
            % Convex hull plot
            hull = convhull(obj.P);
            plot(obj.P(hull,1), obj.P(hull,2),'--r');
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
        
        function C = polynomialCoefficients(obj)
            n = size(obj.P,1) - 1;
            C = [];
            for j=0:n
                Cj = 0;
                for i=0:j
                    Cj = Cj + ( (-1)^(i+j) * obj.P(i+1, :)') / (factorial(i) * factorial(j-i));
                end
                Cj = Cj * factorial(n)/factorial(n-j);
                C = [Cj C];
            end            
        end
        
        function I = snapIntegral(obj)
            
            function p2 = squarePoly(p)
                if length(p)==1
                    p2 = p * p;
               elseif length(p)==2
                   p2 = [p(1)*p(1), 2*p(1)*p(2), p(2)*p(2)];
               else
                   error('Snap supported for n=5 max');
               end
            end
            
           I = 0;
           p = obj.polynomialCoefficients();
           px = p(1,:);
           py = p(2,:);
           
           % Calc fourth derivative (snap)
           for i=1:4
               px = polyder(px);
               py = polyder(py);
           end
           
           % Square norm of snap
           
%            s2_x = conv(px, px);
%            s2_y = conv(py, py);
            s2_x = squarePoly(px);
            s2_y = squarePoly(py);
           
           % Integral in [0,1]
           I = I + sum(polyint(s2_x));
           I = I + sum(polyint(s2_y));
           
        end
        
    end
    
    
end
