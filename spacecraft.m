classdef spacecraft < handle
    properties
        theta_ = zeros(3,1);
        rot = zeros(3,1);
        Tc = zeros(3,1);
        t = 0;
    end
    properties (SetAccess = immutable)
        H = [eye(3) zeros(3)];
        Td = [0.0001 0.0001 0.0001]';
        J = [124.531 0 0; 0 124.53 0; 0 0 0.704];
        orbit = 700;
    end
    properties (Access = private)
        dt = 0.1;
        ode87 = ode87;
    end
    
    methods
        function u = u(self)
            u = [self.Td' self.Tc']';
        end
        function n = n(self)
            %TODO
            n = 1/self.orbit;
        end
        function x = x(self)
            x = [self.theta; self.rot];
        end

        function self = sim(self)
            self.step;
        end    
        function x2d = x2d(self)
            x2d = self.x * 180 / pi;
        end
        function self = step(self)
            X = self.ode87.xout(self, [0 self.dt], self.x);
            self.update_x(X);
            self.t = self.t + self.dt;
        end
        function cv = cv(self)
            cv = self.H * self.x;
        end
        function OM = OM(self)
            omega = self.rot;
            OM = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
        end
        function G = G(self)
            G = [zeros(3) zeros(3); inv(self.J) inv(self.J)];
        end
        function N = N(self)
           th = self.theta;
           N = [1 sin(th(1))*tan(th(2)) cos(th(1))*tan(th(2));
               0 cos(th(1)) -sin(th(1));
               0 sin(th(1))/cos(th(2)) cos(th(1))/cos(th(2))];
        end
        function f = f(self)
            f = [self.N*self.rot+self.ok; -inv(self.J)*self.OM() * self.J*self.rot];
        end
        function x_dot = x_dot(self)
            x_dot = self.f + self.G*self.u;
        end
        function self = update_x(self, x)
            self.theta_ = x(1:3);
            self.rot = x(4:6);
        end
        function ok = ok(self)
           th = self.theta;
           ok = self.n/cos(th(2)) * [sin(th(3)); cos(th(2))*cos(th(3)); sin(th(2))*sin(th(3))];
        end
        function x_d = x_dotx(self, x)
            sc = self;
            sc.update_x(x);
            x_d = sc.x_dot;
        end
        function theta = theta(self)
            theta = self.theta_;
        end
    end
    methods (Static = true)
        function d2r = d2r(c)
            d2r = c*(1/180.*pi);
        end
        function r2d = r2d(c)
            r2d = c * 180 / pi;
        end        
    end    
end