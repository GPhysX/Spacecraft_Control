classdef spacecraft
    properties
        theta = zeros(3,1);
        rot = zeros(3,1);
        Tc = zeros(3,1);
    end
    properties (SetAccess = immutable)
        H = [eye(3) zeros(3)];
        Td = [0.0001 0.0001 0.0001]';
        J = [124.531 0 0; 0 124.53 0; 0 0 0.704];
        orbit = 700;
    end
    properties (Dependent)
        x                                   % state vector
        u                                   % combined input vector
        cv                                  % control variables 
        OM                                  % Omega
        G                                   
        N
        f
        n                                   % orbital period
        ok                                  % orbital kinematics
        od                                  % orbital dynamics
        x_dot                               % state derivative
    end
    
    methods
        function x = get.x(self)
            x = [self.theta; self.rot];
        end
        function u = get.u(self)
            u = [self.Td' self.Tc']';
        end
        function cv = get.cv(self)
            cv = self.H * self.x;
        end
        function OM = get.OM(self)
            omega = self.rot;
            OM = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
        end
        function G = get.G(self)
            G = [zeros(3) zeros(3); inv(self.J) inv(self.J)];
        end
        function N = get.N(self)
           t = self.theta;
           N = [1 sin(t(1))*tan(t(2)) cos(t(1))*tan(t(2));
               0 cos(t(1)) -sin(t(1));
               0 sin(t(1))/cos(t(2)) cos(t(1))/cos(t(2))];
        end
        function f = get.f(self)
            f = [self.N*self.rot+self.ok; -inv(self.J)*self.OM * self.J*self.rot + self.od];
        end
        function n = get.n(self)
            n = 1/self.orbit;
        end
        function ok = get.ok(self)
           th = self.theta;
           ok = self.n/cos(th(2)) * [ sin(th(3)); cos(th(2))*cos(th(3)); sin(th(2))*sin(th(3))];
        end
        function od = get.od(self)
            th = self.theta;
            od = 3*self.n^2 * [ 0 -cos(th(1))*cos(th(2)) sin(th(1))*cos(th(2)); 
                cos(th(1))*cos(th(2)) 0 sin(th(2));
                sin(th(1))*cos(th(2)) -sin(th(2)) 0] * self.J * [
                -sin(th(2)) sin(th(1))*cos(th(2)) cos(th(1))*cos(th(2))]';
        end 
        function x_dot = get.x_dot(self)
            x_dot = self.f + self.G*self.u;
        end        
        function self = update_x(self, x)
            self.theta = x(1:3);
            self.rot = x(4:6);
        end
    end
end