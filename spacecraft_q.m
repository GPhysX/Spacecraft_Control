classdef spacecraft_q < spacecraft
    % This class extends on the spacecraft.m class to add quaternion
    % representation and differential equations. 
    % the class method step() propagates the simulation one timestep(Ts)
    % using an ode87 integration scheme. 
    % X represents the state vector containing the four quaternions
    % (q) and three rotational velocities (rot)
    % the system can be initialized to a certain euler angle attitude using
    % set_orient_r(theta), with theta the three euler angles in radians
    properties
        q = [0 0 0 1]';
    end
    properties (SetAccess = immutable)
        H_ = [[eye(3);zeros(1,3)] zeros(4)];
    end
    methods
        function self = spacecraft_q(self)
            th0 = 30/180*pi;
            self = self.set_orient_r([th0 th0 th0]);
        end
        function q_ = q_(self)
            q_ = self.q(1:3);
        end
        function x = x(self)
            x = [self.q; self.rot];
        end
        function cv = cv(self)
            cv = self.H_ * self.x;
        end
        function G = G(self)
            G = [zeros(4,3) zeros(4,3); inv(self.J) inv(self.J)];
        end
        function OM_q = OM_q(self)
            w = self.rot + self.ok;
            OM_q = [0 w(3) -w(2) w(1); -w(3) 0 w(1) w(2); 
                w(2) -w(1) 0 w(3); -w(1) -w(2) -w(3) 0];
        end
        function f = f(self)
            f = [0.5 * self.OM_q * self.q; -inv(self.J)*self.OM() * self.J*self.rot ];
        end
        function ok = ok(self)
            C = self.C;            
            ok =  self.n*[C(1,2) C(2,2) C(3,2)]';
        end
        function self = update_x(self,x)
            self.q = x(1:4);
            self.rot = x(5:7);
            self.theta_ = self.theta;
        end
        function Q = Q(self)
            Q = [0 -self.q(3) self.q(2); self.q(3) 0 -self.q(1); -self.q(2) self.q(1) 0];
        end
        function C = C(self)
            C = (self.q(4)^2 -self.q_'*self.q_) * eye(3) + 2* self.q_ * self.q_' - 2 *self.q(4) *self.Q;
        end
        function theta = theta(self)
            %3-2-1 axis rotation
            C = self.C;
            theta = [atan2(C(2,3), C(3,3)); asin(-C(1,3)); atan2(C(1,2),C(1,1))];
        end
        function qe = qe(self, qc)
            qe = [ qc(4) qc(3) -qc(2) -qc(1); -qc(3) qc(4) qc(1) -qc(2);
                qc(2) -qc(1) qc(4) -qc(3); qc(1) qc(2) qc(3) qc(4)] * self.q;            
        end
        function self = set_orient_r(self, theta)
            self.q = [  sin(theta(1)/2)*cos(theta(2)/2)*cos(theta(3)/2) - cos(theta(1)/2)*sin(theta(2)/2)*sin(theta(3)/2);
                cos(theta(1)/2)*sin(theta(2)/2)*cos(theta(3)/2) + sin(theta(1)/2)*cos(theta(2)/2)*sin(theta(3)/2);
                cos(theta(1)/2)*cos(theta(2)/2)*sin(theta(3)/2) - sin(theta(1)/2)*sin(theta(2)/2)*cos(theta(3)/2);
                cos(theta(1)/2)*cos(theta(2)/2)*cos(theta(3)/2) + sin(theta(1)/2)*sin(theta(2)/2)*sin(theta(3)/2)];
        end
    end
    methods (Static = true)
        function title = title()
            title = 'quaternions';       
        end
    end
end
