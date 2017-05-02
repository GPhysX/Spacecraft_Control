classdef spacecraft < handle
    % This class provides an attitude simulation of a rigid satelite in a
    % circular orbit, subject to a disturbance and a control torque. The attitude representation is done in euler angles
    % using 3-2-1 rotations. 
    % the class method step() propagates the simulation one timestep(Ts)
    % using an ode87 integration scheme. 
    % X represents the state vector containing the three attitude angles
    % (theta) and three rotational velocities (rot)
    properties
        theta_ = 30/180*pi*ones(3,1);
        rot = zeros(3,1);
        Tc = zeros(3,1);
        t = 0;
        Ts = 0.1;
        Td = [0.0001 0.0001 0.0001]';
        J = [124.531 0 0; 0 124.586 0; 0 0 0.704];
        H = [eye(3) zeros(3)];
        orbit = 700;
        orbit_period = 5917.46;
    end
    properties (Access = private)
        ode87 = ode87(1.e-6);
    end
    methods
        function self = step(self)
            X = self.ode87.xout(self, [0 self.Ts]);
            self.update_x(X);
            self.t = self.t + self.Ts;
        end
        function x_dot = x_dot(self)
            %returns the derivative of x in the current state
            x_dot = self.f + self.G*self.u;
        end
        function G = G(self)
            G = [zeros(3) zeros(3); inv(self.J) inv(self.J)];
        end
        function f = f(self)
            f = [self.N*self.rot+self.ok; -inv(self.J) * self.OM() * self.J*self.rot];
        end
        function OM = OM(self)
            omega = self.rot;
            OM = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
        end
        function N = N(self)
            % Matrix describing the relation between theta (orbital frame) and omega(body frame).
            % theta_dot = N * omega
           th = self.theta;
           N = [1 sin(th(1))*tan(th(2)) cos(th(1))*tan(th(2));
               0 cos(th(1)) -sin(th(1));
               0 sin(th(1))/cos(th(2)) cos(th(1))/cos(th(2))];
        end
        function u = u(self)
            u = [self.Td' self.Tc']';
        end
        function n = n(self)
            %Orbital rate
            n = 2*pi/self.orbit_period;
        end
        function x = x(self)
            x = [self.theta; self.rot];
        end
        function y = x_(self,i)
            x = self.x;
            y = x(i);
        end
        function self = simit(self)
            self.step;
        end    
        function x2d = x2d(self)
            x2d = self.x * 180 / pi;
        end
        function cv = cv(self)
            cv = self.H * self.x;
        end
        function N_d = N_d(self)
            th = self.theta;
            f = self.f;
            theta_d = f(1:3);
            i1 = cos(th(1))*tan(th(2))*theta_d(1)+ sin(th(1))*theta_d(2)/(cos(th(2))^2);
            i2 =  -sin(th(1))*tan(th(2))*theta_d(1)+cos(th(1))*theta_d(2)/(cos(th(2))^2);
            i3 = (cos(th(1))*cos(th(2))*theta_d(1)+sin(th(1))*sin(th(2))*theta_d(2))/(cos(th(2))^2);
            i4 = (-sin(th(1))*cos(th(2))*theta_d(1)+cos(th(1))*sin(th(2))*theta_d(2))/(cos(th(2))^2);
            N_d = [0 i1 i2;
                    0 -sin(th(1))*theta_d(1) -cos(th(1))*theta_d(1);
                    0  i3 i4];
        end
        function self = update_x(self, x)
            self.theta_ = mod(x(1:3)+pi,2*pi)-pi;     % normalized to angles between -pi,pi
            self.rot = x(4:6);
        end
        function ok = ok(self)
           th = self.theta;
           ok = self.n/cos(th(2)) * [sin(th(3)); cos(th(2))*cos(th(3)); sin(th(2))*sin(th(3))];
        end
        function x_d = x_dotx(self, x)  
            %returns the derivative of x given a certain statevector
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
        function title = title()
            title = 'euler_angles';
        end
    end    
end