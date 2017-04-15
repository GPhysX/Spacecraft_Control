classdef pd 
    properties
        zeta = 0.707;
        kp = 1; 
        ref = [0 0 0]'; 
        v = [0 0 0]';
        sc = spacecraft;
        er = [0 0 0]';
    end
    methods
        function self = control(self, x)
%             self.v = - self.kp .* self.error(x) - self.kd .* x(end-2:end);
%             self.v = - self.kp .* self.error(x) - self.kd .* (self.error(x)-self.er)/self.sc.Ts;
%             self.v = -10* (self.er - 0.995*self.error(x));
            self.v = self.kp*5*(0.98*self.er - self.error(x))+0.15*self.v;
%             self.v = self.er - 0.99*self.error(x));
            self.er = self.error(x);
        end
        function kd = kd(self)
            kd = 4*self.kp;
        end
        function self = setref_d(self, ref)
            self.ref = pi/180*ref;
        end
        function error = error(self, x)
            error = x(1:3) - self.ref;
        end
    end
end