classdef pd 
    properties
        zeta = 0.707;
        kp = 1; 
        ref = [0 0 0]'; 
        v = [0 0 0]';
        sc = spacecraft;
        er1 = [0 0 0]';
        er2 = [0 0 0]';
    end
    methods
        function self = control(self, x, s)
            self.v = self.kp*5*(0.99*self.er1 - self.error(x));
            self.er2 = self.er1;
            self.er1 = self.error(x);
        end
        function kd = kd(self)
            kd = sqrt(self.kp);
        end
        function self = setref_d(self, ref)
            self.ref = pi/180*ref;
        end
        function error = error(self, x)
            error = x(1:3) - self.ref;
        end
    end
end