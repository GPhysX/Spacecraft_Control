classdef fracc 
    properties
        zeta = 0.707;
        kp = 1; 
        ref = [0 0 0]'; 
        v = [0 0 0]';
        sc = spacecraft;
        er1 = zeros(3,5);
        v1 = zeros(3,5);
        Ts = 0.1;
    end
    methods
        function self = control(self, x, s)
            self.v = - [self.error(x) self.er1(:,1:3)] * [8.9652  -25.9300   24.9746   -8.0097]' - self.v1(:,1:3) * [-2.0978    1.3077   -0.2085]';
            er2 = self.er1(:,1:end-1);
            self.er1 = [self.error(x) er2];
            v2 = self.v1(:,1:end-1);
            self.v1 = [self.v v2];
        end
        function kd = kd(self)
            kd = 4*sqrt(self.kp);
        end
        function self = setref_d(self, ref)
            self.ref = pi/180*ref;
        end
        function error = error(self, x)
            error = x(1:3) - self.ref;
        end
    end
end