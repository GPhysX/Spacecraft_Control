classdef pd 
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
%             self.v = - [self.error(x) self.er1] * [9.9774  -46.8090   87.6848  -81.9704   38.2353   -7.1181]' - self.v1 * [-3.8318    5.6749   -4.0057    1.3145   -0.1519]';
%             self.v = - [self.error(x) self.er1] * [8.9652  -25.9300   24.9746   -8.0097]' - self.v1 * [-2.0978    1.3077   -0.2085]';
%             self.v = 1/4*([self.error(x) self.er1] * [19.9526  -57.7048   55.5746  -17.8224]' - self.v1 * [-2.0978    1.3076   -0.2085]');            
%             self.v = -self.kp * self.error(x) - self.kd*(self.error(x) - self.er1(:,1))/self.Ts;
            self.v =   -6.6337* (self.error(x)-0.9942*self.er1(:,1));
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