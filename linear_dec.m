classdef linear 
    properties
        zeta = 0.707;
        kp = [1 1 1]; 
        ref = [0 0 0]'; 
        v = [0 0 0]';
        sc = spacecraft;
        er1 = zeros(3,5);
        v1 = zeros(3,5);
        Ts = 0.1;        
    end
    methods
        function self = control(self, x, s)
%             self.v = -self.kp * self.error(x) - self.kd*(self.error(x) - self.er1(:,1))/self.Ts;
%             self.v =   -10* (self.error(x)-0.994*self.er1(:,1));
            self.v = -(self.kp * x(1:3) + self.kd*x(4:6));
            er2 = self.er1(:,1:end-1);
            self.er1 = [self.error(x) er2];
            v2 = self.v1(:,1:end-1);
            self.v1 = [self.v v2];
        end
        function kd = kd(self)
            kd = [self.kd1; self.kd2; self.kd3];
        end
        function self = setref_d(self, ref)
            self.ref = pi/180*ref;
        end
        function error = error(self, x)
            error = x(1:3) - self.ref;
        end
        function uu = uu(self)
            uu = self.kp(1)*self.kp(3)/(self.sc.J(1,1)*self.sc.J(3,3));
        end
        function vv = vv(self)
            vv = self.sc.J(3,3)*self.kp(1) + self.sc.J(1,1)*self.kp(3);
        end
        function ww = ww(self)
            ww = (-self.kp(3) + self.uu*self.sc.J(3,3))/(self.kp(1)-self.uu*self.sc.J(1,1));
        end
        function kd1 = kd1(self)
            kd1 = sqrt(self.vv - self.uu*self.sc.J(1,1)*self.sc.J(3,3)*(4*self.zeta^2 - 2)/(self.sc.J(1,1)/(4*self.zeta^2*self.sc.J(1,1))+self.ww/(2*self.zeta^2)+self.ww^2*self.sc.J(1,1)/(4*self.zeta^2*self.sc.J(3,3))-self.ww));
        end
        function kd2 = kd2(self)
            kd2 = 2 * self.zeta*sqrt(self.kp(2)*self.sc.J(2,2));
        end
        function kd3 = kd3(self)
            kd3 = self.ww * self.kd1;
        end
    end
end