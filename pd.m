classdef pd < spacecraft
    properties
        zeta = 0.707;
        kp = [1 1 1]';
        ref = [0 0 0]';
    end
    methods
        function self = sim(self)
            self.control;
            self.step;  
        end
        function self = control(self)
            Tc = self.kp' .* (self.d2r(self.ref) - self.theta) - self.kd' .* self.rot;
            self.Tc = Tc;
        end
        function kd = kd(self)
            kd = 2*self.zeta*sqrt(self.kp.*(ones(1,3)*self.J));
        end
        function d2r = d2r(self,c)
            d2r = c*(1/180.*pi);
        end
    end
end
        
            
            
            
        
