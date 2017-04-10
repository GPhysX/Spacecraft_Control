classdef indi
    properties
        title = 'INDI';
        u = [0 0 0]';
        om = [0 0 0]';
        alpha = [0 0 0]';
    end
    methods
        function self = control(self, s, v)
            self = self.d_om(s);
            du = s.J *( inv(s.N)*v-self.alpha);
            self.u = self.u + du;
        end
        function self = d_om(self, s)
            self.alpha;
            self.alpha = (s.rot - self.om)/s.dt;
            self.om = s.rot;
        end
            
    end
end