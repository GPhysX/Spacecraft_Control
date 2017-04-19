classdef indi_q
    properties
        title = 'INDI';
        u = [0 0 0]';
        om = [0 0 0]';
        alpha = [0 0 0]';
    end
    methods
        function self = control(self, s, v)
%             qd = -.5 * s.Q * s.rot;
%             du = s.J*(v-s.rot) +  s.OM*s.J*s.rot;
%             a = (v-s.rot);
            self = self.d_om(s);
            du = s.J * (v-self.alpha);
            self.u = self.u + du;
        end
        
        function self = d_om(self, s)
            self.alpha = (s.rot - self.om)/s.Ts;
            self.om = s.rot;
        end
    end
end