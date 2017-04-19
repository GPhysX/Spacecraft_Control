classdef indi
    properties
        title = 'INDI';
        u = [0 0 0]';
        theta_d = [0 0 0]';
        alpha = [0 0 0]';
    end
    methods
        function self = control(self, s, v)
            self = self.d_th(s);
            du = s.J *(v-self.alpha);
            self.u = self.u + du;
        end
        function self = d_th(self, s)
            f = s.f;
            theta_d = f(1:3);
            self.alpha = (theta_d - self.theta_d)/s.Ts;
            self.theta_d = theta_d;
        end
    end
end