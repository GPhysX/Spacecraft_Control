classdef indi_q
    % INDI linearizing control law for the innerloop of the spacecraft using
    % quaternions
    properties
        title = 'INDI';
        u = [0 0 0]';
        om = [0 0 0]';
        alpha = [0 0 0]';
        sc = spacecraft_q;
        Ts = 0.01;        
    end
    methods
        function self = control(self, x, v)
            self.sc = self.sc.update_x(x);
            self = self.d_om(self.sc);
            du = self.sc.J * (v-self.alpha);
            self.u = self.u + du;
        end
        
        function self = d_om(self, sc)
            self.alpha = (sc.rot - self.om)/self.Ts;
            self.om = sc.rot;
        end
    end
end