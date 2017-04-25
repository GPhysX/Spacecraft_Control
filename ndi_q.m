classdef ndi_q
    properties
        title = 'NDI';
        u
        sc = spacecraft_q;
    end
    methods
        function self = control(self, x, v)
            self.sc = self.sc.update_x(x);            
            self.u = self.sc.J*(v) +  self.sc.OM*self.sc.J*self.sc.rot  - self.sc.Td;
        end
    end
end