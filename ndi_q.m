classdef ndi_q
    properties
        title = 'NDI';
        u
    end
    methods
        function self = control(self, s, v)
            self.u = s.J*(v-s.rot) +  s.OM*s.J*s.rot  - s.Td;
        end
    end
end