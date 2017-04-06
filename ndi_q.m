classdef ndi_q
    properties
        u
    end
    methods
        function self = control(self, s, v)
            self.u = s.J*(v-s.rot) +  s.OM*s.J*s.rot  - s.Td;
        end
    end
end
