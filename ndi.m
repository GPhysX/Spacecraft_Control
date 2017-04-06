classdef ndi
    properties
        u
    end
    methods
        function self = control(self, s, v)
            self.u = s.J*inv(s.N)*(v-s.N_d*s.rot) + s.OM*s.J*s.rot - s.Td;
        end
    end
end
