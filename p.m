classdef p
    properties
        u
    end
    methods
        function self = control(self, s, v)
            self.u = s.J*v;
        end
    end
end
