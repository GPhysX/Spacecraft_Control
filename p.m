classdef p
    properties
        title = 'P';
        u
    end
    methods
        function self = control(self, s, v)
            self.u = s.J*v;
        end
    end
end
