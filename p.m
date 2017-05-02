classdef p
    % Controller to translate v to u using a factor J
    properties
        title = 'P';
        u
        sc = spacecraft;
    end
    methods
        function self = control(self, x, v)
            self.u = self.sc.J * v;
        end
    end
end
