classdef wn 
    properties
        v = [0 0 0]';
    end
    methods
        function self = control(self, x)
            self.v = -.1 + .2*rand * [1 1 1]';
        end
        function self = setref_d(self, r)
            return
        end
    end
end