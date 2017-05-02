classdef wn 
    % Outerloop controller to produce quasi gausian white noise, eg. used for system
    % identification.
    properties
        v = [0 0 0]';
    end
    methods
        function self = control(self, x)
            self.v = -.01 + .02*[rand rand rand]';
        end
        function self = setref_d(self, r)
            return
        end
    end
end