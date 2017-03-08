classdef spacecraft_q < spacecraft
    properties
        q = [0 0 0 1]';
    end
    properties (SetAccess = immutable)
        H_ = [[eye(3);zeros(1,3)] zeros(4)];
    end
    methods
        function q_ = q_(self)
            q_ = self.q(1:3);
        end
        function x = x(self)
            x = [self.q; self.rot];
        end
        function cv = cv(self)
            cv = self.H_ * self.x;
        end
        function G = G(self)
            G = [zeros(4,3) zeros(4,3); inv(self.J) inv(self.J)];
        end
        function OM_ = OM_(self)
            w = self.rot;
            OM_ = [0 w(3) -w(2) w(1); -w(3) 0 w(1) w(2); 
                w(2) -w(1) 0 w(3); -w(1) -w(2) -w(3) 0];
        end
        function f = f(self)
            f = [0.5 * self.OM_ * self.q; -inv(self.J)*self.OM() * self.J*self.rot ];
        end
        function ok = ok(self)
            %TODO
            ok = 0;
        end
        function self = update_x(self,x)
            self.q = x(1:4);
            self.rot = x(5:7);
        end        
    end
end
