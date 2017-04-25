classdef ndi
    %%% This object contains the non linear dynamic inversion control law. 
    properties
        title = 'NDI';
        u
        sc = spacecraft;
    end
    methods
        function self = control(self, x, v)
 			% Update internal model to the current state
            self.sc = self.sc.update_x(x);
            % Linearizing control law
            self.u = self.sc.J*inv(self.sc.N)*(v-self.sc.N_d*self.sc.rot) + self.sc.OM*self.sc.J*self.sc.rot - self.sc.Td;
        end
    end
end
