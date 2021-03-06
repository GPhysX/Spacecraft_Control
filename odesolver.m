classdef odesolver
    properties (SetAccess = immutable)        
        s_max = 10^-6;                      %maximum error
        % weights
        A = [ 0 0 0 0 0 0 0;
            1 0 0 0 0 0 0;
            1/4 3/4 0 0 0 0 0;
            11/9 -14/3 40/9 0 0 0 0;
            4843/1458 -3170/243  8056/729  -53/162 0 0 0;
            9017/3168  -355/33  46732/5247  49/176 -5103/18656 0 0;
            35/384 0 500/1113 125/192 -2187/6784  11/84 0]';
        by = [5179/57600 0 7571/16695 393/640 -92097/339200 187/2100 1/40]';
        bz = [  35/384   0  500/1113  125/192  -2187/6784    11/84    0]';
        c = [0 1/5 3/10 4/5 8/9 1 1]';
    end
    properties (SetAccess = private)
        s = 10^-6;
        h = 1;
        k = zeros(7);
    end
    properties
        sc2 = spacecraft;
    end
    methods
        function self = populate_k(self)
            sc = self.sc2;                  %make a copy of the state
            for i = 1:length(self.k)
                k(i) = 
            end
    end
    
        
    
    
    
    
end