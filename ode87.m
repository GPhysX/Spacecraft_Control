classdef ode87 < handle
    properties (SetAccess = immutable)
        % The coefficients of method
        c_i=  [ 1/18, 1/12, 1/8, 5/16, 3/8, 59/400, 93/200, 5490023248/9719169821, 13/20, 1201146811/1299019798, 1, 1]';

        a_i_j = [ 1/18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 
                  1/48, 1/16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 
                  1/32, 0, 3/32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 
                  5/16, 0, -75/64, 75/64, 0, 0, 0, 0, 0, 0, 0, 0, 0; 
                  3/80, 0, 0, 3/16, 3/20, 0, 0, 0, 0, 0, 0, 0, 0; 
                  29443841/614563906, 0, 0, 77736538/692538347, -28693883/1125000000, 23124283/1800000000, 0, 0, 0, 0, 0, 0, 0;
                  16016141/946692911, 0, 0, 61564180/158732637, 22789713/633445777, 545815736/2771057229, -180193667/1043307555, 0, 0, 0, 0, 0, 0;
                  39632708/573591083, 0, 0, -433636366/683701615, -421739975/2616292301, 100302831/723423059, 790204164/839813087, 800635310/3783071287, 0, 0, 0, 0, 0;
                  246121993/1340847787, 0, 0, -37695042795/15268766246, -309121744/1061227803, -12992083/490766935, 6005943493/2108947869, 393006217/1396673457, 123872331/1001029789, 0, 0, 0, 0;
                 -1028468189/846180014, 0, 0, 8478235783/508512852, 1311729495/1432422823, -10304129995/1701304382, -48777925059/3047939560, 15336726248/1032824649, -45442868181/3398467696, 3065993473/597172653, 0, 0, 0;
                  185892177/718116043, 0, 0, -3185094517/667107341, -477755414/1098053517, -703635378/230739211, 5731566787/1027545527, 5232866602/850066563, -4093664535/808688257, 3962137247/1805957418, 65686358/487910083, 0, 0;
                  403863854/491063109, 0, 0, -5068492393/434740067, -411421997/543043805, 652783627/914296604, 11173962825/925320556, -13158990841/6184727034, 3936647629/1978049680, -160528059/685178525, 248638103/1413531060, 0, 0]';

         b_8 = [ 14005451/335480064, 0, 0, 0, 0, -59238493/1068277825, 181606767/758867731,   561292985/797845732,   -1041891430/1371343529,  760417239/1151165299, 118820643/751138087, -528747749/2220607170,  1/4]';

         b_7 = [ 13451932/455176623, 0, 0, 0, 0, -808719846/976000145, 1757004468/5645159321, 656045339/265891186,   -3867574721/1518517206,   465885868/322736535,  53011238/667516719,                  2/45,    0]';
        %  A relative error tolerance that applies to all components of the solution vector. 
        tol = 1.e-6;

        pow = 1/8; % power for step control
    end
    properties (SetAccess = private)
        h_ = 0.05       % initial guess
    end
    methods
        function xout = xout(self, spacecraft,tspan)
            % Maximal step size
            hmax=(tspan(2) - tspan(1))/2.5;
            % Initialization
            nstep = 0;
            t0 = tspan(1);
            tfinal = tspan(2);
            t = t0;
            % Minimal step size
            hmin = 1.e-6;

            % constant for step rejection
            reject = 0;

            t0 = tspan(1);
            tfinal = tspan(2);
            t = t0;
            x = spacecraft.x;          % start point
            f = x*zeros(1,13);  % array f for RHS calculation
            %tout = t;
            xout = x;
            tau = self.tol * max(norm(x,'inf'), 1);  % accuracy

            % The main loop
               while (t < tfinal) && (self.h_ >= hmin)
                  if (t + self.h_) > tfinal 
                     h = tfinal - t;
                  else
                     h = self.h_;
                  end;

            % Compute the RHS for step of method
                  f(:,1) = spacecraft.x_dotx(x);
                  for j = 1: 12
                      f(:,j+1) = spacecraft.x_dotx(x+h*f*self.a_i_j(:,j));
                  end;

            % Two solution 
                  sol2=x+h*f*self.b_8;
                  sol1=x+h*f*self.b_7;

            % Truncation error 
                  error_1 = norm(sol1-sol2);

            % Estimate the error and the acceptable error
                  Error_step = norm(error_1,'inf');
                  tau = self.tol*max(norm(x,'inf'),1.0);

            % Update the solution only if the error is acceptable
                  if Error_step <= tau                      
                     t = t + h;
                     x = sol2; 
                     xout = x;
                     if t == tfinal
                        return;
                     end
                  end;
            % Step control
                  if Error_step == 0.0
                     Error_step = eps*10.0;
                  end;
                  self.h_ = min(hmax, 0.9*self.h_*(tau/Error_step)^self.pow);
                  if (abs(self.h_) <= eps) 
                     if reject == 0
                        disp('Warning!!! ode87. Step is very small!!!');
                        self.h_ = eps * 100;
                        return
                     else
                        disp('Error in ode87. Step is too small.');
                        return; 
                     end;
                  end;
                  

              end;

               if (t < tfinal)
                  disp('Error in ODE87...')
                  disp(t);
               end;
% 
%               if haveoutfun
%                  feval(outfun,t,x,'done',varargin{:});
%               end;
        end
    end
end