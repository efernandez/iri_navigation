% sphere social model: revision for map generation
%
%
%
% 2-Movember-2012
% Gonzalo Ferrer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function f =  sphere ( param ,  person , obstacle  )


%spherical force model, the simplest
K = param(1);
lambda = param(2);
A = param(3);
B = param(4);
d_0 = param(5);
d = person - obstacle;
f_module = A * exp( (d_0 - norm(d) )/ B) ;
d_u = d./norm(d);
f =  f_module .*d_u;
%anisotrophy -> lambda =1
%phi = diffangle(v_person , -obstacle+person);
%if lambda <0 
%	lambda = 0;
%elseif lambda > 1
%	lambda = 1;
%end

%f = f .* (lambda + (1-lambda)*(1 + cos(phi))/2 );

% gradient calculation
%f_gradient = [ 0 0; %k
%					0 0; %lambda
%					f_module/A *. d_u ; %df/dA
%					 .* d_u; %df/dB
%					0 0]; %tau (not necessary using the spherical model)


