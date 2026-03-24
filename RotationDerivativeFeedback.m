function [Fc,Gc] = RotationDerivativeFeedback(var,m,g)
k = 0.004; % Nm/(rad/sec)
p = var(:,10);
q = var(:,11);
r = var(:,12);
Z_c = - k* m* g;
L_c = - k .* p;
M_c = - k .* q;
N_c = - k.* r;
Fc = [0,0,Z_c]';
Gc = [ L_c, M_c, N_c]';
end