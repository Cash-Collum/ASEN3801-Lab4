function motor_forces =  ComputeMotorForces(Fc, Gc, d, km)

Z_c = Fc(1,3);
control_input_array = [Z_c; Gc(1); Gc(2); Gc(3)];
Values = [-1,-1,-1,-1; -d/sqrt(2), -d/sqrt(2), d/sqrt(2), d/sqrt(2);
           d/sqrt(2), -d/sqrt(2), d/sqrt(2), -d/sqrt(2); km, -km, km, -km];

motor_forces = inv(Values) * control_input_array;
end

