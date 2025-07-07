clear all
delete(instrfindall); % 혹시 시리얼이 다른 곳에서 열려 있으면 종료

Nsamples = 400;
EulerSaved = zeros(Nsamples, 3);

dt = 0.01;

[accel_var, gyro_var] = calcurate_sensor_noise();
disp("Finish noise calculation");
pause(1);

Q = diag(gyro_var);
R = diag(accel_var);
R = R(1:2, 1:2);

for k = 1:Nsamples
    [accel, gyro, mag] = imu_read();
    p = gyro(1);
    q = gyro(2);
    r = gyro(3);
    ax = accel(1);
    ay = accel(2);
    az = accel(3);
    [phi, theta] = LinearEulerAccel(ax,ay,az);
    [phi, theta, psi] = EulerEKF([phi theta]', Q, R, [p q r], dt);
    EulerSaved(k,:) = [phi theta psi];
    pause(dt)
end

PhiSaved = EulerSaved(:,1) * (180/pi);
ThetaSaved = EulerSaved(:,2) * (180/pi);
PsiSaved = EulerSaved(:,3) * (180/pi);

t = 0:dt:Nsamples*dt-dt;

figure
plot(t, PhiSaved)
ylim([-30 30])
title('Roll (Phi)')

figure
plot(t, ThetaSaved)
ylim([-30 30])
title('Pitch (Theta)')