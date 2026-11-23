function [est, mem] = estimadorEstado(med, uPrevious, p, mem, dt) %#ok<INUSD>
% P5 EKF: IMU prediction and intermittent GPS correction.
% med: acc,gyro,hay_gps,gps_X,gps_Y. uPrevious preserves the P5 interface;
% this P5 predictor uses IMU data and does not use the command itself.
% mem.xh = [X;Y;psi;vx], mem.P = 4x4 covariance. p.Q:4x4, p.R:2x2.
% This original EKF does not estimate vy or gyro bias: vy=0, r=med.gyro.
xh=mem.xh;
P=mem.P;
X=xh(1); Y=xh(2); psi=xh(3); vx=xh(4);
rMeasured=med.gyro;
axMeasured=med.acc;
xPredicted=[X+vx*cos(psi)*dt;
    Y+vx*sin(psi)*dt;
    psi+rMeasured*dt;
    vx+axMeasured*dt];
F=[1,0,-vx*sin(psi)*dt,cos(psi)*dt;
   0,1, vx*cos(psi)*dt,sin(psi)*dt;
   0,0,1,0;
   0,0,0,1];
predictedCovariance=F*P*F.'+p.Q;
if med.hay_gps
    measurement=[med.gps_X;med.gps_Y];
    H=[1 0 0 0;0 1 0 0];
    innovation=measurement-H*xPredicted;
    S=H*predictedCovariance*H.'+p.R;
    K=predictedCovariance*H.'/S;
    xh=xPredicted+K*innovation;
    P=(eye(4)-K*H)*predictedCovariance;
else
    xh=xPredicted; P=predictedCovariance;
end
mem.xh=xh; mem.P=P;
est.X=xh(1); est.Y=xh(2); est.psi=xh(3); est.vx=xh(4);
est.vy=0;
est.r=med.gyro;
end
