function med = sensores(z, tele, p, k)
% P5 GPS/IMU noise and sampling model, driven by four-wheel plant outputs.
% z supplies X,Y,r. tele.ax supplies the plant's physical longitudinal
% acceleration. No bicycle force/acceleration model is recalculated here.
% p: acc_sigma,gyro_sigma,gyro_bias,gps_sigma,gps_cada,gps_corte,dt.
% k is the 1-based measurement index. Caller controls rng for repeatability.
% tele must correspond to the plant sample being measured.
med.acc=tele.ax+p.acc_sigma*randn;
med.gyro=z(6)+p.gyro_bias+p.gyro_sigma*randn;
t=(k-1)*p.dt;
inOutage=~isempty(p.gps_corte) && t>=p.gps_corte(1) && t<=p.gps_corte(2);
med.hay_gps=(mod(k-1,p.gps_cada)==0) && ~inOutage;
if med.hay_gps
    med.gps_X=z(1)+p.gps_sigma*randn;
    med.gps_Y=z(2)+p.gps_sigma*randn;
else
    med.gps_X=NaN; med.gps_Y=NaN;
end
med.t=t;
end
