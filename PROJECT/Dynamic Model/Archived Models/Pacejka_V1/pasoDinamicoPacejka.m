function [z_next,tele]=pasoDinamicoPacejka(z,u,p,dt)
% Nueva dinámica longitudinal Pacejka con cuatro estados de giro de rueda.
% Interfaz: z (24), u, p, dt -> z_next (24), tele.
% u=[FxTotal;delta] mantiene el reparto nominal; alternativamente
% u=[FxDI;FxDD;FxTI;FxTD;delta] aplica las cuatro demandas independientes.
% Una demanda [N] se convierte en par aplicado mediante T=R*FxSolicitada.
% Motor limitado en potencia; el par de frenado no invierte las ruedas.
% Lateral: rigidez original, saturada por la capacidad restante del círculo.
% No es un ajuste completo de Pacejka combinado ni un modelo de suspensión.
% Desaparecen las pérdidas empíricas eta_slip y eta_brake de esta versión.
validateattributes(z,{'numeric'},{'real','finite','vector','numel',24});
validateattributes(u,{'numeric'},{'real','finite','vector'});
validateattributes(dt,{'numeric'},{'real','finite','scalar','positive'});
z=z(:);u=u(:);
if numel(u)==2
    delta=u(2);
    if u(1)>=0,solicitadas=u(1)*p.drive_w;else,solicitadas=u(1)*p.brake_w;end
elseif numel(u)==5
    solicitadas=u(1:4).';delta=u(5);
else
    error('Pacejka:EntradaInvalida','u debe contener 2 o 5 elementos.');
end
R=p.pacejka.radio;I=p.pacejka.inercia;
n=ceil(dt/p.pacejka.pasoMaximo);h=dt/n;
FxAcum=zeros(1,4);FyAcum=FxAcum;FzAcum=FxAcum;
potenciaAcum=0;calorAcum=0;axAcum=0;ayAcum=0;
for sub=1:n
    vx=z(4);vy=z(5);r=z(6);omega=z(21:24).';
    dw=[delta delta 0 0];
    vxw=vx-r*p.yw;vyw=vy+r*p.xw;
    vxLocal=vxw.*cos(dw)+vyw.*sin(dw);
    vyLocal=-vxw.*sin(dw)+vyw.*cos(dw);
    dLong=p.m*z(7)*p.hcg/p.L;
    dDel=p.kRoll*p.m*z(8)*p.hcg/p.td;
    dTras=(1-p.kRoll)*p.m*z(8)*p.hcg/p.tt;
    Fz=p.Fzw0+[-dLong/2 -dLong/2 dLong/2 dLong/2]+[-dDel dDel -dTras dTras];
    if any(Fz<0)||any(~isfinite(Fz))
        error('Pacejka:CargaInvalida','Carga vertical negativa: se ha salido del alcance del modelo plano.');
    end
    capacidad=p.muw.*Fz;
    kappa=(R*omega-vxLocal)./max(abs(vxLocal),p.pacejka.velocidadRegularizacion);
    Fx=fuerzaPacejka(kappa,Fz,p.muw,p);
    % El ángulo se calcula en el sistema de rueda: no sumar delta otra vez.
    alpha=-atan2(vyLocal,max(abs(vxLocal),p.pacejka.velocidadRegularizacion));
    rigidez=p.Caw.*(Fz./p.Fzw0).^p.nCa;
    FyPura=rigidez.*alpha;
    capacidadLateral=sqrt(max(capacidad.^2-Fx.^2,0));
    Fy=capacidadLateral.*tanh(FyPura./max(capacidadLateral,eps));
    bloqueo=(omega<.05)&(solicitadas<0)&(vxLocal>.5);
    if any(bloqueo)
        % Rueda bloqueada: fricción opuesta a la velocidad de deslizamiento,
        % independiente de hacia dónde apunten las ruedas delanteras.
        modulo=hypot(vxLocal,vyLocal);
        magnitud=abs(fuerzaPacejka(-ones(1,4),Fz,p.muw,p));
        Fx(bloqueo)=-magnitud(bloqueo).*vxLocal(bloqueo)./modulo(bloqueo);
        Fy(bloqueo)=-magnitud(bloqueo).*vyLocal(bloqueo)./modulo(bloqueo);
    end
    parMotor=R*max(solicitadas,0);
    potenciaSolicitada=sum(parMotor.*omega);
    escalaPotencia=1;
    if potenciaSolicitada>p.pacejka.potenciaMaxima
        escalaPotencia=p.pacejka.potenciaMaxima/potenciaSolicitada;
    end
    parMotor=parMotor*escalaPotencia;
    parFreno=R*max(-solicitadas,0);
    omegaNext=max(0,omega+h*(parMotor-parFreno-R*Fx)/I);
    FxChasis=Fx.*cos(dw)-Fy.*sin(dw);
    FyChasis=Fx.*sin(dw)+Fy.*cos(dw);
    ax=(sum(FxChasis)-.5*p.rho*p.Cd*p.Af*vx*abs(vx)-sign(vx)*p.fr*p.m*p.g*cos(p.theta)-p.m*p.g*sin(p.theta))/p.m;
    ay=sum(FyChasis)/p.m;
    dr=sum(p.xw.*FyChasis-p.yw.*FxChasis)/p.Iz;
    psi=z(3);
    z(1:6)=z(1:6)+h*[vx*cos(psi)-vy*sin(psi);vx*sin(psi)+vy*cos(psi);r;ax+vy*r;ay-vx*r;dr];
    z(4)=max(z(4),0);z(7:8)=[ax;ay];z(21:24)=omegaNext(:);
    FxAcum=FxAcum+Fx;FyAcum=FyAcum+Fy;FzAcum=FzAcum+Fz;
    potenciaAcum=potenciaAcum+sum(parMotor.*omega);
    calorAcum=calorAcum+sum(Fx.*(R*omega-vxLocal)-Fy.*vyLocal);
    axAcum=axAcum+ax;ayAcum=ayAcum+ay;
end
z(9:16)=1;z(17:20)=double(bloqueo(:));z_next=z;
tele=struct('Fxw',FxAcum/n,'Fyw',FyAcum/n,'Fzw',FzAcum/n, ...
    'Fmax_w',p.muw.*FzAcum/n,'Fmax_peak_w',p.muw.*FzAcum/n, ...
    'kappa',kappa,'omega',z(21:24).','alpha_w',alpha, ...
    'bloqueo_w',bloqueo,'ax',axAcum/n,'ay',ayAcum/n, ...
    'potenciaMotor',potenciaAcum/n,'potenciaDisipada',calorAcum/n, ...
    'escalaPotencia',escalaPotencia,'Fxw_sol',solicitadas);
end
