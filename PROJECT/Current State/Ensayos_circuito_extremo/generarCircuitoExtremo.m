function pista=generarCircuitoExtremo
% Circuito cerrado de33 curvas. Arcos exactos y rectas tangentes, sin esquinas.
paso=.5;ancho=6;pasoConos=5;
xy=[0 0];tangente=[1 0];curvatura=0;pos=[0 0];rumbo=0;
sectores=struct('tipo',{},'inicio_m',{},'fin_m',{},'radio_m',{},'angulo_deg',{});
distancia=0;
recta(250,'Recta de lanzamiento');
enlace(100,25,'Enlazada rápida');
recta(180,'Frenada desde alta velocidad');
enlace(14,45,'Chicane rápida');
recta(120,'Aceleración y aproximación a horquillas');
horquillas(7,80,180,10,'Horquillas cerradas');
recta(100,'Salida de horquilla');
enlace(25,55,'Enlazada técnica');
recta(220,'Recta de alta velocidad');
longitudLado=pos(1);assert(abs(pos(2))<1e-8&&abs(rumbo)<1e-8);
arco(65,90,'Curva rápida90');
recta(180,'Recta norte');enlace(100,30,'Doble apoyo rápido');recta(300,'Recta larga norte');
longitudVertical=pos(2)-65;
arco(65,90,'Curva90 norte');inicio=pos;
recta(180,'Aceleración oeste');enlace(120,35,'Enlazada rápida oeste');recta(120,'Entrada a horquillas');
horquillas(10,100,200,15,'Horquillas de radio variable');
recta(100,'Aceleración oeste técnica');enlace(12,50,'Chicane cerrada');
restante=longitudLado-(inicio(1)-pos(1));assert(restante>100);
recta(restante,'Recta máxima antes de curva rápida');
arco(65,90,'Curva rápida90 oeste');
recta(100,'Recta sur');enlace(60,35,'Enlazada de retorno');
restante=longitudVertical-(100+4*60*sind(35));assert(restante>100);
recta(restante,'Recta final');arco(65,90,'Curva de meta');
assert(norm(pos)<1e-7 && abs(mod(rumbo+pi,2*pi)-pi)<1e-7,'Circuito sin cierre.');
xy(end,:)=[];tangente(end,:)=[];curvatura(end)=[];
s=[0;cumsum(vecnorm(diff(xy),2,2))];d=circshift(xy,-1)-xy;ds=vecnorm(d,2,2);
normal=[-tangente(:,2) tangente(:,1)];longitud=sum(ds);
pista=struct('centro',xy,'tangente',tangente,'curvatura',curvatura,'s',s, ...
 'longitud',longitud,'ancho',ancho,'segmentoA',xy,'segmentoD',d,'segmentoL',ds);
pista.bordeIzq=xy+ancho/2*normal;pista.bordeDer=xy-ancho/2*normal;
sc=(0:pasoConos:longitud-1e-6)';
pista.conoIzq=interp1([s;longitud],[pista.bordeIzq;pista.bordeIzq(1,:)],sc);
pista.conoDer=interp1([s;longitud],[pista.bordeDer;pista.bordeDer(1,:)],sc);
pista.sectores=struct2table(sectores);
pista.curvas=pista.sectores(pista.sectores.radio_m>0,:);
assert(height(pista.curvas)==33);
% La anchura y los radios aseguran borde interior positivo en todos los arcos.
assert(min(pista.curvas.radio_m)>ancho/2);

 function recta(L,tipo)
  assert(L>0);n=max(1,ceil(L/paso));direccion=[cos(rumbo) sin(rumbo)];
  puntos=pos+(1:n)'/n*L.*direccion;
  agregar(puntos,repmat(direccion,n,1),zeros(n,1),L,tipo,0,0);
 end
 function arco(R,grados,tipo)
  giro=deg2rad(grados);signo=sign(giro);n=max(2,ceil(abs(giro)*R/paso));
  centro=pos+signo*R*[-sin(rumbo) cos(rumbo)];
  theta0=atan2(pos(2)-centro(2),pos(1)-centro(1));theta=theta0+(1:n)'/n*giro;
  puntos=centro+R*[cos(theta) sin(theta)];
  agregar(puntos,signo*[-sin(theta) cos(theta)],signo/R*ones(n,1),abs(giro)*R,tipo,R,grados);
  rumbo=rumbo+giro;
 end
 function enlace(R,grados,tipo)
  arco(R,grados,[tipo ' entrada']);arco(R,-2*grados,[tipo ' contracurva']);arco(R,grados,[tipo ' salida']);
 end
 function horquillas(R,atras,adelante,Rfinal,tipo)
  arco(R,180,[tipo '180 izquierda']);recta(atras,[tipo ' recta intermedia']);
  arco(R,-180,[tipo '180 derecha']);recta(adelante,[tipo ' salida enlazada']);
  arco(Rfinal,-90,[tipo '90 derecha']);recta(4*R-2*Rfinal,[tipo ' conexión']);
  arco(Rfinal,90,[tipo '90 izquierda']);
 end
 function agregar(puntos,t,k,L,tipo,R,grados)
  xy=[xy;puntos];tangente=[tangente;t];curvatura=[curvatura;k];pos=puntos(end,:);
  sectores(end+1)=struct('tipo',tipo,'inicio_m',distancia,'fin_m',distancia+L,'radio_m',R,'angulo_deg',grados);
  distancia=distancia+L;
 end
end
