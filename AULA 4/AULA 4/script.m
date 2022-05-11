Dadostratados = readtable('dadosTratados.csv')
Dadostratados.Properties.VariableNames = {'u','y'}
y = Dadostratados.y;
u = Dadostratados.u ;
Y = zeros(length(y),1);
t = [0:length(y)-1] * 0.1;
j = 0;
numeroDeAmostrasMedia = 3;
medArray = ones(1,numeroDeAmostrasMedia)*0.1;

for i = 1:length(y)
    if i > (numeroDeAmostrasMedia + 1)
        for j = 1:numeroDeAmostrasMedia
            if (y(i-j) < mean(medArray)*20 && y(i-j) > mean(medArray)*0.05)
                medArray(j) = y(i-j);
            else 
                medArray(j) = mean(medArray);
            end
        end
    end
    if y(i) > mean(medArray)*1.6 || y(i) < mean(medArray)*0.4
            Y(i) = ceil(mean(medArray));
    else 
        Y(i) = y(i);
    end
end
%% 
%systemIdentification();

%% Filtro
 s = tf('s')
 
 G = -0.18694/(34.914*s + 1)
 
 Ylsim = lsim(G,u,t);
 
 F = 1/(3.5*s+1) %10x mais rápido 
 
 Yf = lsim(F,Y,t);
 
 figure();
 plot(t,Ylsim,'r',t,Y,'b',t,Yf,'m')
 
 %% Filtro discreto 
 Fz = c2d(F,0.1,'tustin')
 
 [n, d] = tfdata(Fz, 'v');
 
 a = n(1);
 
 b = abs(d(2));
 
 Yfk = zeros(length(y),1);
 
 Yfk(1) = a*Y(1);
 
 for k = 2:length(y)
      Yfk(k) = a*Y(k)+a*Y(k-1)+b*Yfk(k-1);
 end
figure();
plot(t,Ylsim,'r',t,Y,'b',t,Yf,'m',t,Yfk,'g')
figure();
plot(t,Yf,'m',t,Yfk,'g')
%% Simulação
u0 = 52433;
y0= 40982;

%sintese direta
kp = -0.52146;
ti  =  1;
td = 0.0354;

C = kp*(1+1/s*ti);

C = -14.405*(s + 0.0362)/s

Csiso = (-359.37*(s+0.09174)*(s+0.3104))/s;

Gbar = zpk(G*F);

sisotool(Gbar,C);
Cz = c2d(C,0.1,'tustin')
pidstd(C);

