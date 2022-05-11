y = Dadostratados.y;
u = Dadostratados.u ;

Y = y;

t = [0:length(y)-1]*0.1;
j = 0;

for i = 1:length(y)
    if y(i) > 10000 || y(i) < -10000
            y(i) = y(i-1);
            
    end
end

%% Filtro
 s = tf('s')

 
 G = -0.18694/(34.914*s + 1)
 
 Y = lsim(G,u,t);
 
 F = 1/(3.5*s+1) %10x mais rápido 
 
 Yf = lsim(F,y,t);
 
 figure();
 plot(t,Y,'r',t,y,'b',t,Yf,'m')
 
 %% Filtro discreto 
 Fz = c2d(F,0.1,'tustin')

 %% Escrevendo como equações de diferenças 
N = length(t);
S = zeros(1,N);
E = ones(1,N);

a = 0.01408; 
b = 0.9718;
for k=3:N 
S(1,K)  =  a 

end 
