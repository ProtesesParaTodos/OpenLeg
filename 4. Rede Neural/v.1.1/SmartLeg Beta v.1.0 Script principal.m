clear;
A = importdata('teste2-1.xls');

index=A.data(:,4);
tempo=index.*0.008;

anguloCoxa=str2double(A.textdata(:,1));
velocidadeAngular=str2double(A.textdata(:,2));
anguloCanela=str2double(A.textdata(:,3));
anguloJoelho =(anguloCoxa-anguloCanela)+ 20;
entradaVelAngCoxa = [anguloCoxa velocidadeAngular];

plot(tempo,anguloCoxa, 'linewidth',1);
grid on;
xlabel('Tempo [s]');
ylabel('Ângulo [º]');
axis([3 5 -10 140]);
hold on;
title('Gráfico de ângulo em relação ao tempo da caminhada');
plot(tempo,anguloJoelho,'--');

clear entradasAnteriores;
saidaAnterior = 0;
entradasAnteriores = zeros(2,5);
for redeneural1 = 1:size(entradaVelAngCoxa,1)
    entrada = entradaVelAngCoxa(redeneural1,:)';
    [y1, xf1] = myNeuralNetworkFunction1(entrada, entradasAnteriores);
    y1=y1*0.6+saidaAnterior*0.4;
    entradasAnteriores = xf1;
    saidaAnterior=y1;
    saida(redeneural1,1) = y1;
end
plot(tempo,saida, 'linewidth',1);
legend('Angulo adquirido da coxa', 'Angulo adquirido do Joelho', 'Angulo calculado do joelho');
