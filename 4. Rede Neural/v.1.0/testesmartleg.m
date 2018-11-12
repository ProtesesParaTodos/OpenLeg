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
plot(tempo,anguloJoelho, 'linewidth',1);
x1 = entradaVelAngCoxa';

clear entradasAnteriores;
for redeneural1 = 1:size(entradaVelAngCoxa)
    entrada = x1(:,redeneural1);
    for anterior = 1:5
         indice = redeneural1-anterior;
         if indice<1
         entradasAnteriores ((6-anterior), 1) = 0;
         entradasAnteriores ((6-anterior), 2) = 0;
         else
         entradasAnteriores ((6-anterior), 1) = x1(1,indice);
         entradasAnteriores ((6-anterior), 2) = x1(2,indice);
         end
    end
    [y1, xf1] = myNeuralNetworkFunction1(entrada, entradasAnteriores');
   
    saida(redeneural1,1) = y1;
end
plot(tempo,saida, 'linewidth',1);

clear entradasAnteriores;
for redeneural2 = 1:size(entradaVelAngCoxa)
   entrada = x1(:,redeneural2);
   if redeneural2-1 < 1
        entradaOutput = 0;
   else
   entradaOutput = output(redeneural2-1);
   end
   for anterior = 1:5
         indice = redeneural2-anterior;
         if indice<1
         entradasAnteriores ((6-anterior), 1) = 0;
         entradasAnteriores ((6-anterior), 2) = 0;
         else
         entradasAnteriores ((6-anterior), 1) = x1(1,indice);
         entradasAnteriores ((6-anterior), 2) = x1(2,indice);
         end
   end
      for contadorOutput = 1:5
         indice = (redeneural2-contadorOutput);
         if indice<1
         outputAnterior (6-contadorOutput) = 0;
           else
         outputAnterior (6-contadorOutput) = output(indice);
         end
      end
  [y1,xf1,xf2] = myNeuralNetworkFunction2(entrada,entradaOutput,entradasAnteriores',outputAnterior);
  output(redeneural2) = y1;
end
plot(tempo,output, 'linewidth',1);

% clear entradasAnteriores;
% entradasAnteriores = zeros(2,5);
% outputAnterior = zeros(1,5);
% for redeneural2 = 1:size(entradaVelAngCoxa)
%   entrada = x1(:,redeneural2);
%   if redeneural2-1 < 1
%        entradaOutput = 0;
%   else
%        entradaOutput = output(redeneural2-1);
%   end
%   [y1,xf1,xf2] = myNeuralNetworkFunction2(entrada,entradaOutput,entradasAnteriores,outputAnterior);
%   entradasAnteriores = xf1;
%   outputAnterior = xf2;
%   output(redeneural2) = y1;   
% end
% % plot(tempo,output, 'linewidth',1);