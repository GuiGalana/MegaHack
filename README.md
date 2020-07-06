# MegaHack
desenvolvimento para o evendo 
O projeto tem como objetivo fazer a leitira e transmção de informações de embarcações, sendo essas rebocadores, tais como o consumo instantaneo, risco de colisão e 
indicação de estado de trabalho.
Para o risco de colisão caso ele aconteça é dado um alerta ao comandante da embarcação informando ao mesmo que essa colisõao pode acontecer. Essa solução foi feta através
de leitura da posição das embarcações, sendo a embarcação monitorada e as que a mesma pode colidir, e de obstaculos que se encontram parados. Quando o algoritmo identifica 
que existe uma possibilidade de colisão ele emite o alarme e envia a central de comando que essa situação aconteceu. Para identificar essa colisão se utiliza de variaveis 
como velocidade do rebocador e spin, para objetos em movimento se utiliza tambem o spin do mesmo.
Os estados de trabalho são: 
Parado, sendo que este estado indica ausência de movimento do rebocador;
Deslocamento, sendo que este estado indica que o rebocador esta mudando sua posição porem não esta trabalhando;
Trabalhando, sendo que este estado indica que o rebocador esta empurrando ou puxando algum navio.
Para o consumo de combustível quando não tendo esta informação na maquina, as vezes por ser um modelo antigo, é utilizado um calculo que foi feito e testado pelos membros
da equipe em maquinas agricolas, visando o funcionamento de motores disel que se utiliza nos rebocadores. Essa informação é calculada e sera dividido o dado de consumo 
conforme o estado de trabalho da maquina, dando assim a acertividade de onde esta sendo o maior gasto em cada operação realizada.
