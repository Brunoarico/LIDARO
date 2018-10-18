# Introdução

Este trabalho tem por ideia principal desenvolver um sensor para o imageamento de ambientes denominado Lidar (Light Detection and Ranging), de forma que ele seja flexível em sua aplicação. Neste processo de desenvolvimento serão explorados temas de computação embarcada, microcontroladores, coleta de dados por meio de sensores e o tratamento desses dados.
O processo também envolverá uma forma de como extrair características úteis desse conjunto de dados para que seja aplicada principalmente na área de veículos autônomos.

 
# Motivação

Na área de veículos autônomos e robótica móvel, a necessidade de se conhecer o ambiente em que o móvel se encontra torna-se imprescindível. Para isso utiliza-se diversas técnicas de visão compotacional tentando otimizar, de alguma forma, a quantidade de dados coletado do ambiente unido a maior eficiência computacional para fazer isso.
Partindo dessa premissa, o LidaR tem se tornado um sensor muito utilizado por suprir exatamente esses requisitos, o que foi possível graças, essencialmente, ao aumento de poder computacional dos embarcado e a diminuição dos custos de fabricação dos circuitos integrados.
Existem diversas tecnologias diferentes para a implementtação de LiDaR's desde utilizando espelhos móveis, alternativas utilizando CCD's lineares e também construções híbridas, assim tendo este TCC também o foco de estudo de uma implementação que pondere tanto as questões de custo de implementação como de eficiência computacional.

# Objetivos 

O objetivo deste trabalho é de construir um Lidar, um sensor que utiliza um laser para medir distâncias e fazer um mapeamento do ambiente no qual se encontra imerso. 
A construção envolverá desde a construção do hardware, o firmware que operará esse hardware, até o software que fará o tratamento da nuvem de pontos gerada. 
A estrutura do Lidar em sí tambem envolverá a modelagem 3D das peças para que sejam prototipadas usando ferramentas de fabricação digital, como cortadora Laser e impressoras 3D.
