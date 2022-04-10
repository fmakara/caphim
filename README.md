## CapIHM
CapIHM, ou Interface Homem-Maquina CAPacitiva, é a tentativa de criar uma interface barata, configurável e passível de isolação elétrica. Ela consiste de um RP2040 conectado a pads capacitivos e um OLED1306 (128x64)

Em 'cpp/' (precisa de 'pico-sdk/' checkauteado) existe código básico para fazer um sensor capacitivo e driver do OLED.

Em 'pure-micropython-scripts/' estão scripts que descrevem o comportamento interno apenas em micropython. O problema com ele é que ele parece reavaliar a cada leitura o código PIO.

Para solucionar isto, se está passando metade do comportamento basico para um módulo implementado em C de dentro do micropython. Para isto é necessário compilar 'micropython/' usando as adições de 'micropython-mods/', usando os scripts 'prepare-micropython.sh' e 'build-micropython.sh'

A placa e moldes estão sendo re-preparados e serão incluidas diretamente neste repositorio (...eventualmente).