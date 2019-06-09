El fichero importante es el main.c
Se Elige con el programa "STM32MX" las funciones que emplearemos, con sus respectivos pines si fuese necesario,
a continuacion se genera codigo a tratar en uvision, cuyas modificaciones a sustituir son los ficheros de esta carpeta.

-DO PA11 Entrada Digital con Pull-up tecla

-RE PA12 Entrada Digital con Pull-up tecla

-MI PB2 Entrada Digital con Pull-up tecla

-FA PC12 Entrada Digital con Pull-up tecla

-SOL PC13 Entrada Digital con Pull-up tecla

-LA PD2 Entrada Digital con Pull-up tecla

-SI PH0 Entrada Digital con Pull-up tecla

-TIM2_CH1 PA5 Alternate Function Conexión de Altavoz (control por onda cuadrada)

-DAC PA4 Puerto analógico Conexión de Altavoz (control por onda senoidal)

-USART1_RX PB7 Alternate Function Conexión a PC con puente UART-USB
  En la configuración por defecto es un
  LED de la placa, si no se usa el puerto
  serie, no es necesario cambiar la
  configuración de este pin

-USART1_TX PB6 Alternate Function Conexión a PC con puente UART-USB
  En la configuración por defecto es un
  LED de la placa, si no se usa el puerto
  serie, no es necesario cambiar la
  configuración de este pin
