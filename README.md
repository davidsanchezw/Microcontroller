<h1>Diseño de un piano simple a partir de un microprocesador</h1>
<h3>Proyecto de la asignatura de Sistemas Digitales Basados en Microprocesadores</h3>
El microprocesador empleado es el modelo STM32L152, cuya información se puede encontrar en su <a href="https://www.alldatasheet.com/view.jsp?Searchword=STM32L152&sField=2">datasheet</a>.

El siguiente diagrama es una representación del esquema de entradas y salidas de los pines del microprocesador:

<p align="center">
    <img src="https://user-images.githubusercontent.com/51442623/210570887-0f302f2f-235c-4ec9-822b-dc04f08b924d.jpg">
</p>
<h3>Diagrama de flujo</h3>
El siguiente diagrama muestra el flujo de funcionamiento según el código creado para este proyecto:
<p align="center">
    <img src="https://user-images.githubusercontent.com/51442623/210569245-7eaad396-9a85-4a5f-bfb9-9f19067d1b44.jpg">
</p>

El fichero importante modificado para este proyecto es el [main.c](https://github.com/davidsanchezw/Microcontroller/blob/master/main.c)

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
