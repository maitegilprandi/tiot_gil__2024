# tiot_gil__2024

**Breve descripción**
Código del proyecto de TioT 2024

Este proyecto contiene una aplicación básica para la transmisión de datos desde el dispositivo Periférico al Central mediante protocolo BLE. El dispositivo Periférico en donde se corre el código es el microcontrolador MSP430F5529LP, el cual se comunica vía SPI con la placa Proteus-III-SPI. El dispositivo Central se conforma por la PC con diversos softwares corriendo, y un USB Dongle que contiene un módulo BLE Proteus-III. 

Se va a caracterizar el consumo básico de esta aplicación mediante el uso de la placa Power Profiler Kit II de Nordic Semiconductos. Se van a realizar cambios en los parámetros del sistema para obtener distintos escenarios y comparar su consumo.

Los software a utilizar son:
- Code Composer Studio v.12.1.0, de Texas Instruments: para programar y debug el microcontrolador MSP430.
- Smart Commander v1.3.3.0, de Wurth Elektronik: para configurar el módulo BLE del USB Stick, y para la transmisión y recepeción de datos de la conexión establecida entre los módulos BLE.
- Power Profiler, de Nordic Semiconductor: para la configuración y muestreo de la corriente eléctrica del sistema conectado a la PPKII. Este software corre dentro de nRF Connect for Desktop v5.0.1.
- Logic 2 v2.4.14, de Saleae: para configurar y visualizar las entradas digital y analógicas del analizador lógico Logic Pro 8.

El Central va a transmitir los datos: 0xABCD. El Periférico va a recibir datos y si coincide con este valor predeterminado, va a responder con una secuencia de datos que va a corresponder a un contador de 0 a 1000, y de 1000 a 0. Esto genera un flujo considerable de datos desde el Periférico al Central, y es sencillo de verificar la correcta recepción de los datos desde la PC, desde el software Smart Commander (software que configura y comanda al USB stick).

El código espeja la máquina de estados de los estados definidos en la placa Proteus-III. Los estados que se utilizan para este proyecto son: SLEEP, IDLE y CONNECTED. Se utilizan 2 timers, independientes y configurables que permiten transicionar entre estados. El timer T_idle: tiempo de permanencia en el estado IDLE para irse a SLEEP. Y el timer T_connected, tiempo de permanencia en el estado CONNECTED, para luego irse a IDLE.

El archivo ¨interface_ble¨ contiene todas las definiciones, variables y funciones que se utilizan para la interfaz entre el microcontrolador MSP430 y la Proteus-III-SPI. Esto incluye las funciones de transmisión de comandos, de lectura de comandos, de transmisión de datos, y las ISR utilizadas para el manejo de los Timers. El archivo main hace una lectura constante por la máquina de estados, chequea el estado actual y ejecuta las funciones correspondientes según si se dispara determinado evento.

Para mayor detalle de cómo es el funcionamiento del código y el hardware se utiliza para programar esta aplicación, se puede leer el Reporte técnico "Informe_TioT_Proyecto_Maite_Gil___2024".
