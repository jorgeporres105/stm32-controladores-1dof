\# Tabla de pines – STM32 NUCLEO-L432KC + L298N + Motor con encoder



\## Motor DC (potencia)

\- M1 → L298N OUT1 (Motor A)

\- M2 → L298N OUT2 (Motor A)



\## L298N (control Motor A)

\- ENA (PWM) → STM32 PA9  (TIM1\_CH2)

\- IN1 (DIR) → STM32 PB5  (GPIO Output)

\- IN2 (DIR) → STM32 PB4  (GPIO Output)



Notas:

\- Quitar jumper ENA para usar PWM desde STM32.

\- VIN y GND del L298N van a la batería (potencia).



\## Encoder integrado

\- C1 (Canal A) → STM32 PA5 (TIM2\_CH1)

\- C2 (Canal B) → STM32 PA1 (TIM2\_CH2)

\- VCC → 3V3

\- GND → GND común



\## UART a PC (por USB / ST-Link)

\- USART2\_TX → STM32 PA2

\- USART2\_RX → STM32 PA3

\- Baudrate: 115200



\## Alimentación y tierras

\- STM32: alimentado por USB

\- L298N VIN: batería LiPo (+)

\- L298N GND: batería LiPo (−)

\- GND común: STM32 GND ↔ L298N GND ↔ Encoder GND



\## V lógico (5V) del L298N

\- No conectar el pin 5V del L298N al STM32.

\- Si el jumper 5V-EN está puesto, el L298N se autoalimenta lógicamente desde VIN.

\- Si el jumper 5V-EN está quitado, el L298N requiere 5V externos para su lógica (a definir según módulo).

