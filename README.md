# stm32-controladores-1dof
Primer repositorio del proyecto final, aqui se guardaran los controladores usados

# STM32 – Controladores PI, PID y LQR (1-DOF)

Este repositorio contendrá el firmware en STM32CubeIDE para implementar y comparar
controladores PI, PID y LQR en una planta 1-DOF (motor DC con encoder + puente H).

## Estado actual
- [ ] Cableado del sistema
- [ ] Configuración STM32CubeIDE (PWM, Encoder, UART, Timer de control)
- [ ] Control PI
- [ ] Control PID
- [ ] Control LQR
- [ ] Registro de datos por UART (CSV)

## Hardware previsto
- STM32 Nucleo-32 (modelo por confirmar)
- Motor DC con encoder incremental (PPR por confirmar)
- Puente H (modelo por confirmar)
- Batería LiPo (voltaje por confirmar)

## Documentación
Los diagramas y tablas estarán en `docs/`:
- diagrama_bloques.png
- diagrama_conexiones.png
- tabla_pines.md

## Cómo se ejecutará (plan)
1. Abrir el proyecto en `cubeide_project/` con STM32CubeIDE 1.19.0
2. Flashear con ST-Link
3. Conectar UART a PC y registrar CSV
4. Correr pruebas de escalón para cada controlador

