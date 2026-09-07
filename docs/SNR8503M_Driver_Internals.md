# SNR8503M BLDC Driver - Internal Architecture & Physics Reference

Este documento detalla la ingeniería interna, física, algoritmos de control y parámetros de hardware del controlador **SNR8503M** (núcleo ARM Cortex-M0 de Snaner Semiconductor), extraídos e inspeccionados directamente del código fuente oficial del SDK (`SNR8503M_BLDC_SNLS_LIB_V33_RELEASE_230418`).

---

## 1. Parámetros de Hardware y Potencia

| Parámetro | Valor de Hardware / SDK | Descripción |
| :--- | :---: | :--- |
| **Reloj Principal MCU (`MCU_MCLK`)** | `48 MHz` | Frecuencia del reloj interno del microcontrolador |
| **Frecuencia PWM Inversor (`PWM_FREQ`)** | `16 kHz` | Frecuencia de conmutación de los MOSFETs |
| **Período PWM (`PWM_PERIOD`)** | `1500 ticks` | Resolución de modulación PWM a 16 kHz |
| **Duty Cycle Mínimo (`MIN_PWM_DUTY`)** | `150 ticks (10%)` | Límite inferior para evitar apagado errático |
| **Duty Cycle Máximo (`MAX_PWM_DUTY`)** | `1500 ticks (100%)` | Saturación máxima de modulación |
| **Resistencia Shunt (`RSHUNT`)** | `0.004 Ω (4 mΩ)` | Shunt de corriente de bus DC de baja inductancia |
| **Ganancia Amplificador OPA (`AMPLIFICATION_GAIN`)** | `18.18 V/V` | Circuito diferencial $200\text{k}\Omega / (10\text{k}\Omega + 1\text{k}\Omega)$ |
| **Escala de Corriente Máxima Medible** | `49.5 A` | $3.6\text{V} / (0.004\Omega \times 18.18)$ |
| **Divisor de Voltaje DC (`VOLTAGE_SHUNT_RATIO`)** | `1 / 34 (0.02941)` | Circuito $1.0\text{k}\Omega / (33\text{k}\Omega + 1.0\text{k}\Omega)$ |
| **Rango de Tensión DC de Operación** | `6.0 V a 78.0 V` | Rango de alimentación del bus |

---

## 2. Secuencia de Arranque y Transición de Estados

La máquina de estados principal (`m1m1_app_mainstate_t`) y sub-estados de marcha (`m1m1_run_substate_t`) gobiernan el arranque del motor BLDC sensorless:

```
[ M1_MainState_Stop ]
         |
         v (Comando de marcha SpeedCMD > 0 o VSP > 300)
[ M1_RunState_Calib ] ----> Calibra offset de ADC de corriente (512 muestras)
         |
         v
[ M1_RunState_Ready ] ----> Pre-carga condensadores de bootstrap (100 ms)
         |
         v
[ M1_RunState_Align ] ----> Inyección de rotor / alineación en fase
         |
         v
[ M1_RunState_Startup ] --> Arrastre en lazo abierto (Open-Loop Drag):
         |                  - Duty PWM inicial: 10% (150 ticks)
         |                  - Tiempo de arrastre: 100 ms (STARTUP_DRAG_TIME)
         v
[ M1_RunState_Spin ] -----> Lazo cerrado (Closed-Loop BEMF Zero-Crossing)
         |
         v (Comando SpeedCMD == 0 o Falla)
[ M1_RunState_Brake / Stop ]
```

---

## 3. Algoritmo de Medición y Conmutación FOC / 6 Pasos

1. **Base de Tiempo de Conmutación (`TIMER1_TIMEBASE`)**:
   El temporizador de conmutación corre con un tick base de **$6\ \mu\text{s}$**.
2. **Cálculo Recíproco de Velocidad en Silicio**:
   El controlador acumula el período de las 6 fases eléctricas consecutivas:
   $$T_{\text{elec}} = \sum_{i=0}^{5} \text{MotorElePhaseValue}[i]$$
   Y calcula la velocidad mecánica en RPM mediante división recíproca exacta:
   $$\text{RPM} = \frac{\text{MOTOR\_SPEED\_X}}{T_{\text{elec}}}$$
   Donde:
   $$\text{MOTOR\_SPEED\_X} = \frac{60 \times 10^6}{\text{Poles} \times \text{TIMER1\_TIMEBASE}}$$
   Para $P = 7$ pares de polos (14 polos) y base de $6\ \mu\text{s}$:
   $$\text{MOTOR\_SPEED\_X} = \frac{60\,000\,000}{14 \times 6} \approx 714\,285$$

---

## 4. Lazos de Control PI y Límites de Aceleración

### A. Regulador de Velocidad (PI Loop)
- **Frecuencia de Actualización**: Cada $2\text{ ms}$ (`SPEED_PI_PRC = 2`).
- **Constantes de Ganancia (formato Q15)**:
  - $K_p = \text{Q15}(0.05) = 1638$
  - $K_i = \text{Q15}(0.01) = 328$
  - $K_c = \text{Q15}(0.5) = 16384$ (Anti-windup clamping)

### B. Rampas de Aceleración y Desaceleración
- **Aceleración Máxima (`SPEED_ACC_MS`)**: $5.0\text{ RPM/ms} \implies 5\,000\text{ RPM/s}$.
- **Desaceleración Máxima (`SPEED_DEC_MS`)**: $5.0\text{ RPM/ms} \implies 5\,000\text{ RPM/s}$.
- Esto garantiza que el comando hacia el inversor nunca sufra un escalón abrupto que dispare sobrecorriente.

### C. Límite de Corriente Dinámico (`PiCurrentLimitPWMDuty`)
- Si la corriente de bus excede los $18\text{ A}$ (`MAX_BUS_CURRENT_SETTINT`), el lazo PI de corriente satura el Duty Cycle PWM del inversor para mantener la operación en zona segura sin apagar el motor.

---

## 5. Umbrales de Protección y Códigos de Falla

| Falla | Máscara Bit | Umbral Físico | Tiempo de Disparo | Acción del Driver |
| :--- | :---: | :---: | :---: | :--- |
| **Cortocircuito (`SHORT_ERROR`)** | `0x0001` | $80\text{ A}$ | Inmediato (Hardware DAC Comp) | Apagado instantáneo de PWM |
| **Subvoltaje (`LOW_VOL_ERROR`)** | `0x0002` | $< 5.5\text{ V}$ ($500\text{ ms}$) o $< 6.0\text{ V}$ ($50\text{ ms}$) | $50\text{ ms} / 500\text{ ms}$ | Detención por batería baja |
| **Sobrevoltaje (`HIG_VOL_ERROR`)** | `0x0004` | $> 78.0\text{ V}$ | $10\text{ ms}$ | Corte por sobretensión de frenado |
| **Rotor Bloqueado (`BLOCK_ERROR`)** | `0x0008` | Falla de cruce por cero en $50\text{ ms}$ | $50\text{ ms}$ (`MOTOR_BLOCK_DETECT_CNT`) | Detención por stall |
| **Offset ADC (`DC_OFFSET_ERROR`)** | `0x0010` | Offset corriente fuera de rango | Calibración inicial | Bloqueo de arranque |
| **Sobretemperatura MOS (`MOS_OVER_ERROR`)** | `0x0020` | $> 95^\circ\text{C}$ ($R_{\text{NTC}} < 1.0\text{k}\Omega$) | $500\text{ ms}$ | Corte térmico (Recupera a $60^\circ\text{C}$) |
| **Baja Temp MOS (`MOS_LOW_ERROR`)** | `0x0040` | Sensor NTC desconectado | $500\text{ ms}$ | Falla de sensado |
| **Sobrecorriente Nivel 1 (`OVER_LOAD_ERROR`)** | `0x0200` | $> 20.0\text{ A}$ | $1000\text{ ms}$ | Detención por sobrecarga sostenida |
| **Sobrecorriente Nivel 2 (`OVER_LOAD_ERROR`)** | `0x0200` | $> 21.0\text{ A}$ | $200\text{ ms}$ | Detención rápida |
| **Pérdida de Fase (`PHASE_DROP_ERROR`)** | `0x0400` | Desconexión de cable de fase | Ciclo de conmutación | Detención |
| **Falla en MOSFETs (`MOSFET_ERROR`)** | `0x0800` | Corto interno detectado en autoprueba | Arranque | Bloqueo total |
