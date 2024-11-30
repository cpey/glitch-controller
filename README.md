# Glitch Controller

Controller firmware for the STM32F072B-DISCO board.

The waveform below shows the shortest glitch achieved by driving the DAC of the [power-glitcher](https://github.com/cpey/power-glitcher) board:

![Waveform showing a voltage glitch driving the DAC of the power-glitcher board.](https://github.com/cpey/glitch-controller/blob/main/images/pg_dac_glitch.png)

The following waveform shows the glitch obtained when using the power-glitcher in on/off mode, bypassing the DAC:
![Waveform showing a voltage glitch bypassing the DAC of the power-glitcher board.](https://github.com/cpey/glitch-controller/blob/main/images/pg_sig_glitch.png)
