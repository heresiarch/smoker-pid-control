# smoker-pid-control
Arduino based PID Controller with PT1000 elements to control heat in BBQ grill using a blower fan
* PWM Output 0-255 (my fan starts not below 30 in cold mode but continue to rotate until 10 if already rotates)
* Manual controller mode
* PID based controller mode
* Fuzzy based controller mode
* OLED Display
* Rotary Encoder user interface

## Field Test
PID Mode works very well with Weber 57 cm BBQ Grill. Only small overshots in the beginning. I use a BBQ Guru Viper Pit 12 V Fan which has no PWM, PWM is converted to linear voltage with a simple Buck converter. PID Values are very much dependend on the type of BBQ Grill. I have a hard cut off rule above 1.0 °C which counters some overshots in the beginning. Autotune experiments are a good waste of time IMHO, better is to use PID values based on experiments. 
 
Fuzzy mode is very stable and gets good steady state for setpoint temperature if top air vent of Weber Grill is closed by around 75%. Fuzzy mode is controlled by Fuzzyset based on temperature difference. You should change rules sets based on your setup. Fuzzy update rate is 5 seconds. I derived a simple fuzzy set from the much more sophisticated example from the link below, which has unfortunately too many rules for my Arduino Micro memory: https://github.com/rvt/bbq-controller/blob/master/lib/bbq/bbqfanonly.cpp.

Overall I could hold constant temperature of 110 °C over 4 hours with 1 char basket filled with some weber charcoal briquettes, and still had enough of them left for some hours. With 2 char baskets starting on each side with 2-3 hot coals in ignition cord style and with grill fully meat loaded I got superior results in temperature stability with the Fuzzy Sets defined. The best way is to run in Manual Mode in the pre-heat phase and switch afterwards to Fuzzy or PID Mode.

## TODO
* build case
* eagle circuit/pcb
* pictures and real live plot recordings
* implement meat probe
* ~implement count down~

## References and similar Projects
* [Smokerduino on Instructables](https://www.instructables.com/id/Smokerduino/)
* [Fuzzy based BBQ Controler Project](https://github.com/rvt/bbq-controller)
* [Small OLED Library ss_oled](https://github.com/bitbank2/ss_oled)
* [Another small OLED Library SSD1306Ascii](https://github.com/greiman/SSD1306Ascii)
