# smoker-pid-control
Arduino based PID Controller with PT1000 elements to control heat in BBQ grill using a blower fan
* PWM Output 0-255 (my fan starts not below 30 in cold mode but continue to rotate until 10)
* manual controller mode
* PID based controller mode
* Fuzzy based controller mode

## Field Test
PWM Mode works very well with Weber 57 cm BBQ Grill. Only small overshots in + 1-2 ° Celsius. I use a BBQ Guru Viper Pit 12 V Fan which has no PWM, PWM is converted to linear voltage with a simple Buck converter. 
 
Fuzzy mode tends mor oscillating, but not more than -2 to 2.5 ° Celsius. 

Overall I could hold constant temperature of 110 °C over 4 hours with 1 char basket filled with some weber charcoal briquettes, and still had enough of them left for some hours. 


## TODO
* build case
* eagle circuit/pcb
* pictures and real live plot recordings
