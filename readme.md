# Quadcopter STM32 project

Calculate attitudes of robot.
Read GY-86 MPU6050, HMC5883L, MS5611.
[roll, pitch, yaw, respective rates, altitude ].
# Pinouts
## GY86: PB6, PB7, PC13
## HCSR 04: 
## PPM
PE1
## Motors
PE9,PE11,PE13,PE14

# Copter modes.
+ Stabilize.
+ Altitude Hold.
+ Auto Landing.

# Copter Configuration
## IMU behaviors
+ yaw clockwise -> yaw increase
+ roll so quad moves to right -> roll decreases
+ pitch so quad moves to front -> pitch increases
## RC

+ channels[0]: elev ( nut ben phai len xuong ).  bottom: 1100
+ channels[1]: aile. leftmost: 1100
+ channels[3]: rudder. rightmost: 1100
+ channels[2]: throttle. bottom: 1100
+ channels[4]: GEAR switch. bottom: 1100
+ channels[5]: MIX switch. bottom: 1100
+ channels[6]: AUX2. increases clockwise

## Quadcopter control confirmed.
### RC channels
+ ELEV increases -> Motor 1 and Motor 2 increase.
+ AILE increases -> Motor 0 and Motor 1 increase
### quad behavior.
+ pitch the head down -> Motor 0,3 must increase to main tain zero setpoint. -> check ok
+ roll so it tends to move to left -> Motor 0,1 must increase -> check ok

# To dos.
+ ~~Implement yaw PID -> ok~~
+ Flight mode selection added, but what happens when in arm state, and switch flight mode?? -> need considering. ( For example from mode Stabilize to Alt Hold)
+ Check before going to while loop, rate RPY (and other data ) is good data? (mean square root< k), sometimes when quad at rest it has values>1.0 -> bad
+ Lora Telemetry
+ EEPROM
https://hshop.vn/products/mach-eeprom-24c256-giao-tiep-i2c
+ SD card. -> Can be read by computers. 
+ Indicator LEDs features.
+ Display monitor.
+ Buzzer
https://hshop.vn/products/coi-buzzer-bao-dong-3-24vdc
+ Altitude measurement, combination between SONAR and BAROMETER, smooth transition between the two.
+ Xem lại mấy cái đo thời gian vì nó chưa bao gồm trường hợp bị tràn tick count.

+ Problem with yaw readings when run with 4 escs running. -> HMC5883 is affected by motors magnetic field
+ Fix initial `alt_setpoint` when switch from Stabilize to Althold. ( a kind of interrupt function when we switch modes)
# Mount STM32 on the frame. Hardware.
+ PCB. Tự thiêt kế mạch, có các jumper cắm, ý tưởng như Pixhawk.
## Doable.
DIY PCB. Tự thiết kế mạch, có header cắm sẵn với stm32, một bên cho stm32 ( cắm phía trên ). Đường power thì mở rộng từ pin 5V làm thêm các header cắm vào để cho các module khác lấy nguồn. 
+ Phần module nguốn cấp cho STM32 lấy từ BEC của pin Phantom chính.
+ Thiết kế dự phòng để có thể mở rộng dễ dàng.
+ Chú ý cần thiết thì làm chân cắm chắc.
+ Vì thiết kế PCB nên có thể hàn được cho chắc các chân cắm biến, vì cảm biến thì có thể thay chân cắm hàn lại được.
+ Để phòng trường hợp không tháo ra vào được, thì ta thiết kế theo kiểu ... Ví dụ dễ hiểu.
# Preferences.

+ Arducopter

+ LaunchPadFlightController.
Keil uVision project. rate PID design.
Build OK

+ MahoRotorF4-Discovery
STM32-based project.
GY-86
