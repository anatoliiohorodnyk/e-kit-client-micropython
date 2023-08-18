from machine import Pin, PWM, I2C
from pico_i2c_lcd import I2cLcd
from umqtt.simple import MQTTClient
import machine
import network
import ubinascii
import time
import utime
import json

power_btn = Pin(21, Pin.IN)

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)

I2C_ADDR = i2c.scan()[0]

lcd = I2cLcd(i2c, I2C_ADDR, 2, 16)

lcd.backlight_off()

mac_address = ubinascii.hexlify(network.WLAN().config('mac'),':').decode()

rgb = (255, 255, 255)

RED = 0
GREEN = 1
BLUE = 2
pwm_pins = [18,17,16]
pwms = [PWM(Pin(pwm_pins[RED])), PWM(Pin(pwm_pins[GREEN])), PWM(Pin(pwm_pins[BLUE]))]
[pwm.freq(1000) for pwm in pwms]


MQTT_BROKER = "10.42.0.1"
CLIENT_ID = ubinascii.hexlify(machine.unique_id())

POS_TOPIC_TO_SET_REPLY = f"{mac_address}/set.to_server"
SUB_TOPIC_FROM_SET_REPLY = f"{mac_address}/set.from_server"
SUB_TOPIC_FROM_GET_REPLY = f"{mac_address}/get.from_server"
payload = {"command": "register",
           "parameters": {"name": mac_address, "device": "rpi-pico-w", "power": 25, "is_consumer": True, "state": False, "priority": 1, "blocked": False}}
payload_power_on = {"command": "power_on",
           "parameters": {"is_active": 1, "state": 1, "priority": 1, "blocked": 0}}
payload_power_off = {"command": "register",
           "parameters": {"is_active": 1, "state": 1, "priority": 1, "blocked": 0}}
# led = Pin(25, Pin.OUT)

def map_range(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def turn_off_rgb():
    pwms[RED].duty_u16(0)
    pwms[GREEN].duty_u16(0)
    pwms[BLUE].duty_u16(0)
    time.sleep(0.1)

def rgb_led_high(red, green, blue):
    light = 0
    while light < 65535:
        try:
            pwms[RED].duty_u16(map_range(red, 0, 255, 0, light))
            pwms[GREEN].duty_u16(map_range(green, 0, 255, 0, light))
            pwms[BLUE].duty_u16(map_range(blue, 0, 255, 0, light))
        except:
            pwms[RED].deinit()
            pwms[GREEN].deinit()
            pwms[BLUE].deinit()
        light += 10
        utime.sleep(0.000001)
        
def rgb_led_low(red, green, blue):
    light = 65535
    while light > 0:
        try:
            pwms[RED].duty_u16(map_range(red, 0, 255, 0, light))
            pwms[GREEN].duty_u16(map_range(green, 0, 255, 0, light))
            pwms[BLUE].duty_u16(map_range(blue, 0, 255, 0, light))
        except:
            pwms[RED].deinit()
            pwms[GREEN].deinit()
            pwms[BLUE].deinit()
        light -= 10
        utime.sleep(0.000001)

def sub_callback(topic, msg):
    data = json.loads(msg)
    
    print((topic, msg))
    
    if len(data['parameters']) == 0:
        if data['result'] == False:
                print("Device already turned off")
                lcd.backlight_on()
                lcd.clear()
                lcd.putstr("Device already turned off")
                rgb_led_high(0, 0, 205)
                time.sleep(2)
                rgb_led_low(0, 0, 205)
                lcd.clear()
                lcd.backlight_off()
    
    action = " ".join(str(x) for x in list(data['parameters'].keys()))
    
    if action == "is_active":
            print("Registration passed")
            lcd.backlight_on()
            lcd.clear()
            lcd.putstr("Registration passed")
            time.sleep(4)
            lcd.clear()
            lcd.backlight_off()
            return;
    
    if list(data['parameters'].values())[0] == True:
        print("HERE 1")
        if action == "state":
            if data['result'] == True:
                print("Device turned on")
                lcd.backlight_on()
                lcd.clear()
                lcd.putstr("Device turned on")
                rgb_led_high(0, 128, 0)
            else:
                print("Device already turned on")
                lcd.clear()
                lcd.putstr("Device already turned on")
                rgb_led_low(0, 128, 0)
                rgb_led_high(255, 234, 0)
                time.sleep(5)
                rgb_led_low(255, 234, 0)
                rgb_led_high(0, 128, 0)
                lcd.clear()
                lcd.putstr("Device turned on")
                
    else:
        print("HERE 2")
        if action == "state":
            if data['result'] == True:
                print("Device turned off")
                lcd.clear()
                lcd.putstr("Device turned off")
                rgb_led_low(0, 128, 0)
                rgb_led_high(75, 0, 130)
                time.sleep(3)
                rgb_led_low(75, 0, 130)
                lcd.clear()
                lcd.backlight_off()
    return;
    
def network_connect(ssid = "rpi-pow", password = "rpi-Power"):
    wifi = network.WLAN(network.STA_IF)
    wifi.active(True)
    while not wifi.isconnected():
        wifi.connect(ssid, password)
        print("No conneection yet")
        time.sleep(1)
    print("Успішно підключено до Wi-Fi")
    print("IP-адреса:", wifi.ifconfig()[0])

network_connect()

def connect_to_mqtt():
    mqttClient = MQTTClient(CLIENT_ID, MQTT_BROKER, keepalive=60)
    mqttClient.set_callback(sub_callback)
    try:
        mqttClient.connect() #try catch
    except:
        pass
    print("Connected to broker")
    mqttClient.subscribe(SUB_TOPIC_FROM_SET_REPLY)
    mqttClient.subscribe(SUB_TOPIC_FROM_GET_REPLY)
    print("Subscribed to broker")
    return mqttClient;


mqttClient = connect_to_mqtt()

print(f"MQTT Publish: Topic = {POS_TOPIC_TO_SET_REPLY}, payload = {payload}")
mqttClient.publish(topic=POS_TOPIC_TO_SET_REPLY, msg=json.dumps(payload))
print("Published to broker")


time.sleep(1)


while True:
    try:
        mqttClient.check_msg()
    except:
        mqttClient = connect_to_mqtt()
    if power_btn.value() == True:
        print("power on")
        time.sleep(1)
        mqttClient.publish(topic=POS_TOPIC_TO_SET_REPLY, msg=json.dumps(payload_power_on))
        print("power on publish")
    else:
        pass




# potentiometer = ADC(Pin(12))
# potentiometer.atten(ADC.ATTN_11DB)
# 
# while True:
#     potentiometer_val = potentiometer.read()
#     print(potentiometer_val)
#     time.sleep(0.2)
#      led.value(1)
#      time.sleep(1)
#      led.value(0)
#      time.sleep(1)

# import network
# import time
# 
# ssid = "coolHacker"
# password = "Hacker228"
# 
# wifi = network.WLAN(network.STA_IF)
# 
# wifi.active(True)
# 
# # wifi.connect(ssid, password)
# 
# while not wifi.isconnected():
#     wifi.connect(ssid, password)
#     print("")
#     time.sleep(1)
# 
# print("Успішно підключено до Wi-Fi")
# print("IP-адреса:", wifi.ifconfig()[0])


