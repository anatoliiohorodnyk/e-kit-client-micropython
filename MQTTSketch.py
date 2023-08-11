from machine import Pin, PWM
from umqtt.simple import MQTTClient
import machine
import network
import ubinascii
import time
import json

mac_address = ubinascii.hexlify(network.WLAN().config('mac'),':').decode()


rgb = (255, 255, 255)

RED = 0
GREEN = 1
BLUE = 2
pwm_pins = [18,17,16]
pwms = [PWM(Pin(pwm_pins[RED])), PWM(Pin(pwm_pins[GREEN])), PWM(Pin(pwm_pins[BLUE]))]
[pwm.freq(1000) for pwm in pwms]


MQTT_BROKER = "192.168.4.10"
CLIENT_ID = ubinascii.hexlify(machine.unique_id())

POS_TOPIC_TO_SET_REPLY = f"{mac_address}/set.to_server"
SUB_TOPIC_FROM_SET_REPLY = f"{mac_address}/set.from_server"
SUB_TOPIC_FROM_GET_REPLY = f"{mac_address}/get.from_server"
payload = {"command": "register",
           "parameters": {"name": mac_address, "device": "rpi-pico-w", "power": 25, "is_consumer": True, "state": False, "priority": 1, "blocked": False}}

# led = Pin(25, Pin.OUT)

def map_range(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def turn_off_rgb():
    pwms[RED].duty_u16(0)
    pwms[GREEN].duty_u16(0)
    pwms[BLUE].duty_u16(0)
    time.sleep(0.1)

def rgb_led():
    try:
        red, green, blue = rgb
            
        pwms[RED].duty_u16(map_range(red, 0, 255, 0, 65535))
        pwms[GREEN].duty_u16(map_range(green, 0, 255, 0, 65535))
        pwms[BLUE].duty_u16(map_range(blue, 0, 255, 0, 65535))
    except:
        pwms[RED].deinit()
        pwms[GREEN].deinit()
        pwms[BLUE].deinit()

def sub_callback(topic, msg):
    data = json.loads(msg)
    print((topic, msg))
    
    action = " ".join(str(x) for x in list(data['parameters'].keys()))
    
    if action == "is_active":
            print("Registration passed")
    
    if list(data['parameters'].values())[0] == True:
        if action == "state":
            if data['result'] == True:
                print("Device turned on")
            else:
                print("Device already turned on")
            rgb_led()
    else:
        if action == "state":
            if data['result'] == False:
                print("Device already turned off")
            else:
                print("Device turned off")
            turn_off_rgb()
    return;
    
def network_connect(ssid = "e-kit-server", password = "globallogic"):
    wifi = network.WLAN(network.STA_IF)
    wifi.active(True)
    while not wifi.isconnected():
        wifi.connect(ssid, password)
        print("No conneection yet")
        time.sleep(1)
    print("Успішно підключено до Wi-Fi")
    print("IP-адреса:", wifi.ifconfig()[0])

network_connect()

mqttClient = MQTTClient(CLIENT_ID, MQTT_BROKER, keepalive=60)
mqttClient.set_callback(sub_callback)
mqttClient.connect()
print("Connected to broker")

mqttClient.subscribe(SUB_TOPIC_FROM_SET_REPLY)
mqttClient.subscribe(SUB_TOPIC_FROM_GET_REPLY)
print("Subscribed to broker")

print(f"MQTT Publish: Topic = {POS_TOPIC_TO_SET_REPLY}, payload = {payload}")
mqttClient.publish(topic=POS_TOPIC_TO_SET_REPLY, msg=json.dumps(payload))
print("Published to broker")

while True:
    time.sleep(1)
    try:
        mqttClient.check_msg()
    except:
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

