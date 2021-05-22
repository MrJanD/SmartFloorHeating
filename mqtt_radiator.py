#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import RPi.GPIO as GPIO
import threading 
import time
import sys

class GPIO_Tuple(object):
    def __init__(self, gpio_bcm, gpio_led, inverse = False):
        self.PORT = gpio_bcm
        self.LED = gpio_led
        self.duty_cycle = 0.1
        self.frequency = 50.0
        self.inverse = inverse
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.PORT, GPIO.OUT, initial = GPIO.HIGH if self.inverse else GPIO.LOW)
        GPIO.setup(self.LED, GPIO.OUT, initial=GPIO.LOW)
        self.PWM = GPIO.PWM(self.LED, self.frequency)
        self.update_led()

    def update_led(self):
        if self.get_gpio():
            self.PWM.start(self.duty_cycle) if not self.inverse else self.PWM.stop()
        else:
            self.PWM.stop() if not self.inverse else self.PWM.start(self.duty_cycle)

    def set_gpio(self, on_off):
        GPIO.output(self.PORT, on_off if not self.inverse else not on_off)
        self.update_led()
        pin_state = " high" if self.get_gpio() else " low"
        print("GPIO" + str(self.PORT) + " set logically " + pin_state)

    def get_gpio(self):
        return GPIO.input(self.PORT)

class Power_LED(object):
    def __init__(self, gpio_bcm, inverse = False):
        self.duty_cycle = 0.1
        self.frequency = 50.0
        self.inverse = inverse
        self.LED = gpio_bcm
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.LED, GPIO.OUT, initial=GPIO.LOW)
        self.PWM = GPIO.PWM(self.LED, self.frequency)

    def on(self):
        self.PWM.start(self.duty_cycle) if not self.inverse else self.PWM.stop()

    def off(self):
        self.PWM.stop() if not self.inverse else self.PWM.start(self.duty_cycle)

class Temperature_Handler(object):
    def __init__(self):
        self.flow_path = "/sys/bus/w1/devices/28-0316a279fa82/w1_slave"
        self.return_path = "/sys/bus/w1/devices/28-0319a279267e/w1_slave"

    def get_temp(self, path):
        file = open(path)
        temperature = float(file.read().split("\n")[1].split(" ")[9][2:]) / 1000
        file.close()
        return(temperature)

    def get_temps(self):
        return self.get_temp(self.flow_path), self.get_temp(self.return_path)

class MQTT_Handler(object):
    def __init__(self):
        self.mqtt_server = "192.168.0.11"
        self.mqtt_port = 1883
        self.mqtt_sub_kitchen = "Kitchen/Radiator/set"
        self.mqtt_sub_diningroom = "Diningroom/Radiator/set"
        self.mqtt_sub_livingroom = "Livingroom/Radiator/set"
        self.mqtt_sub_pump = "Server/Radiator/Pump/set"
        self.mqtt_sub_request = "Server/Radiator/get"
        self.mqtt_pub_kitchen = "Kitchen/Radiator/state"
        self.mqtt_pub_diningroom = "Diningroom/Radiator/state"
        self.mqtt_pub_livingroom = "Livingroom/Radiator/state"
        self.mqtt_pub_pump = "Server/Radiator/Pump/state"
        self.mqtt_pub_flow = "Server/Radiator/flow/temperature"
        self.mqtt_pub_return = "Server/Radiator/return/temperature"
        self.mqtt_pub_state = "Server/Radiator/state"
        self.client = mqtt.Client()

        self.kitchen_tuple = GPIO_Tuple(24, 12, inverse = True)
        self.diningroom_tuple = GPIO_Tuple(25, 16, inverse = True)
        self.livingroom_tuple = GPIO_Tuple(8, 20, inverse = True)
        self.pump_tuple = GPIO_Tuple(15, 18, inverse = True)
        self.power_led = Power_LED(7)

        self.temperature = Temperature_Handler()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        client.subscribe(self.mqtt_sub_kitchen)
        client.subscribe(self.mqtt_sub_diningroom)
        client.subscribe(self.mqtt_sub_livingroom)
        client.subscribe(self.mqtt_sub_request)
        client.subscribe(self.mqtt_sub_pump)
        client.message_callback_add(self.mqtt_sub_kitchen, self.kitchen_radiator)
        client.message_callback_add(self.mqtt_sub_diningroom, self.diningroom_radiator)
        client.message_callback_add(self.mqtt_sub_livingroom, self.livingroom_radiator)
        client.message_callback_add(self.mqtt_sub_pump, self.pump_radiator)
        client.message_callback_add(self.mqtt_sub_request, self.radiator_state)
        client.publish(self.mqtt_pub_state, "Online", retain = True)
        self.power_led.on()

    def rollback(self):
        self.power_led.off()
        self.kitchen_tuple.set_gpio(False)
        self.diningroom_tuple.set_gpio(False)
        self.livingroom_tuple.set_gpio(False)
        GPIO.cleanup()
        # client.publish(self.mqtt_pub_kitchen, self.get_tuple_mqtt_state(self.kitchen_tuple), retain = True)
        # client.publish(self.mqtt_pub_diningroom, self.get_tuple_mqtt_state(self.diningroom_tuple), retain = True)
        # client.publish(self.mqtt_pub_livingroom, self.get_tuple_mqtt_state(self.livingroom_tuple), retain = True)

    def on_disconnect(self, client, userdata, rc):
        self.power_led.off()
        self.kitchen_tuple.set_gpio(False)
        self.diningroom_tuple.set_gpio(False)
        self.livingroom_tuple.set_gpio(False)
        GPIO.cleanup()

    def kitchen_radiator(self, client, userdata, message):
        print("kitchen_radiator: " + message.topic + " " + str(message.payload))
        self.kitchen_tuple.set_gpio("on" in str(message.payload))
        client.publish(self.mqtt_pub_kitchen, self.get_tuple_mqtt_state(self.kitchen_tuple))

    def diningroom_radiator(self, client, userdata, message):
        print("diningroom_radiator: " + message.topic + " " + str(message.payload))
        self.diningroom_tuple.set_gpio("on" in str(message.payload))
        client.publish(self.mqtt_pub_diningroom, self.get_tuple_mqtt_state(self.diningroom_tuple))

    def livingroom_radiator(self, client, userdata, message):
        print("livingroom_radiator: " + message.topic + " " + str(message.payload))
        self.livingroom_tuple.set_gpio("on" in str(message.payload))
        client.publish(self.mqtt_pub_livingroom, self.get_tuple_mqtt_state(self.livingroom_tuple))

    def pump_radiator(self, client, userdata, message):
        print("pump_radiator: " + message.topic + " " + str(message.payload))
        self.pump_tuple.set_gpio("on" in str(message.payload))
        client.publish(self.mqtt_pub_pump, self.get_tuple_mqtt_state(self.pump_tuple))

    def radiator_state(self, client, userdata, message):
        flow_temp, return_temp = self.temperature.get_temps()
        print("Flow temperature: " + str(flow_temp))
        print("Return temperature: " + str(return_temp))
        client.publish(self.mqtt_pub_flow, str(flow_temp))
        client.publish(self.mqtt_pub_return, str(return_temp))

    def get_tuple_mqtt_state(self, tuple):
        if tuple.get_gpio():
            return "on" if not tuple.inverse else "off"
        else:
            return "off" if not tuple.inverse else "on"

    def run_temperature_emitter(self):
        while True:
            flow_temp, return_temp = self.temperature.get_temps()
            print("Flow temperature: " + str(flow_temp))
            print("Return temperature: " + str(return_temp))
            self.client.publish(self.mqtt_pub_flow, str(flow_temp))
            self.client.publish(self.mqtt_pub_return, str(return_temp))
            time.sleep(300)

    def run_actor(self):
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.will_set(self.mqtt_pub_state, "Offline", retain = True)
        self.client.connect(self.mqtt_server, self.mqtt_port, 60)
        self.client.loop_forever()

    def run(self):
        try:
            emitter_thread = threading.Thread(target=self.run_temperature_emitter)
            emitter_thread.daemon = True
            emitter_thread.start()
            self.run_actor()
        except (KeyboardInterrupt):
            print("Shutting down floor heating system control due to user request!")
            self.rollback()
            #emitter_thread.join()
            sys.exit()
        except (SystemExit):
            print("Shutting down floor heating system control due to a system exit!")
            self.rollback()
            #emitter_thread.join()
            sys.exit()

mqtt_handler = MQTT_Handler()
mqtt_handler.run()
 
