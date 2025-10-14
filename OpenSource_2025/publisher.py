# publisher.py
import paho.mqtt.client as mqtt
import time
import random

BROKER = "broker.hivemq.com"
TOPIC = "iot/room/temperature"

client = mqtt.Client()
client.connect(BROKER, 1883, 60)

while True:
    temp = round(random.uniform(20, 35), 2)
    client.publish(TOPIC, temp)
    print(f"Published temperature: {temp}°C")
    time.sleep(2)
