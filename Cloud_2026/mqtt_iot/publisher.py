import paho.mqtt.client as mqtt
import time
import random

BROKER = "broker.hivemq.com"
TOPIC = "iot/room/temperature"

client = mqtt.Client()
client.connect(BROKER, 1883, 60)

while True:
    temperature = round(random.uniform(20, 22), 2)
    client.publish(TOPIC, temperature)
    print(f"Published temperature: {temperature}°C")
    time.sleep(2)