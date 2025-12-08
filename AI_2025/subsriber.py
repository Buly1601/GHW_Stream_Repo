# subscriber.py
import paho.mqtt.client as mqtt

BROKER = "broker.hivemq.com"
TOPIC = "iot/room/temperature"

def on_message(client, userdata, message):
    temp = float(message.payload.decode())
    status = "Fan ON" if temp > 30 else "Fan OFF"
    print(f"Received: {temp}°C → {status}")

client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER, 1883, 60)
client.subscribe(TOPIC)
print("Listening for temperature updates...")

client.loop_forever()