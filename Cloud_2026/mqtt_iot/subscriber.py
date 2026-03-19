import paho.mqtt.client as mqtt

BROKER = "broker.hivemq.com"
TOPIC = "iot/room/temperature"

def on_message(client, userdata, message):
    """
    This function takes care of handling the messages 
    when they are sent through the broker
    """
    temperature = float(message.payload.decode())
    status = "Fan On" if temperature > 25 else "Fan Off"
    print(f"Received {temperature}°C and status is: {status}")


client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER, 1883, 60)
client.subscribe(TOPIC)
print("Listening for temperature updates...")
client.loop_forever()
