import paho.mqtt.client as mqtt

broker = "broker.hivemq.com"
topic = "home/livingroom/light"

client = mqtt.Client()
client.connect(broker, 1883, 60)

print("Connected to MQTT Broker!")
print("Type 'on' or 'off' to control the light. Type 'exit' to quit.\n")

while True:
    cmd = input("Command: ").strip().lower()
    if cmd == "exit":
        break
    elif cmd in ["on", "off"]:
        client.publish(topic, cmd.upper())
        print(f"Sent: {cmd.upper()}")
    else:
        print("Invalid command, try 'on' or 'off'")
