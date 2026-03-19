import paho.mqtt.client as mqtt

BROKER = "broker.hivemq.com"
TOPIC = "iot/garden/light"

client = mqtt.Client()
client.connect(BROKER, 1883, 60)

print("Type 'ON' or 'OFF' to cotnrol the light, type 'EXIT' to quit")
while True:
    cmd = input("Command: ".strip().upper())
    if cmd == "EXIT":
        break
    elif cmd in ["ON", "OFF"]:
        client.publish(TOPIC, cmd)
    else:
        print("Invalid Command!!!")
