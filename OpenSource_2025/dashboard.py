import streamlit as st
import paho.mqtt.client as mqtt
from collections import deque
import time

BROKER = "broker.hivemq.com"
TOPIC = "iot/room/temperature"

temps = deque(maxlen=50)
st.title("🌡️ IoT Temperature Dashboard")
chart = st.line_chart([])

def on_message(client, userdata, message):
    temp = float(message.payload.decode())
    temps.append(temp)

client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER, 1883, 60)
client.subscribe(TOPIC)
client.loop_start()

placeholder = st.empty()

# Main Streamlit update loop
while True:
    if temps:
        placeholder.line_chart(list(temps))
    time.sleep(1)
