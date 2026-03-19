import streamlit as st
import paho.mqtt.client as mqtt
from collections import deque
import time

BROKER = "broker.hivemq.com"
TOPIC = "iot/room/temperature"

temperatures = deque(maxlen=50)
st.title("IoT Temperature Dashboard")
chart = st.line_chart([])

def on_message(client, userdata, message):
    temperature_message = float(message.payload.decode())
    temperatures.append(temperature_message)

client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER, 1883, 60)
client.subscribe(TOPIC)
client.loop_start()

placeholder = st.empty()

# main streamlit loop
while True:
    if temperatures:
        placeholder.line_chart(list(temperatures))
    time.sleep(1)