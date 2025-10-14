# device_simulator.py
import streamlit as st
import paho.mqtt.client as mqtt
import time

BROKER = "broker.hivemq.com"
TOPIC_TEMP = "iot/room/temperature"
TOPIC_LIGHT = "iot/room/light"

client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
client.connect(BROKER, 1883, 60)

st.title("🧠 MQTT IoT Device Simulator")

temp = st.slider("Temperature Sensor 🌡️", min_value=10, max_value=40, value=25)
light = st.toggle("Light Switch 💡", value=False)

if st.button("Send Data"):
    client.publish(TOPIC_TEMP, temp)
    client.publish(TOPIC_LIGHT, "ON" if light else "OFF")
    st.success(f"✅ Data sent! Temp={temp}, Light={'ON' if light else 'OFF'}")
    time.sleep(1)
