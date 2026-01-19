from flask import Flask, render_template
import requests
import os

app = Flask(__name__)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/json")
def show_json():
    # Example public weather API (no key needed)
    url = "https://api.open-meteo.com/v1/forecast?latitude=52.52&longitude=13.41&current_weather=true"
    response = requests.get(url)
    data = response.json()
    return render_template("json.html", data=data)

@app.route("/image")
def show_image():
    return render_template("image.html")

@app.route("/error")
def error():
    return render_template("error.html")

if __name__ == "__main__":
    app.run(debug=True)
