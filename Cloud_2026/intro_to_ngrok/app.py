from flask import Flask, request, jsonify, render_template, Response
from datetime import datetime
import json
import time
import requests as pyrequests

app = Flask(__name__)

requests_log = []

# ====== GEOLOCATION ======
def get_location(ip):
    try:
        res = pyrequests.get(
            f"http://ip-api.com/json/{ip}?fields=country,city,lat,lon",
            timeout=2
        )
        data = res.json()
        return {
            "city": data.get("city", ""),
            "country": data.get("country", ""),
            "lat": data.get("lat", 0),
            "lon": data.get("lon", 0)
        }
    except:
        return {
            "city": "Unknown",
            "country": "",
            "lat": 0,
            "lon": 0
        }

# ====== REAL-TIME STREAM ======
def event_stream():
    last_len = 0
    while True:
        if len(requests_log) != last_len:
            last_len = len(requests_log)
            yield f"data: {json.dumps(requests_log[:20])}\n\n"
        time.sleep(1)

@app.route("/stream")
def stream():
    return Response(event_stream(), mimetype="text/event-stream")

# ====== INSPECT ENDPOINT ======
@app.route("/inspect", methods=["GET", "POST", "PUT", "DELETE"])
def inspect():
    ip = request.headers.get("X-Forwarded-For", request.remote_addr)

    loc = get_location(ip)

    data = {
        "time": datetime.now().strftime("%H:%M:%S"),
        "ip": ip,
        "location": f"{loc['city']}, {loc['country']}",
        "lat": loc["lat"],
        "lon": loc["lon"],
        "method": request.method,
        "path": request.path,
        "headers": dict(request.headers),
        "body": request.get_data(as_text=True)
    }

    requests_log.insert(0, data)

    return jsonify({"status": "received"})

@app.route("/")
def dashboard():
    return render_template("index.html")

# ====== RUN ======
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
