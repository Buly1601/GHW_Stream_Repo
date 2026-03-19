from flask import Flask, request, jsonify
from plyer import notification

app = Flask(__name__)
PORT = 3000


@app.route('/')
def index():
    return 'Your webhook server is running! Point your webhooks to /webhook'


@app.route('/webhook', methods=['POST'])
def webhook():
    print('--- WEBHOOK RECEIVED! ---')

    # Get the JSON payload from the request
    data = request.json 

    # Log the headers (optional, but useful for debugging)
    print('Headers:', request.headers)

    # Log the body (this is the payload from GitHub, Stripe, etc.)
    print('Body:', data)

    # Send an OK response to let the service know you received it
    return 'Webhook received successfully!', 200


@app.route('/notification_webhook', methods=['POST'])
def notification_webhook():
    data = request.json
    

    # This specific example works great with GitHub push events
    try:
        user = data.get("user", "No User")
        message = data.get("message", "No Message")
        message = f"{user} says: {message}!"
    except:
        message = "Webhook data received!"

    # Trigger the native desktop notification
    notification.notify(
        title='Webhook Alert!',
        message=message,
        app_name='ngrok Demo',
        timeout=10
    )

    return 'Notification sent!', 200


if __name__ == '__main__':
    app.run(port=PORT)