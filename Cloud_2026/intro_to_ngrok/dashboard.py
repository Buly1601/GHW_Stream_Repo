from flask import Flask, request, render_template
from flask_socketio import SocketIO, emit

app = Flask(__name__)
# A secret key is needed for SocketIO
app.config['SECRET_KEY'] = 'your-secret-key!' 
# Initialize SocketIO
socketio = SocketIO(app)


@app.route('/')
def index():
    """
    Serves the main dashboard page.
    """

    # Renders the HTML file from the 'templates' folder
    return render_template('dashboard.html')


@app.route('/github_webhook', methods=['POST'])
def handle_github():
    """
    Catches the GitHub webhook.
    """

    data = request.json
    event_type = request.headers.get('X-GitHub-Event', 'unknown')

    message = "GitHub Event: Unknown" # Default
    
    # Let's handle the Pull Request event we made!
    if event_type == 'pull_request':
        action = data.get('action')
        user = data.get('sender', {}).get('login', 'Someone')
        pr_title = data.get('pull_request', {}).get('title', 'No Title')
        message = f"PR {action.title()} by {user}: {pr_title}"

    # Send the 'message' to every browser connected on the 'new_event' channel.
    socketio.emit('new_event', {'data': message})

    return 'GitHub OK', 200


@app.route('/stripe-webhook', methods=['POST'])
def handle_stripe():
    """
    Catches a fake Stripe webhook.
    """

    data = request.json
    event_type = data.get('type')

    message = "Stripe Event: Unknown" # Default
    
    # Example: A 'charge.succeeded' event
    if event_type == 'charge.succeeded':
        amount = data.get('amount', 0) / 100 # Stripe uses cents
        customer = data.get('customer_email', 'customer@example.com')
        message = f"Sale! ${amount:.2f} from {customer}"

    # Send this event to the dashboard too!
    socketio.emit('new_event', {'data': message})
    
    return 'Stripe OK', 200


if __name__ == '__main__':
    # socketio.run() is special, it starts a server that supports WebSockets
    socketio.run(app, port=3000, debug=True)