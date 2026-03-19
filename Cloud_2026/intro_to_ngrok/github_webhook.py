from flask import Flask, request
from plyer import notification
import threading

app = Flask(__name__)
PORT = 3000

def send_notification(title, message):
    """
    Helper function to send a notification in a separate thread.
    This stops it from blocking the web server.
    """
    try:
        notification.notify(
            title=title,
            message=message,
            app_name='GitHub Webhook',
            timeout=10
        )
    except Exception as e:
        print(f"Error sending notification: {e}")

@app.route('/github_webhook', methods=['POST'])
def webhook():
    # Get the event type from the GitHub headers
    github_event = request.headers.get('X-GitHub-Event')

    if github_event == 'pull_request':
        data = request.json
        
        # --- This is where we parse the PR data ---
        action = data.get('action')
        pr_title = data.get('pull_request', {}).get('title', 'No Title')
        user = data.get('sender', {}).get('login', 'Someone')

        # Craft a nice message
        notif_title = f"New Pull Request: {action.title()}"
        notif_message = f"{pr_title}\nby {user}"
        
        # Send the notification!
        threading.Thread(target=send_notification, args=(notif_title, notif_message)).start()
        
    elif github_event == 'ping':
        # This is GitHub testing the connection
        print("GitHub ping event received!")
        threading.Thread(target=send_notification, args=("Webhook Connected!", "GitHub is successfully connected.")).start()

    else:
        print(f"Received unhandled event: {github_event}")

    # Always send a 200 OK back to GitHub
    return 'Webhook received!', 200

if __name__ == '__main__':
    app.run(port=PORT)