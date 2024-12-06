from pyngrok import ngrok
import json
import os
import dotenv

dotenv.load_dotenv()

def setup_tunnel(port=8001):
    """
    Set up a persistent ngrok tunnel for the robot server
    Returns the public URL that can be used to access the server
    """
    try:
        # Configure ngrok to create a persistent tunnel
        tunnel_config = ngrok.connect(port, "http", options={
            "bind_tls": True,  # Enable HTTPS
        })
        
        tunnel_url = tunnel_config.public_url
        print(f"Tunnel established at: {tunnel_url}")
        print("Add this URL to your Streamlit secrets.toml file as:")
        print(f"ROBOT_URL = '{tunnel_url}'")
        
        return tunnel_url
        
    except Exception as e:
        print(f"Error setting up tunnel: {e}")
        return None

if __name__ == "__main__":
    # Set your ngrok auth token
    ngrok.set_auth_token(os.getenv('NGROK_AUTH_TOKEN'))
    # Setup the tunnel
    tunnel_url = setup_tunnel()
    
    if tunnel_url:
        print("Keep this script running to maintain the tunnel")
        try:
            # Keep the tunnel alive
            ngrok_process = ngrok.get_ngrok_process()
            ngrok_process.proc.wait()
        except KeyboardInterrupt:
            print("Shutting down tunnel...")
            ngrok.kill()