from pyngrok import ngrok
import json
import os
from dotenv import load_dotenv

load_dotenv()

def setup_tunnel(port=8001):
    """
    Set up an ngrok tunnel for the robot server
    Returns the public URL that can be used to access the server
    """
    try:
        # Start ngrok tunnel
        public_url = ngrok.connect(port)
        
        # Get the public URL
        tunnels = ngrok.get_tunnels()
        if tunnels:
            tunnel_url = tunnels[0].public_url
            
            # Save the URL to a file that can be read by the Streamlit app
            with open('tunnel_url.json', 'w') as f:
                json.dump({'url': tunnel_url}, f)
                
            print(f"Tunnel established at: {tunnel_url}")
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