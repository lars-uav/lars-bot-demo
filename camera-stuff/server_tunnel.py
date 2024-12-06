from pyngrok import ngrok, conf
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
        # Configure ngrok
        pyngrok_config = conf.PyngrokConfig(
            auth_token=os.getenv('NGROK_AUTH_TOKEN'),
            region="us"
        )
        ngrok.set_auth_token(os.getenv('NGROK_AUTH_TOKEN'))
        
        # Start tunnel with basic configuration
        tunnel = ngrok.connect(port)
        tunnel_url = tunnel.public_url
        
        print(f"Tunnel established at: {tunnel_url}")
        print("Add this URL to your Streamlit secrets.toml file as:")
        print(f"ROBOT_URL = '{tunnel_url}'")
        
        return tunnel_url
        
    except Exception as e:
        print(f"Error setting up tunnel: {e}")
        return None

if __name__ == "__main__":
    # Verify auth token exists
    if not os.getenv('NGROK_AUTH_TOKEN'):
        print("Error: NGROK_AUTH_TOKEN not found in environment variables")
        exit(1)
        
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
    else:
        print("Failed to establish tunnel")