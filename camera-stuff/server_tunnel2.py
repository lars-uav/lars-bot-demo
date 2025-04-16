from pyngrok import ngrok
import os
import dotenv
import json

# Load environment variables
dotenv.load_dotenv()

def setup_tunnels(ports=[8000, 8001]):
    """
    Set up ngrok tunnels for multiple servers
    Returns a dictionary of port:url mappings
    """
    try:
        # Set auth token
        ngrok.set_auth_token(os.getenv('NGROK_AUTH_TOKEN'))
        
        # Create tunnels for each port
        tunnel_urls = {}
        for port in ports:
            tunnel = ngrok.connect(port, "http")
            tunnel_urls[port] = tunnel.public_url
            print(f"Tunnel for port {port} established at: {tunnel.public_url}")
        
        # Save URLs to a file that can be read by the Streamlit app
        config = {
            'VIDEO_URL': tunnel_urls[8000],
            'ROBOT_URL': tunnel_urls[8001]
        }
        
        print("\nAdd these URLs to your Streamlit secrets.toml file as:")
        print(f"VIDEO_URL = '{config['VIDEO_URL']}'")
        print(f"ROBOT_URL = '{config['ROBOT_URL']}'")
        
        return tunnel_urls
        
    except Exception as e:
        print(f"Error setting up tunnels: {e}")
        return None

if __name__ == "__main__":
    # Verify auth token exists
    if not os.getenv('NGROK_AUTH_TOKEN'):
        print("Error: NGROK_AUTH_TOKEN not found in environment variables")
        exit(1)
    
    # Setup the tunnels
    tunnel_urls = setup_tunnels()
    
    if tunnel_urls:
        print("\nKeeping tunnels alive. Press CTRL+C to stop.")
        try:
            ngrok_process = ngrok.get_ngrok_process()
            ngrok_process.proc.wait()
        except KeyboardInterrupt:
            print("\nShutting down tunnels...")
            ngrok.kill()
