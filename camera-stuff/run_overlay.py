from grid_overlay import GridOverlay

def main():
    try:
        overlay = GridOverlay()
        overlay.run()
    except Exception as e:
        print(f"Error: {str(e)}")

if __name__ == "__main__":
    main()
