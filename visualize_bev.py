import pandas as pd
import matplotlib.pyplot as plt

def plot_bev():
    # Read recorded spatial positions
    try:
        df = pd.read_csv("spatial_records.csv")
    except FileNotFoundError:
        print("CSV file not found, please confirm if the recording script ran successfully.")
        return

    plt.figure(figsize=(12, 8))
    # Plot lane marking points (World X, World Y)
    plt.scatter(df['world_x'], df['world_y'], s=1, c='blue', alpha=0.5, label='Detected Lane Points')
    
    plt.title("Bird's-Eye View: Infrastructure Mapping Result")
    plt.xlabel("Global X (meters)")
    plt.ylabel("Global Y (meters)")
    plt.axis('equal') # Ensure correct aspect ratio
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    
    print("Generating BEV visualization plot...")
    plt.savefig("bev_reconstruction.png")
    plt.show()

if __name__ == "__main__":
    plot_bev()
