# python3 Light_Sensor_Determination.py -dataFile TERP_2-3-4.csv -downSampleFactor 1000
# Read in light Sensor values and output a graph of them
# Dependencies
# pip install numpy pandas matplotlib

import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Plot lux values against time')
    parser.add_argument('-dataFile', metavar='DATAFILE', type=str, help='path to TERP CSV file', required=True)
    parser.add_argument('-downSampleFactor', metavar='DOWNSAMPLEFACTOR', type=int, help='Cluster Size of Timed Variables', required=False, default=1)
    args = parser.parse_args()

    # Read the CSV file
    lux_column_name = "LightSensor lux"
    time_column_name = "Time (ms)"
    csv_file = args.dataFile
    try:
        data_types = {
            lux_column_name: float,
            time_column_name: float
        }
        df = pd.read_csv(csv_file, header=0, usecols=['Time (ms)', 'LightSensor lux'], dtype=data_types)
    except pd.errors.ParserError as e:
        print(f"Error reading CSV file: {e}")
        return

    # Extract columns
    lux_values = df[lux_column_name]
    time_values = df[time_column_name]
    time_values = [str(value) for value in time_values]

    # Downsample factor (adjust as needed)
    downsample_factor = args.downSampleFactor

    # Downsample the lux values by taking the mean within fixed intervals
    downsampled_lux_values = []
    for i in range(0, len(lux_values), downsample_factor):
        lux_chunk = lux_values[i:i+downsample_factor]
        if not lux_chunk.empty:  # Check if lux_chunk is not empty
            downsampled_lux_values.append(np.mean(lux_chunk))

    # Downsample the time values accordingly
    downsampled_time_values = time_values[::downsample_factor]

    # Create the graph
    print("Creating Lux vs Time Graph")
    plt.figure(figsize=(10, 6))  # Adjust size if needed
    plt.plot(downsampled_time_values, downsampled_lux_values, marker='o', color='b', linestyle='-')
    plt.title('Lux vs Time')
    plt.xlabel('Time')
    plt.ylabel('Lux')
    plt.xticks(rotation=90)  # Rotate x-axis labels for better readability
    plt.tight_layout()

    # Show the graph
    plt.show()

if __name__ == "__main__":
    main()
