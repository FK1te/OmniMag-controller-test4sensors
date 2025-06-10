import os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# Define paths
log_dir = os.path.join(os.getcwd(), 'Data', 'controller_tests', 'rotation_logs')
fig_dir = os.path.join(os.getcwd(), 'Data', 'controller_tests', 'figs')

# Ensure the output directory exists
os.makedirs(fig_dir, exist_ok=True)

# Iterate over each .csv file in the log directory
for filename in os.listdir(log_dir):
    if filename.endswith('.csv'):
        csv_path = os.path.join(log_dir, filename)
        fig_name = filename.replace('.csv', '.png')
        fig_path = os.path.join(fig_dir, fig_name)

        print(csv_path)
        print(fig_path)

        try:
            # Read the CSV file
            df = pd.read_csv(csv_path)

            # Ensure there are at least 3 columns
            if df.shape[1] >= 8:
                time = df.iloc[:, 0]
                value = df.iloc[:, 7]

                # Plot the data with larger figure and higher resolution
                plt.figure(figsize=(12, 6))
                plt.plot(time, value)
                plt.xlabel('Time')
                plt.ylabel('Value (Column 3)')
                plt.title(f"Plot of {filename}")

                ax = plt.gca()
                ax.grid(True)

                # Y-axis: grid and ticks every 5 units
                ax.yaxis.set_major_locator(ticker.MultipleLocator(2))
                ax.xaxis.set_major_locator(ticker.MultipleLocator(1))

                # Save the figure with high resolution
                plt.savefig(fig_path, dpi=300)
                plt.close()
            else:
                print(f"Skipping {filename}: not enough columns.")
        except Exception as e:
            print(f"Error processing {filename}: {e}")
