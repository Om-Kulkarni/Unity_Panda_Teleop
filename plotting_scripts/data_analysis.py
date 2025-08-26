# main.py
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

# --- Configuration ---
# Set a consistent style for all plots for a professional look.
sns.set_theme(style="whitegrid", palette="colorblind")
CSV_FILE_PATH = 'raw_data.csv'

# --- Helper Functions ---
def check_sample_csv():
    try:
        pd.read_csv(CSV_FILE_PATH)
        print(f"'{CSV_FILE_PATH}' found. Proceeding with analysis.")
    except FileNotFoundError:
        print(f"'{CSV_FILE_PATH}' not found. Creating a sample file for demonstration.")


def plot_total_completion_time(df):
    """
    Generates and saves a box plot comparing the total task completion time
    between the two groups.
    """
    plt.figure(figsize=(8, 6))
    sns.boxplot(x='Group', y='Total_Time_s', data=df)
    plt.title('Total Task Completion Time by Control Group', fontsize=16, fontweight='bold')
    plt.xlabel('Control Group', fontsize=12)
    plt.ylabel('Total Time (seconds)', fontsize=12)
    plt.savefig('1_total_completion_time.png', dpi=300)
    print("Generated '1_total_completion_time.png'")

def plot_learning_curve(df):
    """
    Generates and saves a line graph showing the learning curve (time per cube)
    for each group.
    """
    # Reshape data from wide to long format for easier plotting
    time_cols = [f'Cube{i}_Time_s' for i in range(1, 7)]
    df_long = df.melt(id_vars=['Participant_ID', 'Group'], value_vars=time_cols,
                      var_name='Cube_Trial', value_name='Time_s')
    # Extract trial number for a clean x-axis
    df_long['Trial'] = df_long['Cube_Trial'].str.extract('(\d+)').astype(int)

    plt.figure(figsize=(10, 6))
    sns.lineplot(x='Trial', y='Time_s', hue='Group', data=df_long, marker='o', errorbar='ci')
    plt.title('Learning Curve: Time per Cube Trial', fontsize=16, fontweight='bold')
    plt.xlabel('Cube Trial Number', fontsize=12)
    plt.ylabel('Average Time (seconds)', fontsize=12)
    plt.xticks(range(1, 7))
    plt.legend(title='Control Group')
    plt.savefig('2_learning_curve.png', dpi=300)
    print("Generated '2_learning_curve.png'")

def plot_user_errors(df):
    """
    Generates and saves a bar chart comparing the number of user errors
    between the two groups.
    """
    plt.figure(figsize=(8, 6))
    sns.barplot(x='Group', y='User_Errors_Count', data=df, capsize=.1)
    plt.title('Average User Errors by Control Group', fontsize=16, fontweight='bold')
    plt.xlabel('Control Group', fontsize=12)
    plt.ylabel('Average Number of Errors', fontsize=12)
    plt.savefig('3_user_errors.png', dpi=300)
    print("Generated '3_user_errors.png'")

def plot_nasa_tlx_radar(df):
    """
    Generates and saves a radar chart comparing the NASA-TLX workload scores
    between the two groups.
    """
    # Define the six subscales for the radar chart
    tlx_labels = [
        'Mental Demand', 'Physical Demand', 'Temporal Demand',
        'Performance (Inverted)', 'Effort', 'Frustration'
    ]
    tlx_cols = [
        'TLX_Mental_Demand', 'TLX_Physical_Demand', 'TLX_Temporal_Demand',
        'TLX_Performance_Inverted', 'TLX_Effort', 'TLX_Frustration'
    ]

    # Calculate the mean for each subscale, grouped by the control method
    group_means = df.groupby('Group')[tlx_cols].mean()

    # Setup for the radar chart
    num_vars = len(tlx_labels)
    angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()
    angles += angles[:1] # Close the loop

    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(polar=True))

    # Plot data for each group
    for i, (group_name, row) in enumerate(group_means.iterrows()):
        values = row.tolist()
        values += values[:1] # Close the loop
        ax.plot(angles, values, label=group_name, linewidth=2)
        ax.fill(angles, values, alpha=0.25)

    # Formatting the chart
    ax.set_yticklabels([])
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(tlx_labels, size=12)
    ax.set_title('NASA-TLX Perceived Workload Comparison', size=16, fontweight='bold', y=1.1)
    ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))
    plt.savefig('4_nasa_tlx_radar.png', dpi=300, bbox_inches='tight')
    print("Generated '4_nasa_tlx_radar.png'")


# --- Main Execution ---
if __name__ == "__main__":
    # 1. Ensure the data file exists
    check_sample_csv()

    # 2. Load the data from the CSV file
    try:
        df = pd.read_csv(CSV_FILE_PATH)
    except Exception as e:
        print(f"Error loading CSV file: {e}")
        exit()

    # 3. Preprocess the data: Create new columns for analysis
    # Calculate total completion time for each participant
    time_cols = [f'Cube{i}_Time_s' for i in range(1, 7)]
    df['Total_Time_s'] = df[time_cols].sum(axis=1)

    # Invert the 'Performance' score so a higher value means worse performance (higher workload)
    # This aligns it with the other TLX scales. Assumes a 0-100 scale.
    df['TLX_Performance_Inverted'] = 100 - df['TLX_Performance']

    # 4. Generate and save all plots
    print("\n--- Generating Plots ---")
    plot_total_completion_time(df)
    plot_learning_curve(df)
    plot_user_errors(df)
    plot_nasa_tlx_radar(df)
    print("\n--- All plots have been saved as PNG files in the script's directory. ---")

    # 5. Display the plots (optional, can be commented out if you only want to save files)
    plt.show()
