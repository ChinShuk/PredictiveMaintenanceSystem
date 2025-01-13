import pandas as pd
import numpy as np

# Constants
CSV_FILE_PATH = "predictive_maintenance_data.csv"  # Path to your CSV file
VARIANCE_THRESHOLD = 0.20  # 20% variance threshold

def label_anomalies(csv_path, variance_threshold):
    # Read the CSV data
    df = pd.read_csv(csv_path)

    # Check if required columns exist
    required_columns = ['fft_peak_value', 'rms_vibration']
    for col in required_columns:
        if col not in df.columns:
            raise ValueError(f"Column {col} is missing in the dataset")

    # Calculate mean and acceptable ranges for key features
    mean_fft_peak = df['fft_peak_value'].mean()
    mean_rms = df['rms_vibration'].mean()

    fft_range = (mean_fft_peak * (1 - variance_threshold), mean_fft_peak * (1 + variance_threshold))
    rms_range = (mean_rms * (1 - variance_threshold), mean_rms * (1 + variance_threshold))

    # Apply anomaly detection rules
    def detect_anomaly(row):
        if not (fft_range[0] <= row['fft_peak_value'] <= fft_range[1]):
            return 1  # Anomaly detected
        if not (rms_range[0] <= row['rms_vibration'] <= rms_range[1]):
            return 1  # Anomaly detected
        return 0  # No anomaly

    # Add a new column for anomaly labels
    df['label'] = df.apply(detect_anomaly, axis=1)

    # Save the labeled data back to a CSV file
    labeled_csv_path = "labeled_" + csv_path
    df.to_csv(labeled_csv_path, index=False)
    print(f"Labeled data saved to {labeled_csv_path}")

    # Return labeled DataFrame
    return df

if __name__ == "__main__":
    labeled_data = label_anomalies(CSV_FILE_PATH, VARIANCE_THRESHOLD)
    print("Sample of labeled data:")
    print(labeled_data.head())

