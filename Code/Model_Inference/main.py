import paho.mqtt.client as mqtt
import json
import pandas as pd
import os
import numpy as np
from scipy.fft import fft
from queue import Queue
import threading
import time
from datetime import datetime

# Generate a timestamped CSV filename
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_file = f"sensor_data_{timestamp}.csv"

# Initialize the CSV file
df = pd.DataFrame(columns=["Vx", "Vy", "Vz", "fft_peak_value", "natural_frequency",
                           "rms_vibration", "kurtosis", "thd", "zero_crossing_rate", "label"])
df.to_csv(csv_file, index=False)

# Data queues for thread-safe communication
vibration_queue = Queue()

# CSV write lock
csv_lock = threading.Lock()

# Global variables for ideal baseline and flag
calculate_ideal_baseline = True
ideal_baseline = None

# Flag to control processing of zero crossing rate
process_zero_crossing = False  # Set this to False to disable zero crossing rate calculation


# Save ideal baseline to file
def save_ideal_baseline(baseline, filename="ideal_baseline.json"):
    with open(filename, "w") as file:
        json.dump(baseline, file)
    print(f"Ideal baseline saved to {filename}")


# Load ideal baseline from file
def load_ideal_baseline(filename="ideal_baseline.json"):
    global ideal_baseline
    try:
        with open(filename, "r") as file:
            ideal_baseline = json.load(file)
            print(f"Ideal baseline loaded from {filename}")
    except FileNotFoundError:
        print(f"No ideal baseline file found at {filename}.")
        ideal_baseline = None


# FFT function
def compute_fft(data, sampling_rate):
    n = len(data)
    fft_result = fft(data)
    freq = np.fft.fftfreq(n, d=1 / sampling_rate)
    return freq[:n // 2], np.abs(fft_result)[:n // 2]


# Feature extraction function
def extract_fft_features(data_x, data_y, data_z, sampling_rate=50):
    try:
        # Ensure data is valid
        if len(data_x) < 2 or len(data_y) < 2 or len(data_z) < 2:
            raise ValueError("Insufficient data for FFT computation.")
        if not (all(isinstance(val, (int, float)) for val in data_x + data_y + data_z)):
            raise ValueError("Non-numeric values detected in input data.")

        # Check for variation in the data
        min_variance_threshold = 1e-10
        if np.var(data_x) < min_variance_threshold or np.var(data_y) < min_variance_threshold or np.var(
                data_z) < min_variance_threshold:
            print("Warning: Very low variation detected in input data. Proceeding with caution.")

        freq_x, fft_x = compute_fft(data_x, sampling_rate)
        freq_y, fft_y = compute_fft(data_y, sampling_rate)
        freq_z, fft_z = compute_fft(data_z, sampling_rate)

        fft_peak_value = max(max(fft_x), max(fft_y), max(fft_z))
        natural_frequency = max(max(freq_x), max(freq_y), max(freq_z))
        rms_vibration = np.sqrt(np.mean(np.square(data_x + data_y + data_z)))

        variance = np.var(data_x)
        kurtosis = (
            float(np.sum((data_x - np.mean(data_x)) ** 4) / len(data_x) / (variance ** 2))
            if variance > 1e-10 else 0  # Avoid division by zero for very low variance
        )
        thd = (
            float(np.sqrt(np.sum(np.array(data_x[1:]) ** 2)) / abs(data_x[0]))
            if abs(data_x[0]) > 1e-3 else 0  # Avoid division by very small values
        )

        # Zero crossing rate calculation only if enabled
        zero_crossing_rate = (
            len(np.where(np.diff(np.sign(data_x)))[0]) / len(data_x)
            if process_zero_crossing and len(data_x) > 0 else 0  # Avoid division by zero
        )

        features = {
            "fft_peak_value": fft_peak_value,
            "natural_frequency": natural_frequency,
            "rms_vibration": rms_vibration,
            "kurtosis": kurtosis,
            "thd": thd,
        }

        if process_zero_crossing:
            features["zero_crossing_rate"] = zero_crossing_rate

        return features
    except Exception as e:
        print(f"Error computing FFT features: {e}, Data_x: {data_x[:10]}, Data_y: {data_y[:10]}, Data_z: {data_z[:10]}")
        return None


# Label calculation based on ideal baseline
def calculate_label(features):
    if ideal_baseline is None:
        return "unknown"

    thresholds = {"needs_maintenance": 0.1, "breakdown_imminent": 0.2}
    max_variance = 0

    for key, ideal in ideal_baseline.items():
        if key == "zero_crossing_rate" and features.get(key, 0.0) == 0.0:
            return "faulty_Data"
        if key in features:
            variance = abs(features[key] - ideal) / ideal
            max_variance = max(max_variance, variance)

    if max_variance > thresholds["breakdown_imminent"]:
        return "breakdown_imminent"
    elif max_variance > thresholds["needs_maintenance"]:
        return "needs_maintenance"
    else:
        return "good_condition"


# Thread-safe CSV writer
def write_to_csv(row):
    global csv_file
    with csv_lock:  # Ensure only one thread writes at a time
        try:
            print(f"Writing to CSV: {row}")  # Debug statement
            df = pd.DataFrame([row])
            df.to_csv(csv_file, mode="a", header=False, index=False)
            print(f"Data written successfully to {csv_file}")  # Debug statement
        except Exception as e:
            print(f"Error writing to CSV: {e}")


# FFT computation thread
def fft_thread():
    global ideal_baseline
    data_x, data_y, data_z = [], [], []
    all_features = []  # To collect features for baseline calculation
    iteration_count = 0

    while True:
        try:
            new_data = vibration_queue.get()

            # Append new data to buffers
            vx = float(new_data.get("Vx", 0))
            vy = float(new_data.get("Vy", 0))
            vz = float(new_data.get("Vz", 0))
            data_x.append(vx)
            data_y.append(vy)
            data_z.append(vz)

            if len(data_x) >= 256:
                if not all(map(lambda v: isinstance(v, (int, float)) and v != 0, data_x + data_y + data_z)):
                    print(f"Invalid or zero values detected in buffers. Clearing buffers. Data_x: {data_x[:10]}")
                    data_x, data_y, data_z = [], [], []
                    continue

                features = extract_fft_features(data_x, data_y, data_z)
                print(f"Extracted Features: {features}")  # Debug statement

                if features:
                    if calculate_ideal_baseline:
                        all_features.append(features)
                        iteration_count += 1
                        if iteration_count >= 100:
                            ideal_baseline = {
                                key: np.mean([f[key] for f in all_features])
                                for key in all_features[0]
                            }
                            print("Ideal baseline calculated:", ideal_baseline)
                            save_ideal_baseline(ideal_baseline)
                            print("Program exiting gracefully.")
                            os._exit(0)
                    else:
                        label = calculate_label(features)
                        print(f"Calculated Label: {label}")  # Debug statement
                        new_row = {
                            "Vx": data_x[-1],
                            "Vy": data_y[-1],
                            "Vz": data_z[-1],
                            **features,
                            "label": label,
                        }
                        print(f"Row to Write: {new_row}")  # Debug statement
                        write_to_csv(new_row)

                data_x, data_y, data_z = [], [], []

        except Exception as e:
            print(f"Error in FFT thread: {e}, Data_x: {data_x[:10]}, Data_y: {data_y[:10]}, Data_z: {data_z[:10]}")
        finally:
            time.sleep(0.01)


# MQTT message handler
def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        print(f"Data: {data}")

        if msg.topic == "vibration":
            for entry in data:
                vx = entry.get("Vx")
                vy = entry.get("Vy")
                vz = entry.get("Vz")

                if vx is None or vy is None or vz is None:
                    print(f"Missing expected keys in the vibration data: {entry}")
                    continue

                vibration_queue.put({"Vx": vx, "Vy": vy, "Vz": vz})

    except Exception as e:
        print(f"Error processing MQTT message: {e}")


# Start MQTT client
def start_mqtt_client():
    broker = "192.168.187.72"  # "10.0.0.140"
    port = 1883

    client = mqtt.Client()
    client.on_message = on_message

    client.connect(broker, port)
    client.subscribe("vibration")
    client.loop_start()


if __name__ == "__main__":
    if not calculate_ideal_baseline:
        load_ideal_baseline()

    mqtt_thread = threading.Thread(target=start_mqtt_client)
    mqtt_thread.start()

    fft_processing_thread = threading.Thread(target=fft_thread)
    fft_processing_thread.start()
