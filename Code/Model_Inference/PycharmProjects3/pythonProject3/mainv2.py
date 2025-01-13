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
temperature = 0
# Initialize the CSV file
df = pd.DataFrame(columns=["Vx", "Vy", "Vz", "temperature", "current",
                            "fft_peak_value", "natural_frequency",
                            "rms_vibration", "kurtosis", "thd", "zero_crossing_rate"])
df.to_csv(csv_file, index=False)

# Data queues for thread-safe communication
vibration_queue = Queue()
temperature_queue = Queue()
current_queue = Queue()
sensor_data_queue = Queue()

# FFT function
def compute_fft(data, sampling_rate):
    n = len(data)
    fft_result = fft(data)
    freq = np.fft.fftfreq(n, d=1 / sampling_rate)
    return freq[:n // 2], np.abs(fft_result)[:n // 2]

# Feature extraction function
def extract_fft_features(data_x, data_y, data_z, sampling_rate=50):
    try:
        # Compute FFT for each axis
        freq_x, fft_x = compute_fft(data_x, sampling_rate)
        freq_y, fft_y = compute_fft(data_y, sampling_rate)
        freq_z, fft_z = compute_fft(data_z, sampling_rate)

        # Extract features
        fft_peak_value = max(max(fft_x), max(fft_y), max(fft_z))
        natural_frequency = max(max(freq_x), max(freq_y), max(freq_z))
        rms_vibration = np.sqrt(np.mean(np.square(data_x + data_y + data_z)))
        kurtosis = float(np.sum((data_x - np.mean(data_x))**4) / len(data_x) /
                         (np.var(data_x)**2)) if np.var(data_x) > 0 else 0
        thd = float(np.sqrt(np.sum(np.array(data_x[1:])**2)) / data_x[0]) if data_x[0] != 0 else 0
        zero_crossing_rate = len(np.where(np.diff(np.sign(data_x)))[0]) / len(data_x)

        return {
            "fft_peak_value": fft_peak_value,
            "natural_frequency": natural_frequency,
            "rms_vibration": rms_vibration,
            "kurtosis": kurtosis,
            "thd": thd,
            "zero_crossing_rate": zero_crossing_rate,
        }
    except Exception as e:
        print(f"Error computing FFT features: {e}")
        return None

# FFT computation thread
def fft_thread():
    data_x, data_y, data_z = [], [], []
    while True:
        try:

            new_data = vibration_queue.get()

            data_x.append(float(new_data["Vx"]))
            data_y.append(float(new_data["Vy"]))
            data_z.append(float(new_data["Vz"]))

            if len(data_x) >= 256:
                features = extract_fft_features(data_x, data_y, data_z)
                if features:
                    new_row = {
                        "Vx": data_x[-1],
                        "Vy": data_y[-1],
                        "Vz": data_z[-1],
                        "temperature": None,  # Placeholder for the temperature thread
                        "current": None,      # Placeholder for the temperature thread
                        **features,
                    }

                    df = pd.DataFrame([new_row])
                    df.to_csv(csv_file, mode="a", header=False, index=False)

                data_x, data_y, data_z = [], [], []

        except Exception as e:
            print(f"Error in FFT thread: {e}")
        finally:
            time.sleep(0.01)  # Prevent CPU overuse


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

        elif msg.topic == "temperature":
            temperature = data.get("temperature")
            if temperature is not None:
                sensor_data_queue.put({"temperature": temperature, "current": None})

        elif msg.topic == "current":
            current = data.get("current")
            if current is not None:
                sensor_data_queue.put({"temperature": None, "current": current})

    except Exception as e:
        print(f"Error processing MQTT message: {e}")

def sensor_thread():
    while True:
        try:

            new_data = sensor_data_queue.get()

            new_row = {
                "Vx": None,
                "Vy": None,
                "Vz": None,
                "temperature": new_data.get("temperature"),
                "current": new_data.get("current"),
                "fft_peak_value": None,
                "natural_frequency": None,
                "rms_vibration": None,
                "kurtosis": None,
                "thd": None,
                "zero_crossing_rate": None,
            }

            df = pd.DataFrame([new_row])
            df.to_csv(csv_file, mode="a", header=False, index=False)

        except Exception as e:
            print(f"Error in Sensor thread: {e}")
        finally:
            time.sleep(0.01)

def start_mqtt_client():
    broker = "10.0.0.140"
    port = 1883

    client = mqtt.Client()
    client.on_message = on_message

    client.connect(broker, port)
    client.subscribe("vibration")
    client.subscribe("temperature")
    client.subscribe("current")
    client.loop_start()

if __name__ == "__main__":

    mqtt_thread = threading.Thread(target=start_mqtt_client)
    mqtt_thread.start()

    fft_processing_thread = threading.Thread(target=fft_thread)
    fft_processing_thread.start()

    temp_current_thread = threading.Thread(target=sensor_thread)
    temp_current_thread.start()
