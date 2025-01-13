import json
import logging
import threading
import time
from queue import Queue
import numpy as np
from scipy.fft import fft
import paho.mqtt.client as mqtt
import pickle

# AWS IoT Core Configuration
THING_NAME = "Group_8_Device"
CERT_DIR = "certificates/Chinmay"
ENDPOINT = "a1wg482ic8xn9d-ats.iot.ca-central-1.amazonaws.com"
CERT_FILE = f"{CERT_DIR}/certificate.pem.crt"
KEY_FILE = f"{CERT_DIR}/private.pem.key"
CA_FILE = f"{CERT_DIR}/AmazonRootCA1.pem"
AWS_TOPIC = "maintenance_prediction"
New_RF = 1

# Local MQTT Broker Configuration
LOCAL_BROKER = "10.0.0.140"
LOCAL_PORT = 1883
LOCAL_TOPICS = {
    "vibration": "vibration",
    "temperature": "temperature",
    "current": "current"
}

# Configure Logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Load Pickle Model
MODEL_PATH = "random_forest_predictiveMaintainance.h5"
with open(MODEL_PATH, 'rb') as model_file:
    rf_model = pickle.load(model_file)

# Labels Mapping
LABELS = ["Good Condition", "Needs Maintenance", "Breakdown Imminent"]

# Data queues for thread-safe communication
vibration_queue = Queue()
temperature_queue = Queue()
current_queue = Queue()

# Setup AWS IoT MQTT Client
aws_client = mqtt.Client(client_id=THING_NAME)

def on_aws_connect(client, userdata, flags, rc):
    if rc == 0:
        logging.info("Connected to AWS IoT Core")
    else:
        logging.error(f"Failed to connect to AWS IoT Core, return code {rc}")

def on_aws_publish(client, userdata, mid):
    logging.info(f"Message {mid} published successfully to AWS IoT Core")

aws_client.tls_set(ca_certs=CA_FILE, certfile=CERT_FILE, keyfile=KEY_FILE)
aws_client.tls_insecure_set(False)
aws_client.on_connect = on_aws_connect
aws_client.on_publish = on_aws_publish
aws_client.connect(ENDPOINT, port=8883, keepalive=60)

# Setup Local MQTT Client
local_client = mqtt.Client()

def on_local_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        if msg.topic == LOCAL_TOPICS["vibration"]:
            for entry in data:
                vibration_queue.put({
                    "Vx": float(entry["Vx"]),
                    "Vy": float(entry["Vy"]),
                    "Vz": float(entry["Vz"])
                })
        elif msg.topic == LOCAL_TOPICS["temperature"]:
            temperature_queue.put(float(data["temperature"]))
        elif msg.topic == LOCAL_TOPICS["current"]:
            current_queue.put(float(data["current"]))
    except Exception as e:
        logging.error(f"Error processing local MQTT message: {e}")

local_client.on_message = on_local_message

# FFT and Feature Extraction Functions
def compute_fft(data, sampling_rate):
    n = len(data)
    fft_result = fft(data)
    freq = np.fft.fftfreq(n, d=1 / sampling_rate)
    return freq[:n // 2], np.abs(fft_result)[:n // 2]

def extract_fft_features(data_x, data_y, data_z, sampling_rate=50):
    try:
        freq_x, fft_x = compute_fft(data_x, sampling_rate)
        freq_y, fft_y = compute_fft(data_y, sampling_rate)
        freq_z, fft_z = compute_fft(data_z, sampling_rate)
        fft_peak_value = max(max(fft_x), max(fft_y), max(fft_z))
        natural_frequency = max(max(freq_x), max(freq_y), max(freq_z))
        rms_vibration = np.sqrt(np.mean(np.square(data_x + data_y + data_z)))
        kurtosis = float(np.sum((data_x - np.mean(data_x))**4) / len(data_x) / (np.var(data_x)**2)) if np.var(data_x) > 0 else 0
        return fft_x, fft_y, fft_z, {
            "fft_peak_value": float(fft_peak_value),
            "natural_frequency": float(natural_frequency),
            "rms_vibration": float(rms_vibration),
            "kurtosis": float(kurtosis)
        }
    except Exception as e:
        logging.error(f"Error extracting FFT features: {e}")
        return None, None, None, None

# Feature Extraction and Forwarding to AWS
def feature_extraction_thread():
    data_x, data_y, data_z = [], [], []
    while True:
        try:
            new_data = vibration_queue.get()
            data_x.append(new_data["Vx"])
            data_y.append(new_data["Vy"])
            data_z.append(new_data["Vz"])
            if len(data_x) >= 50:
                fft_x, fft_y, fft_z, features = extract_fft_features(data_x, data_y, data_z)
                if features:
                    features['temperature'] = float(temperature_queue.get() if not temperature_queue.empty() else 0)
                    features['current'] = float(current_queue.get() if not current_queue.empty() else 0)
                    features['Vx'] = [float(v) for v in data_x]
                    features['Vy'] = [float(v) for v in data_y]
                    features['Vz'] = [float(v) for v in data_z]
                    features['fftx'] = fft_x[:10].tolist()  # Include FFT data for Vx (limited to 10 points)
                    features['ffty'] = fft_y[:10].tolist()  # Include FFT data for Vy (limited to 10 points)
                    features['fftz'] = fft_z[:10].tolist()  # Include FFT data for Vz (limited to 10 points)

                    # Validate features before inference
                    logging.debug(f"Features prepared for inference: {features}")
                    if New_RF == 1:
                        inference_input = np.array([[
                            features["fft_peak_value"],      # fft_peak_value
                            features["natural_frequency"],   # natural_frequency
                            features["rms_vibration"],       # rms_vibration
                            features["kurtosis"]             # kurtosis
                        ]])  # Ensure input is 2D and matches the expected order
                    else:
                        inference_input = np.array([[
                            float(np.mean(features["Vx"])),  # Vx
                            float(np.mean(features["Vy"])),  # Vy
                            float(np.mean(features["Vz"])),  # Vz
                            features["fft_peak_value"],  # fft_peak_value
                            features["natural_frequency"],  # natural_frequency
                            features["rms_vibration"],  # rms_vibration
                            features["kurtosis"]  # kurtosis
                        ]])  # Ensure input is 2D and matches the expected order

                    logging.debug(f"Inference Input: {inference_input}")

                    # Run inference
                    if inference_input.shape[1] == 7:  # Validate input shape
                        prediction = rf_model.predict(inference_input)
                        predicted_label = LABELS[int(prediction[0])]  # Map integer to label

                        # Prepare JSON payload with FFT values
                        payload = {
                            "prediction": str(predicted_label),
                            "temperature": str(features['temperature']),
                            "current": str(features['current']),
                            "vibration_Vx": [str(v) for v in features['Vx'][:10]],  # Include only first 10 points
                            "vibration_Vy": [str(v) for v in features['Vy'][:10]],  # Include only first 10 points
                            "vibration_Vz": [str(v) for v in features['Vz'][:10]],  # Include only first 10 points
                            "fft_fftx": [str(v) for v in features['fftx']],  # FFT data for Vx
                            "fft_ffty": [str(v) for v in features['ffty']],  # FFT data for Vy
                            "fft_fftz": [str(v) for v in features['fftz']]   # FFT data for Vz
                        }

                        # Trim payload to fit within 1000 characters if necessary
                        payload_str = json.dumps(payload)
                        if len(payload_str) > 1000:
                            payload['vibration_Vx'] = payload['vibration_Vx'][:5]
                            payload['vibration_Vy'] = payload['vibration_Vy'][:5]
                            payload['vibration_Vz'] = payload['vibration_Vz'][:5]
                            payload['fft_fftx'] = payload['fft_fftx'][:5]
                            payload['fft_ffty'] = payload['fft_ffty'][:5]
                            payload['fft_fftz'] = payload['fft_fftz'][:5]

                        send_data_to_aws(payload)
                    else:
                        logging.error(f"Invalid input shape for prediction: {inference_input.shape}")

                # Reset data buffers
                data_x, data_y, data_z = [], [], []
        except Exception as e:
            logging.error(f"Error in feature extraction thread: {e}, Data_x: {data_x}, Data_y: {data_y}, Data_z: {data_z}")
        finally:
            time.sleep(0.01)

def send_data_to_aws(features):
    payload = json.dumps(features)
    aws_client.publish(AWS_TOPIC, payload=payload, qos=1)
    logging.info(f"Sent data to AWS IoT Core: {payload}")

# Main Function
def main():
    aws_client.loop_start()
    local_client.connect(LOCAL_BROKER, LOCAL_PORT)
    local_client.subscribe([(LOCAL_TOPICS["vibration"], 0), (LOCAL_TOPICS["temperature"], 0), (LOCAL_TOPICS["current"], 0)])
    local_client.loop_start()
    threading.Thread(target=feature_extraction_thread, daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Shutting down...")
        local_client.loop_stop()
        local_client.disconnect()
        aws_client.loop_stop()
        aws_client.disconnect()

if __name__ == "__main__":
    main()
