import json
import logging
import threading
import time
from queue import Queue
import numpy as np
from scipy.fft import fft
import paho.mqtt.client as mqtt
from collections import OrderedDict

# AWS IoT Core Configuration
THING_NAME = "Group_8_Device"
CERT_DIR = "certificates/Chinmay"
ENDPOINT = "a1wg482ic8xn9d-ats.iot.ca-central-1.amazonaws.com"
CERT_FILE = f"{CERT_DIR}/certificate.pem.crt"
KEY_FILE = f"{CERT_DIR}/private.pem.key"
CA_FILE = f"{CERT_DIR}/AmazonRootCA1.pem"
AWS_TOPIC = "maintenance_prediction"

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
        thd = float(np.sqrt(np.sum(np.array(data_x[1:])**2)) / data_x[0]) if data_x[0] != 0 else 0
        zero_crossing_rate = len(np.where(np.diff(np.sign(data_x)))[0]) / len(data_x)
        return {
            "fft_peak_value": fft_peak_value,
            "natural_frequency": natural_frequency,
            "rms_vibration": rms_vibration,
            "kurtosis": kurtosis,
            "thd": thd,
            "zero_crossing_rate": zero_crossing_rate,
            "fft_data": {
                "freq_x": freq_x.tolist(),
                "fft_x": fft_x.tolist(),
                "freq_y": freq_y.tolist(),
                "fft_y": fft_y.tolist(),
                "freq_z": freq_z.tolist(),
                "fft_z": fft_z.tolist()
            }
        }
    except Exception as e:
        logging.error(f"Error extracting FFT features: {e}")
        return None

# Feature Extraction and Forwarding to AWS
def feature_extraction_thread():
    data_x, data_y, data_z = [], [], []
    while True:
        try:
            new_data = vibration_queue.get()
            data_x.append(new_data["Vx"])
            data_y.append(new_data["Vy"])
            data_z.append(new_data["Vz"])
            if len(data_x) >= 256:
                features = extract_fft_features(data_x, data_y, data_z)
                if features:
                    features['temperature'] = temperature_queue.get() if not temperature_queue.empty() else None
                    features['current'] = current_queue.get() if not current_queue.empty() else None
                    send_data_to_aws(features)
                data_x, data_y, data_z = [], [], []
        except Exception as e:
            logging.error(f"Error in feature extraction thread: {e}")
        finally:
            time.sleep(0.01)

# def send_data_to_aws(features):
#     payload = json.dumps(features)
#     aws_client.publish(AWS_TOPIC, payload=payload, qos=1)
#     logging.info(f"Sent data to AWS IoT Core: {payload}")

def send_data_to_aws(features):
    # Ensure 'temperature' and 'current' are at the top of the payload
    payload = OrderedDict()
    payload['temperature'] = features.get('temperature', None)
    payload['current'] = features.get('current', None)

    # Add the rest of the features
    for key, value in features.items():
        if key not in ['temperature', 'current']:
            payload[key] = value

    # Convert to JSON and send to AWS IoT Core
    aws_payload = json.dumps(payload)
    aws_client.publish(AWS_TOPIC, payload=aws_payload, qos=1)
    logging.info(f"Sent data to AWS IoT Core: {aws_payload}")

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
