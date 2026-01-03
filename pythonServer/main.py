import socket
import pyautogui
import psutil
import time
import threading
import random

# CONFIGURATION
UDP_IP = "0.0.0.0"
LISTEN_PORT = 5005
ESP32_IP = "192.168.0.239"
ESP32_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, LISTEN_PORT))
sock.settimeout(0.1)

print(f"🔄 Bi-Directional Server Running.")
print(f"   Listening on {LISTEN_PORT}")
print(f"   Sending to {ESP32_IP}:{ESP32_PORT}")

def get_system_stats():
    cpu = int(psutil.cpu_percent())
    ram = int(psutil.virtual_memory().percent)
    return f"STATS:{cpu}:{ram}"

try:
    last_send_time = 0
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            message = data.decode("utf-8")
            print(f"📥 CMD: {message}")
            
            if message == "VOL_UP": pyautogui.press('volumeup')
            elif message == "VOL_DOWN": pyautogui.press('volumedown')
            elif message == "CLICK": pyautogui.press('playpause')
            
            if addr[0] != ESP32_IP:
                ESP32_IP = addr[0]
                
        except socket.timeout:
            pass

        if time.time() - last_send_time > 0.5:
            stats_msg = get_system_stats()
            sock.sendto(stats_msg.encode(), (ESP32_IP, ESP32_PORT))
            last_send_time = time.time()

except KeyboardInterrupt:
    print("\nStopping.")
