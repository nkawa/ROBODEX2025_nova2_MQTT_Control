import tkinter as tk
from tkinter import messagebox
import serial
import threading
import time

# ===== シリアルポート設定 =====
PORT = "/dev/usbserial_conveyor"   # 例: Windowsでは COM3, Linuxでは /dev/ttyUSB0
BAUDRATE = 115200

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
except serial.SerialException as e:
    ser = None
    print("シリアルポートを開けませんでした:", e)

# ===== 送信関数 =====
def send_command(cmd):
    if ser and ser.is_open:
        ser.write(cmd.encode('utf-8'))
        print(f"送信: {cmd}")
    else:
        messagebox.showerror("エラー", "シリアルポートが開かれていません。")

# ===== 自動送信制御 =====
running = False  # 自動送信フラグ

def auto_send():
    global running
    cmd = "CW"
    try:
        interval = float(interval_var.get())
    except ValueError:
        messagebox.showerror("エラー", "間隔には数値を入力してください。")
        return

    while running:
        send_command(cmd)
        cmd = "CCW" if cmd == "CW" else "CW"  # トグル
        time.sleep(interval)
    print("自動送信停止")

def start_auto():
    global running
    if not running:
        running = True
        thread = threading.Thread(target=auto_send, daemon=True)
        thread.start()
        print("自動送信開始")

def stop_auto():
    global running
    running = False

# ===== GUI設定 =====
root = tk.Tk()
root.title("Serial Control GUI")
root.geometry("250x350")

frame = tk.Frame(root, padx=20, pady=20)
frame.pack(expand=True)

# 手動ボタン
btn_cw = tk.Button(frame, text="CW", width=15, height=2, bg="#90EE90",
                   command=lambda: send_command("CW"))
btn_cw.pack(pady=5)

btn_stop = tk.Button(frame, text="STOP", width=15, height=2, bg="#FFA07A",
                     command=lambda: send_command("STOP"))
btn_stop.pack(pady=5)

btn_ccw = tk.Button(frame, text="CCW", width=15, height=2, bg="#ADD8E6",
                    command=lambda: send_command("CCW"))
btn_ccw.pack(pady=5)

# 間隔設定
interval_label = tk.Label(frame, text="間隔（秒）:")
interval_label.pack(pady=(15, 0))
interval_var = tk.StringVar(value="2.0")
interval_entry = tk.Entry(frame, textvariable=interval_var, width=10, justify="center")
interval_entry.pack(pady=5)

# 自動開始・停止ボタン
btn_auto_start = tk.Button(frame, text="自動送信開始", width=15, height=2, bg="#FFD700",
                           command=start_auto)
btn_auto_start.pack(pady=5)

btn_auto_stop = tk.Button(frame, text="自動送信停止", width=15, height=2, bg="#D3D3D3",
                          command=stop_auto)
btn_auto_stop.pack(pady=5)

# 終了時処理
def on_closing():
    global running
    running = False
    if ser and ser.is_open:
        ser.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
