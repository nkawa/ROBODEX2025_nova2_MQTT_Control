# Cobotta ProをMQTTで制御する

import json
import logging
from paho.mqtt import client as mqtt
import multiprocessing as mp
import multiprocessing.shared_memory

from multiprocessing import Process

import os
from datetime import datetime
import numpy as np
import time
import sys

## ここでUUID を使いたい
import uuid

package_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.append(package_dir)
from nova2.config import SHM_NAME, SHM_SIZE
from nova2.nova2_monitor import Nova2_MON
from nova2.nova2_control import Nova2_CON, Nova2_CON_Archiver
from nova2_monitor_gui import run_joint_monitor_gui

from dotenv import load_dotenv

# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
MQTT_SERVER = os.getenv("MQTT_SERVER", "sora2.uclab.jp")
MQTT_CTRL_TOPIC = os.getenv("MQTT_CTRL_TOPIC", "control")
# ROBOT_UUID = os.getenv("ROBOT_UUID","nova2-real")
# ROBOT_MODEL = os.getenv("ROBOT_MODEL","nova2-real")
ROBOT_MODEL = os.getenv("ROBOT_MODEL","robodex2025-demo-nova2")
ROBOT_UUID = os.getenv("ROBOT_UUID","robodex2025-demo-nova2")

MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "mgr")
MQTT_MANAGE_RCV_TOPIC = os.getenv("MQTT_MANAGE_RCV_TOPIC", "dev")+"/"+ROBOT_UUID
MQTT_FORMAT = os.getenv("MQTT_FORMAT", "NOVA2_Control_IK")
MQTT_MODE = os.getenv("MQTT_MODE", "metawork")

class Cobotta_Pro_MQTT:
    def __init__(self):
        self.mqtt_ctrl_topic = None
        self.last_registered = None
 
    def on_connect(self, client, userdata, connect_flags, reason_code, properties):
        # ロボットのメタ情報の中身はとりあえず
        date = datetime.now().strftime('%c')
        if MQTT_MODE == "metawork":
            info = {
                "date": date,
                "device": {
                    "agent": "none",
                    "cookie": "none",
                },
                "devType": "robot",
                "type": ROBOT_MODEL,
                "version": "none",
                "devId": ROBOT_UUID,
            }
            self.client.publish(MQTT_MANAGE_TOPIC + "/register", json.dumps(info))
            with self.mqtt_control_lock:
                info["topic_type"] = "mgr/register"
                info["topic"] = MQTT_MANAGE_TOPIC + "/register"
                self.mqtt_control_dict.clear()
                self.mqtt_control_dict.update(info)
            self.logger.info("publish to: " + MQTT_MANAGE_TOPIC + "/register")
            self.last_registered = time.time()
            self.client.subscribe(MQTT_MANAGE_RCV_TOPIC)
            self.logger.info("subscribe to: " + MQTT_MANAGE_RCV_TOPIC)
        else:
            self.logger.info("MQTT:Connected with result code " + str(reason_code),
                             "subscribe ctrl", MQTT_CTRL_TOPIC)
            self.mqtt_ctrl_topic = MQTT_CTRL_TOPIC
            self.client.subscribe(self.mqtt_ctrl_topic)

    def on_disconnect(
        self,
        client,
        userdata,
        disconnect_flags,
        reason_code,
        properties,
    ):
        if reason_code != 0:
            self.logger.warning("MQTT Unexpected disconnection.")

    def on_message(self, client, userdata, msg):
        if msg.topic == self.mqtt_ctrl_topic:
            js = json.loads(msg.payload)


            if MQTT_FORMAT == "NOVA2_Control_IK":
                joint_q = js["joints"]
            elif MQTT_FORMAT == "Denso-Cobotta-Pro-Control-IK":
                # 7要素入っているが6要素でよいため
                rot = js["joints"][:6]
                joint_q = [x for x in rot]
                # NOTE: j5の基準がVRと実機とでずれているので補正。将来的にはVR側で修正?
                # NOTE(20250530): 現状はこれでうまくいくがVR側との意思疎通が必要
                # joint_q[4] = joint_q[4] + 90
                # NOTE(20250604): 一時的な対応。VR側で修正され次第削除。
                # joint_q[0] = joint_q[0] - 180
            else:
                raise ValueError
            self.pose[6:12] = joint_q 

            if "grip" in js:
                if js['grip'][0]:
                    # NOVA2_Control_IKの場合のみ？
                    self.pose[13] = 1
                else:
                    self.pose[13] = 2
            
            if "tool_change" in js:
                if self.pose[17] == 0:
                    tool = js["tool_change"]
                    self.pose[16] = 1
                    self.pose[17] = tool
            
            if "put_down_box" in js:
                if self.pose[21] == 0:
                    if js["put_down_box"]:
                        self.pose[16] = 1
                        self.pose[21] = 1
            
            if "line_cut" in js:
                if self.pose[38] == 0:
                    if js["line_cut"]:
                        self.pose[16] = 1
                        self.pose[38] = 1

            self.pose[20] = 1
            with self.mqtt_control_lock:
                js["topic_type"] = "control"
                js["topic"] = msg.topic
                self.mqtt_control_dict.clear()
                self.mqtt_control_dict.update(js)

        elif msg.topic == MQTT_MANAGE_RCV_TOPIC:
            if MQTT_MODE == "metawork":
                js = json.loads(msg.payload)
                goggles_id = js["devId"]
                mqtt_ctrl_topic = MQTT_CTRL_TOPIC + "/" + goggles_id
                if mqtt_ctrl_topic != self.mqtt_ctrl_topic:
                    if self.mqtt_ctrl_topic is not None:
                        self.client.unsubscribe(self.mqtt_ctrl_topic)    
                    self.mqtt_ctrl_topic = mqtt_ctrl_topic
                self.client.subscribe(self.mqtt_ctrl_topic)
                self.logger.info("subscribe to: " + self.mqtt_ctrl_topic)
                with self.mqtt_control_lock:
                    js["topic_type"] = "dev"
                    js["topic"] = msg.topic
                    self.mqtt_control_dict.clear()
                    self.mqtt_control_dict.update(js)
        else:
            self.logger.warning("not subscribe msg" + msg.topic)

    def connect_mqtt(self):
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        # MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.on_message = self.on_message         # メッセージ到着時のコールバック
        self.client.connect(MQTT_SERVER, 1883, 60)
        self.client.loop_start()   # 通信処理開始

    def setup_logger(self, log_queue):
        self.logger = logging.getLogger("MQTT")
        if log_queue is not None:
            self.handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.handler = logging.StreamHandler()
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.INFO)

    def run_proc(self, mqtt_control_dict, mqtt_control_lock, log_queue):
        self.setup_logger(log_queue)
        self.logger.info("Process started")
        self.sm = mp.shared_memory.SharedMemory(SHM_NAME)
        self.pose = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        self.mqtt_control_dict = mqtt_control_dict
        self.mqtt_control_lock = mqtt_control_lock
        self.connect_mqtt()
        while True:
            # 30分ごとに再登録
            now = time.time()
            if (self.last_registered is not None and 
                self.last_registered + 60 * 30 < now):
                date = datetime.now().strftime('%c')
                if MQTT_MODE == "metawork":
                    info = {
                        "date": date,
                        "device": {
                            "agent": "none",
                            "cookie": "none",
                        },
                        "devType": "robot",
                        "type": ROBOT_MODEL,
                        "version": "none",
                        "devId": ROBOT_UUID,
                    }
                    self.client.publish(
                        MQTT_MANAGE_TOPIC + "/register", json.dumps(info))
                    with self.mqtt_control_lock:
                        info["topic_type"] = "mgr/register"
                        info["topic"] = MQTT_MANAGE_TOPIC + "/register"
                        self.mqtt_control_dict.clear()
                        self.mqtt_control_dict.update(info)
                    self.logger.info(
                        "re-publish to: " + MQTT_MANAGE_TOPIC + "/register")
                    self.last_registered = now

            # プロセス終了時
            if self.pose[32] == 1:
                if MQTT_MODE == "metawork":
                    info = {"devId": ROBOT_UUID}
                    self.client.publish(
                        MQTT_MANAGE_TOPIC + "/unregister", json.dumps(info))
                    self.logger.info(
                        "publish to: " + MQTT_MANAGE_TOPIC + "/unregister")
                self.client.loop_stop()
                self.client.disconnect()
                self.sm.close()
                time.sleep(1)
                self.logger.info("Process stopped")
                self.handler.close()
                break

            time.sleep(1)

class ProcessManager:
    def __init__(self):
        # mp.set_start_method('spawn')
        sz = SHM_SIZE * np.dtype('float32').itemsize
        try:
            self.sm = mp.shared_memory.SharedMemory(create=True,size = sz, name=SHM_NAME)
        except FileExistsError:
            self.sm = mp.shared_memory.SharedMemory(size = sz, name=SHM_NAME)
        # self.arの要素の説明
        # [0:6]: 関節の状態値
        # [6:12]: 関節の目標値
        # [12]: ハンドの状態値
        # [13]: ハンドの目標値
        # [14]: 0: 必ず通常モード。1: 基本的にスレーブモード（通常モードになっている場合もある）　!!Nova2では指定なし!!
        # [15]: 0: mqtt_control実行中でない。1: mqtt_control実行中
        # [16]: 1: リアルタイム制御停止命令（mqtt_control停止命令ではないことに注意）
        # [17]: ツールチェンジの実行フラグ。0: 終了。0以外: 開始。次のツール番号
        # [18]: ツールチェンジ完了状態。0: 未定義。1: 成功。2: 失敗
        # [19]: 制御開始後の状態値の受信フラグ
        # [20]: 制御開始後の目標値の受信フラグ
        # [21]: 棚の上の箱を作業台に置くデモの実行フラグ。0: 終了。1: 開始
        # [22]: 棚の上の箱を作業台に置くデモの完了状態。0: 未定義。1: 成功。2: 失敗
        # [23]: 現在のツール番号
        # [24:30]: 関節の制御値
        # [31]: エリア機能の有効/無効状態。0: 無効。1: 有効
        # [32]: プロセス終了フラグ
        # [33]: ログ出力先の変更フラグ(control用)
        # [34]: ログ出力先の変更フラグ(monitor用)
        # [35]: ログ出力先の変更フラグ(contol-archiver用)
        # [36]: 0: 非常停止でない。1: 非常停止
        # [37]: スレーブモードの状態値。0: 通常モード。1: スレーブモード
        # [38]: カッター移動の実行フラグ。0: 終了。1: 開始
        # [39]: カッター移動の完了状態。0: 未定義。1: 成功。2: 失敗
        # [40]: ハンドの把持力。
        # [41]: ツールチェンジなど後の制御可能フラグ。0: 制御不可。1: 制御可能
        self.ar = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf) # 共有メモリ上の Array
        self.ar[:] = 0
        self.manager = multiprocessing.Manager()
        self.monitor_dict = self.manager.dict()
        self.monitor_lock = self.manager.Lock()
        self.mqtt_control_dict = self.manager.dict()
        self.mqtt_control_lock = self.manager.Lock()
        self.slave_mode_lock = multiprocessing.Lock()
        self.main_to_control_pipe, self.control_pipe = multiprocessing.Pipe()
        self.main_to_monitor_pipe, self.monitor_pipe = multiprocessing.Pipe()
        self.state_recv_mqtt = False
        self.state_monitor = False
        self.state_control = False
        self.state_monitor_gui = False
        self.log_queue = multiprocessing.Queue()
        self.recvP = None
        self.monP = None
        self.ctrlP = None
        self.monitor_guiP = None
        self.ctrl_archiverP = None
        self.control_to_archiver_queue = multiprocessing.Queue()
        self.main_to_control_archiver_pipe, self.control_archiver_pipe = \
            multiprocessing.Pipe()

    def startRecvMQTT(self):
        self.recv = Cobotta_Pro_MQTT()
        self.recvP = Process(
            target=self.recv.run_proc,
            args=(self.mqtt_control_dict,
                  self.mqtt_control_lock,
                  self.log_queue),
            name="MQTT-recv")
        self.recvP.start()
        self.state_recv_mqtt = True

    def startMonitor(self, logging_dir: str | None = None, disable_mqtt: bool = False):
        self.mon = Nova2_MON()
        self.monP = Process(
            target=self.mon.run_proc,
            args=(self.monitor_dict,
                  self.monitor_lock,
                  self.slave_mode_lock,
                  self.log_queue,
                  self.monitor_pipe,
                  logging_dir,
                  disable_mqtt),
            name="Nova2-monitor")
        self.monP.start()
        self.state_monitor = True
        print("command: start monitor")

    def startControl(self, logging_dir: str | None = None):
        self.ctrl = Nova2_CON()
        self.ctrlP = Process(
            target=self.ctrl.run_proc,
            args=(self.control_pipe, self.slave_mode_lock, self.log_queue, logging_dir, self.control_to_archiver_queue),
            name="Nova2-control")
        self.ctrlP.start()

        self.ctrl_archiver = Nova2_CON_Archiver()
        self.ctrl_archiverP = Process(
            target=self.ctrl_archiver.run_proc,
            args=(self.control_archiver_pipe,
                  self.log_queue,
                  logging_dir,
                  self.control_to_archiver_queue,
                  ),
            name="Nova2-control-archiver")
        self.ctrl_archiverP.start()
        self.state_control = True
        print("command: start control")

    def startMonitorGUI(self):
        self.monitor_guiP = Process(
            target=run_joint_monitor_gui,
            name="Nova2-monitor-gui",
        )
        self.monitor_guiP.start()
        self.state_monitor_gui = True

    def stop_all_processes(self):
        self.ar[32] = 1
        self.ar[16] = 1
        if self.recvP is not None:
            self.recvP.join()
        if self.monP is not None:
            self.monP.join()
        if self.ctrlP is not None:
            self.ctrlP.join()
        if self.ctrl_archiverP is not None:
            self.ctrl_archiverP.join()
        if self.monitor_guiP is not None:
            self.monitor_guiP.join()
        self.sm.close()
        self.sm.unlink()
        self.manager.shutdown()
        self.main_to_control_pipe.close()
        self.control_pipe.close()
        self.main_to_monitor_pipe.close()
        self.monitor_pipe.close()
        self.control_to_archiver_queue.close()

    def _send_command_to_control(self, command):
        wait = command.get("wait", False)
        self.main_to_control_pipe.send(command)
        if wait:
            return self.main_to_control_pipe.recv()

    def _send_command_to_control_archiver(self, command):
        self.main_to_control_archiver_pipe.send(command)

    def _send_command_to_monitor(self, command):
        self.main_to_monitor_pipe.send(command)

    def enable(self):
        self._send_command_to_control({"command": "enable", "wait": True})

    def disable(self):
        self._send_command_to_control({"command": "disable", "wait": True})

    def set_area_enabled(self, enable: bool):
        self._send_command_to_control({"command": "set_area_enabled", "params": {"enable": enable}, "wait": True})

    def tidy_pose(self):
        self._send_command_to_control({"command": "tidy_pose", "wait": True})

    def clear_error(self):
        self._send_command_to_control({"command": "clear_error", "wait": True})

    def release_hand(self):
        self._send_command_to_control({"command": "release_hand", "wait": True})

    def line_cut(self):
        self._send_command_to_control({"command": "line_cut", "wait": True})

    def start_mqtt_control(self):
        self._send_command_to_control({"command": "start_mqtt_control", "wait": False})

    def stop_mqtt_control(self):
        # mqtt_control中のみシグナルを出す
        if self.state_mqtt_control:
            self.ar[16] = 1

    @property
    def state_mqtt_control(self):
        return self.ar[15] == 1

    def tool_change(self, tool_id: int):
        self.ar[17] = tool_id
        self._send_command_to_control({"command": "tool_change", "wait": True})

    def jog_joint(self, joint, direction):
        self._send_command_to_control({"command": "jog_joint", "params": {"joint": joint, "direction": direction}, "wait": False})

    def jog_tcp(self, axis, direction):
        self._send_command_to_control({"command": "jog_tcp", "params": {"axis": axis, "direction": direction}, "wait": False})

    def move_joint(self, joints: list[float], wait: bool = False):
        self._send_command_to_control({"command": "move_joint", "params": {"joints": joints}, "wait": wait})

    def demo_put_down_box(self):
        self._send_command_to_control({"command": "demo_put_down_box", "wait": True})

    def get_current_monitor_log(self):
        with self.monitor_lock:
            monitor_dict = self.monitor_dict.copy()
        return monitor_dict
    
    def get_current_mqtt_control_log(self):
        with self.mqtt_control_lock:
            mqtt_control_dict = self.mqtt_control_dict.copy()
        return mqtt_control_dict

    def change_log_file(self, logging_dir: str):
        # モニタプロセス
        self.ar[34] = 1
        self._send_command_to_monitor({"command": "change_log_file", "params": {"logging_dir": logging_dir}})
        # 制御プロセス
        self.ar[33] = 1
        self._send_command_to_control({"command": "change_log_file", "params": {"logging_dir": logging_dir}, "wait": True})
        # 制御プロセスはMQTTControl時は一旦停止させる
        self.stop_mqtt_control()
        # 制御記録用プロセス
        self.ar[35] = 1
        self._send_command_to_control_archiver({"command": "change_log_file", "params": {"logging_dir": logging_dir}})
