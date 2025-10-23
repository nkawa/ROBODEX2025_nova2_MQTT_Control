# Cobotta Pro の状態をモニタリングする

import logging
from typing import Any, Dict, List, TextIO
from paho.mqtt import client as mqtt
from nova2.config import SHM_NAME, SHM_SIZE, T_INTV
from nova2_robot import Nova2Robot

import datetime
import time

import os
import sys
import json
import psutil

import multiprocessing as mp

import numpy as np

from dotenv import load_dotenv

from nova2.tools import tool_infos, tool_classes

# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
ROBOT_IP = os.getenv("ROBOT_IP", "192.168.5.1")
HAND_IP = os.getenv("HAND_IP", "192.168.5.46")
# ROBOT_UUID = os.getenv("ROBOT_UUID","nova2-real")
ROBOT_UUID = os.getenv("ROBOT_UUID","robodex2025-demo-nova2")
MQTT_SERVER = os.getenv("MQTT_SERVER", "sora2.uclab.jp")
MQTT_ROBOT_STATE_TOPIC = os.getenv("MQTT_ROBOT_STATE_TOPIC", "robot")+"/"+ROBOT_UUID
MQTT_FORMAT = os.getenv("MQTT_FORMAT", "NOVA2_Control_IK")
MQTT_MODE = os.getenv("MQTT_MODE", "metawork")
SAVE = os.getenv("SAVE", "true") == "true"
# 基本的に運用時には固定するパラメータ
save_state = SAVE


class Nova2_MON:
    def __init__(self):
        pass

    def init_robot(self):
        self.robot = Nova2Robot(host=ROBOT_IP, logger=self.robot_logger, port=[30004])

        self.robot.start()
        # self.robot.clear_error()



    def init_realtime(self):
        os_used = sys.platform
        process = psutil.Process(os.getpid())
        if os_used == "win32":  # Windows (either 32-bit or 64-bit)
            process.nice(psutil.REALTIME_PRIORITY_CLASS)
        elif os_used == "linux":  # linux
            rt_app_priority = 80
            param = os.sched_param(rt_app_priority)
            try:
                os.sched_setscheduler(0, os.SCHED_FIFO, param)
            except OSError:
                self.logger.warning("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
            else:
                self.logger.info("Process real-time priority set to: %u" % rt_app_priority)

    def on_connect(self, client, userdata, connect_flags, reason_code, properties):
        # 接続できた旨表示
        self.logger.info("MQTT connected with result code: " + str(reason_code))
        
    def on_disconnect(
        self,
        client,
        userdata,
        disconnect_flags,
        reason_code,
        properties,
    ):
        if reason_code != 0:
            self.logger.warning("MQTT unexpected disconnection.")

    def connect_mqtt(self, disable_mqtt: bool = False):
        if disable_mqtt:
            self.client = None
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.connect(MQTT_SERVER, 1883, 60)
        self.client.loop_start()   # 通信処理開始



    def monitor_start(self, f: TextIO | None = None):
        last = 0
        last_error_monitored = 0
        is_in_tool_change = False
        is_put_down_box = False
        last_is_in_servo_mode = None
        last_is_emergency_stopped = None
        while True:
            # ログファイル変更時
            if self.pose[34] == 1:
                return True

            now = time.time()
            if last == 0:
                last = now
            if last_error_monitored == 0:
                last_error_monitored = now

            actual_joint_js = {}
                
            # TCP姿勢
            try:
                actual_tcp_pose = self.robot.get_current_pose()
            except Exception as e:
                # if type(e) is ORiNException and self.robot.is_error_level_0(e):
                #     self.logger.warning(f"{self.robot.format_error_wo_desc(e)}")
                # else:
                #     self.logger.error(f"{self.robot.format_error_wo_desc(e)}")
                # self.reconnect_after_timeout(e)
                self.logger.error("fail to get tcp pose")
                self.logger.error(e)
                actual_tcp_pose = None
                
            # 関節
            try:
                actual_joint = self.robot.get_current_joint()
            except Exception as e:
                # if type(e) is ORiNException and self.robot.is_error_level_0(e):
                #     self.logger.warning(f"{self.robot.format_error_wo_desc(e)}")
                # else:
                #     self.logger.error(f"{self.robot.format_error_wo_desc(e)}")
                # self.reconnect_after_timeout(e)
                self.logger.error("fail to get joints")
                self.logger.error(e)
                actual_joint = None

            if actual_joint is not None:
                if MQTT_FORMAT == 'UR-realtime-control-MQTT':        
                    joints = ['j1','j2','j3','j4','j5','j6']
                    actual_joint_js.update({
                        k: v for k, v in zip(joints, actual_joint)})
                elif MQTT_FORMAT == 'Denso-Cobotta-Pro-Control-IK':
                    # 7要素送る必要があるのでダミーの[0]を追加
                    actual_joint_js.update({"joints": list(actual_joint) + [0]})
                    # NOTE: j5の基準がVRと実機とでずれているので補正。将来的にはVR側で修正?
                    # NOTE(20250530): 現状はこれでうまく行くがVR側と意思疎通が必要
                    # actual_joint_js["joints"][4] = actual_joint_js["joints"][4] - 90
                    # NOTE(20250604): 一時的な対応。VR側で修正され次第削除。
                    # actual_joint_js["joints"][0] = actual_joint_js["joints"][0] + 180
                elif MQTT_FORMAT == "NOVA2_Control_IK":
                    actual_joint_js.update({"joints": list(actual_joint) + [0]})

                    #joints = ['j1','j2','j3','j4','j5','j6']
                    #actual_joint_js.update({
                    #    k: v for k, v in zip(joints, actual_joint)})
                else:
                    raise ValueError
            
            # 型: 整数、単位: ms
            time_ms = int(now * 1000)
            actual_joint_js["time"] = time_ms
            # [X, Y, Z, RX, RY, RZ]: センサ値の力[N]とモーメント[Nm]

                        
            # モータがONか
            try:
                enabled = self.robot.is_enabled()
            except Exception as e:
                # if type(e) is ORiNException and self.robot.is_error_level_0(e):
                #     self.logger.warning(f"{self.robot.format_error_wo_desc(e)}")
                # else:
                #     self.logger.error(f"{self.robot.format_error_wo_desc(e)}")
                # self.reconnect_after_timeout(e)
                self.logger.error(e)
                enabled = False
            actual_joint_js["enabled"] = enabled



            is_emergency_stopped = False
            error = {}
            # スレーブモード中にエラー情報や非常停止状態を取得しようとすると、
            # スレーブモードが切断される。
            # 制御プロセスでスレーブモードを前提とした処理をしている間 (lock中) は、
            # エラー情報や非常停止状態を取得しないようにする。
            

            if self.pose[15] == 0:
                actual_joint_js["mqtt_control"] = "OFF"
            else:
                actual_joint_js["mqtt_control"] = "ON"
            
            area_enabled = bool(self.pose[31])
            actual_joint_js["area_enabled"] = area_enabled

            if error:
                actual_joint_js["error"] = error

            if actual_joint is not None:
                self.pose[:len(actual_joint)] = actual_joint
                self.pose[19] = 1

            if now-last > 0.3 :
                jss = json.dumps(actual_joint_js)
                if self.client is not None:
                    self.client.publish(MQTT_ROBOT_STATE_TOPIC, jss)
                with self.monitor_lock:
                    actual_joint_js["topic_type"] = "robot"
                    actual_joint_js["topic"] = MQTT_ROBOT_STATE_TOPIC
                    if actual_tcp_pose is not None:
                        actual_joint_js["poses"] = actual_tcp_pose
                    self.monitor_dict.clear()
                    self.monitor_dict.update(actual_joint_js)
                last = now

            # MQTT手動制御モード時のみ記録する
            # それ以外の時のエラーはstate情報は必要ないと考えたため
            if f is not None and self.pose[15] == 1:
                forces = None #Nova2は力センサなし
                width = 0
                force = 0
                tool_id = 2
                datum = dict(
                    time=now,
                    kind="state",
                    joint=actual_joint,
                    pose=actual_tcp_pose,
                    width=width,
                    force=force,
                    forces=forces,
                    error=error,
                    enabled=enabled,
                    # TypeError: Object of type float32 is not JSON
                    # serializableへの対応
                    tool_id=float(tool_id),
                )
                # js = json.dumps(datum, ensure_ascii=False)
                # f.write(js + "\n")

            if self.pose[32] == 1:
                return False

            t_elapsed = time.time() - now
            t_wait = T_INTV - t_elapsed
            if t_wait > 0:
#                self.logger.info("Sleep wait for " + str(t_wait))
                time.sleep(t_wait)

    def setup_logger(self, log_queue):
        self.logger = logging.getLogger("MON")
        if log_queue is not None:
            self.handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.handler = logging.StreamHandler()
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.INFO)
        self.robot_logger = logging.getLogger("MON-ROBOT")
        if log_queue is not None:
            self.robot_handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.robot_handler = logging.StreamHandler()
        self.robot_logger.addHandler(self.robot_handler)
        self.robot_logger.setLevel(logging.INFO)

    def get_logging_dir_and_change_log_file(self) -> None:
        command = self.monitor_pipe.recv()
        logging_dir = command["params"]["logging_dir"]
        self.logger.info("Change log file")
        self.logging_dir = logging_dir
        self.pose[34] = 0

    def run_proc(self, monitor_dict, monitor_lock, slave_mode_lock, log_queue, monitor_pipe, logging_dir, disable_mqtt: bool = False):
        self.setup_logger(log_queue)
        self.logger.info("Monitor Process started")
        self.sm = mp.shared_memory.SharedMemory(SHM_NAME)
        self.pose = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        self.monitor_dict = monitor_dict
        self.monitor_lock = monitor_lock
        self.slave_mode_lock = slave_mode_lock
        self.monitor_pipe = monitor_pipe
        self.logging_dir = logging_dir

        self.init_realtime()
        self.init_robot()
        self.connect_mqtt(disable_mqtt=disable_mqtt)
        self.logger.info("Starting monitor loop")
        while True:
            try:
                if save_state:
                    with open(
                        os.path.join(self.logging_dir, "state.jsonl"), "a"
                    ) as f:
                        will_change_log_file = self.monitor_start(f)
                else:
                    will_change_log_file = self.monitor_start()
                if will_change_log_file:
                    self.get_logging_dir_and_change_log_file()
            except Exception as e:
                self.logger.error("Error in monitor",e)
            if self.pose[32] == 1:
                if self.client is not None:
                    self.client.loop_stop()
                    self.client.disconnect()
                # self.robot.close_port()
                self.sm.close()
                time.sleep(2)
                self.logger.info("Process stopped")
                self.handler.close()
                self.robot_handler.close()
                break


if __name__ == '__main__':
    cp = Nova2_MON()
    cp.init_realtime()
    cp.init_robot()
    cp.connect_mqtt()

    try:
        cp.monitor_start()
    except KeyboardInterrupt:
        print("Monitor Main Stopped")
        cp.robot.disable()
        cp.robot.stop()
