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
ROBOT_UUID = os.getenv("ROBOT_UUID","nova2-real")
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
        self.robot = Nova2Robot(host=ROBOT_IP, logger=self.robot_logger)
        self.robot.start()
        self.robot.clear_error()
        # tool_id = int(os.environ["TOOL_ID"])
        tool_id = 2
        self.find_and_setup_hand(tool_id)

    def find_and_setup_hand(self, tool_id):
        connected = False
        tool_info = self.get_tool_info(tool_infos, tool_id)
        name = tool_info["name"]
        hand = tool_classes[name]()
        if tool_id != -1:
            connected = hand.connect_and_setup()
            if not connected:
                raise ValueError(f"Failed to connect to hand: {name}")
        else:
            hand = None
        self.hand_name = name
        self.hand = hand
        self.tool_id = tool_id


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

    def get_tool_info(
        self, tool_infos: List[Dict[str, Any]], tool_id: int) -> Dict[str, Any]:
        return [tool_info for tool_info in tool_infos
                if tool_info["id"] == tool_id][0]

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

            # 不要なはず？--------------   
            # ツール番号
            tool_id = int(self.pose[23].copy())
            # 起動時（共有メモリが初期化されている）のみ初期値を使用
            if tool_id == 0:
                tool_id = self.tool_id
            # それ以外は制御プロセスからの値を使用
            else:
                self.tool_id = tool_id

            # ツールチェンジ
            status_tool_change = None
            next_tool_id = int(self.pose[17].copy())
            if next_tool_id != 0:
                if not is_in_tool_change:
                    is_in_tool_change = True
                    self.logger.info("Tool change started")
            else:
                if self.pose[41] == 1 and is_in_tool_change:
                    is_in_tool_change = False
                    status_tool_change = bool(self.pose[18] == 1)
                    self.logger.info("Tool change finished")
            if status_tool_change is not None:
                self.logger.info(f"Tool change status: {status_tool_change}")
                # 終了した場合のみキーを追加
                actual_joint_js["tool_change"] = status_tool_change
                # 成功した場合のみ接続
                if status_tool_change:
                    next_tool_info = self.get_tool_info(tool_infos, tool_id)
                    name = next_tool_info["name"]
                    hand = tool_classes[name]()
                    connected = hand.connect_and_setup()
                    if not connected:
                        self.logger.warning(
                            f"Failed to connect to hand: {name}")
                    self.hand_name = name
                    self.hand = hand

            # 棚の上の箱を作業台に置くデモ
            status_put_down_box = None
            put_down_box = self.pose[21].copy()
            if put_down_box != 0:
                if not is_put_down_box:
                    is_put_down_box = True
            else:
                if self.pose[41] == 1 and is_put_down_box:
                    is_put_down_box = False
                    status_put_down_box = bool(self.pose[22] == 1)
            # 終了した場合のみキーを追加
            if status_put_down_box is not None:
                actual_joint_js["put_down_box"] = status_put_down_box

            # カッター移動            
            status_line_cut = None
            line_cut = self.pose[38].copy()
            if line_cut != 0:
                if not line_cut:
                    line_cut = True
            else:
                if self.pose[41] == 1 and line_cut:
                    line_cut = False
                    status_line_cut = bool(self.pose[39] == 1)
            # 終了した場合のみキーを追加
            if status_line_cut is not None:
                actual_joint_js["line_cut"] = status_line_cut
            # ----------------
                
                
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
                    joints = ['j1','j2','j3','j4','j5','j6']
                    actual_joint_js.update({
                        k: v for k, v in zip(joints, actual_joint)})
                else:
                    raise ValueError
            
            # 型: 整数、単位: ms
            time_ms = int(now * 1000)
            actual_joint_js["time"] = time_ms
            # [X, Y, Z, RX, RY, RZ]: センサ値の力[N]とモーメント[Nm]

            # 不要？------- 
            actual_joint_js["tool_id"] = int(tool_id)
            # ツールチェンジ中はツールのセンサは使用しない
            if is_in_tool_change:
                width = None
                force = None
            else:
                width = None
                force = None
                if self.hand_name == "onrobot_2fg7":
                    width = self.pose[12]
                    if width == 0:
                        width = None
                    else:
                        width = float(width - 100)
                    force = self.pose[40]
                    if force == 0:
                        force = None
                    else:
                        force = float(force - 100)
            # ---------
                        
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
            
            # エラー関連の取得は未調査。nova2_robotの対応関数も未編集
            # with self.slave_mode_lock:
            #     if self.pose[14] == 0:
            #         try:
            #             errors = self.robot.get_cur_error_info_all()
            #         except Exception as e:

            #             self.logger.error(f"{self.robot.format_error(e)}")
            #             self.reconnect_after_timeout(e)
            #             errors = []
            #         # 制御プロセスのエラー検出と方法が違うので、
            #         # 直後は状態プロセスでエラーが検出されないことがある
            #         # その場合は次のループに検出を持ち越す
            #         if len(errors) > 0:
            #             error = {"errors": errors}
            #             # 自動復帰可能エラー
            #             auto_recoverable = \
            #                 self.robot.are_all_errors_stateless(errors)
            #             error["auto_recoverable"] = auto_recoverable
            #         try:
            #             is_emergency_stopped = self.robot.is_emergency_stopped()
            #         except Exception as e:
            #             # self.logger.error(f"{self.robot.format_error_wo_desc(e)}")
            #             self.logger.error(e)
            #             self.reconnect_after_timeout(e)
                        
                        
            # # 切り替わるときにログを出す
            # if is_emergency_stopped != last_is_emergency_stopped:
            #     if is_emergency_stopped:
            #         self.logger.error("Emergency stop is ON")
            #     else:
            #         self.logger.info("Emergency stop is OFF")
            # last_is_emergency_stopped = is_emergency_stopped
            # actual_joint_js["emergency_stopped"] = is_emergency_stopped
            # self.pose[36] = int(is_emergency_stopped)

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

            if now-last > 0.3 or "tool_change" in actual_joint_js or "put_down_box" in actual_joint_js:
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
                js = json.dumps(datum, ensure_ascii=False)
                f.write(js + "\n")

            if self.pose[32] == 1:
                return False

            t_elapsed = time.time() - now
            t_wait = T_INTV - t_elapsed
            if t_wait > 0:
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
        self.robot_logger.setLevel(logging.WARNING)

    def get_logging_dir_and_change_log_file(self) -> None:
        command = self.monitor_pipe.recv()
        logging_dir = command["params"]["logging_dir"]
        self.logger.info("Change log file")
        self.logging_dir = logging_dir
        self.pose[34] = 0

    def run_proc(self, monitor_dict, monitor_lock, slave_mode_lock, log_queue, monitor_pipe, logging_dir, disable_mqtt: bool = False):
        self.setup_logger(log_queue)
        self.logger.info("Process started")
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
                self.logger.error("Error in monitor")
                # self.logger.error(f"{self.robot.format_error_wo_desc(e)}")
            if self.pose[32] == 1:
                if self.client is not None:
                    self.client.loop_stop()
                    self.client.disconnect()
                self.sm.close()
                time.sleep(1)
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
