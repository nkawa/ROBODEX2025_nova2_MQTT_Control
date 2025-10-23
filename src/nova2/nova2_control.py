# nova2 を制御する

import logging
import queue
from typing import Any, Dict, List, Literal, TextIO, Tuple
import datetime
import time
import traceback

import os
import sys
import json
import psutil

import multiprocessing as mp
import threading

import modern_robotics as mr
import numpy as np
from dotenv import load_dotenv

from nova2.config import SHM_NAME, SHM_SIZE, ABS_JOINT_LIMIT, T_INTV

package_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'vendor'))
sys.path.append(package_dir)

from filter import SMAFilter
from interpolate import DelayedInterpolator
from nova2.tools import tool_infos, tool_classes, tool_base
from nova2_robot import Nova2Robot

# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
ROBOT_IP = os.getenv("ROBOT_IP", "192.168.5.45")
HAND_IP = os.getenv("HAND_IP", "192.168.5.46")
SAVE = os.getenv("SAVE", "true") == "true"
MOVE = os.getenv("MOVE", "true") == "true"

# 基本的に運用時には固定するパラメータ
# 実際にロボットを制御するかしないか (VRとの結合時のデバッグ用)
move_robot = MOVE
# 平滑化の方法
# NOTE: 実際のVRコントローラとの結合時に、
# 遅延などを考慮すると改良が必要かもしれない。
# そのときのヒントとして残している
filter_kind: Literal[
    "original",
    "target",
    "state_and_target_diff",
    "moveit_servo_humble",
    "control_and_target_diff",
    "feedback_pd_traj",
] = "original"
speed_limits = np.array([240, 200, 240, 300, 300, 475])
speed_limit_ratio = 0.35
# NOTE: 加速度制限。スマートTPの最大加速度設定は単位が[rev/s^2]だが、[deg/s^2]とみなして、
# その値をここで設定すると、エラーが起きにくくなる (観測範囲でエラーがなくなった)
accel_limits = np.array([4040, 4033.33, 4040, 5050, 5050, 4860])
accel_limit_ratio = 0.35
stopped_velocity_eps = 1e-4
servo_mode = 0x202
use_interp = True
n_windows = 10
if filter_kind == "original":
    n_windows = 10
if servo_mode == 0x102:
    t_intv = 0.004
else:
    t_intv = T_INTV
    # t_intv = 0.03 #Nova2の最小制御間隔は30ms
n_windows *= int(0.008 / t_intv)
reset_default_state = True
default_joints = {
    # TCPが台の中心の上に来る初期位置
    "tidy": [4.7031, -0.6618, 105.5149, 0.0001, 75.1440, 94.7100],
    # NOTE: j5の基準がVRと実機とでずれているので補正。将来的にはVR側で修正?
    "vr": [159.3784, 10.08485, 122.90902, 151.10866, -43.20116 + 90, 20.69275],
    # NOTE: 2025/04/18 19:25の新しい位置?VRとの対応がおかしい気がする
    # 毎回の値も[0, 0, 0, 0, 0, 0]が飛んでくる気がする
    "vr2": [115.55677, 5.86272, 135.70465, 110.53529, -15.55474 + 90, 35.59977],
    # NOTE: 2025/05/30での新しい位置
    "vr3": [113.748, 5.645, 136.098, 109.059, 75.561, 35.82],
    # NOTE: 2025/06/05 での新しい位置
    "vr4": [-46.243, 10.258, 128.201, 125.629, 62.701, 32.618],
    "vr5": [-66.252, 5.645, 136.098, 109.059, 75.561, 35.82],
}
abs_joint_limit = ABS_JOINT_LIMIT
abs_joint_limit = np.array(abs_joint_limit)
abs_joint_soft_limit = abs_joint_limit - 10
# 外部速度。単位は%
speed_normal = 20
speed_tool_change = 2
# 目標値が状態値よりこの制限より大きく乖離した場合はロボットを停止させる
# 設定値は典型的なVRコントローラの動きから決定した
target_state_abs_joint_diff_limit = [30, 30, 40, 40, 40, 60]

save_control = SAVE


class StopWatch:
    def __init__(self):
        self.start_t = None
        self.last_t = None
        self.last_msg = None
        self.laps = []

    def start(self, msg: str = "") -> None:
        t = time.perf_counter()
        self.start_t = t
        self.last_t = t
        self.last_msg = msg
        self.laps = []

    def lap(self, msg: str = "") -> None:
        if ((self.start_t is None) or
            (self.last_t is None) or
            (self.last_msg is None)):
            raise RuntimeError("StopWatch has not been started.")
        t = time.perf_counter()
        self.laps.append({
            "msg": self.last_msg,
            "lap": t - self.last_t,
            "split": t - self.start_t,
        })
        self.last_t = t
        self.last_msg = msg

    def stop(self) -> None:
        self.lap()
        self.start_t = None
        self.last_t = None
        self.last_msg = None
    
    def summary(self) -> str:
        s = "StopWatch summary:\n"
        s += "| No. | Lap (ms) | Split (ms) | Message |\n"
        s += "| - | - | - | - |\n"
        for i, lap in enumerate(self.laps):
            s += f"| {i} | {lap['lap']*1000:.3f} | {lap['split']*1000:.3f} | {lap['msg']} |\n"
        return s


class Nova2_CON:
    def __init__(self):
        self.default_joint = default_joints["vr5"]
        self.tidy_joint = default_joints["tidy"]

    def init_robot(self):
        try:
            self.robot = Nova2Robot(
                host=ROBOT_IP,
                default_servo_mode=servo_mode,
                logger=self.robot_logger,
                port=[30003, 29999]
            )
            self.robot.start()
            # self.robot.clear_error()
            
        except Exception as e:
            self.logger.error("Error in initializing robot: ")
            self.logger.error(f"{self.robot.format_error(e)}")

    def get_hand_state(self):        
        # # ハンドの状態値を取得して共有メモリに格納する
        # width = None
        # force = None
        # # NOTE: グリッパー。幅はVR表示に必要かもしれない。
        # # 力は把持の有無に変換してもよいかもしれない。
        # if self.hand_name == "onrobot_2fg7":
        #     width = self.hand.get_ext_width()
        #     force = self.hand.get_force()
        # # NOTE: 真空グリッパー。真空度しか取得できないのでどう使うか不明。
        # elif self.hand_name == "onrobot_vgc10":
        #     width = 0
        #     force = 0
        # if width is None:
        #     width = 0
        # else:
        #     # 0に意味があるのでオフセットをもたせる
        #     width += 100
        # if force is None:
        #     force = 0
        # else:
        #     force += 100
        
        # Nova2の真空グリッパからの情報取得方法は要調査(dashPortから)．そもそも不要？
        self.pose[12] = 0
        self.pose[40] = 0
    
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

    def format_error(self, e: Exception) -> str:
        s = "\n"
        s = s + "Error trace: " + traceback.format_exc() + "\n"
        return s

    def hand_control_loop(self, stop_event, error_event, lock, error_info):
        self.logger.info("Start Hand Control Loop")
        last_tool_corrected = None
        t_intv_hand = 0.16
        last_tool_corrected_time = time.time()
        while True:
            now = time.time()
            if stop_event.is_set():
                break
            # 現在情報を取得しているかを確認
            if self.pose[19] != 1:
                time.sleep(t_intv_hand)
                continue
            # 目標値を取得しているかを確認
            if self.pose[20] != 1:
                time.sleep(t_intv_hand)
                continue
            # ツールの値を取得
            # 値0が意味を持つので共有メモリではオフセットをかけている
            tool = self.pose[13]
            if tool == 0:
                time.sleep(t_intv_hand)
                continue
            tool_corrected = tool
            if tool_corrected != last_tool_corrected:
                try:
                    if tool_corrected == 1:
                        self.send_grip()
                    elif tool_corrected == 2:
                        self.send_release()
                except Exception as e:
                    with lock:
                        error_info['kind'] = "hand"
                        error_info['msg'] = self.format_error(e)
                        error_info['exception'] = e
                    error_event.set()
                    break
            # ハンドの状態値を取得
            # 情報を常に取得するとアームの制御ループの処理間隔を乱し
            # 情報が必要なのはハンドに制御値を送った少し後だけなので以下のようにする
            # NOTE: 常に取得した方がいいかもしれない。処理間隔を乱すなら別プロセス化の方がいいかもしれない
            if now - last_tool_corrected_time < 1:
                try:
                    self.get_hand_state()
                except Exception as e:
                    with lock:
                        error_info['kind'] = "hand"
                        error_info['msg'] = self.format_error(e)
                        error_info['exception'] = e
                    error_event.set()
                    break
            if tool_corrected != last_tool_corrected:
                last_tool_corrected = tool_corrected
                last_tool_corrected_time = now
            # 適度に間隔を開ける
            t_elapsed = time.time() - now
            t_wait = t_intv_hand - t_elapsed
            if t_wait > 0:
                time.sleep(t_wait)
        self.logger.info("Stop Hand Control Loop")

    def control_loop(self, f: TextIO | None = None) -> bool:
        """リアルタイム制御ループ"""
        self.last = 0
        self.logger.info("Start Control Loop")
        # 状態値が最新の値になるようにする
        time.sleep(1)
        self.pose[19] = 0
        self.pose[20] = 0
        target_stop = None
        sw = StopWatch()
        stop_event = threading.Event()
        error_event = threading.Event()
        lock = threading.Lock()
        error_info = {}
        last_target = None
        use_hand_thread = True
        if use_hand_thread:
            hand_thread = threading.Thread(
                target=self.hand_control_loop,
                args=(stop_event, error_event, lock, error_info)
            )
            hand_thread.start()
        else:
            last_tool_corrected = None

        while True:
            sw.start("Get shared memory")
            now = time.time()

            # TODO: これがメインスレッドを遅くしている可能性ありだが
            # この1行だけでとも思う。要検証
            # 但しハンド由来のエラーでループを終了できなくなる
            if not hand_thread.is_alive():
                break

            # NOTE: テスト用データなど、時間が経つにつれて
            # targetの値がstateの値によらずにどんどん
            # 変化していく場合は、以下で待ちすぎると
            # 制御値のもとになる最初のtargetの値が
            # stateから大きく離れるので、t_intv秒と短い時間だけ
            # 待っている。もしもっと待つと最初に
            # ガッとロボットが動いてしまう。実際のシステムでは
            # targetはstateに依存するのでまた別に考える
            stop = self.pose[16]
            if stop:
                stop_event.set()

            # 現在情報を取得しているかを確認
            if self.pose[19] != 1:
                time.sleep(t_intv)
                # self.logger.info("Wait for monitoring")
                # 取得する前に終了する場合即時終了可能
                if stop:
                    break
                # ロボットにコマンドを送る前は、非常停止が押されているかを
                # スレーブモードが解除されているかで確認する
                continue

            # ツールチェンジなど後の制御可能フラグ
            self.pose[41] = 1

            # 目標値を取得しているかを確認
            if self.pose[20] != 1:
                time.sleep(t_intv)
                # self.logger.info("Wait for target")
                # 取得する前に終了する場合即時終了可能
                if stop:
                    break
                # ロボットにコマンドを送る前は、非常停止が押されているかを
                # スレーブモードが解除されているかで確認する
                continue

            # NOTE: 最初にVR側でロボットの状態値を取得できていれば追加してもよいかも
            # state = self.pose[:6].copy()
            # target = self.pose[6:12].copy()
            # if np.any(np.abs(state - target) > 0.01):
            #     continue

            # 関節の状態値
            state = self.pose[:6].copy()

            # 目標値
            target = self.pose[6:12].copy()
            sw.lap("Check target")
            target_raw = target

            # 目標値の角度が360度の不定性が許される場合 (1度と-359度を区別しない場合) でも
            # 実機の関節の角度は360度の不定性が許されないので
            # 状態値に最も近い目標値に規格化する
            # TODO: VRと実機の関節の角度が360度の倍数だけずれた状態で、
            # 実機側で制限値を超えると動くVRと動かない実機との間に360度の倍数で
            # ないずれが生じ、急に実機が動く可能性があるので、VR側で
            # 実機との比較をし、実機側で制限値を超えることがないようにする必要がある
            # とりあえず急に動こうとすれば止まる仕組みは入れている
            target = state + (target - state + 180) % 360 - 180

            # TODO: VR側でもソフトリミットを設定したほうが良い
            # abs_joint_soft_limitよりも10degree余裕を持たせる設計のため公式限界の+10°でconfigは設定
            target_th = np.maximum(target, -abs_joint_soft_limit)
            if (target != target_th).any():
                self.logger.warning("target reached minimum threshold")
            target_th = np.minimum(target_th, abs_joint_soft_limit)
            if (target != target_th).any():
                self.logger.warning("target reached maximum threshold")
            target = target_th

            # 目標値が状態値から大きく離れた場合
            # すでに目標値を受け取っていない場合はすぐに、
            # 目標値を受け取っている場合は前回の目標値からも大きく離れた場合に停止させる
            if (
                (np.abs(target - state) > 
                target_state_abs_joint_diff_limit).any()
            ) and (
                last_target is None or
                (np.abs(target - last_target) > 
                target_state_abs_joint_diff_limit).any()
            ):
                # 強制停止する。強制停止しないとエラーメッセージを返すのが複雑になる
                msg = f"Target and state are too different. State: {state}, Target: {target}, Last target: {last_target}, Diff limit: {target_state_abs_joint_diff_limit}"
                with lock:
                    error_info['kind'] = "robot"
                    error_info['msg'] = msg
                    error_info['exception'] = ValueError(msg)
                error_event.set()
                stop_event.set()
                break

            last_target = target
            sw.lap("First target")
            if self.last == 0:
                self.logger.info("Start sending control command")
                # 制御する前に終了する場合即時終了可能
                if stop:
                    break
                self.last = now
                # 目標値を遅延を許して極力線形補間するためのセットアップ
                if use_interp:
                    di = DelayedInterpolator(delay=0.1)
                    di.reset(now, target)
                    target_delayed = di.read(now, target)
                else:
                    target_delayed = target
                self.last_target_delayed = target_delayed
                # 移動平均フィルタのセットアップ（t_intv秒間隔）
                if filter_kind == "original":
                    self.last_control = state
                    _filter = SMAFilter(n_windows=n_windows)
                    _filter.reset(state)
                # 速度制限をフィルタの手前にも入れてみる
                if True:
                    assert filter_kind in ["original", "feedback_pd_traj"]
                    self.last_target_delayed_velocity = np.zeros(6)
                if filter_kind == "original":
                    self.last_control_velocity = np.zeros(6)
                continue

            if (self.last == now):
                time.sleep(0.01)
                continue
            sw.lap("Check stop")
            # 制御値を送り済みの場合は
            # 目標値を状態値にしてロボットを静止させてから止める
            # 厳密にはここに初めて到達した場合は制御値は送っていないが
            # 簡潔さのため同じように扱う
            if stop:
                if target_stop is None:
                    target_stop = state
                target = target_stop

            sw.lap("Read delayed interpolator")
            # target_delayedは、delay秒前の目標値を前後の値を
            # 使って線形補間したもの
            if use_interp:
                target_delayed = di.read(now, target)
            else:
                target_delayed = target

            sw.lap("1st speed limit")
            # 速度制限をフィルタの手前にも入れてみる
            if True:
                assert filter_kind in ["original", "feedback_pd_traj"]
                target_diff = target_delayed - self.last_target_delayed
                # 速度制限
                dt = now - self.last
                v = target_diff / dt
                ratio = np.abs(v) / (speed_limit_ratio * speed_limits)
                max_ratio = np.max(ratio)
                if max_ratio > 1:
                    v /= max_ratio
                target_diff_speed_limited = v * dt

                # 加速度制限
                a = (v - self.last_target_delayed_velocity) / dt
                accel_ratio = np.abs(a) / (accel_limit_ratio * accel_limits)
                accel_max_ratio = np.max(accel_ratio)
                if accel_max_ratio > 1:
                    a /= accel_max_ratio
                v = self.last_target_delayed_velocity + a * dt
                target_diff_speed_limited = v * dt

                # 速度がしきい値より小さければ静止させ無駄なドリフトを避ける
                # NOTE: スレーブモードを落とさないためには前の速度が十分小さいとき (しきい値は不明) 
                # にしか静止させてはいけない
                if np.all(target_diff_speed_limited / dt < stopped_velocity_eps):
                    target_diff_speed_limited = np.zeros_like(
                        target_diff_speed_limited)
                    v = target_diff_speed_limited / dt

                self.last_target_delayed_velocity = v
                target_delayed = self.last_target_delayed + target_diff_speed_limited

            self.last_target_delayed = target_delayed

            sw.lap("Get filtered target")
            # 平滑化
            if filter_kind == "original":
                # 成功している方法
                # 速度制限済みの制御値で平滑化をしており、
                # moveit servoなどでは見られない処理
                target_filtered = _filter.predict_only(target_delayed)
                target_diff = target_filtered - self.last_control
            else:
                raise ValueError

            sw.lap("2nd speed limit")
            if filter_kind != "feedback_pd_traj":
                # 速度制限
                dt = now - self.last
                v = target_diff / dt
                ratio = np.abs(v) / (speed_limit_ratio * speed_limits)
                max_ratio = np.max(ratio)
                if max_ratio > 1:
                    v /= max_ratio
                target_diff_speed_limited = v * dt

                # 加速度制限
                a = (v - self.last_control_velocity) / dt
                accel_ratio = np.abs(a) / (accel_limit_ratio * accel_limits)
                accel_max_ratio = np.max(accel_ratio)
                if accel_max_ratio > 1:
                    a /= accel_max_ratio
                v = self.last_control_velocity + a * dt
                target_diff_speed_limited = v * dt

                # 速度がしきい値より小さければ静止させ無駄なドリフトを避ける
                # NOTE: スレーブモードを落とさないためには前の速度が十分小さいとき (しきい値は不明) 
                # にしか静止させてはいけない
                if np.all(target_diff_speed_limited / dt < stopped_velocity_eps):
                    target_diff_speed_limited = np.zeros_like(
                        target_diff_speed_limited)
                    v = target_diff_speed_limited / dt

                self.last_control_velocity = v

            sw.lap("Get control")
            # 平滑化の種類による対応
            if filter_kind == "original":
                control = self.last_control + target_diff_speed_limited
                # 登録するだけ
                _filter.filter(control)
            else:
                raise ValueError

            sw.lap("Put control to shared memory")
            self.pose[24:30] = control

            sw.lap("Save control - gather data")
            # 分析用データ保存
            datum = [
                dict(
                    time=now,
                    kind="target",
                    joint=target_raw.tolist(),
                ),
                dict(
                    time=now,
                    kind="target_delayed",
                    joint=target_delayed.tolist(),
                ),
                dict(
                    time=now,
                    kind="control",
                    joint=control.tolist(),
                    max_ratio=max_ratio,
                    accel_max_ratio=accel_max_ratio,
                ),
            ]
            sw.lap("Save control - save to queue")
            self.control_to_archiver_queue.put(datum)

            sw.lap("Check elapsed before command")
            t_elapsed = time.time() - now
            if t_elapsed > t_intv * 2:
                # self.logger.warning(
                #     f"Control loop is 2 times as slow as expected before command: "
                #     f"{t_elapsed} seconds")
                # self.logger.warning(sw.summary())
                pass

            if move_robot:
                sw.lap("Send arm command")
                try:
                    self.robot.move_joint_servo(control.tolist())
                except Exception as e:
                    self.logger.warning(f"{self.robot.format_error(e)}")
                    # with lock:
                    #     error_info['kind'] = "robot"
                    #     error_info['msg'] = self.robot.format_error(e)
                    #     error_info['exception'] = e
                    # error_event.set()
                    # stop_event.set()
                    #     break

                if not use_hand_thread:
                    sw.lap("Send hand command")
                    tool = self.pose[13]
                    tool_corrected = tool
                    if tool_corrected != last_tool_corrected:
                        if tool_corrected == 1:
                            th1 = threading.Thread(target=self.send_grip)
                            th1.start()
                        elif tool_corrected == 2:
                            th2 = threading.Thread(target=self.send_release)
                            th2.start()
                        last_tool_corrected = tool_corrected

            sw.lap("Wait control loop")
            t_elapsed = time.time() - now
            t_wait = t_intv - t_elapsed
            if t_wait > 0:
                # if (move_robot and servo_mode == 0x102) or (not move_robot):
                time.sleep(t_wait)

            sw.lap("Check elapsed after command")
            t_elapsed = time.time() - now
            if t_elapsed > t_intv * 2:
                # self.logger.warning(
                #     f"Control loop is 2 times as slow as expected after command: "
                #     f"{t_elapsed} seconds")
                # self.logger.warning(sw.summary())
                pass

            sw.stop()
            if stop:
                # スレーブモードでは十分低速時に2回同じ位置のコマンドを送ると
                # ロボットを停止させてスレーブモードを解除可能な状態になる
                if (control == self.last_control).all():
                    break
                
            self.last_control = control
            self.last = now
 
         # ツールチェンジなど後の制御可能フラグ
        self.pose[41] = 0

        hand_thread.join()
        if error_event.is_set():
            # TODO: これで例外発生元のスタックトレースが取得できればこれで十分
            raise error_info['exception']
        return True

    def send_grip(self) -> None:
        self.robot.toolDo(1,1)
    
    def send_release(self) -> None:
        self.robot.toolDo(1,0)

    def enable(self) -> None:
        try:
            self.robot.enable_robot()
        except Exception as e:
            self.logger.error("Error enabling robot")
            self.logger.error(f"{self.robot.format_error(e)}")

    def disable(self) -> None:
        try:
            self.robot.disable()
        except Exception as e:
            self.logger.error("Error disabling robot")
            self.logger.error(f"{self.robot.format_error(e)}")

    def set_area_enabled(self, enable: bool) -> None:
        try:
            self.robot.SetAreaEnabled(0, enable=enable)
            self.pose[31] = int(enable)
        except Exception as e:
            self.logger.error("Error setting area enabled")
            self.logger.error(f"{self.robot.format_error(e)}")

    def tidy_pose(self) -> None:
        try:
            self.robot.move_joint_until_completion(self.tidy_joint) #共有メモリ越しでなく直で現在角度を受け取りながら制御するため同プロセスでポートに繋ぐ前提．dashPort, feedbackPortで角度は取得可能
        except Exception as e:
            self.logger.error("Error moving to tidy pose")
            self.logger.error(f"{self.robot.format_error(e)}")

    def move_joint(self, joints: List[float]) -> None:
        try:
            self.robot.move_joint_until_completion(joints)
        except Exception as e:
            self.logger.error("Error moving to specified joint")
            self.logger.error(f"{self.robot.format_error(e)}")

    def clear_error(self) -> None:
        try:
            self.robot.clear_error()
        except Exception as e:
            self.logger.error("Error clearing robot error")
            self.logger.error(f"{self.robot.format_error(e)}")

    def leave_servo_mode(self):
        # self.pose[14]は0のとき必ず通常モード。
        # self.pose[14]は1のとき基本的にスレーブモードだが、
        # 変化前後の短い時間は通常モードの可能性がある。
        self.pose[14] = 0

    def control_loop_w_recover_automatic(self) -> bool:
        """自動復帰を含むリアルタイム制御ループ"""
        self.logger.info("Start Control Loop with Automatic Recover")
        # 自動復帰ループ
        while True:
            try:
                # 制御ループ
                # 停止するのは、ユーザーが要求した場合か、自然に内部エラーが発生した場合
                self.control_loop()
                # ここまで正常に終了した場合、ユーザーが要求した場合が成功を意味する
                if self.pose[16] == 1:
                    self.pose[16] = 0
                    self.logger.info("User required stop and succeeded")
                    return True
            except Exception as e:
                # 自然に内部エラーが発生した場合、自動復帰を試みる
                # 自動復帰の前にエラーを確実にモニタするため待機
                time.sleep(1)
                self.logger.error("Error in control loop")
                self.logger.error(f"{self.robot.format_error(e)}")

                # 目標値が状態値から大きく離れた場合は自動復帰しない
                if str(e) == "Target and state are too different":
                    self.pose[16] = 0
                    return False


                # 非常停止ボタンの状態値を最新にするまで待つ必要がある
                time.sleep(1)
                # 非常停止ボタンが押された場合は自動復帰しない
                if self.pose[36] == 1:
                    self.logger.error("Emergency stop is pressed")
                    self.pose[16] = 0
                    return False

                # タイムアウトの場合は接続からやり直す
                # if ((type(e) is ORiNException and
                #     e.hresult == HResult.E_TIMEOUT) or
                #    (type(e) is not ORiNException)):
                #         for i in range(1, 11):
                #             try:
                #                 self.robot.start()
                #                 self.robot.clear_error()

                #                 # NOTE: タイムアウトした場合の数回に1回、
                #                 # 制御権が取得できない場合がある。しかし、
                #                 # このメソッドのこの例外から抜けた後に
                #                 # GUIでClearError -> Enable -> StartMQTTControl
                #                 # とすると制御権が取得できる。
                #                 # ここで制御権を取得しても、GUIから制御権を取得しても
                #                 # 内部的には同じ関数を呼んでいるので原因不明
                #                 # (ソケットやbCAPClientのidが両者で同じことも確認済み)
                #                 # 0. 元
                #                 # self.robot.take_arm()
                #                 # 1. ここをイネーブルにしても変わらない
                #                 # self.robot.enable_robot(ext_speed=speed_normal)
                #                 # 2. manual_resetを追加しても変わらない
                #                 # self.robot.manual_reset()
                #                 # self.robot.take_arm()
                #                 # 3. 待っても変わらない
                #                 # time.sleep(5)
                #                 # self.robot.take_arm()
                #                 # time.sleep(5)

                #                 self.logger.info(
                #                     "Reconnected to robot successfully"
                #                     " after timeout")
                #                 break
                #             except Exception as e_reconnect:
                #                 self.logger.error(
                #                     "Error in reconnecting robot")
                #                 self.logger.error(
                #                     f"{self.robot.format_error(e_reconnect)}")
                #                 if i == 10:
                #                     self.logger.error(
                #                         "Failed to reconnect robot after"
                #                         " 10 attempts")
                #                     self.pose[16] = 0
                #                     return False
                #             time.sleep(1)
                
                # ここまでに接続ができている場合
                # try:
                #     errors = self.robot.get_cur_error_info_all() ##ポートに接続していないからおそらくerrorを見に行けない
                #     self.logger.error(f"Errors in teach pendant: {errors}")
                #     # 自動復帰可能エラー
                #     if self.robot.are_all_errors_stateless(errors):
                #         # 自動復帰を試行。失敗またはエラーの場合は通常モードに戻る。
                #         # エラー直後の自動復帰処理に失敗しても、
                #         # 同じ復帰処理を手動で行うと成功することもあるので
                #         # 手動で操作が可能な状態に戻す
                #         ret = self.robot.recover_automatic_enable()
                #         if not ret:
                #             raise ValueError(
                #                 "Automatic recover failed in enable timeout")
                #         self.logger.info("Automatic recover succeeded")                    
                #     # 自動復帰不可能エラー
                #     else:
                #         self.logger.error(
                #             "Error is not automatically recoverable")
                #         self.pose[16] = 0
                #         return False
                # except Exception as e_recover:
                #     self.logger.error("Error during automatic recover")
                #     self.logger.error(f"{self.robot.format_error(e_recover)}")
                #     self.pose[16] = 0
                #     return False

                self.pose[16] = 0
                return False


    def mqtt_control_loop(self) -> None:
        """MQTTによる制御ループ"""
        self.logger.info("Start MQTT Control Loop")
        self.pose[15] = 1
        while True:
            # 停止するのは、ユーザーが要求した場合か、自然に内部エラーが発生した場合
            success_stop = self.control_loop_w_recover_automatic()
            # 停止フラグが成功の場合は、ユーザーが要求した場合のみありうる
            # next_tool_id = self.pose[17].copy()
            # put_down_box = self.pose[21].copy()
            # line_cut = self.pose[38].copy()
            change_log_file = self.pose[33].copy()
            if success_stop:
                # ログファイルを変更することが要求された場合
                if change_log_file != 0:
                    self.logger.info("User required change log file")
                    # ログファイルを変更する
                    self.get_logging_dir_and_change_log_file()
                # 単なる停止が要求された場合は、ループを抜ける
                else:
                    break
            # 停止フラグが失敗の場合は、ユーザーが要求した場合か、
            # 自然に内部エラーが発生した場合
            else:
                if change_log_file != 0:
                    # 要求コマンドのみリセット
                    self.pose[33] = 0
                break
        self.pose[15] = 0

    def get_tool_info(
        self, tool_infos: List[Dict[str, Any]], tool_id: int) -> Dict[str, Any]:
        return [tool_info for tool_info in tool_infos
                if tool_info["id"] == tool_id][0]

    def tool_change(self, next_tool_id: int) -> None:
        if next_tool_id == self.tool_id:
            self.logger.info("Selected tool is current tool.")
            return
        tool_info = self.get_tool_info(tool_infos, self.tool_id)
        next_tool_info = self.get_tool_info(tool_infos, next_tool_id)
        # ツールチェンジはワークから十分離れた場所で行うことを仮定
        current_joint = self.robot.get_current_joint()
        self.robot.move_joint(self.tidy_joint)
        # ツールチェンジの場所が移動可能エリア外なので、エリア機能を無効にする
        self.robot.SetAreaEnabled(0, False)
        self.pose[31] = 0
        # アームの先端の位置で制御する（現在のツールに依存しない）
        self.robot.set_tool(0)

        # 現在のツールとの接続を切る
        # 現在ツールが付いていないとき
        if tool_info["id"] == -1:
            assert next_tool_info["id"] != -1
        # 現在ツールが付いているとき
        else:
            self.hand.disconnect()

        # 現在ツールが付いていないとき
        if tool_info["id"] == -1:
            assert next_tool_info["id"] != -1
            wps = next_tool_info["holder_waypoints"]
            self.robot.move_pose(wps["enter_path"])
            self.robot.move_pose(wps["disengaged"])
            self.robot.ext_speed(speed_tool_change)
            self.robot.move_pose(wps["tool_holder"])
            time.sleep(1)
            self.robot.move_pose(wps["locked"])
            name = next_tool_info["name"]
            hand = tool_classes[name]()
            connected = hand.connect_and_setup()
            # NOTE: 接続できなければ止めたほうが良いと考える
            if not connected:
                raise ValueError(f"Failed to connect to hand: {name}")
            self.hand_name = name
            self.hand = hand
            self.tool_id = next_tool_id
            self.pose[23] = next_tool_id
            self.robot.ext_speed(speed_normal)
            self.robot.move_pose(wps["exit_path_1"])
            self.robot.move_pose(wps["exit_path_2"])
        # 現在ツールが付いているとき
        else:
            wps = tool_info["holder_waypoints"]
            self.robot.move_pose(wps["exit_path_2"])
            self.robot.move_pose(wps["exit_path_1"])
            self.robot.move_pose(wps["locked"])
            self.robot.ext_speed(speed_tool_change)
            self.robot.move_pose(wps["tool_holder"])
            time.sleep(1)
            self.robot.move_pose(wps["disengaged"])
            if next_tool_info["id"] == -1:
                self.robot.ext_speed(speed_normal)
                self.robot.move_pose(wps["enter_path"])
            elif tool_info["holder_region"] == next_tool_info["holder_region"]:
                wps = next_tool_info["holder_waypoints"]
                self.robot.ext_speed(speed_normal)
                self.robot.move_pose(wps["disengaged"])
                self.robot.ext_speed(speed_tool_change)
                self.robot.move_pose(wps["tool_holder"])
                time.sleep(1)
                self.robot.move_pose(wps["locked"])
                name = next_tool_info["name"]
                hand = tool_classes[name]()
                connected = hand.connect_and_setup()
                # NOTE: 接続できなければ止めたほうが良いと考える
                if not connected:
                    raise ValueError(f"Failed to connect to hand: {name}")
                self.hand_name = name
                self.hand = hand
                self.tool_id = next_tool_id
                self.pose[23] = next_tool_id
                self.robot.ext_speed(speed_normal)
            elif tool_info["holder_region"] != next_tool_info["holder_region"]:
                self.robot.ext_speed(speed_normal)
                self.robot.move_pose(wps["enter_path"])
                self.robot.move_pose(tool_base, fig=-3)
                wps = next_tool_info["holder_waypoints"]
                self.robot.ext_speed(speed_normal)
                self.robot.move_pose(wps["disengaged"])
                self.robot.ext_speed(speed_tool_change)
                self.robot.move_pose(wps["tool_holder"])
                time.sleep(1)
                self.robot.move_pose(wps["locked"])
                name = next_tool_info["name"]
                hand = tool_classes[name]()
                connected = hand.connect_and_setup()
                # NOTE: 接続できなければ止めたほうが良いと考える
                if not connected:
                    raise ValueError(f"Failed to connect to hand: {name}")
                self.hand_name = name
                self.hand = hand
                self.tool_id = next_tool_id
                self.pose[23] = next_tool_id
                self.robot.ext_speed(speed_normal)
            self.robot.move_pose(wps["exit_path_1"])
            self.robot.move_pose(wps["exit_path_2"])
                
        # 以下の移動後、ツールチェンジ前後でのTCP位置は変わらない
        # （ツールの大きさに応じてアームの先端の位置が変わる）
        if next_tool_info["id"] != -1:
            self.robot.SetToolDef(
                next_tool_info["id_in_robot"], next_tool_info["tool_def"])
        self.robot.set_tool(next_tool_info["id_in_robot"])
        self.robot.move_joint(self.tidy_joint)
        # エリア機能を有効にする
        self.robot.SetAreaEnabled(0, True)
        self.pose[31] = 1
        if next_tool_info["id"] == 4:
            # 箱の前だがやや離れた、VRでも到達可能な姿勢
            self.robot.move_joint(
                [-157.55, -18.18, 116.61, 95.29, 67.08, -99.31]
            )
        else:
            # ツールチェンジ後に実機をVRに合わせる場合
            # ツールチェンジ前の位置だけでなく関節角度も合わせる必要がある
            # ツールチェンジ後にVRを実機に合わせる場合は必ずしも
            # その限りではないが、関節空間での補間に悪影響があるかもしれないので
            # 関節角度を前後で合わせることを推奨
            self.robot.move_joint(current_joint)
        return

    def tool_change_not_in_rt(self) -> None:
        while True:
            next_tool_id = self.pose[17]
            if next_tool_id != 0:
                try:
                    self.logger.info(f"Tool change to: {next_tool_id}")
                    self.pose[41] = 0
                    self.tool_change(next_tool_id)
                    self.pose[18] = 1
                except Exception as e:
                    self.logger.error("Error during tool change")
                    self.logger.error(f"{self.robot.format_error(e)}")
                    self.pose[18] = 2
                finally:
                    self.pose[17] = 0
                    self.pose[41] = 1
                    break

    def jog_joint(self, joint: int, direction: float) -> None:
        try:
            self.robot.jog_joint(joint, direction)
        except Exception as e:
            self.logger.error("Error during joint jog")
            self.logger.error(f"{self.robot.format_error(e)}")

    def jog_tcp(self, axis: int, direction: float) -> None:
        try:
            self.robot.jog_tcp(axis, direction)
        except Exception as e:
            self.logger.error("Error during TCP jog")
            self.logger.error(f"{self.robot.format_error(e)}")

    def setup_logger(self, log_queue):
        self.logger = logging.getLogger("CTRL")
        if log_queue is not None:
            self.handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.handler = logging.StreamHandler()
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.INFO)
        self.robot_logger = logging.getLogger("CTRL-ROBOT")
        if log_queue is not None:
            self.robot_handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.robot_handler = logging.StreamHandler()
        self.robot_logger.addHandler(self.robot_handler)
        self.robot_logger.setLevel(logging.WARNING)

    def get_logging_dir_and_change_log_file(self) -> None:
        command = self.control_pipe.recv()
        logging_dir = command["params"]["logging_dir"]
        self.logger.info("Change log file")
        self.change_log_file(logging_dir)

    def change_log_file(self, logging_dir: str) -> None:
        self.logging_dir = logging_dir
        self.pose[33] = 0

    def run_proc(self, control_pipe, slave_mode_lock, log_queue, logging_dir, control_to_archiver_queue):
        self.setup_logger(log_queue)
        self.logger.info("Process started")
        self.sm = mp.shared_memory.SharedMemory(SHM_NAME)
        self.pose = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        self.slave_mode_lock = slave_mode_lock
        self.control_pipe = control_pipe
        self.logging_dir = logging_dir
        self.control_to_archiver_queue = control_to_archiver_queue

        self.init_robot()
        self.init_realtime()
        while True:
            if control_pipe.poll(timeout=1):
                command = control_pipe.recv()
                if command["command"] == "enable":
                    self.enable()
                elif command["command"] == "disable":
                    self.disable()
                elif command["command"] == "set_area_enabled":
                    self.set_area_enabled(**command["params"])
                elif command["command"] == "tidy_pose":
                    self.tidy_pose()
                elif command["command"] == "release_hand":
                    self.logger.info("Release hand")
                    self.send_release()
                elif command["command"] == "line_cut":
                    self.logger.info("Line cut not during MQTT control")
                    self.line_cut()
                elif command["command"] == "clear_error":
                    self.clear_error()
                    self.robot.get_robot_mode()
                    self.robot.get_error_id()
                elif command["command"] == "start_mqtt_control":
                    self.mqtt_control_loop()
                elif command["command"] == "tool_change":
                    self.logger.info("Tool change not during MQTT control")
                    self.tool_change_not_in_rt()
                elif command["command"] == "jog_joint":
                    self.jog_joint(**command["params"])
                elif command["command"] == "jog_tcp":
                    self.jog_tcp(**command["params"])
                elif command["command"] == "move_joint":
                    self.logger.info("Move joint not during MQTT control")
                    self.move_joint(**command["params"])
                elif command["command"] == "demo_put_down_box":
                    self.logger.info("Demo put down box not during MQTT control")
                    self.demo_put_down_box()
                elif command["command"] == "change_log_file":
                    # MQTTControl時以外にログファイルを変更する場合に対応
                    self.logger.info("Change log file")
                    self.change_log_file(**command["params"])
                else:
                    self.logger.warning(
                        f"Unknown command: {command['command']}")
                wait = command.get("wait", False)
                if wait:
                    control_pipe.send({"status": True})
            if self.pose[32] == 1:
                self.robot.close()
                self.sm.close()
                self.control_to_archiver_queue.close()
                time.sleep(1)
                self.logger.info("Process stopped")
                self.handler.close()
                self.robot_handler.close()
                break

# 後回し
class Nova2_CON_Archiver:
    def monitor_start(self, f: TextIO | None = None):
        while True:
            # ログファイル変更時
            if self.pose[35] == 1:
                return True
            try:
                datum = self.control_to_archiver_queue.get(
                    block=True, timeout=T_INTV)
            except queue.Empty:
                datum = None
            if ((f is not None) and 
                (datum is not None)):
                s = ""
                for d in datum:
                    s = s + json.dumps(d, ensure_ascii=False) + "\n"
                f.write(s)
            # プロセス終了時
            if self.pose[32] == 1:
                return False

    def setup_logger(self, log_queue):
        self.logger = logging.getLogger("CTRL-ARCV")
        if log_queue is not None:
            self.handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.handler = logging.StreamHandler()
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.INFO)

    def get_logging_dir_and_change_log_file(self) -> None:
        command = self.control_arcv_pipe.recv()
        logging_dir = command["params"]["logging_dir"]
        self.logger.info("Change log file")
        self.change_log_file(logging_dir)

    def change_log_file(self, logging_dir: str) -> None:
        self.logging_dir = logging_dir
        self.pose[35] = 0

    def run_proc(self, control_arcv_pipe, log_queue, logging_dir, control_to_archiver_queue):
        self.setup_logger(log_queue)
        self.logger.info("Process started")
        self.sm = mp.shared_memory.SharedMemory(SHM_NAME)
        self.pose = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        self.control_arcv_pipe = control_arcv_pipe
        self.logging_dir = logging_dir
        self.control_to_archiver_queue = control_to_archiver_queue

        while True:
            try:
                # 基本はmonitor_start内のループにいるが、
                # ログファイル変更またはプロセス終了時に
                # monitor_startから抜ける
                if save_control:
                    with open(
                        os.path.join(self.logging_dir, "control.jsonl"), "a"
                    ) as f:
                        will_change_log_file = self.monitor_start(f)
                else:
                    will_change_log_file = self.monitor_start()
                # ログファイル変更時は大きいループを継続
                if will_change_log_file:
                    self.get_logging_dir_and_change_log_file()
            except Exception as e:
                self.logger.error("Error in control archiver")
                self.logger.error(e)
            # プロセス終了時は大きいループを抜ける
            if self.pose[32] == 1:
                self.sm.close()
                self.control_to_archiver_queue.close()
                time.sleep(1)
                self.logger.info("Process stopped")
                self.handler.close()
                break