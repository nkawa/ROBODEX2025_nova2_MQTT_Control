import copy
import logging
import os
import time
import traceback
from typing import Any, Dict, List, Literal, Optional, Tuple

import threading

import numpy as np
import pandas as pd
import sys

package_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'vendor'))
sys.path.append(package_dir)

from dobot_api import *
# from dobot.dobot_api import DobotApiDashboard, DobotApiFeedBack, DobotApiDashboard


# エラーコード

# ベースライン
E = 0x100000000

def python_error_to_original_error_str(hr: int) -> str:
    # Reference: https://github.com/ShoheiKobata/orin_bcap_python_samples/blob/master/SimpleSamples/06_error_handling.py
    # 正負で変換式が異なる
    if hr < 0:
        return hex(E + hr)
    else:
        return hex(hr)

def original_error_to_python_error(e: int) -> int:
    return e - E



LABEL_ROBOT_MODE = {
    1:	"ROBOT_MODE_INIT",
    2:	"ROBOT_MODE_BRAKE_OPEN",
    3:	"",
    4:	"ROBOT_MODE_DISABLED",
    5:	"ROBOT_MODE_ENABLE",
    6:	"ROBOT_MODE_BACKDRIVE",
    7:	"ROBOT_MODE_RUNNING",
    8:	"ROBOT_MODE_RECORDING",
    9:	"ROBOT_MODE_ERROR",
    10:	"ROBOT_MODE_PAUSE",
    11:	"ROBOT_MODE_JOG"
}

DOBOT_PORT_TYPES = {
    "dashboard": 29999,
    "motion_control": 30003,
    "realtime_feedback": 30004
}


class Nova2Robot:
    """DOBOT Nova2の制御クラス。

    ハンド OnRobot 2FG7 を使う場合はツール座標系を
    状態取得プロセスと制御プロセスで揃える必要があるので、
    どちらのプロセスから呼び出す場合にもuse_hand = Trueを指定する
    """
    def __init__(
        self,
        name: str = "dobot_nova2",
        default_servo_mode: int = 0x001,
        default_fig: int = -2,
        host: str = "192.168.5.45",
        port: int = [0],
        timeout: float = 5,
        use_hand: bool = False,
        hand_host: str = "192.168.5.46",
        hand_tool_id: int = 1,
        hand_parameters: Dict[str, Any] = {
            "TwofgFingerLength": 8.5,
            "TwofgFTWidth": 5,
            "TwofgOrientation": 2,
            "TwofgGripMode": 1,
        },
        logger: Optional[logging.Logger] = None,
    ):
        if logger is None:
            self.logger = logging.getLogger(__name__)
        else:
            self.logger = logger
        self.name = name
        self._bcap = None
        self._hRob = 0
        self._hCtrl = 0
        assert default_servo_mode in [0x001, 0x101, 0x201, 0x002, 0x102, 0x202]
        self._default_servo_mode = default_servo_mode
        # 形態自動設定
        # -2: 形態が大きく変化するのを抑制する
        # -3: ソフトリミットエラーや可動範囲外エラーにならない
        self._default_fig = default_fig
        self.logger.info(("default_servo_mode", self._default_servo_mode))
        self.logger.info(("default_fig", self._default_fig))
        # パラメータ
        self.host = host
        self.port = port
        self.timeout = timeout
        # 接続しているコントローラからメッセージを取得する周期を ms 単位で指定します．デフォルト値は 100ms です．
        # e.g. "Interval=8"
        # 変えても変化は見られなかった
        self.controller_connect_option = ""
        # Robotモジュール内でハンドを使用する場合
        # ツール座標系番号
        self._use_hand = use_hand
        self._hand_host = hand_host
        self._hand_tool_id = hand_tool_id
        self._hand_parameters = hand_parameters
        self._default_grip_width = None
        self._default_release_width = None
        
        self.global_state = {}
        self.global_state["connect"] = False
        self._raw_feedback = None

    def start(self):
        self.logger.info("Robot start")
        # モーター起動など長い時間がかかるコマンドがあるので、
        # それに合わせてタイムアウト時間は長めに設定する
        # このときスレーブモードの制御コマンドはタイムアウトエラーではなく
        # 指示値生成遅延エラーで通知される
        self.text_log = ""
        # need try
        for p in self.port:
            match(p):
                case 29999:
                    self.client_dash = DobotApiDashboard("192.168.5.1", 29999, self.text_log)
                    self.logger.info("Connect dashboard port")
                case 30003:
                    # ServoP only accepted in this port.
                    self.client_move = DobotApiDashboard("192.168.5.1", 30003, self.text_log)
                    self.logger.info("Connect motion control port")
                case 30004:
                    self.client_feed = DobotApiFeedBack("192.168.5.1", 30004,self.text_log)
                    self.logger.info("Connect feedback port")
                case _:
                    self.logger.warning("Unknown port")

        self.global_state["connect"] = not self.global_state["connect"]
        if hasattr(self, 'client_feed'):
            self._set_feed_back()
            # すぐには取得できない？
        
    def _set_feed_back(self):
        if self.global_state["connect"]:
            thread = threading.Thread(target=self._feed_back)
            thread.daemon = True
            thread.start()
            
    def _feed_back(self):
        last = 0
        self.logger.info("Now running  Feed Back Thread")
        while True:
            now = time.time()
            if last == 0:
                last = now
                continue

            if now - last > 3:
                self._raw_feedback = self.client_feed.feedBackData()
                time.sleep(0.020)   # 20ミリ秒）待つ               
    
     # monitorから叩く
    def get_current_pose(self):
        if self._raw_feedback is None:
            return None
        cur_pos = self._raw_feedback[0][27]
        return cur_pos
        # x, y, z, rx, ry, rz = cur_pos
    
    def get_current_joint(self):
        if self._raw_feedback is None:
            return None
        cur_jnt = self._raw_feedback[0][23]
        return cur_jnt
    
    
    def enable_robot(self) -> bool:
        #本来は以下のようにしたい
        # # STO状態（セーフティ状態）を解除する
        # self.manual_reset()
        # # ティーチングペンダントのエラーをクリアする
        # self.clear_error()
        # self.enable_wo_clear_error()
        try:
            self.client_dash.EnableRobot(load=1.0, centerX=0, centerY=0, centerZ=70) #TCPハードコードしないようにしたい
            return True
        except Exception as e:
            self.logger.error(f"Enable failed")
            self.logger.warning(f"{self.format_error(e)}")
            return False
        
    def disable(self):
        self.logger.info("disable")
        # disableするかのチェックもしたい
        # if self._hRob == 0 or self._bcap is None:
        #     self.logger.warning(f"Disable undone: {self._hRob=}, {self._bcap=}")
        #     return
        try:
            self.client_dash.DisableRobot()
        except Exception as e:
            self.logger.warning("Error disabling motor but ignored.")
            self.logger.warning(f"{self.format_error(e)}")

    # gripperでmodbusとか使わない限りはNova2では不要？
    def stop(self):
        self.logger.info("stop")
        # if self._hRob != 0:
        #     self._bcap.robot_release(self._hRob)
        #     self._hRob = 0
        # if self._hCtrl != 0:
        #     self._bcap.controller_disconnect(self._hCtrl)
        #     self._hCtrl = 0
        # if self._bcap is not None:
        #     self._bcap.service_stop()
        #     self._bcap = None


    def format_error(self, e: Exception) -> str:
        s = "\n"
        s = s + "Error trace: " + traceback.format_exc() + "\n"
        return s
    
    def is_enabled(self):
        if self._raw_feedback is None:
            return None
        is_enable = self._raw_feedback[0][50]
        return bool(is_enable)
    
    def get_robot_mode(self):
        # dashPortからenableかの判定可能!!
        if hasattr(self, 'client_dash'):
            ret = self.client_dash.RobotMode()
            print("get robot mode")
            print(ret)
        else:
            self.logger.warning("client_dash is not initialized")
    
    # !!Nova2で使えるかは不明 Error Code Descriptions: https://docs.trossenrobotics.com/dobot_cr_cobots_docs/tcpip_protocol/functions.html#error-code-descriptions
    def get_error_id(self):
        """
        Return: ErrorID,{[[id,…,id], [id], [id], [id], [id], [id], [id]]},GetErrorID();
        [id,..., id] is the alarm information of the controller and algorithm, 
        where the collision detection value is -2, the safe skin collision detection value is -3.
        The last six [id] represent the alarm information of six servos respectively.
        """
        try:
            ret = self.client_dash.GetErrorID()
            print("get error id")
            print(ret)
        except Exception as e:
            self.logger.warning(f"{self.format_error(e)}")

    def power_on(self):
        if hasattr(self, 'client_dash'):
            ret = self.client_dash.PowerOn()
            # NOTE Once the robot is powered on, you can enable the robot after about 10 seconds.    
        else:
            self.logger.warning("client_dash is not initialized")
    
    def set_collision_level(self,level):
        """"
        Set the collision detection level.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        level     int     collision detection level, 0: switch off collision detection, 1 – 5: the larger the number, the higher the sensitivity.
        """
        try:
            self.client_dash.SetCollisionLevel(level)
        except Exception as e:
            self.logger.warning(f"{self.format_error(e)}")

    
    def clear_error(self):
        if hasattr(self, 'client_dash'):
            self.client_dash.ClearError()
        else:
            self.logger.warning("client_dash is not initialized")

    # gripperでmodbusとか使わない限りはNova2では不要？
    def stop(self):
        self.logger.info("stop")
        # if self._hRob != 0:
        #     self._bcap.robot_release(self._hRob)
        #     self._hRob = 0
        # if self._hCtrl != 0:
        #     self._bcap.controller_disconnect(self._hCtrl)
        #     self._hCtrl = 0
        # if self._bcap is not None:
        #     self._bcap.service_stop()
        #     self._bcap = None
    
    def move_joint_servo(
        self, pose
    ) -> Tuple[float, Tuple[float, float, float, float, float, float]]:
        """
        スレーブモードでの関節制御。

        時間と現在の状態関節を返す。

        この時間はコントローラ内部の時間で、ユーザースクリプトの
        時間と少しずれているので使わないほうがよい。
        """
        str_joint = [str(x) for x in pose]      
        ret = self.client_move.sendRecvMsg("ServoJ("+str_joint[0]+","+str_joint[1]+","+str_joint[2]+","+str_joint[3]+","+str_joint[4]+","+str_joint[5]+")")
        self.logger.info("ServoJ("+str_joint[0]+","+str_joint[1]+","+str_joint[2]+","+str_joint[3]+","+str_joint[4]+","+str_joint[5]+")")
    
    # 真空グリッパ制御
    def toolDo(self, index, status):
        str_index = str(index)
        str_status = str(status)
        ret = self.client_dash.sendRecvMsg("ToolDOExecute("+str_index+","+str_status+")")

    def leave_servo_mode(self):
        self.logger.info("leave_servo_mode")
        pass

    def is_in_servo_mode(self) -> bool:
        """
        スレーブモード中にあるかどうか。
        非常停止中も実行できる。
        """
        # 要フィードバック参照
        return True




    def take_arm(self) -> None:
        """
        制御権の取得要求を行います．
        スレーブモードでは使用できない。
        複数プロセスで同時に制御権を取得することはできない。
        """
        # 引数1: 付加軸を含まないロボットのみのアームグループ番号
        # 引数2: 現在の内部速度，カレントツール番号，カレントワーク番号を変更せず，保持
        self._bcap.robot_execute(self._hRob, "Takearm", [0, 1])

    def take_arm_state(self) -> int:
        """
        指定したアームグループが制御権を取得されているかどうかを返します。
        
        指定したアームグループの制御権がいずれかのタスクに取得されているときは"1"、
        どのタスクにも取得されていないときは"0"を返します。

        自タスクが軸の制御権を取得していないときに、
        引数アームグループに-1を指定した場合は"0"を返します。
        """
        # 引数1: アームグループ番号
        return self._bcap.robot_execute(self._hRob, "TakeArmState", -1)

    def give_arm(self) -> None:
        """
        制御権の解放要求を行います.
        """
        self._bcap.robot_execute(self._hRob, "Givearm")

    def set_tool(self, tool_id: int) -> None:
        """
        ロボットのツール座標系を変更します.
        """
        self.robot_change(f"Tool{tool_id}")

    def wait_until_set_tool(self, timeout: float = 60) -> bool:
        t_start = time.time()
        while True:
            cur_tool = self.CurTool()
            if cur_tool == self._tool:
                return True
            if time.time() - t_start > timeout:
                return False
            time.sleep(1)


    def manual_reset(self) -> None:
        # STO状態（セーフティ状態）を解除する
        # このコマンドの前にセーフティ状態が解除できる状態になっていなければならない
        # 例えば非常停止ボタンをONにしてセーフティ状態に入った場合
        # OFFにしてからこのコマンドを実行しなければセーフティ状態は解除できない
        # さもなくばエラー(0x83500178「非常停止ON中は実行できません。」が出る)
        self._bcap.controller_execute(self._hCtrl, "ManualReset")

    # def clear_error(self) -> None:
    #     """
    #     ティーチングペンダントのエラーをクリアする。
    #     GetCurErrorInfoなどで取得できるエラーもクリアする。
    #     例えば、非常停止中でもエラーをクリアすることはできるが、
    #     非常停止状態がOFFになるわけではないため、
    #     その後一部のコマンドが実行できないことに注意。
    #     気軽にクリアしてしまうと現在のエラーがわからなくなるので注意。
    #     """
    #     if self._hCtrl == 0 or self._bcap is None:
    #         self.logger.warning(
    #             f"ClearError undone: {self._hCtrl=}, {self._bcap=}")
    #         return
    #     self._bcap.controller_execute(self._hCtrl, "ClearError")

    # def enable(self) -> bool:
    #     # STO状態（セーフティ状態）を解除する
    #     self.manual_reset()
    #     # ティーチングペンダントのエラーをクリアする
    #     self.clear_error()
    #     self.enable_wo_clear_error()

    def enable_wo_clear_error(self) -> None:
        # Cobotta Proの手動モードではイネーブル前に
        # マニュアルリセットするのでそれに倣ったフローにしている
        # STO状態（セーフティ状態）を解除する
        self.manual_reset()
        # ロボットの軸の制御権
        # 引数1: 現在の内部速度，カレントツール番号，カレントワーク番号を変更せず，保持
        self._bcap.robot_execute(self._hRob, "Takearm", [0, 1])
        cur_tool = self.CurTool()
        if cur_tool != self._tool:
            self.robot_change(f"Tool{self._tool}")
        if self._use_hand:
            if not self.setup_hand():
                return False
        # 外部速度(%)の設定
        # スレーブモードでは外部速度は反映されない
        self._bcap.robot_execute(self._hRob, "ExtSpeed", [20])
        self._bcap.robot_execute(self._hRob, "Motor", 1)
        return True

    def set_default_pose(self, pose):
        self._default_pose = pose

    def get_default_pose(self):
        return self._default_pose

    def move_default_pose(self):
        self.logger.info("move_default_pose")
        self.move_pose(self._default_pose)

    def move_pose(self, pose, interpolation: int = 1, fig: Optional[int] = None, path: str = "@E", option: str = ""):
        """
        option: 動作オプション。"NEXT": 非同期実行オプション
        """
        pose = self._add_fig_if_necessary(pose)
        if fig is not None:
            pose[6] = fig
        prev_servo_mode = self.is_in_servo_mode()
        if prev_servo_mode:
            self.leave_servo_mode()
        x, y, z, rx, ry, rz, fig = pose
        # 参考：https://www.fa-manuals.denso-wave.com/jp/COBOTTA%20PRO/016664/
        # 第2引数
        # 補間指定
        # 1: MOVE P（PTP動作（最短時間経路））
        # 2: MOVE L (直線補間動作（最短距離経路)）
        # 3: MOVE C（円弧補間動作）
        # 4: MOVE S
        # 第3引数
        # @0は、目標位置に動作指令値が到達したら次のコマンドに移行する。
        # @Eは、目標位置にエンコーダ値（測定位置）が到達するまで待ち停止する。このときの位置は関節角度。
        # @Pは、目標位置の近く（自動設定）にエンコーダ値が到達したら次のコマンドに移行する。
        # @<数字>は、@Pを、目標位置の近くを数字（mm）に設定した上で実行。
        # P(X, Y, Z, Rx, Ry, Rz, Fig)はTCP点の位置、姿勢、形態
        self._bcap.robot_move(self._hRob, interpolation, f"{path} P({x}, {y}, {z}, {rx}, {ry}, {rz}, {fig})", option)
        if prev_servo_mode:
            self.enter_servo_mode()

    def jog_tcp(self, axis: int, direction: float) -> None:
        poses = self.get_current_pose()
        poses = np.asarray(poses)
        poses[axis] += direction
        # 直接補間動作、形態が大きく変化するのを抑制する、繰り返し動作させるので途中で毎回停止させない
        self.move_pose(poses.tolist(), interpolation=2, fig=-3, path="@0")

    def move_pose_by_diff(self, diff: List[float], option: str = "") -> None:
        current_pose = self.get_current_pose()
        target_pose = (np.asarray(current_pose) + np.asarray(diff)).tolist()
        self.move_pose(target_pose, option=option)

    def move_pose_until_completion(
        self,
        pose: List[float],
        precisions: Optional[List[float]] = None,
        check_interval: float = 1,
        timeout: float = 60,
    ) -> None:
        self.move_pose(pose)
        if precisions is None:
            precisions = [1, 1, 1, 1, 1, 1]
        precisions = np.asarray(precisions)
        t_start = time.time()
        while True:
            current_pose = self.get_current_pose()
            diff = np.abs(np.asarray(current_pose) - np.asarray(pose))
            if np.all(diff < precisions):
                done = True
                break
            time.sleep(check_interval)
            if time.time() - t_start > timeout:
                self.logger.info("Timeout before reaching destination.")
                done = False
                break
        # 位置が十分近くなった後念のため少し待つ
        time.sleep(1)
        return done

    def move_joint(self, joint, option: str = ""):
        """
        option: 動作オプション。"NEXT": 非同期実行オプション
        """
        prev_servo_mode = self.is_in_servo_mode()
        if prev_servo_mode:
            self.leave_servo_mode()
        # 参考：https://www.fa-manuals.denso-wave.com/jp/COBOTTA%20PRO/016664/
        # 第2引数
        # 補間指定
        # 1: MOVE P（PTP動作（最短時間経路））
        # 2: MOVE L (直線補間動作（最短距離経路)）
        # 3: MOVE C（円弧補間動作）
        # 4: MOVE S
        # 第3引数
        # @Eは、目標位置にエンコーダ値（測定位置）が到達するまで待ち停止する。このときの位置は関節角度。
        # @Pは、目標位置の近く（自動設定）にエンコーダ値が到達したら次のコマンドに移行する。
        # @<数字>は、@Pを、目標位置の近くを数字（mm）に設定した上で実行。
        # J(J1, J2, J3, J4, J5, J6, J7, J8)はTCP点の位置、姿勢、形態
        # Cobotta Pro 900では関節数は6
        joint_all = [0] * 8
        for i, j in enumerate(joint):
            joint_all[i] = j
        joint_str = ', '.join(map(str, joint_all))
        self._bcap.robot_move(self._hRob, 1, f"@E J({joint_str})", option)
        if prev_servo_mode:
            self.enter_servo_mode()

    def jog_joint(self, joint: int, direction: float) -> None:
        joints = self.get_current_joint()
        joints = np.asarray(joints)
        joints[joint] += direction
        self.move_joint(joints.tolist())

    def move_joint_until_completion(
        self,
        pose: List[float],
        precisions: Optional[List[float]] = None,
        check_interval: float = 1,
        timeout: float = 60,
    ) -> None:
        self.move_joint(pose)
        if precisions is None:
            precisions = [1, 1, 1, 1, 1, 1]
        precisions = np.asarray(precisions)
        t_start = time.time()
        while True:
            current_pose = self.get_current_joint()
            diff = np.abs(np.asarray(current_pose) - np.asarray(pose))
            if np.all(diff < precisions):
                done = True
                break
            time.sleep(check_interval)
            if time.time() - t_start > timeout:
                self.logger.info("Timeout before reaching destination.")
                done = False
                break
        # 位置が十分近くなった後念のため少し待つ
        time.sleep(1)
        return done

    # def get_current_pose(self):
    #     cur_pos = self._bcap.robot_execute(self._hRob, "CurPos")
    #     # x, y, z, rx, ry, rz, fig = cur_pos

    #     # ロボットコントローラからタイムスタンプも追加で返すことができる
    #     # cur_pos_ex = bcap.robot_execute(hRob, "CurPosEx")
    #     # t, x, y, z, rx, ry, rz, fig = cur_pos_ex
    #     # t: コントローラ電源ONからの時間（msec）
    #     # 他の処理と比較するには同じプログラムで時間を計算したほうが便利なので
    #     # 使用しない
    #     # NOTE: b-CAPからハンドを使うとデータが届かないためcur_posがNoneになることがある
    #     return cur_pos[:6]

    def enter_servo_mode(self):
        self.logger.info("enter_servo_mode")
        self.enter_servo_mode_by_mode(self._default_servo_mode)

    # def leave_servo_mode(self):
    #     self.logger.info("leave_servo_mode")
    #     self.slvChangeMode = 0x000
    #     if self._hRob == 0 or self._bcap is None:
    #         self.logger.warning(
    #             f"leave_servo_mode undone: {self._hRob=}, {self._bcap=}")
    #         return
    #     self._bcap.robot_execute(self._hRob, "slvChangeMode", self.slvChangeMode)

    # def disable(self):
    #     self.logger.info("disable")
    #     if self._hRob == 0 or self._bcap is None:
    #         self.logger.warning(f"Disable undone: {self._hRob=}, {self._bcap=}")
    #         return
    #     try:
    #         self._bcap.robot_execute(self._hRob, "Motor", 0)
    #     except Exception as e:
    #         self.logger.warning("Error disabling motor but ignored.")
    #         self.logger.warning(f"{self.format_error(e)}")
    #     self._bcap.robot_execute(self._hRob, "Givearm")

    # def stop(self):
    #     self.logger.info("stop")
    #     if self._hRob != 0:
    #         self._bcap.robot_release(self._hRob)
    #         self._hRob = 0
    #     if self._hCtrl != 0:
    #         self._bcap.controller_disconnect(self._hCtrl)
    #         self._hCtrl = 0
    #     if self._bcap is not None:
    #         self._bcap.service_stop()
    #         self._bcap = None

    def _format_servo_mode(self, servo_mode: int) -> str:
        # (ex.) s = "0x100"
        s = f"{servo_mode:#05x}"
        return s

    def get_suggested_servo_interval(self):
        base = 0.008
        s = self._format_servo_mode(self._default_servo_mode)
        move_mode = s[2]
        if move_mode == "0":
            return base
        elif move_mode == "1":
            return base
        elif move_mode == "2":
            return 0

    # def is_in_servo_mode(self) -> bool:
    #     """
    #     スレーブモード中にあるかどうか。
    #     非常停止中も実行できる。
    #     """
    #     if self._hRob == 0 or self._bcap is None:
    #         self.logger.warning(
    #             f"is_in_servo_mode undone: {self._hRob=}, {self._bcap=}")
    #         return False
    #     return self._bcap.robot_execute(self._hRob, "slvGetMode") != 0x000

    def enter_servo_mode_by_mode(self, mode: int = 0x001):
        # スレーブモードで実行可
        # STO状態（セーフティ状態）を解除する
        self.manual_reset()
        # return以降ははスレーブモードでなければ実行不可
        if self.is_in_servo_mode():
            return
        # スレーブモードの出力設定
        s = self._format_servo_mode(mode)
        type_mode = s[4]
        # 0x001はタイムスタンプを追加
        recv_format_str = "0x001" + type_mode
        recv_format_int = int(recv_format_str, base=16)
        self.set_servo_format(output_mode=recv_format_int, timestamp_unit=1)
        # スレーブモードでは実行不可
        self.clear_error()
        # 0x000: モード解除、0x001: P型モード0設定、0x101: P型モード1設定、0x201: P型モード2設定
        # エラー時には自動的に0に戻る
        # スレーブモードでは実行不可
        self.slvChangeMode = mode
        self._bcap.robot_execute(self._hRob, "slvChangeMode", self.slvChangeMode)

    def move_pose_servo(self, pose) -> Tuple[float, Tuple[float, float, float, float, float, float]]:
        """
        スレーブモードでの位置制御。

        時間と現在の状態位置を返す。

        この時間はコントローラ内部の時間で、ユーザースクリプトの
        時間と少しずれているので使わないほうがよい。
        """
        self.logger.debug("move_pose_servo")
        pose = self._add_fig_if_necessary(pose)
        # 内部でrecvをしない場合は少し高速化する
        # ret = self._move_pose_servo_mode(pose)
        # t, ret_pose = ret
        # ret_pose = ret_pose[:6]
        # return t, ret_pose
        self._move_pose_servo_mode(pose)


    def _move_pose_servo_mode(self, pose):
        s = self._format_servo_mode(self.slvChangeMode)
        move_mode = s[2]
        ret = None
        if move_mode == "0":
            # スレーブモード0は必要なデータが返ってくる
            ret = self._move_pose_servo_mode_0(pose)
        elif move_mode == "1":
            self._move_pose_servo_mode_1(pose)
        elif move_mode == "2":
            # スレーブモード2は待機することが重要
            self._move_pose_servo_mode_2(pose)
        else:
            raise ValueError
        return ret

    def try_restart(self, e: Exception) -> bool:
        if type(e) is ORiNException:
            hr = e.hresult
            if hr < 0:
                if hr in (
                    E_VEL_LARGE_JOINTS +
                    E_ACCEL_LARGE_JOINTS +
                    [E_NOT_IN_SLAVE_MODE] +
                    [E_MOTOR_OFF] +
                    [E_ORDER_DELAY]
                ):
                    self.recover_automatic_servo()
                    time.sleep(1)
                    return True
        return False

    def _move_pose_servo_mode_0(self, target_pose):
        while True:
            try:
                ret = self._bcap.robot_execute(
                    self._hRob, "slvMove", target_pose
                )
                return ret
            except ORiNException as e:
                hr = e.hresult
                # モード0はバッファオーバーフロー時の挙動を
                # ユーザーが選択できるがここではモード2と同じく
                # 何もせずに同じコマンドを送るようにしている
                # 通信の分だけこちらの方が間隔にばらつきがでるかもしれない
                if hr == E_BUF_FULL:
                    continue
                raise e
            except Exception as e:
                raise e

    def _move_pose_servo_mode_1(self, target_pose):
        # どっちでも把持と両立
        # with self._bcap._lock:
        #     self._bcap._bcap_send(self._bcap._serial, self._bcap._version, 64, [self._hRob, "slvMove", target_pose])
        #     if self._bcap._serial >= 0xFFFF:
        #         self._bcap._serial  = 1
        #     else:
        #         self._bcap._serial += 1
        self._bcap.robot_execute(self._hRob, "slvMove", target_pose)

    def _move_pose_servo_mode_2(self, target_pose):
        # ここでrecvしなくても他のコマンドでrecvするときにb-CAPでのハンドの制御と競合して意味がない
        # with self._bcap._lock:
        #     self._bcap._bcap_send(self._bcap._serial, self._bcap._version, 64, [self._hRob, "slvMove", target_pose])
        #     if self._bcap._serial >= 0xFFFF:
        #         self._bcap._serial  = 1
        #     else:
        #         self._bcap._serial += 1
        self._bcap.robot_execute(self._hRob, "slvMove", target_pose)

    def set_servo_format(self, output_mode: int = 0x0001, timestamp_unit: int = 0):
        """
        output_mode:
          0x0001: 位置のみ
          0x0011: タイムスタンプと位置
        timestamp_unit:
          0: ms
          1: us
        """
        self._bcap.robot_execute(self._hRob, "slvRecvFormat", [output_mode, timestamp_unit])

    def stop_move_pose_servo(self, last_target_pose):
        self.move_pose_servo(last_target_pose)
        if not self.is_in_servo_mode():
            self.recover_automatic_servo()

    def stop_move_joint_servo(self, last_target_pose):
        self.move_joint_servo(last_target_pose)
        if not self.is_in_servo_mode():
            self.recover_automatic_servo()

    def StoState(self) -> bool:
        """
        STO 状態（セーフティ状態）を返します。
        非常停止中も実行できます。
        """
        return self._bcap.controller_execute(self._hCtrl, "StoState")

    def get_cur_error_info_all(self) -> List[Dict[str, Any]]:
        """
        現在発生しているエラーの情報を返す。
        非常停止中も実行できる。
        """
        # スレーブモード時実行不可
        n_current_error_infos = self._bcap.controller_execute(self._hCtrl, "GetCurErrorCount")
        # iは0が最新なので古い順から返す
        infos = []
        for i in list(range(n_current_error_infos))[::-1]:
            info = self.get_cur_error_info(i)
            infos.append(info)
        return infos

    def get_cur_error_info(self, i: int) -> Dict[str, Any]:
        """現在発生しているエラーの情報を返す"""
        info = self._bcap.controller_execute(self._hCtrl, "GetCurErrorInfo", i)
        py_err_code = info[0]  # エラーコード
        err_msg = info[1]
        # sub_code = info[2]  # サブコード、観測範囲でエラーコードと一致
        org_err_code_str = python_error_to_original_error_str(py_err_code)
        # APIからdatetime型のUTCの現在時刻は返ってきて正しいがとりあえず使わない
        # timestamp = info[7]
        return {
            "error_code": org_err_code_str,
            "error_message": err_msg,
        }

    def get_error_log(self, i: int) -> Dict[str, Any]:
        """エラーログの情報を取得する"""
        log = self._bcap.controller_execute(self._hCtrl, "GetErrorLog", i)
        py_err_code = log[0]  # エラーコード
        # sub_code = log[13]  # オリジナルエラーコード、観測範囲でエラーコードと一致
        org_err_code_str = python_error_to_original_error_str(py_err_code)
        err_msg = log[12]
        # APIから現在時刻は返ってくるが正しくない
        # year, month, weekday, day, hour, minute, second, millisecond = log[1:9]
        # timestamp = datetime.datetime(
        #     year, month, day, hour, minute, second,
        #     microsecond=millisecond * 10 ** 3)
        timestamp = time.time()
        return {
            "time": timestamp,
            "error_code": org_err_code_str,
            "error_message": err_msg,
        }

    def GetCurErrorInfoAll(self) -> bool:
        """現在発生しているエラーの情報を返します"""
        # スレーブモード中実行不可
        # エラーの情報: エラーコード、エラーメッセージ、サブコード、ファイルID＋行番号、プログラム名、行番号、ファイルID
        n_errors = self._bcap.controller_execute(self._hCtrl, "GetCurErrorCount")
        for i in range(n_errors):
            # i = 0が最新のエラー
            self.logger.error(self._bcap.controller_execute(self._hCtrl, "GetCurErrorInfo", i))

    def GetErrorLogAll(self) -> bool:
        """エラーログの情報を取得します"""
        # 現在発生しているもの以外のすべてのエラーの情報を返すため、通常は使用しないと思われる
        # エラーログ: エラーコード、時間、プログラム名、行番号、エラーメッセージ、オリジナルエラーコード、呼び出し元、IPアドレス
        n_errors = self._bcap.controller_execute(self._hCtrl, "GetErrorLogCount")
        for i in range(n_errors):
            # i = 0が最新のエラー
            self.logger.error(self._bcap.controller_execute(self._hCtrl, "GetErrorLog", i))

    def SceneInfo(self):
        cur_scene = self._bcap.robot_execute(self._hRob, "CurScene")
        cur_sub_scene = self._bcap.robot_execute(self._hRob, "CurSubScene")
        scene_info = self._bcap.robot_execute(self._hRob, "SceneInfo", [cur_scene, cur_sub_scene])
        self.logger.info(f"{scene_info=}")

    def motion_skip(self):
        self._bcap.robot_execute(self._hRob, "MotionSkip", [-1, 0])

    def log_error(self, e: Exception):
        self.logger.error("Error trace:", exc_info=True)
        if type(e) is ORiNException:
            self.logger.error("ORiN exception in controller")
            if self._hCtrl == 0:
                self.logger.error("Controller handler is dead")
            else:
                self.logger.error("Controller handler is alive")
            hr = e.hresult
            self.logger.error(f"Error code: {python_error_to_original_error_str(hr)}")
            desc = self._bcap.controller_execute(self._hCtrl, "GetErrorDescription", hr)
            self.logger.error(f"Error description: {desc}")

    def format_error_wo_desc(self, e: Exception) -> str:
        try:
            s = "\n"
            s = s + "Error trace: " + traceback.format_exc() + "\n"
            if type(e) is ORiNException:
                s += "Error type: ORiN exception in controller\n"
                hr = e.hresult
                s += f"Error code: {python_error_to_original_error_str(hr)}\n"
            return s
        # エラーフォーマット時に例外を起こさないようにする
        # 例えばGetErrorDescriptionはタイムアウトの場合に例外を投げることを確認している
        # のでそれらをキャッチしておく
        except Exception as e:
            s += "Error in format error; during handling of the above exception, " \
                 "another exception occurred:\n\n"
            s = s + "Error trace: " + traceback.format_exc() + "\n"
            return s

    def is_in_range(self, target_pose) -> bool:
        # 返り値は、
        # 0: 可動範囲内、1~63: ソフトリミットである軸のビット、
        # -1: 軸構成上計算不可能な位置、-2: 特異点
        ret = self._bcap.robot_execute(self._hRob, "OutRange", target_pose)
        return ret == 0

    def recover_automatic_servo(self, max_trials: int = 3):
        """
        エラー状態からスレーブモードまで自動復帰する。
        recover_automatic_enableの使用を推奨。
        """
        # 以下の方法で復帰できるエラーに対してのみ呼ぶこと
        # 頻繁に呼ばれうるので無駄な処理は入れないこと

        # 位置監視以外のエラーは自動で復帰できる
        # STO状態（セーフティ状態）を解除する
        self.manual_reset()
        # ティーチングペンダントのエラーをクリアする
        self.clear_error()
        # 不要。制御権は解除されない
        # self._bcap.robot_execute(self._hRob, "Takearm", [0, 1])
        # 不要。スレーブモードに外部速度は関係ない
        # self._bcap.robot_execute(self._hRob, "ExtSpeed", [20])

        # モーターをできるだけONにしようとする
        # モーター損傷回避のためmax_trials以上はエラーとしておく
        i_trials = 0
        while True:
            while True:
                i_trials += 1
                try:
                    # 第3引数のリストの2番目はデフォルトで0で完了待ち（ブロッキング）
                    self._bcap.robot_execute(self._hRob, "Motor", [1, 0])
                    break
                # E_MOTOR_ON_WHILE_OFF_TRANSITIONの場合にやり直す
                # 他の場合はエラー処理しない
                except ORiNException as e:
                    if i_trials == max_trials:
                        raise e
                    if e.hresult == E_MOTOR_ON_WHILE_OFF_TRANSITION:
                        # エラークリアが必要
                        self.clear_error()
                        # 0.008だと待ちすぎなので下げる
                        time.sleep(0.001)
                    else:
                        raise e
                except Exception as e:
                    raise e

            # モーターがONになったかどうかはslvChangeModeが成功してはじめてわかる
            try:
                # スレーブモードがOFFになっているのでONにする
                # 元のslvChangeModeを使う
                self._bcap.robot_execute(self._hRob, "slvChangeMode", self.slvChangeMode)
                break
            # E_MOTOR_OFFの場合にやり直す
            # 他の場合はエラー処理しない
            except ORiNException as e:
                if i_trials == max_trials:
                    raise e
                if e.hresult == E_MOTOR_OFF:
                    # エラークリアが必要
                    self.clear_error()
                    time.sleep(0.001)
                else:
                    raise e
            except Exception as e:
                raise e
    
    def recover_automatic_enable(self, timeout: float = 10) -> bool:
        """エラー状態からイネーブル状態まで自動復帰する"""
        # STO状態（セーフティ状態）を解除する
        self.manual_reset()
        # ティーチングペンダントのエラーをクリアする
        self.clear_error()
        # 制御権を取得
        self.take_arm()
        # モータをONにし、完了待ちする
        # 観測範囲では完了してもモータがONでないことがある
        self._bcap.robot_execute(self._hRob, "Motor", [1, 0])
        t_start = time.time()
        while True:
            # モータがONかを別の方法で確認する
            if self.is_enabled():
                return True
            else:
                if time.time() - t_start > timeout:
                    return False
            time.sleep(0.008)

    def _add_fig_if_necessary(self, pose):
        assert len(pose) in [6, 7]
        if len(pose) == 6:
            pose = copy.deepcopy(pose)
            pose.append(self._default_fig)
        return pose

    def __del__(self):
        self.leave_servo_mode()
        self.disable()
        self.stop()
        self.logger.info("Robot deleted")

    # def get_current_joint(self):
    #     # コントローラ内部で一定周期（8ms）に更新された現在位置をJ 型で取得する
    #     # 8関節分出るがCobotta Pro 900は6関節分のみ有効
    #     # 非常停止中も実行できる
    #     # 所要時間は1ms程度
    #     cur_jnt = self._bcap.robot_execute(self._hRob, "CurJnt")
    #     return cur_jnt[:6]

    def cur_spd(self):
        # 内部速度の設定値を返す
        return self._bcap.robot_execute(self._hRob, "CurSpd")

    def cur_acc(self):
        # 内部加速度の設定値を返す
        return self._bcap.robot_execute(self._hRob, "CurAcc")

    def cur_ext_spd(self):
        # 外部速度の設定値を返す
        return self._bcap.robot_execute(self._hRob, "CurExtSpd")

    def cur_ext_acc(self):
        # 外部加速度の設定値を返す
        return self._bcap.robot_execute(self._hRob, "CurExtAcc")

    def speed(self, value: float):
        # 内部速度を設定する(100%まで)
        # -1: 手先速度を表す
        # 通常TakeArmで100%に初期化される
        self._bcap.robot_speed(self._hRob, -1, value)

    def ext_speed(
        self,
        speed: float = 20,
        accel: float = -2, 
        decel: float = -2,
    ):
        # 外部速度、加速度、減速度を設定する(100%まで。-1はそのまま、-2は
        # 外部速度の二乗を100で割った値)
        # 実測度は外部速度と内部速度の掛け算で決定される
        return self._bcap.robot_execute(
            self._hRob,
            "ExtSpeed",
            [speed, accel, decel],
        )

    def accelerate(self, accel: float = -1, decel: float = -1):
        # 内部加速度、減速度を設定する(100%まで。-1はそのまま)
        # -1: 手先加速度を表す
        # 通常TakeArmで100%に初期化される
        self._bcap.robot_accelerate(self._hRob, -1, accel, decel)

    def motion_complete(self, mode: int = 1) -> bool:
        """
        mode:
            0: 動作命令完了状態取得
            1: 動作完了状態取得
        """
        return self._bcap.robot_execute(self._hRob, "MotionComplete", [-1, mode])

    def move_pose_by_diff_until_completion(
        self,
        diff: List[float],
        precisions: Optional[List[float]] = None,
        check_interval: float = 1,
        timeout: float = 60,
    ) -> None:
        current_pose = self.get_current_pose()
        target_pose = (np.asarray(current_pose) + np.asarray(diff)).tolist()
        self.move_pose_until_completion(
            target_pose,
            precisions=precisions,
            check_interval=check_interval,
            timeout=timeout,
        )

    def CurTool(self) -> int:
        """
        現在のツール番号を取得します.

        所要時間は1ms程度
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "CurTool",
        )
        return ret

    def GetToolDef(self, tool_no: int) -> List[float]:
        """
        ツール番号で指定したツール定義を取得します.

        PacScriptのToolPosに対応。

        ツール座標系のメカニカルインターフェース座標系からのオフセットを
        [x, y, z, rx, ry, rz]の順で返す。
        ツール座標系の原点の位置をTCP位置という。
        例えばアーム先端 (フランジ) にハンドとしてOnRobotの2FG7を付けると、
        TCP位置はアーム先端ではなくハンド先端が望ましいのでツール座標系を使う。
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "GetToolDef",
            tool_no,
        )
        return ret
    
    def SetToolDef(self, tool_no: int, tool_pos: List[float]) -> None:
        """
        ツール番号で指定したツール定義を設定します.

        PacScriptのToolに対応。

        制御権が必要。

        このコマンドで設定した値は、コントローラの電源OFFまで有効ですが、
        電源OFF後は保持されません。
        """
        x, y, z, rx, ry, rz = tool_pos
        tool_def = f"P({x}, {y}, {z}, {rx}, {ry}, {rz})"
        self._bcap.robot_execute(
            self._hRob,
            "SetToolDef",
            [tool_no, tool_def],
        )

    def robot_change(self, name: str) -> None:
        """
        ロボットのツール座標系/ワーク座標系を変更します.

        このメソッドは PacScript 言語の CHANGETOOL 及び CHANGEWORK 命令に対応。

        引数例:
            name = "Tool1"
            name = "Work1"

        このコマンドで変更した座標系番号は、コントローラの電源OFF時まで保持されます。
        電源ON時には元の座標系番号に戻ります。

        このコマンドを実行するには、タスクがロボット軸の制御権を取得しなければなりません。
        """
        ret = self._bcap.robot_change(
            self._hRob,
            name,
        )
        return ret

    def HandRelease(self) -> None:
        """
        各ハンドに設定された開放動作用のコマンドを，
        パラメータに記録された値を引数にして実行します.

        PacScript 言語の HandRelease 命令に対応。
        NOTE: PacScriptのWebマニュアルに記載なし。パラメータも不明。

        ツール座標系をTool0 (メカニカルインターフェース座標系と一致)
        以外に変換する必要がある
        メカニカルインターフェース座標系は、
        アーム先端のフランジ中心を原点、アーム先端の方向をz軸、
        それに垂直な方向をx軸、y軸とした座標系
        ツール座標系はメカニカルインターフェース座標系を、
        オフセットした座標系で、原点はTCPと呼ばれる
        (ロボットハンドではフランジの先にハンドを付け、
        ハンド先端を原点としたいため)
        ツール座標系の変換の影響を受けるのはApproach, Departなど
        一部のコマンド。Moveなどには影響しない
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "HandRelease",
        )
        return ret

    def HandGrip(self) -> None:
        """
        各ハンドに設定された把持動作用のコマンドを，パラメータに記録された値を引数にして実行します.

        PacScript 言語の HandGrip 命令に対応

        NOTE: PacScriptのWebマニュアルに記載なし。パラメータも不明。
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "HandGrip",
        )
        return ret

    def TwofgIsConn(self) -> int:
        """
        2FG7 との接続状態を取得します.

        0：未接続
        1：接続中
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgIsConn",
        )
        return ret

    def TwofgIsBusy(self) -> int:
        """
        動作中かどうかを取得します．

        0：停止中
        1：動作中
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgIsBusy",
        )
        return ret

    def TwofgIsGrip(self) -> int:
        """
        ワークを把持しているかどうかを取得します．

        0：ワークを把持していない
        1：ワークを把持している
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgIsBusy",
        )
        return ret

    def TwofgGetErrorCode(self) -> int:
        """
        エラー状態を取得します．

        0：エラーなし
        1：較正エラー
        2：線形センサーエラー
        3：較正エラーと線形センサーエラー
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgGetErrorCode",
        )
        return ret

    def TwofgGetWidth(self) -> int:
        """
        フィンガー間の幅を取得します．
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgGetWidth",
        )
        return ret

    def TwofgGetMinWidth(self) -> int:
        """
        現在の設定でのフィンガー間の幅の最小値を取得します．
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgGetMinWidth",
        )
        return ret

    def TwofgGetMaxWidth(self) -> int:
        """
        現在の設定でのフィンガー間の幅の最大値を取得します．
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgGetMaxWidth",
        )
        return ret

    def TwofgGetForce(self) -> int:
        """
        把持力を取得します．
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgGetForce",
        )
        return ret

    def TwofgGetOrientation(self) -> int:
        """
        フィンガーの取り付け向きを取得します．

        1：内向き
        2：外向き
        
        内向きのほうが、外向きより、フィンガー間の距離が狭くなる取り付け方である
        |  | 内向き | 外向き |
        | - | - | - |
        | 外部把持範囲[mm] | 1-39 | 35-73 |
        | 内部把持範囲[mm] | 11-49 | 45-83 |

        内向きにすると外部把持範囲よりフィンガー間の内側の距離は1mmまで縮められる。
        フィンガーの厚み5mmと仮定すると、フィンガー間の外側の距離は11mmとなり、
        これは内部把持範囲と一致する。
        これよりフィンガーの厚みの仮定は正しい。
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgGetOrientation",
        )
        return ret

    def TwofgGetFingerLength(self) -> float:
        """
        フィンガーの長さを取得します．

        フィンガー: ハンドの先のL字パーツ。ネジの進む方向から見て、
        視線に垂直な平面上での、パーツのネジ穴からグリップまでの距離が、
        フィンガーの長さである。
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgGetFingerLength",
        )
        return ret

    def TwofgGetFTWidth(self) -> float:
        """
        フィンガーチップの幅を取得します．

        データシートから予測するにフィンガー (L字パーツ)の厚み。
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgGetFTWidth",
        )
        return ret

    def TwofgGetMode(self) -> int:
        """
        把持モード（外部把持・内部把持）を取得します．

        1：外部把持
        2：内部把持

        外部把持 (外径把持、ピンチ把持、external grip): ワーク（物体）の外側から把持する。
        内部把持 (内径把持、インサート把持、internal grip): ハンドのつめ部分を
        ワークの穴の部分に差し込んでからつめを外側に広げ、穴の内側の壁をつかむ。
        穴のあるワークに最適。
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgGetMode",
        )
        return ret

    def TwofgSetOrientation(self, orientation: int) -> None:
        """
        フィンガーの取り付け向きを設定します．

        1：内向き
        2：外向き
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgSetOrientation",
            orientation,
        )
        return ret

    def TwofgSetFingerLength(self, length: float) -> None:
        """
        フィンガーの長さを設定します．
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgSetFingerLength",
            length,
        )
        return ret

    def TwofgSetFTWidth(self, width: float) -> None:
        """
        フィンガーチップの幅を設定します．
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgSetFTWidth",
            width,
        )
        return ret

    def TwofgSwitchToExternal(self) -> None:
        """
        フィンガーの外側の面で把持を行う外部把持モードに移行します．

        NOTE: デンソーのユーザーマニュアルの上記のモードの説明は、
        OnRobotや一般的なモードの説明の逆になっているが実装は問題ないか?
                """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgSwitchToExternal",
        )
        return ret

    def TwofgSwitchToInternal(self) -> None:
        """
        フィンガーの内側の面で把持を行う内部把持モードに移行します．

        NOTE: デンソーのユーザーマニュアルの上記のモードの説明は、
        OnRobotや一般的なモードの説明の逆になっているが実装は問題ないか?
        """
        ret = self._bcap.robot_execute(
            self._hRob,
            "TwofgSwitchToInternal",
        )
        return ret

    def TwofgGrip(
        self,
        width: Optional[float] = None,
        force: int = 20,
        speed: int = 10,
        waiting: int = 1,
    ) -> None:
        """
        指定した設定で把持動作を実行します．

        TwofgGrip(<fWidth>, <lForce>, <lSpeed>, <lWaiting>)
        <fWidth> ： 目標となフィンガー間の幅[mm]（VT_R4）
        <lForce> ： ワークを把持する力[N]（VT_I4）
        <lSpeed> ： フィンガーの開閉速度[%]（VT_I4）
        <lWaiting> ： 待機設定（VT_I4）
                        0：待機しない
                        1：目標ワーク幅に達するまで待機する

        OR_2FG_GRIP(instance, width, force, speed, waiting)
        width シングル 現在のモードで適切な幅を [mm]単位、小数点精度 1 で定
        義します。
        force 整数 適切な把持力を [N]単位で定義します。有効な範囲は 20
        ～140N です。
        speed 整数 適切な把持速度を [%]単位で定義します。有効な範囲は
        10～100%です。
        waiting 整数
        0：プログラムはグリッパーが動作中も続行されます。
        1：プログラムはグリッパーが動作を終了するまで待って
        から把持のステータスをチェックします。把持が発生し
        ていなかった場合、プログラムは停止します。
        """
        if width is None:
            if self._default_grip_width is not None:
                width = self._default_grip_width
            else:
                mode = self.TwofgGetMode()
                # 外部把持
                if mode == 1:
                    width = self.TwofgGetMinWidth()
                # 内部把持
                elif mode == 2:
                    width = self.TwofgGetMaxWidth()
                else:
                    raise ValueError(mode)
            

        # スタンダードな方法 (robot_execute) で呼び出すと
        # メインスレッドの制御ループに遅延をもたらす
        # | 条件 | 遅延 (s) |
        # | - | - |
        # | waiting = 0, メインスレッドで呼び出し | 1s |
        # | waiting = 0, 別スレッドで呼び出し | 0.2s |
        # | waiting = 1, メインスレッドで呼び出し | 0.2s |
        # | waiting = 1, 別スレッドで呼び出し | 0.2s |
        if waiting:
            self._bcap.robot_execute(
                self._hRob,
                "TwofgGrip",
                [width, force, speed, waiting]
            )
        else:
            # robot_executeでは内部でパケットのsendとrecvを行うが
            # TwofgGripでwaiting = 0で使う場合はsendだけで良いので
            # 内部のsendを抜き出して使う
            # メインスレッドの制御ループの遅延は、
            # メインスレッドで呼び出すと、0.012s
            # 別スレッドで呼び出すと、0.008s
            # で十分速くなる

            # ハンドルは複数のソケットで使い回せなかった
            # with self._bcap_hand._lock:
            #     self._bcap_hand._bcap_send(self._bcap_hand._serial, self._bcap_hand._version, 64, [self._hRob, "TwofgGrip", [width, force, speed, waiting]])
            #     if self._bcap_hand._serial >= 0xFFFF:
            #         self._bcap_hand._serial  = 1
            #     else:
            #         self._bcap_hand._serial += 1

            with self._bcap._lock:
                self._bcap._bcap_send(self._bcap._serial, self._bcap._version, 64, [self._hRob, "TwofgGrip", [width, force, speed, waiting]])
                if self._bcap._serial >= 0xFFFF:
                    self._bcap._serial  = 1
                else:
                    self._bcap._serial += 1
        
    def TwofgRelease(
        self,
        width: Optional[float] = None,
        waiting: int = 1,
    ) -> None:
        """
        指定したワーク幅まで開放動作を実行します．

        TwofgRelease(<fWidth>, <lWaiting>)
        <fWidth> ： 目標となるフィンガー間の距離[mm]（VT_R4）
        <lWaiting> ： 待機設定（VT_I4）
                        0：待機しない
                        1：目標ワーク幅に達するまで待機する

        OR_2FG_RELEASE(instance, width, waiting)
        width シングル 現在のセットアップで適切な幅を [mm]単位、小数点精度
        1 で定義します。
        waiting 整数
        0：プログラムはグリッパーが動作中も続行されます。
        1：プログラムはグリッパーが動作を停止するまで待ちま
        す。

        widthがNoneの場合、把持モードに応じて最大まで開放する。
        """
        if width is None:
            if self._default_release_width is not None:
                width = self._default_release_width
            else:
                mode = self.TwofgGetMode()
                # 外部把持
                if mode == 1:
                    width = self.TwofgGetMaxWidth()
                # 内部把持
                elif mode == 2:
                    width = self.TwofgGetMinWidth()
                else:
                    raise ValueError(mode)

        if waiting:
            self._bcap.robot_execute(
                self._hRob,
                "TwofgRelease",
                [width, waiting]
            )
        else:
            # TwofgGripと同様の理由で低遅延になるように実装

            # ハンドルは複数のソケットで使い回せなかった
            # with self._bcap_hand._lock:
            #     self._bcap_hand._bcap_send(self._bcap_hand._serial, self._bcap_hand._version, 64, [self._hRob, "TwofgRelease", [width, waiting]])
            #     if self._bcap_hand._serial >= 0xFFFF:
            #         self._bcap_hand._serial  = 1
            #     else:
            #         self._bcap_hand._serial += 1

            with self._bcap._lock:
                self._bcap._bcap_send(self._bcap._serial, self._bcap._version, 64, [self._hRob, "TwofgRelease", [width, waiting]])
                if self._bcap._serial >= 0xFFFF:
                    self._bcap._serial  = 1
                else:
                    self._bcap._serial += 1

    def is_error_level_0(self, e) -> bool:
        """
        エラーレベルが0かどうか返す。
        エラーレベルの特徴は以下のとおりなので、通常無視してよい。
        | エラーのレベル | エラー概要 | ロボット動作 | 通常タスク | モータ電源 | I/O |
        | - | - | - | - | - | - |
        | 0 | 軽微な注意エラー | 影響しない | 影響しない | 維持 | なし |
        """
        hr = e.hresult
        # ex. es = '0x80070057'
        es = python_error_to_original_error_str(hr)
        return es[3] == "0"


    def fetch_from_sys_state(self, sys_state: int, idx: int) -> bool:
        """
        SysStateから特定のステータスを取得します。
        各ビットの意味は以下の通りです。
        0ビット目は最下位ビットのことです。
        SYSSTATE_POS_TO_VALUE = {
            0: 'ロボット運転中(プログラム動作中)',
            1: 'ロボット異常',
            2: 'サーボON中',  # "Motor"コマンドでONにしたときに対応  
            3: 'ロボット初期化完了(I/O 標準、MiniIO専用モード選択時)/ ロボット電源入り完了(I/O 互換モード選択時)',
            4: '自動モード',
            5: '自動モードで起動権がスマートTP以外にある場合',
            6: 'バッテリ切れ警告',
            7: 'ロボット警告',
            8: 'コンティニュスタート許可',
            9: '予約',
            10: '非常停止状態',
            11: '自動運転イネーブル',
            12: '防護停止',
            13: '停止処理中',
            14: '予約',
            15: '予約',
            16: 'プログラムスタートリセット',
            17: 'Cal完了',
            18: '手動モード',
            19: '1サイクル完了',
            20: 'ロボット動作中(指令値レベル)',
            21: 'ロボット動作中(エンコーダレベル)',
            22: '予約',
            23: '予約',
            24: 'コマンド処理完了',
            25: '予約',
            26: '予約',
            27: '予約',
            28: '予約',
            29: '予約',
            30: '予約',
            31: '予約'
        }
        """
        # ビット表示
        # 出力例: 0b100000100100111100
        # 仕様では32ビットだが手元での出力は18ビットであり、後半のビットが省かれていると
        # 考えられる
        b = bin(sys_state)
        # 先頭の"0b"を削る
        b = b[2:]
        if idx < 0 or idx >= len(b):
            raise ValueError(f"idx {idx} is out of range 0-{len(b)-1}")
        # 左端が0ビット目になるように反転
        b = b[::-1]
        return b[idx] == "1"
    
    def is_emergency_stopped(self) -> bool:
        """
        非常停止状態かどうか。
        スレーブモードでは実行できません。
        """
        i = self.SysState()
        return self.fetch_from_sys_state(i, 10)
