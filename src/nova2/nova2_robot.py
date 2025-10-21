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
        port: int = 5007,
        timeout: float = 5,

        logger: Optional[logging.Logger] = None,
    ):
        
        if logger is None:
            self.logger = logging.getLogger(__name__)
        else:
            self.logger = logger
            logger.info("Using external logger in Nova2Robot")
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
        logger.info(("default_fig", self._default_fig))
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
        self._default_grip_width = None
        self._default_release_width = None
        
        self.global_state = {}
        self.global_state["connect"] = False
        self._raw_feedback = None
        self.logger.info("Start Nova2Robot __init__")

    def start(self):
        self.logger.info("Robot start")
        # モーター起動など長い時間がかかるコマンドがあるので、
        # それに合わせてタイムアウト時間は長めに設定する
        # このときスレーブモードの制御コマンドはタイムアウトエラーではなく
        # 指示値生成遅延エラーで通知される
        self.text_log = ""
        self.logger.info("Connect feedback port")
        # need try
        # self.client_dash = DobotApiDashboard(
        #             "192.168.5.1", 29999, self.text_log)
        self.client_feed = DobotApiFeedBack(
                    "192.168.5.1", 30004,self.text_log)

        #ServoP only accepted in this port.
        # self.client_move = DobotApiDashboard("192.168.5.1", 30003, self.text_log)

        self.global_state["connect"] = not self.global_state["connect"]
        
        # self.text_log.insert(tk.END,"Connect!"+str(self.global_state["connect"])+"\n")
      

        self._set_feed_back()
        self.logger.info("Robot started")
        
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

        return cur_pos
    
    def get_current_joint(self):
        if self._raw_feedback is None:
            return None
        cur_jnt = self._raw_feedback[0][23]
        return cur_jnt
    
    def is_enabled(self):
        if self._raw_feedback is None:
            return None
        is_enable = self._raw_feedback[0][50]
        return bool(is_enable)
    
    def enable(self) -> bool:
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
        self.logger.info("manual_reset not yet implemented")

    def clear_error(self) -> None:
        """
        ティーチングペンダントのエラーをクリアする。
        GetCurErrorInfoなどで取得できるエラーもクリアする。
        例えば、非常停止中でもエラーをクリアすることはできるが、
        非常停止状態がOFFになるわけではないため、
        その後一部のコマンドが実行できないことに注意。
        気軽にクリアしてしまうと現在のエラーがわからなくなるので注意。
        """
        # not yet implemented
        self.logger.warning("clear_error: not yet implemented")
        pass

    
    def enable_robot(self, ext_speed: int = 20) -> None:
        # STO状態（セーフティ状態）を解除する
        self.manual_reset()

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
        
        
#        self._bcap.robot_move(self._hRob, interpolation, f"{path} P({x}, {y}, {z}, {rx}, {ry}, {rz}, {fig})", option)


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

    def move_joint_servo(
        self, pose
    ) -> Tuple[float, Tuple[float, float, float, float, float, float]]:
        """
        スレーブモードでの関節制御。

        時間と現在の状態関節を返す。

        この時間はコントローラ内部の時間で、ユーザースクリプトの
        時間と少しずれているので使わないほうがよい。
        """
        self.logger.debug("move_joint_servo")
        joint = pose
        joint_all = [0] * 8
        for i, j in enumerate(joint):
            joint_all[i] = j
        # 内部でrecvをしない場合は少し高速化する
        # ret = self._move_pose_servo_mode(joint_all)
        # t, ret_pose = ret
        # ret_pose = ret_pose[:6]
        # return t, ret_pose
        self._move_pose_servo_mode(joint_all)

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


    def __del__(self):
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
