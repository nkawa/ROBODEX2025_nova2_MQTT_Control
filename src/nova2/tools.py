from nova2.dummy_hand_control import DummyHandControl


# ホルダーの座標系は、ベース座標系（ツール座標系ではない）であることに注意
tool_infos = [
    # {
    #     "id": -1,
    #     "name": "no_tool",
    #     "tool_def": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #     "id_in_robot": 0,
    # },
    {
        "id": 1,
        "name": "onrobot_2fg7",
        "holder_waypoints": {
            "enter_path": [402.57, 445.05, 21.20 + 200, 180, 0, -90],
            "disengaged": [402.57, 445.05, 21.20 + 30, 180, 0, -90],
            "tool_holder": [402.57, 445.05, 21.20, 180, 0, -90],
            "locked": [402.57, 445.05 + 30, 21.20, 180, 0, -90],
            "exit_path_1": [402.57, 445.05 + 120, 21.20, 180, 0, -90],
            "exit_path_2": [402.57, 445.05 + 120, 21.20 + 200, 180, 0, -90],
        },
        # ツールチェンジャー + 平行グリッパーのグリッパパーツの先端（中央でない）までの距離
        "tool_def": [0.0, 0.0, 190.0, 0.0, 0.0, 0.0],
        "id_in_robot": 1,
        "holder_region": 1,
    },
    {
        "id": 2,
        "name": "onrobot_vgc10",
        "holder_waypoints": {
            "enter_path": [541.98, 444.11, 21.20 + 200, 180, 0, -90],
            "disengaged": [541.98, 444.11, 21.20 + 30, 180, 0, -90],
            "tool_holder": [541.98, 444.11, 21.20, 180, 0, -90],
            "locked": [541.98, 444.11 + 30, 21.20, 180, 0, -90],
            "exit_path_1": [541.98, 444.11 + 120, 21.20, 180, 0, -90],
            "exit_path_2": [541.98, 444.11 + 120, 21.20 + 200, 180, 0, -90],
        },
        # ツールチェンジャー + 真空グリッパーの先端までの距離
        "tool_def": [0.0, 0.0, 255.0, 0.0, 0.0, 0.0],
        "id_in_robot": 2,
        "holder_region": 1,
    },
    {
        "id": 3,
        "name": "cutter",
        "holder_waypoints": {
            "enter_path": [680.48, 443.48, 22.15 + 200, 180, 0, -90],
            "disengaged": [680.48, 443.48, 22.15 + 30, 180, 0, -90],
            "tool_holder": [680.48, 443.48, 22.15, 180, 0, -90],
            "locked": [680.48, 443.48 + 30, 22.15, 180, 0, -90],
            "exit_path_1": [680.48, 443.48 + 120, 22.15, 180, 0, -90],
            "exit_path_2": [680.48, 443.48 + 120, 22.15 + 200, 180, 0, -90],
        },
        # ツールチェンジャー + アーム先端から一番遠いカッターの刃のホルダーまでの距離
        "tool_def": [0.0, 0.0, 118.0, 0.0, 0.0, 0.0],
        "id_in_robot": 3,
        "holder_region": 1,
    },
    {
        "id": 4,
        "name": "plate_holder",
        "holder_waypoints": {
            "enter_path": [396.89, -448.34, 22.56 + 200, 180, 0, 90],
            "disengaged": [396.89, -448.34, 22.56 + 30, 180, 0, 90],
            "tool_holder": [396.89, -448.34, 22.56, 180, 0, 90],
            "locked": [396.89, -448.34 - 30, 22.56, 180, 0, 90],
            "exit_path_1": [396.89, -448.34 - 120, 22.56, 180, 0, 90],
            "exit_path_2": [396.89, -448.34 - 120, 22.56 + 200, 180, 0, 90],
        },
        # ツールチェンジャー + ホルダーの厚み
        "tool_def": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "id_in_robot": 4,
        "holder_region": 2,
    },
]

tool_classes = {
    "no_tool": DummyHandControl,
    "onrobot_2fg7": DummyHandControl,
    "onrobot_vgc10": DummyHandControl,
    "cutter": DummyHandControl,
    "plate_holder": DummyHandControl,
}

tool_base = [360.03, 149.97, 460.03, -180, 0, -90]
