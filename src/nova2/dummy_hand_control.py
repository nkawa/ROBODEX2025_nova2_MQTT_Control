class DummyHandControl:
    def __init__(self) -> None:
        pass

    def connect_and_setup(self) -> bool:
        """
        接続してハンドのパラメータを取得します.
        """
        return True
    
    def disconnect(self) -> None:
        """
        接続を切断します
        """
        pass
