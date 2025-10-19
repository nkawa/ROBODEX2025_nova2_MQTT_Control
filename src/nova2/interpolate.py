import numpy as np


class DelayedInterpolator:
    def __init__(
        self,
        delay: float = 0.1,
    ) -> None:
        self.delay = delay
        self.ts = []
        self.data = []

    def reset(self, t: float, datum: np.ndarray) -> None:
        self.ts = [t]
        self.data = [datum]

    def read(self, t: float, datum: np.ndarray) -> np.ndarray:
        self.create(t, datum)
        # 現在の時間 - 遅延の前後のデータを取り出す
        has_new = False
        has_old = False
        for i, _t in enumerate(self.ts):
            if _t < t - self.delay:
                has_old = True
            else:
                has_new = True
                break
        if has_old:
            # 現在の時間 - 遅延の直前のデータ    
            t1 = self.ts[i - 1]
            d1 = self.data[i - 1]
            if has_new:
                # 現在の時間 - 遅延の直後のデータ
                t2 = self.ts[i]
                d2 = self.data[i]
                # 線形補間
                d = d1 + (d2 - d1) / (t2 - t1) * (t - self.delay - t1)
                return d
            else:
                return d1
        else:
            # assert has_new
            # 現在の時間 - 遅延の直後のデータ
            t2 = self.ts[i]
            d2 = self.data[i]
            return d2

    def create(self, t: float, datum: np.ndarray) -> None:
        self.update(t)
        if len(self.data) == 0 or (not np.all(self.data[-1] == datum)):
            self.ts.append(t)
            self.data.append(datum)

    def update(self, t: float) -> None:
        if len(self.ts) == 0:
            return
        indices = range(len(self.ts))[::-1]
        has_old = False 
        too_old = False
        for i in indices:
            _t = self.ts[i]
            if _t < t - self.delay:
                too_old = _t < t - self.delay * 2
                has_old = True
                break
        if has_old:
            # 古いデータがあればその中で最新のものが古すぎれば全部捨てる
            if too_old:
                self.ts = self.ts[i + 1:]
                self.data = self.data[i + 1:]
            # 古いデータがあればその中で最新のものだけ残す
            else:
                self.ts = self.ts[i:]
                self.data = self.data[i:]
