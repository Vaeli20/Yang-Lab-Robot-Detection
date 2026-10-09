from collections import deque


class VelocityFilter:
    """
    单关节速度滤波：中值滤波去尖峰 + EMA 平滑 + 死区。

    这里的速度主要用于连续轨迹播放时生成速度上限，不用于闭环控制，
    所以滤波不能太重，否则会明显滞后。
    """

    def __init__(self, median_window=5, ema_alpha=0.25, deadband=0.002):
        if median_window < 1 or median_window % 2 == 0:
            raise ValueError("median_window 必须是正奇数")
        if not 0.0 < ema_alpha <= 1.0:
            raise ValueError("ema_alpha 必须在 (0, 1] 内")

        self.median_window = int(median_window)
        self.ema_alpha = float(ema_alpha)
        self.deadband = float(deadband)
        self._buf = deque(maxlen=self.median_window)
        self._ema_prev = None

    def filter(self, value):
        value = float(value)
        self._buf.append(value)

        if len(self._buf) < self.median_window:
            median_val = value
        else:
            sorted_vals = sorted(self._buf)
            median_val = sorted_vals[len(sorted_vals) // 2]

        if self._ema_prev is None:
            out = median_val
        else:
            out = self.ema_alpha * median_val + (1.0 - self.ema_alpha) * self._ema_prev

        if abs(out) < self.deadband:
            out = 0.0

        self._ema_prev = out
        return out

    def reset(self):
        self._buf.clear()
        self._ema_prev = None
