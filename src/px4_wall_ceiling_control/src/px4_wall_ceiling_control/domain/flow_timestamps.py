"""Bounded MAVROS clock mapping tolerance, with source replay/clock checks."""
import math

class FlowTimestampGuard:
    def __init__(self, timeout=.6, future_tolerance=.1):
        if (not math.isfinite(timeout) or timeout <= 0
                or not math.isfinite(future_tolerance) or not 0 <= future_tolerance <= .1):
            raise ValueError('invalid optical flow timestamp limits')
        self.timeout=timeout;self.future_tolerance=future_tolerance
        self.last_source=self.last_source_receipt=self.last_receipt=None
        self.frame_error=''

    def observe(self, source, receipt):
        previous_source,previous_receipt=self.last_source,self.last_receipt
        previous_source_receipt=self.last_source_receipt
        self.last_receipt=receipt
        self.frame_error=''
        if not math.isfinite(source) or source <= 0:
            self.frame_error='光流源时间戳缺失或无效'
        elif not math.isfinite(receipt):
            self.frame_error='光流接收时钟无效'
        elif previous_receipt is not None and receipt < previous_receipt:
            self.frame_error='光流接收时钟倒退'
        elif previous_source is not None and previous_source > 0 and source <= previous_source:
            self.frame_error='光流源时间戳重复' if source == previous_source else '光流源时间戳倒退'
        elif receipt-source > self.timeout:
            self.frame_error='光流源时间戳过期'
        elif source-receipt > self.future_tolerance+1e-6:
            self.frame_error='光流源时间戳超前超限'
        elif (previous_source is not None and previous_source > 0 and previous_source_receipt is not None
              and abs((source-previous_source)-(receipt-previous_source_receipt)) > max(.2,2*self.future_tolerance)+1e-6):
            self.frame_error='光流源时钟明显跳变，等待重新确认'
        if not self.frame_error or '明显跳变' in self.frame_error:
            self.last_source,self.last_source_receipt=source,receipt
        return self.frame_error

    def reason(self, source, now, frame_error=None):
        error=self.frame_error if frame_error is None else frame_error
        if error:return error
        if not math.isfinite(source) or source <= 0:return '光流源时间戳缺失或无效'
        age=now-source
        if not math.isfinite(age):return '光流时钟无效'
        if age > self.timeout:return '光流源时间戳过期'
        if age < -self.future_tolerance-1e-6:return '光流源时间戳超前超限'
        return ''
