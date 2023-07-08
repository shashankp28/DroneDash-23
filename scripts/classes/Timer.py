from time import time_ns

class Timer:
    
    def __init__(self):
        self.timeout = -1
    
    def set_timeout(self, timeout_ms=0 ,timeout_s=0):
        if timeout_ms != 0:
            self.timeout = time_ns() + (timeout_ms)*(10**6)
        else:
            self.timeout = time_ns() + (timeout_s)*(10**9)
    
    def check_timeout(self):
        if self.timeout == -1:
            return True
        if time_ns() > self.timeout:
            self.timeout = -1
            return True
        return False
