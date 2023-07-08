class FixedList(list):
    def __init__(self, max_size, *args):
        super(FixedList, self).__init__(*args)
        self.max_size = max_size
    
    def fixedAppend(self, item):
        self.append(item)
        if len(self) > self.max_size:
            self.pop(0)