import numpy as np


class fofinho(node):
    def __init__(self):
        super().__init__("publisher")
        self.publisher = self.create_publisher()
        self.subscription = self.create_subscription()
            
    def listerner_callback(self, msg):
        self.objetivo = np.array[10.00, 7.00]