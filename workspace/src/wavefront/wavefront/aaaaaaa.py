import numpy as np


class fofinho(Node):
    def __init__(self):
        super().__init__("publisher")
        self.publisher = self.create_publisher()
        self.subscription = self.create_subscription()
        self.objetivo = np.array[10.00, 7.00]
        self.inicio = [0.0, 0.0]

        



        

        