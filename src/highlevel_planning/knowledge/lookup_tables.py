class LookupTable:
    def __init__(self, data_type):
        self.data = dict()
        self.data_type = data_type

    def add(self, new_label, new_entry):
        self.data[new_label] = new_entry

    def get(self, label):
        return self.data[label]
