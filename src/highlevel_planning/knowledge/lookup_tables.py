class LookupTable:
    def __init__(self, data_type):
        self.data = dict()
        self.data_type = data_type
        self.counter = -1

    def add(self, new_entry, new_label=None):
        if new_label is None:
            self.counter += 1
            new_label = "{}_sample_{}".format(self.data_type, self.counter)
        self.data[new_label] = new_entry
        return new_label

    def remove(self, label):
        del self.data[label]

    def get(self, label):
        return self.data[label]
