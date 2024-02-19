class PacketQueue:
    def __init__(self, max_size = 0):
        self._data = []
        self._max_size = max_size

        self.queue_active = True

    def put(self, item):
        """Add an item to the end of the FIFO."""
        self._data.append(item)

    def get(self):
        """Return the first item from the FIFO WITHOUT removing."""
        if not self.is_empty():
            return self._data[0]
        else:
            raise IndexError("FIFO is empty")

    def get_packet_list(self):
        return self._data

    def pop(self):
        if not self.is_empty():
            return self._data.pop(0)
        else:
            raise IndexError("FIFO is empty")

    def put_at(self, index, item):
        """Insert an item at the specified index, maintaining FIFO order."""
        self._data.insert(index, item)

    def put_first(self, item):
        self._data.insert(0, item)

    def is_empty(self):
        """Check if the FIFO is empty."""
        return len(self._data) == 0

    def size(self):
        """Return the number of elements in the FIFO."""
        return len(self._data)

    def max_size(self):
        """Return the number of elements in the FIFO."""
        return self._max_size

    def set_queue_active(self, queue_active):
        self.queue_active = queue_active

    def get_queue_active(self):
        return self.queue_active
    
    def dump_queue(self):
        self._data = []

    def __str__(self):
        return str(self._data)