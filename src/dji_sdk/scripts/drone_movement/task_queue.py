class TaskQueue:
    def __init__(self):
        self._data = []
        self._max_size = 0

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

    def get_task_list(self):
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

    def set_queue_active(self, queue_active):
        self.queue_active = queue_active

    def get_queue_active(self):
        return self.queue_active
    
    def dump_queue(self):
        self._data = []

    def __str__(self):
        return str(self._data)




if __name__ == "__main__":
    # Example usage.
    fifo = TaskQueue()
    fifo.put(1)
    fifo.put(2)
    fifo.put(3)
    print(fifo)  # Output: [1, 2, 3]

    fifo.put_at(1, 10)  # Insert 10 at index 1
    print(fifo)  # Output: [1, 10, 2, 3]

    item = fifo.get()
    print(item)  # Output: 1
    fifo.put_first("hello")
    print(fifo)  # Output: [10, 2, 3]

    print(fifo.pop())
    print(fifo.pop())
    print(fifo)
