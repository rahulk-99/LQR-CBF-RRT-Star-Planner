"""
Queue implementations for different ordering strategies.
This module provides three queue implementations:
1. FIFO (First-In-First-Out) queue
2. LIFO (Last-In-First-Out) queue
3. Priority queue with customizable ordering

These queues are used in various planning algorithms for node exploration and path finding.

Authors: 
- Tirth Sadaria
- Kunj Golwala
- Rahul Kumar
Date: 16th May 2025
"""

import collections
import heapq


class QueueFIFO:
    """
    First-In-First-Out (FIFO) Queue implementation.
    Elements are added to the back and removed from the front.
    Uses collections.deque for efficient O(1) operations.
    """

    def __init__(self):
        """
        Initialize an empty FIFO queue using collections.deque.
        """
        self.queue = collections.deque()

    def empty(self):
        """
        Check if the queue is empty.
        
        Returns:
            bool: True if queue is empty, False otherwise
        """
        return len(self.queue) == 0

    def put(self, node):
        """
        Add a node to the back of the queue.
        
        Args:
            node: The node to be added to the queue
        """
        self.queue.append(node)  # enter from back

    def get(self):
        """
        Remove and return the node from the front of the queue.
        
        Returns:
            The node at the front of the queue
        """
        return self.queue.popleft()  # leave from front


class QueueLIFO:
    """
    Last-In-First-Out (LIFO) Queue implementation.
    Elements are added to the back and removed from the back (stack behavior).
    Uses collections.deque for efficient O(1) operations.
    """

    def __init__(self):
        """
        Initialize an empty LIFO queue using collections.deque.
        """
        self.queue = collections.deque()

    def empty(self):
        """
        Check if the queue is empty.
        
        Returns:
            bool: True if queue is empty, False otherwise
        """
        return len(self.queue) == 0

    def put(self, node):
        """
        Add a node to the back of the queue.
        
        Args:
            node: The node to be added to the queue
        """
        self.queue.append(node)  # enter from back

    def get(self):
        """
        Remove and return the node from the back of the queue.
        
        Returns:
            The node at the back of the queue
        """
        return self.queue.pop()  # leave from back


class QueuePrior:
    """
    Priority Queue implementation.
    Elements are ordered based on their priority value.
    Uses heapq for efficient O(log n) operations.
    Lower priority values are dequeued first.
    """

    def __init__(self):
        """
        Initialize an empty priority queue using a list and heapq.
        """
        self.queue = []

    def empty(self):
        """
        Check if the queue is empty.
        
        Returns:
            bool: True if queue is empty, False otherwise
        """
        return len(self.queue) == 0

    def put(self, item, priority):
        """
        Add an item to the queue with specified priority.
        
        Args:
            item: The item to be added to the queue
            priority: The priority value of the item (lower values have higher priority)
        """
        heapq.heappush(self.queue, (priority, item))  # reorder using priority

    def get(self):
        """
        Remove and return the item with the highest priority (lowest priority value).
        
        Returns:
            The item with the highest priority
        """
        return heapq.heappop(self.queue)[1]  # pop out the smallest item

    def enumerate(self):
        """
        Get all items in the queue without removing them.
        
        Returns:
            List of (priority, item) tuples in the queue
        """
        return self.queue
