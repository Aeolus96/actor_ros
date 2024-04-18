#!/usr/bin/env python3


class Demo:
    def __init__(self):
        self.a = 10
        self.b = 20

    def add(self):
        return self.a + self.b

    def multiply(self, x=1, y=1):
        return x * y

    def divide(self, func, *args, **kwargs):
        if callable(func):
            print(*args)
            print(*kwargs)
            print(*kwargs.values())
            return func(*args, **kwargs) / 10


test = Demo()

print(callable(test.add))  # returns True
print(callable(test.multiply))  # returns True
print(callable(test.multiply(10, 20)))  # returns False because of arguments
print(test.divide(test.multiply, 10, 20))  # works with arguments separated
print(test.divide(test.multiply, x=10, y=20))  # works with keyword arguments
