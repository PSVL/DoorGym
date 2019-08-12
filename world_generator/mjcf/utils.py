import functools


def capture_kwargs(f):
    @functools.wraps(f)
    def wrapper(self, *args, **kwargs):
        self.call_kwargs = kwargs
        f(self, *args, **kwargs)
    return wrapper
