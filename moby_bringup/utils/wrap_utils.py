from functools import wraps


##
# @class try_wrap
# @brief try and print eror
class try_wrap(object):
    def __init__(self, print_message=None, print_only_first=True, silence=False):
        self.print_message = print_message
        self.print_only_first = print_only_first
        self.is_error_before = False
        self.silence = silence

    def __call__(self, func):
        @wraps(func)
        def wrapped_f(_self, *args, **kwargs):
            try:
                res = func(_self, *args, **kwargs)
                if self.is_error_before:
                    _self.get_logger().info(f"Function {func.__name__} restored")
                self.is_error_before = False
                return res
            except Exception as e:
                if not self.silence:
                    if not self.is_error_before or not self.print_only_first:
                        if self.print_message is not None:
                            _self.get_logger().error(self.print_message)
                        else:
                            _self.get_logger().error(f"ERROR on Function {func.__name__}")
                        _self.get_logger().debug(str(e))
                self.is_error_before = True
        return wrapped_f