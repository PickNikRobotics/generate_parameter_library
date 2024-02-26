# -*- coding: utf-8 -*-
def valid_string_cpp(description):
    """
    Filter a string to make it a valid C++ string literal.

    Args:
      description (str): The input string to be filtered.

    Returns:
      str: The filtered string that is a valid C++ string.
    """
    if description:
        # remove possible markdown/rst syntax, but add proper indent for cpp-header files.
        filtered_description = (
            description.replace('\\', '\\\\').replace('`', '').replace('\n', '\\n    ')
        )
        return f'"{filtered_description}"'
    else:
        return '""'


def valid_string_python(description):
    """
    Filter a string to make it a valid Python string literal.

    Args:
      description (str): The input string to be filtered.

    Returns:
      str: The filtered string that is a valid Python string.
    """
    if description:
        return description.replace('\n', '\\n    ')
    else:
        return ''
