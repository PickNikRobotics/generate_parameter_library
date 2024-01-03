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
        filtered_description = (
            description.replace('\\', '\\\\').replace('"', '\\"').replace('`', '')
        )
        # create a quote delimited string for every line
        filtered_description = '\n'.join(
            f'"{line}"' for line in filtered_description.splitlines()
        )
        return filtered_description
    else:
        return '""'
