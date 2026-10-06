# Array validators

Use these validators with array parameters. Element bounds apply to numeric arrays (`int_array` and `double_array`); size validators constrain the number of elements. Use `element_bounds<>`, not `bounds<>`, to check every numeric element.

| Function               | Arguments           | Description                                         |
| ---------------------- | ------------------- | --------------------------------------------------- |
| `unique<>`               | []                  | Contains no duplicates                              |
| `subset_of<>`            | [[val1, val2, ...]] | Every element is one of the list                    |
| `fixed_size<>`           | [length]            | Number of elements is specified length              |
| `size_gt<>`              | [length]            | Number of elements is greater than specified length |
| `size_lt<>`              | [length]            | Number of elements is less than specified length    |
| `not_empty<>`            | []                  | Has at least one element                            |
| `element_bounds<>`       | [lower, upper]      | Bounds checking each element (inclusive)            |
| `lower_element_bounds<>` | [lower]             | Lower bound for each element (inclusive)            |
| `upper_element_bounds<>` | [upper]             | Upper bound for each element (inclusive)            |

Note: `element_bounds<>` cannot be mixed with `lower_element_bounds<>` or `upper_element_bounds<>`.
