# Scalar validators

Use bounds and comparisons for `int` and `double` parameters. `one_of<>` checks membership in a list of allowed scalar values.

| Function | Arguments           | Description                          |
| -------- | ------------------- | ------------------------------------ |
| `bounds<>` | [lower, upper]      | Bounds checking (inclusive)          |
| `lt<>`     | [value]             | parameter < value                    |
| `gt<>`     | [value]             | parameter > value                    |
| `lt_eq<>`  | [value]             | parameter <= value                   |
| `gt_eq<>`  | [value]             | parameter >= value                   |
| `one_of<>` | [[val1, val2, ...]] | Value is one of the specified values |

Note: `lt<>`, `gt<>`, `lt_eq<>`, or `gt_eq<>` cannot be used together with `bounds<>`.
