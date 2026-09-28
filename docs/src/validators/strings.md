# String validators

Use these validators with `string` parameters.

| Function     | Arguments           | Description                                    |
| ------------ | ------------------- | ---------------------------------------------- |
| `fixed_size<>` | [length]            | String length is specified length              |
| `size_gt<>`    | [length]            | String length is greater than specified length |
| `size_lt<>`    | [length]            | String length is less than specified length    |
| `not_empty<>`  | []                  | String parameter is not empty                  |
| `one_of<>`     | [[val1, val2, ...]] | String is one of the specified values          |
