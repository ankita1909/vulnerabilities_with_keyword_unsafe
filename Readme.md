This project contains the use of `unsafe` keyword to exploit various safety features provided by Rust. main.rs contains the following security bugs introduced by using `unsafe`

- Stack overflow: call the function fib_func(n) with large value of n to see that memory over the stack is also written and the global static variables e.g DATA is overwritten.
- Dangling Pointer
- Implicit Coercion
- Null Pointer dereferencing
- Buffer Overread
- Defied Immutability: variable a is declared immutable, still the value is updated using the pointer reference to a in an `unsafe` block.

The project can be run on `Cypress PSOC6 062 WIFI BT` board using Visual Studio Code. The `launch.json` contains the required setting for debugging the code.