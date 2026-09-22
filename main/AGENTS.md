# Simplicity and Complexity

Prefer the simplest correct implementation.

Do not over-engineer solutions.

When multiple implementations are possible, prefer the one that is:

- easier to read
- easier to understand
- easier to maintain
- easier to modify later

Do not introduce additional abstraction layers unless they clearly reduce complexity.

Avoid:

- unnecessary design patterns
- unnecessary classes
- deep inheritance
- excessive templates
- excessive generic programming
- complex callback chains
- unnecessary lambdas
- unnecessary indirection
- premature optimization
- clever one-liners
- abstractions used only once
- frameworks or subsystems that are not required by the task

Prefer:

- straightforward control flow
- small functions
- explicit state
- descriptive variable names
- simple data structures
- direct function calls
- clear intermediate variables
- obvious logic

If a simple implementation and a more sophisticated implementation have equivalent behavior, choose the simpler implementation.

Do not refactor working code solely to make it more architecturally sophisticated.

Make the smallest change that correctly solves the requested problem.

Do not:

- rewrite unrelated code
- redesign modules unnecessarily
- create abstractions for hypothetical future requirements
- change unrelated naming or formatting
- optimize code that is not part of the problem

Complexity is acceptable only when it is required for:

- correctness
- safety
- timing requirements
- memory constraints
- algorithmic performance
- hardware limitations

If additional complexity is necessary, keep it localized and explain why it is required.
