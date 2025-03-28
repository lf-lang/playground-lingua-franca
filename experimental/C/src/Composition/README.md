This directory contains examples that explore the potential for creative patterns of composition to
give rise to new connection patterns.

A possible guideline for such things is that the amount of generated IR should not grow faster than
logarithmically wrt the amount of source code nor wrt the size of the data structure. I'm afraid
that even this guideline might be too permissive; furthermore, it is not yet clear how easy it is to
meet even this guideline when working with numbers that are not powers of small numbers such as 2 or
3.

Some examples in this directory involve extensive code duplication that could be resolved by
allowing reactor constructors to be parameterized by each other, by allowing arithmetic expressions
in port widths and especially in reactor parameters, and possibly also by allowing some natural
interconversion between church numerals and numbers that can be used in port bank widths. Current
limitations of the language make some of these programs about as readable as assembly would be
without a notion of a return address register.

By the way, I originally made the mistake of associating parameterization of reactors by each other
as some sort of type-level metaprogramming because in OOP, classes correspond to types. However,
there is no meaningful sense in which reactor "classes" correspond to types because reactors are
never passed around or placed into typed variables; instead, all we define is a constructor, not a
type. If we think of reactor "classes" as actually just constructors, which are actually just
functions, then the parameterization that I am talking about is a very mainstream, normal sort of
functional programming, with the important characteristics that it is supposed to be used in a
limited way, that it should be easy to get started without thinking about functional
programming, and that the constructor-level programs would be evaluated totally at compile time.
