/**
 * @file
 * @author Edward A. Lee (eal@berkeley.edu)
 * @copyright (c) 2020-2024, The University of California at Berkeley.
 * License: <a href="https://github.com/lf-lang/reactor-c/blob/main/LICENSE.md">BSD 2-clause</a>
 * @brief Macros providing an API for use in inline reaction bodies.
 *
 * This set of macros is defined prior to each reaction body and undefined after the reaction body
 * using the set_undef.h header file.  If you wish to use these macros in external code, such as
 * that implementing a bodiless reaction, then you can include this header file (and at least
 * reactor.h, plus possibly a few other header files) in your code.
 *
 * The purpose for these macros is to provide a semblance of polymorphism even though C does not support
 * polymorphism. For example, `lf_set(port, value)` is a macro where the first argument is a specific
 * port struct and the second type is a value with a type corresponding to the port's type. It is not
 * possible in C to provide a function that can be called with a port struct and a value of any type.
 *
 * Some of the macros are provided for convenience. For example, the macro can automatically provide
 * common arguments such as the environment and can cast arguments to required base types to suppress
 * warning.
 *
 * Note for target language developers. This is one way of developing a target language where
 * the C core runtime is adopted. This file is a translation layer that implements Lingua Franca
 * APIs which interact with the internal APIs.
 */

// Prevent inclusion twice in a row without an intervening inclusion of reaction_macros_undef.h.
#ifndef REACTION_MACROS_H
#define REACTION_MACROS_H

// NOTE: According to the "Swallowing the Semicolon" section on this page:
//    https://gcc.gnu.org/onlinedocs/gcc-3.0.1/cpp_3.html
// some of the following macros should use an odd do-while construct to avoid
// problems with if ... else statements that do not use braces around the
// two branches. Specifically, if the macro expands to more than one statement,
// then the odd construct is needed.

/**
 * @brief Mark a port present.
 *
 * This sets the is_present field of the specified output to true.
 *
 * This macro is a thin wrapper around the lf_set_present() function.
 * It simply casts the argument to `lf_port_base_t*` to suppress warnings.
 *
 * @param out The output port (by name).
 */
#define lf_set_present(out) lf_set_present((lf_port_base_t*)out)

/**
 * @brief Set the specified output (or input of a contained reactor) to the specified value.
 *
 * If the value argument is a primitive type such as int,
 * double, etc. as well as the built-in types bool and string,
 * the value is copied and therefore the variable carrying the
 * value can be subsequently modified without changing the output.
 * This also applies to structs with a type defined by a typedef
 * so that the type designating string does not end in '*'.
 *
 * If the value argument is a pointer
 * to memory that the calling reaction has dynamically allocated,
 * the memory will be automatically freed once all downstream
 * reactions no longer need the value.
 * If 'lf_set_destructor' is called on 'out', then that destructor
 * will be used to free 'value'.
 * Otherwise, the default void free(void*) function is used.
 *
 * @param out The output port (by name) or input of a contained
 *  reactor in form input_name.port_name.
 * @param value The value to insert into the self struct.
 */
#define lf_set(out, val)                                                                                               \
  do {                                                                                                                 \
    out->value = val;                                                                                                  \
    lf_set_present(out);                                                                                               \
    if (((token_template_t*)out)->token != NULL) {                                                                     \
      /* The cast "*((void**) &out->value)" is a hack to make the code */                                              \
      /* compile with non-token types where value is not a pointer. */                                                 \
      lf_token_t* token = _lf_initialize_token_with_value((token_template_t*)out, *((void**)&out->value), 1);          \
      out->token = token;                                                                                              \
    }                                                                                                                  \
  } while (0)

/**
 * @brief Set the specified output (or input of a contained reactor)
 * to the specified array with the given length.
 *
 * The array is assumed to be in dynamically allocated memory.
 * The deallocation is delegated to downstream reactors, which
 * automatically deallocate when the reference count drops to zero.
 *
 * @param out The output port (by name).
 * @param val The array to send (a pointer to the first element).
 * @param length The length of the array to send.
 */
#ifndef __cplusplus
#define lf_set_array(out, val, len)                                                                                    \
  do {                                                                                                                 \
    lf_set_present(out);                                                                                               \
    lf_token_t* token = _lf_initialize_token_with_value((token_template_t*)out, val, len);                             \
    out->token = token;                                                                                                \
    out->value = token->value;                                                                                         \
    out->length = len;                                                                                                 \
  } while (0)
#else
#define lf_set_array(out, val, len)                                                                                    \
  do {                                                                                                                 \
    lf_set_present(out);                                                                                               \
    lf_token_t* token = _lf_initialize_token_with_value((token_template_t*)out, val, len);                             \
    out->token = token;                                                                                                \
    out->value = static_cast<decltype(out->value)>(token->value);                                                      \
    out->length = len;                                                                                                 \
  } while (0)
#endif

/**
 * @brief Set the specified output (or input of a contained reactor)
 * to the specified token value.
 *
 * Tokens in the C runtime wrap messages that are in dynamically allocated memory and
 * perform reference counting to ensure that memory is not freed prematurely.
 *
 * @param out The output port (by name).
 * @param token A pointer to token obtained from an input, an action, or from `lf_new_token()`.
 */
#ifndef __cplusplus
#define lf_set_token(out, newtoken)                                                                                    \
  do {                                                                                                                 \
    lf_set_present(out);                                                                                               \
    _lf_replace_template_token((token_template_t*)out, newtoken);                                                      \
    out->value = newtoken->value;                                                                                      \
    out->length = newtoken->length;                                                                                    \
  } while (0)
#else
#define lf_set_token(out, newtoken)                                                                                    \
  do {                                                                                                                 \
    lf_set_present(out);                                                                                               \
    _lf_replace_template_token((token_template_t*)out, newtoken);                                                      \
    out->value = static_cast<decltype(out->value)>(newtoken->value);                                                   \
    out->length = newtoken->length;                                                                                    \
  } while (0)
#endif

/**
 * @brief Set the destructor associated with the specified port.
 *
 * The destructor will be used to free any value sent through the specified port when all
 * downstream users of the value are finished with it.
 *
 * @param out The output port (by name) or input of a contained reactor in form reactor.port_name.
 * @param dtor A pointer to a void function that takes a pointer argument
 * (or NULL to use the default void free(void*) function.
 */
#define lf_set_destructor(out, dtor) ((token_type_t*)out)->destructor = dtor

/**
 * @brief Set the copy constructor associated with the specified port.
 *
 * The copy constructor will be used to copy any value sent through the specified port whenever
 * a downstream user of the value declares a mutable input port or calls `lf_writable_copy()`.
 *
 * @param out The output port (by name) or input of a contained reactor in form reactor.port_name.
 * @param dtor A pointer to a void function that takes a pointer argument
 * (or NULL to use the default void `memcpy()` function.
 */
#define lf_set_copy_constructor(out, cpy_ctor) ((token_type_t*)out)->copy_constructor = cpy_ctor

#ifdef MODAL_REACTORS

/**
 * Sets the next mode of a modal reactor. As with `lf_set` for outputs, only
 * the last value will have effect if invoked multiple times at any given tag.
 * This works only in reactions with the target mode declared as effect.
 *
 * @param mode The target mode to set for activation.
 */
#define lf_set_mode(mode) _LF_SET_MODE_WITH_TYPE(mode, _lf_##mode##_change_type)

#endif // MODAL_REACTORS

/////////// Convenience macros.
// For simplicity and backward compatability, don't require the environment-pointer when calling the timing API.
// As long as this is done from the context of a reaction, `self` is in scope and is a pointer to the self-struct
// of the current reactor.

/**
 * Return the current tag of the environment invoking this reaction.
 */
#define lf_tag() lf_tag(self->base.environment)

/**
 * Return the current logical time in nanoseconds of the environment invoking this reaction.
 */
#define lf_time_logical() lf_time_logical(self->base.environment)

/**
 * Return the current logical time of the environment invoking this reaction relative to the
 * start time in nanoseconds.
 */
#define lf_time_logical_elapsed() lf_time_logical_elapsed(self->base.environment)

/**
 * @brief Return the instance name of the reactor.
 *
 * The instance name is the name of given to the instance created by the `new` operator in LF.
 * If the instance is in a bank, then the name will have a suffix of the form `[bank_index]`.
 *
 * @param reactor The reactor to get the name of.
 */
#define lf_reactor_name(reactor) lf_reactor_name(&reactor->base)

/**
 * @brief Return the fully qualified name of the reactor.
 *
 * The fully qualified name of a reactor is the instance name of the reactor concatenated with the names of all
 * of its parents, separated by dots. If the reactor or any of its parents is a bank, then the name
 * will have a suffix of the form `[bank_index]`.
 *
 * @param reactor The reactor to get the name of.
 */
#define lf_reactor_full_name(reactor) lf_reactor_full_name(&reactor->base)

#endif // REACTION_MACROS_H
