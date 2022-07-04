/**
 * @file
 * @author Soroush Bateni (soroush@utdallas.edu)
 * @autohr Hou Seng Wong (housengw@berkeley.edu)
 *
 * @section LICENSE
Copyright (c) 2022, The University of California at Berkeley.
Copyright (c) 2021, The University of Texas at Dallas.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 * @section DESCRIPTION
 * Implementation of functions defined in @see pythontarget.h
 */

#include "pythontarget.h"
#include "python_tag.c"
#include "python_port.c"
#include "python_action.c"
#include "python_time.c"
#include "core/utils/util.h"
#include "core/tag.h"
#include "modal_models/definitions.h"

//////////// schedule Function(s) /////////////
/**
 * Prototype for the internal API. @see reactor_common.c
 **/
lf_token_t* _lf_initialize_token_with_value(lf_token_t* token, void* value, size_t length);

/**
 * Prototype for API function. @see lib/core/reactor_common.c
 **/
trigger_t* _lf_action_to_trigger(void* action);

/**
 * Schedule an action to occur with the specified time offset
 * with no payload (no value conveyed). This function is callable
 * in Python by calling action_name.schedule(offset).
 * Some examples include:
 *  action_name.schedule(5)
 *  action_name.schedule(NSEC(5))
 * See schedule_token(), which this uses, for details.
 * @param self Pointer to the calling object.
 * @param args contains:
 *      - action: Pointer to an action on the self struct.
 *      - offset: The time offset over and above that in the action.
 **/
static PyObject* py_schedule(PyObject *self, PyObject *args) {
    generic_action_capsule_struct* act = (generic_action_capsule_struct*)self;
    long long offset;
    PyObject* value = NULL;

    if (!PyArg_ParseTuple(args, "L|O", &offset, &value))
        return NULL;
    
    void* action = PyCapsule_GetPointer(act->action,"action");
    if (action == NULL) {
        lf_print_error("Null pointer received.");
        exit(1);
    }

    trigger_t* trigger = _lf_action_to_trigger(action);
    lf_token_t* t = NULL;

    // Check to see if value exists and token is not NULL
    if (value && (trigger->token != NULL)) {
        // DEBUG: adjust the element_size (might not be necessary)
        trigger->token->element_size = sizeof(PyObject*);
        trigger->element_size = sizeof(PyObject*);
        t = _lf_initialize_token_with_value(trigger->token, value, 1);

        // Also give the new value back to the Python action itself
        Py_INCREF(value);
        act->value = value;
    }

    
    // Pass the token along
    _lf_schedule_token(action, offset, t);

    // FIXME: handle is not passed to the Python side

    Py_INCREF(Py_None);
    return Py_None;
}


/**
 * Schedule an action to occur with the specified value and time offset
 * with a copy of the specified value.
 * See reactor.h for documentation.
 */
static PyObject* py_schedule_copy(PyObject *self, PyObject *args) {
    generic_action_capsule_struct* act;
    long long offset;
    PyObject* value;
    int length;

    if (!PyArg_ParseTuple(args, "OLOi" ,&act, &offset, &value, &length))
        return NULL;

    void* action = PyCapsule_GetPointer(act->action,"action");
    if (action == NULL) {
        lf_print_error("Null pointer received.");
        exit(1);
    }
    
    _lf_schedule_copy(action, offset, value, length);

    // FIXME: handle is not passed to the Python side

    Py_INCREF(Py_None);
    return Py_None;
}

/**
 * Prototype for the main function.
 */
int lf_reactor_c_main(int argc, char *argv[]);

/**
 * Prototype for lf_request_stop().
 * @see reactor.h
 */
void lf_request_stop();

///////////////// Other useful functions /////////////////////
/**
 * Stop execution at the conclusion of the current logical time.
 */
static PyObject* py_request_stop(PyObject *self, PyObject *args) {
    lf_request_stop();
    
    Py_INCREF(Py_None);
    return Py_None;
}

/**
 * Parse Python's 'argv' (from sys.argv()) into a pair of C-style 
 * 'argc' (the size of command-line parameters array) 
 * and 'argv' (an array of char* containing the command-line parameters).
 * 
 * This function assumes that argc is already allocated, and will fail if it
 * isn't.
 * 
 * @param py_argv The returned value by 'sys.argv()'
 * @param argc Will contain an integer which is the number of arguments
 *  passed on the command line.
 * @return A list of char*, where each item contains an individual
 *  command-line argument.
 */
char** _lf_py_parse_argv_impl(PyObject* py_argv, size_t* argc) {
    if (argc == NULL) {
        lf_print_error_and_exit("_lf_py_parse_argv_impl called with an unallocated argc argument.");
    }

    // List of arguments
    char** argv;

    // Read the optional argvs
    PyObject* py_argv_parsed = NULL;

    if (!PyArg_ParseTuple(py_argv, "|O", &py_argv_parsed)) {
        PyErr_SetString(PyExc_TypeError, "Could not get argvs.");
        return NULL;
    }

    if (py_argv_parsed == NULL) {
        // Build a generic argv with just one argument, which
        // is the module name.
        *argc = 1;
        argv = malloc(2 * sizeof(char*));
        argv[0] = TOSTRING(MODULE_NAME);
        argv[1] = NULL;
        return argv;
    }

    Py_ssize_t argv_size = PyList_Size(py_argv_parsed);
    argv = malloc(argv_size * sizeof(char *));
    for (Py_ssize_t i = 0; i < argv_size; i++) {
        PyObject* list_item = PyList_GetItem(py_argv_parsed, i);
        if (list_item == NULL) {
            if (PyErr_Occurred()) {
                PyErr_Print();
            }
            lf_print_error_and_exit("Could not get argv list item %u.", i);
        }

        PyObject *encoded_string = PyUnicode_AsEncodedString(list_item, "UTF-8", "strict");
        if (encoded_string == NULL) {
            if (PyErr_Occurred()) {
                PyErr_Print();
            }
            lf_print_error_and_exit("Failed to encode argv list item %u.", i);
        }

        argv[i] = PyBytes_AsString(encoded_string);

        if (PyErr_Occurred()) {
            PyErr_Print();
            lf_print_error_and_exit("Could not convert argv list item %u to char*.", i);
        }
    }
    *argc = argv_size;
    return argv;
}

//////////////////////////////////////////////////////////////
///////////// Main function callable from Python code
/**
 * The main function of this Python module.
 * 
 * @param py_args A single object, which should be a list
 *  of arguments taken from sys.argv().
 */
static PyObject* py_main(PyObject* self, PyObject* py_args) {

    LF_PRINT_DEBUG("Initializing main.");

    size_t argc;
    char** argv = _lf_py_parse_argv_impl(py_args, &argc);

    // Initialize the Python interpreter
    Py_Initialize();

    // Load the pickle module
    if (global_pickler == NULL) {
        global_pickler = PyImport_ImportModule("pickle");
        if (global_pickler == NULL) {
            if (PyErr_Occurred()) {
                PyErr_Print();
            }
            lf_print_error_and_exit("Failed to load the module 'pickle'.");
        }
    }

    LF_PRINT_DEBUG("Initialized the Python interpreter.");

    Py_BEGIN_ALLOW_THREADS
    lf_reactor_c_main(argc, argv);
    Py_END_ALLOW_THREADS

    Py_INCREF(Py_None);
    return Py_None;
}

///// Python Module Built-ins
/**
 * Bind Python function names to the C functions.
 * The name of this struct is dynamically generated because
 * MODULE_NAME is given by the generated code. This struct
 * will be named as MODULE_NAME_methods.
 * For example, for MODULE_NAME=Foo, this struct will
 * be called Foo_methods.
 * start() initiates the main loop in the C core library
 * @see schedule_copy
 * @see get_elapsed_logical_time
 * @see get_logical_time
 * @see get_physical_time
 * @see get_elapsed_physical_time
 * @see request_stop
 */
static PyMethodDef GEN_NAME(MODULE_NAME,_methods)[] = {
  {"start", py_main, METH_VARARGS, NULL},
  {"schedule_copy", py_schedule_copy, METH_VARARGS, NULL},
  {"get_logical_time", py_get_logical_time, METH_NOARGS, NULL},
  {"get_elapsed_logical_time", py_get_elapsed_logical_time, METH_NOARGS, NULL},
  {"get_physical_time", py_get_physical_time, METH_NOARGS, NULL},
  {"get_elapsed_physical_time", py_get_elapsed_physical_time, METH_NOARGS, NULL},
  {"get_start_time", py_get_start_time, METH_NOARGS, NULL},
  {"tag", py_lf_tag, METH_NOARGS, NULL},
  {"get_current_tag", py_get_current_tag, METH_NOARGS, NULL},
  {"get_microstep", py_get_microstep, METH_NOARGS, NULL},
  {"compare_tags", py_compare_tags, METH_VARARGS, NULL},
  {"tag_compare", py_tag_compare, METH_VARARGS, NULL},
  {"request_stop", py_request_stop, METH_NOARGS, NULL},
  {NULL, NULL, 0, NULL}
};


/**
 * Define the Lingua Franca module.
 * The MODULE_NAME is given by the generated code.
 */
static PyModuleDef MODULE_NAME = {
    PyModuleDef_HEAD_INIT,
    TOSTRING(MODULE_NAME),
    "LinguaFranca Python Module",
    -1,
    GEN_NAME(MODULE_NAME,_methods)
};

//////////////////////////////////////////////////////////////
/////////////  Module Initialization
/*
 * The Python runtime will call this function to initialize the module.
 * The name of this function is dynamically generated to follow
 * the requirement of PyInit_MODULE_NAME. Since the MODULE_NAME is not
 * known prior to compile time, the GEN_NAME macro is used.
 * The generated function will have the name PyInit_MODULE_NAME.
 * For example for a module named LinguaFrancaFoo, this function
 * will be called PyInit_LinguaFrancaFoo
 */
PyMODINIT_FUNC
GEN_NAME(PyInit_,MODULE_NAME)(void) {
    PyObject *m;

    // Initialize the port_capsule type
    if (PyType_Ready(&py_port_capsule_t) < 0) {
        return NULL;
    }

    // Initialize the action_capsule type
    if (PyType_Ready(&py_action_capsule_t) < 0) {
        return NULL;
    }

    // Initialize the Tag type
    if (PyType_Ready(&PyTagType) < 0) {
        return NULL;
    }

    // Initialize the Time type
    if (PyType_Ready(&PyTimeType) < 0) {
        return NULL;
    }

    m = PyModule_Create(&MODULE_NAME);

    if (m == NULL) {
        return NULL;
    }

    initialize_mode_capsule_t(m);

    // Add the port_capsule type to the module's dictionary
    Py_INCREF(&py_port_capsule_t);
    if (PyModule_AddObject(m, "port_capsule", (PyObject *) &py_port_capsule_t) < 0) {
        Py_DECREF(&py_port_capsule_t);
        Py_DECREF(m);
        return NULL;
    }


    // Add the action_capsule type to the module's dictionary
    Py_INCREF(&py_action_capsule_t);
    if (PyModule_AddObject(m, "action_capsule_t", (PyObject *) &py_action_capsule_t) < 0) {
        Py_DECREF(&py_action_capsule_t);
        Py_DECREF(m);
        return NULL;
    }

    // Add the Tag type to the module's dictionary
    Py_INCREF(&PyTagType);
    if (PyModule_AddObject(m, "Tag", (PyObject *) &PyTagType) < 0) {
        Py_DECREF(&PyTagType);
        Py_DECREF(m);
        return NULL;
    }

    // Add the Time type to the module's dictionary
    Py_INCREF(&PyTimeType);
    if (PyModule_AddObject(m, "time", (PyObject *) &PyTimeType) < 0) {
        Py_DECREF(&PyTimeType);
        Py_DECREF(m);
        return NULL;
    }

    return m;
}

//////////////////////////////////////////////////////////////
/////////////  Python Helper Functions
/// These functions are called in generated C code for various reasons.
/// Their main purpose is to facilitate C runtime's communication with
/// Python code.
/**
 * A function that destroys action capsules
 **/
void destroy_action_capsule(PyObject* capsule) {
    free(PyCapsule_GetPointer(capsule, "action"));
}

/**
 * A function that is called any time a Python reaction is called with
 * ports as inputs and outputs. This function converts ports that are
 * either a multiport or a non-multiport into a port_capsule.
 * 
 * First, the void* pointer is stored in a PyCapsule. If the port is not
 * a multiport, the value and is_present fields are copied verbatim. These
 * feilds then can be accessed from the Python code as port.value and
 * port.is_present.
 * If the value is absent, it will be set to None.
 * 
 * For multiports, the value of the port_capsule (i.e., port.value) is always
 * set to None and is_present is set to false.
 * Individual ports can then later be accessed in Python code as port[idx].
 */
PyObject* convert_C_port_to_py(void* port, int width) {
    generic_port_instance_struct* cport;
    if (width == -2) {
        // Not a multiport
        cport = (generic_port_instance_struct *)port;
    }
    // Create the port struct in Python
    PyObject* cap = 
        (PyObject*)PyObject_GC_New(generic_port_capsule_struct, &py_port_capsule_t);
    if (cap == NULL) {
        lf_print_error_and_exit("Failed to convert port.");
    }

    // Create the capsule to hold the void* port
    PyObject* capsule = PyCapsule_New(port, "port", NULL);
    if (capsule == NULL) {
        lf_print_error_and_exit("Failed to convert port.");
    }

    // Fill in the Python port struct
    ((generic_port_capsule_struct*)cap)->port = capsule;
    ((generic_port_capsule_struct*)cap)->width = width;
    FEDERATED_ASSIGN_FIELDS(((generic_port_capsule_struct*)cap), cport);

    if (width == -2) {
        ((generic_port_capsule_struct*)cap)->is_present = 
            cport->is_present;


        if (cport->value == NULL) {
            // Value is absent
            Py_INCREF(Py_None);
            ((generic_port_capsule_struct*)cap)->value = Py_None;
            return cap;
        }

        //Py_INCREF(cport->value);
        ((generic_port_capsule_struct*)cap)->value = cport->value;
    } else {
        // Multiport. Value of the multiport itself cannot be accessed, so we set it to
        // None.
        Py_INCREF(Py_None);
        ((generic_port_capsule_struct*)cap)->value = Py_None;
        ((generic_port_capsule_struct*)cap)->is_present = false;
    }

    return cap;
}

/**
 * A helper function to convert C actions to Python action capsules
 * @see xtext/org.icyphy.linguafranca/src/org/icyphy/generator/CGenerator.xtend for details about C actions
 * Python actions have the following fields (for more informatino @see generic_action_capsule_struct):
 *   PyObject_HEAD
 *   PyObject* action;   
 *   PyObject* value; 
 *   bool is_present; 
 *   
 * The input to this function is a pointer to a C action, which might or 
 * might not contain a value and an is_present field. To simplify the assumptions
 * made by this function, the "value" and "is_present" are passed to the function
 * instead of expecting them to exist.
 * 
 * The void* pointer to the C action instance is encapsulated in a PyCapsule instead of passing an exposed pointer through
 * Python. @see https://docs.python.org/3/c-api/capsule.html
 * This encapsulation is done by calling PyCapsule_New(action, "name_of_the_container_in_the_capsule", NULL), 
 * where "name_of_the_container_in_the_capsule" is an agreed-upon container name inside the capsule. This 
 * capsule can then be treated as a PyObject* and safely passed through Python code. On the other end 
 * (which is in schedule functions), PyCapsule_GetPointer(received_action,"action") can be called to retrieve 
 * the void* pointer into received_action.
 **/
PyObject* convert_C_action_to_py(void* action) {
    // Convert to trigger_t
    trigger_t* trigger = _lf_action_to_trigger(action);

    // Create the action struct in Python
    PyObject* cap = (PyObject*)PyObject_GC_New(generic_action_capsule_struct, &py_action_capsule_t);
    if (cap == NULL) {
        lf_print_error_and_exit("Failed to convert action.");
    }

    // Create the capsule to hold the void* action
    PyObject* capsule = PyCapsule_New(action, "action", NULL);
    if (capsule == NULL) {
        lf_print_error_and_exit("Failed to convert action.");
    }

    // Fill in the Python action struct
    ((generic_action_capsule_struct*)cap)->action = capsule;
    ((generic_action_capsule_struct*)cap)->is_present = trigger->status;
    FEDERATED_ASSIGN_FIELDS(((generic_port_capsule_struct*)cap), ((generic_action_instance_struct*)action));

    // If token is not initialized, that is all we need to set
    if (trigger->token == NULL) {
        Py_INCREF(Py_None);
        ((generic_action_capsule_struct*)cap)->value = Py_None;
        return cap;
    }

    // Default value is None
    if (trigger->token->value == NULL) {
        Py_INCREF(Py_None);
        trigger->token->value = Py_None;
    }

    // Actions in Python always use token type
    ((generic_action_capsule_struct*)cap)->value = trigger->token->value;

    return cap;
}

/** 
 * Invoke a Python func in class[instance_id] from module.
 * Class instances in generated Python code are always instantiated in a 
 * list of template classs[_class(params), _class(params), ...] (note the extra s) regardless 
 * of whether a bank is used or not. If there is no bank, or a bank of width 1, the list will be
 * instantiated as classs[_class(params)].
 * 
 * This function would thus call classs[0] to access the first instance in a bank and so on.
 * 
 * Possible optimizations include: - Not loading the module each time (by storing it in global memory),
 *                                 - Keeping a persistent argument table
 * @param module The Python module to load the function from. In embedded mode, it should
 *               be set to "__main__"
 * @param class The name of the list of classes in the generated Python code
 * @param instance_id The element number in the list of classes. class[instance_id] points to a class instance
 * @param func The reaction functino to be called
 * @param pArgs the PyList of arguments to be sent to function func()
 * @return The function or NULL on error.
 */
PyObject*
get_python_function(string module, string class, int instance_id, string func) {
    LF_PRINT_DEBUG("Starting the function start().");

    // Necessary PyObject variables to load the react() function from test.py
    PyObject* pFileName = NULL;
    PyObject* pModule = NULL;
    PyObject* pDict = NULL;
    PyObject* pClasses = NULL;
    PyObject* pClass = NULL;
    PyObject* pFunc = NULL;

    // According to
    // https://docs.python.org/3/c-api/init.html#non-python-created-threads
    // the following code does the following:
    // - Register this thread with the interpreter
    // - Acquire the GIL (Global Interpreter Lock)
    // - Store (return) the thread pointer
    // When done, we should always call PyGILState_Release(gstate);
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    // If the Python module is already loaded, skip this.
    if (globalPythonModule == NULL) {    
        // Decode the MODULE name into a filesystem compatible string
        pFileName = PyUnicode_DecodeFSDefault(module);
        
        // Set the Python search path to be the current working directory
        char cwd[PATH_MAX];
        if ( getcwd(cwd, sizeof(cwd)) == NULL) {
            lf_print_error_and_exit("Failed to get the current working directory.");
        }

        wchar_t wcwd[PATH_MAX];

        mbstowcs(wcwd, cwd, PATH_MAX);

        Py_SetPath(wcwd);

        LF_PRINT_DEBUG("Loading module %s in %s.", module, cwd);

        pModule = PyImport_Import(pFileName);

        LF_PRINT_DEBUG("Loaded module %p.", pModule);

        // Free the memory occupied by pFileName
        Py_DECREF(pFileName);

        // Check if the module was correctly loaded
        if (pModule != NULL) {
            // Get contents of module. pDict is a borrowed reference.
            pDict = PyModule_GetDict(pModule);
            if (pDict == NULL) {
                PyErr_Print();
                lf_print_error("Failed to load contents of module %s.", module);
                /* Release the thread. No Python API allowed beyond this point. */
                PyGILState_Release(gstate);
                return NULL;
            }

            Py_INCREF(pModule);
            globalPythonModule = pModule;
            Py_INCREF(pDict);
            globalPythonModuleDict = pDict;

        }
    }

    if (globalPythonModule != NULL && globalPythonModuleDict != NULL) {
        Py_INCREF(globalPythonModule);
        // Convert the class name to a PyObject
        PyObject* list_name = PyUnicode_DecodeFSDefault(class);

        // Get the class list
        Py_INCREF(globalPythonModuleDict);
        pClasses = PyDict_GetItem(globalPythonModuleDict, list_name);
        if (pClasses == NULL){
            PyErr_Print();
            lf_print_error("Failed to load class list \"%s\" in module %s.", class, module);
            /* Release the thread. No Python API allowed beyond this point. */
            PyGILState_Release(gstate);
            return NULL;
        }

        Py_DECREF(globalPythonModuleDict);

        pClass = PyList_GetItem(pClasses, instance_id);
        if (pClass == NULL) {
            PyErr_Print();
            lf_print_error("Failed to load class \"%s[%d]\" in module %s.", class, instance_id, module);
            /* Release the thread. No Python API allowed beyond this point. */
            PyGILState_Release(gstate);
            return NULL;
        }

        LF_PRINT_DEBUG("Loading function %s.", func);

        // Get the function react from test.py
        pFunc = PyObject_GetAttrString(pClass, func);

        LF_PRINT_DEBUG("Loaded function %p.", pFunc);

        // Check if the funciton is loaded properly
        // and if it is callable
        if (pFunc && PyCallable_Check(pFunc)) {
            LF_PRINT_DEBUG("Calling function %s from class %s[%d].", func , class, instance_id);
            Py_INCREF(pFunc);
            /* Release the thread. No Python API allowed beyond this point. */
            PyGILState_Release(gstate);
            return pFunc;
        }
        else {
            // Function is not found or it is not callable
            if (PyErr_Occurred()) {
                PyErr_Print();
            }
            lf_print_error("Function %s was not found or is not callable.", func);
        }
        Py_XDECREF(pFunc);
        Py_DECREF(globalPythonModule); 
    } else {
        PyErr_Print();
        lf_print_error("Failed to load \"%s\".", module);
    }
    
    LF_PRINT_DEBUG("Done with start().");

    Py_INCREF(Py_None);
    /* Release the thread. No Python API allowed beyond this point. */
    PyGILState_Release(gstate);
    return Py_None;
}
