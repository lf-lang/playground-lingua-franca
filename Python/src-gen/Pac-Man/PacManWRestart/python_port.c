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

#include "python_port.h"

PyTypeObject py_port_capsule_t;

//////////// set Function(s) /////////////
/**
 * Set the value and is_present field of self which is of type
 * LinguaFranca.port_capsule
 * 
 * Each LinguaFranca.port_capsule includes a void* pointer of 
 * the C port (a.k.a. generic_port_instance_struct*).
 * @see generic_port_capsule_struct in pythontarget.h
 * 
 * This function calls the underlying _LF_SET API.
 * @see xtext/org.icyphy.linguafranca/src/lib/core/reactor.h
 * 
 * This function can be used to set any type of PyObject ranging from
 * primitive types to complex lists and tuples. Moreover, this function
 * is callable from Python target code by using port_name.out(value)
 * 
 * Some examples include
 *  port_name.out("Hello")
 *  port_name.out(5)
 *  port_name.out(["Hello", 5 , (2.8, "X")])
 * 
 * The port type given in the Lingua Franca is only used as a "suggestion"
 * as per Python's duck typing principles. The end-user is responsible for
 * appropriately handling types on the recieveing end of this port.
 * @param self The output port (by name) or input of a contained
 *                 reactor in form instance_name.port_name.
 * @param args contains:
 *      - val: The value to insert into the port struct.
 */
PyObject* py_port_set(PyObject *self, PyObject *args) {
    generic_port_capsule_struct* p = (generic_port_capsule_struct*)self;
    PyObject* val = NULL;

    if (!PyArg_ParseTuple(args, "O", &val)) {
        PyErr_SetString(PyExc_TypeError, "Could not set objects.");
        return NULL;
    }

    generic_port_instance_struct* port = 
        PyCapsule_GetPointer(p->port, "port");
    if (port == NULL) {
        error_print("Null pointer received.");
        exit(1);
    }
    
    if (val) {
        LF_PRINT_DEBUG("Setting value %p.", val);
        Py_XDECREF(port->value);
        Py_INCREF(val);
        // Call the core lib API to set the port
        _LF_SET(port, val);

        Py_INCREF(val);
        // Also set the values for the port capsule.      
        p->value = val;
        p->is_present = true;
    }

    Py_INCREF(Py_None);
    return Py_None;
}

/**
 * Called when a port_capsule has to be deallocated (generally by the Python
 * garbage collector).
 * @param self An instance of generic_port_instance_struct*
 */
void py_port_capsule_dealloc(generic_port_capsule_struct *self) {
    Py_XDECREF(self->port);
    Py_XDECREF(self->value);
    Py_TYPE(self)->tp_free((PyObject *) self);
}

/**
 * Create a new port_capsule. Note that a LinguaFranca.port_capsule PyObject
 * follows the same structure as the @see generic_port_capsule_struct.
 * 
 * To initialize the port_capsule, this function first initializes a 
 * generic_port_capsule_struct* self using the tp_alloc property of 
 * port_capsule (@see py_port_capsule_t) and then assigns the members
 * of self with default values of port= NULL, value = NULL, is_present = false,
 * current_index = 0, width = -2.
 * @param type The Python type object. In this case, py_port_capsule_t
 * @param args The optional arguments that are:
 *      - port: A capsule that holds a void* to the underlying C port
 *      - value: value of the port
 *      - is_present: An indication of whether or not the value of the port
 *                       is present at the current logical time.
 *      - current_index: Used to reference multiports in the iterator
 *      - width: Used to indicate the width of a multiport. If the port
 *                   is not a multiport, this field will be -2.
 * @param kwds Keywords (@see Python keywords)
 */
PyObject *py_port_capsule_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    generic_port_capsule_struct *self;
    self = (generic_port_capsule_struct *) type->tp_alloc(type, 0);
    if (self != NULL) {
        self->port = NULL;
        Py_INCREF(Py_None);
        self->value = Py_None;
        self->is_present = false;
        self->current_index = 0;
        self->width = -2;
    }
    return (PyObject *) self;
}

/**
 * Return an iterator for self, which is a port.
 * This function just have to exist to tell Python that ports are iterable.
 * 
 * For example to make
 *     for p in foo_multiport:
 *         p.set(42)
 * possible in Python.
 */
PyObject *py_port_iter(PyObject *self) {
  generic_port_capsule_struct* port = (generic_port_capsule_struct*)self;
  port->current_index = 0;
  Py_INCREF(self);
  return self;
}

/**
 * The function that is responsible for getting the next item in the iterator for a multiport.
 * 
 * This would make the following code possible in the Python target:
 *     for p in foo_multiport:
 *         p.set(42)
 */
PyObject *py_port_iter_next(PyObject *self) {
    generic_port_capsule_struct* port = (generic_port_capsule_struct*)self;
    generic_port_capsule_struct* pyport = (generic_port_capsule_struct*)self->ob_type->tp_new(self->ob_type, NULL, NULL);

    if (port->width < 1) {
        PyErr_Format(PyExc_TypeError,
                "Non-multiport type is not iteratable.");
        return NULL;
    }

    if (port->current_index >= port->width) {
        port->current_index = 0;
        return NULL;
    }

    generic_port_instance_struct **cport = 
        (generic_port_instance_struct **)PyCapsule_GetPointer(port->port,"port");
    if (cport == NULL) {
        error_print_and_exit("Null pointer received.");
    }

    // Py_XINCREF(cport[index]->value);
    pyport->port = PyCapsule_New(cport[port->current_index], "port", NULL);
    pyport->value = cport[port->current_index]->value;
    pyport->is_present = cport[port->current_index]->is_present;
    pyport->width = -2;

    port->current_index++;

    if (pyport->value == NULL) {
        Py_INCREF(Py_None);
        pyport->value = Py_None;
    }

    Py_XINCREF(pyport);
    return (PyObject*)pyport;
}
/**
 * Get an item from a Linugua Franca port capsule type.
 * If a port is a not a multiport, it will have a width of
 * -2 (@see CGenerator.xtend). In this case, this function will
 * return the port capsule itself.
 * If a port is a multiport, this function will convert the index
 * item and convert it into a C long long, and use it to access
 * the underlying array stored in the PyCapsule as "port". A new
 * non-multiport capsule is created and returned, which in turn can be
 * used as an ordinary LinguaFranca.port_capsule.
 * @param self The port which can be a multiport or a singular port
 * @param key The index (key) which is used to retrieve an item from the underlying
 *             C array if the port is a multiport.
 */
PyObject *py_port_capsule_get_item(PyObject *self, PyObject *key) {
    generic_port_capsule_struct* port = (generic_port_capsule_struct*)self;

    // Port is not a multiport
    if (port->width == -2) {
        return self;
    }

    if (PyObject_TypeCheck(key, &PyLong_Type) == 0) {
        PyErr_Format(PyExc_TypeError,
                     "Multiport indices must be integers, not %.200s",
                     Py_TYPE(key)->tp_name);
        return NULL;
    }

    generic_port_capsule_struct* pyport = 
        (generic_port_capsule_struct*)self->ob_type->tp_new(self->ob_type, NULL, NULL);
    long long index = -3;

    index = PyLong_AsLong(key);
    if (index == -3) {
        PyErr_Format(PyExc_TypeError,
                     "Multiport indices must be integers, not %.200s",
                     Py_TYPE(key)->tp_name);
        return NULL;
    }

    generic_port_instance_struct **cport = 
        (generic_port_instance_struct **)PyCapsule_GetPointer(port->port,"port");
    if (cport == NULL) {
        error_print_and_exit("Null pointer received.");
    }

    // Py_INCREF(cport[index]->value);
    pyport->port = PyCapsule_New(cport[index], "port", NULL);
    pyport->value = cport[index]->value;
    pyport->is_present = cport[index]->is_present;
    pyport->width = -2;


    LF_PRINT_LOG("Getting item index %d. Is present is %d.", index, pyport->is_present);

    
    if (pyport->value == NULL) {
        Py_INCREF(Py_None);
        pyport->value = Py_None;
    }

    //Py_INCREF(((generic_port_capsule_struct*)port)->value);
    Py_XINCREF(pyport);
    //Py_INCREF(self);
    return (PyObject*)pyport;
}

/**
 * This function is overloaded to prevent directly assigning to multiports.
 * The set function currently is the only way to assign value to ports.
 * @param self The port of type LinguaFranca.port_capsule
 * @param item The index (which is ignored)
 * @param value The value to be assigned (which is ignored)
 */
int py_port_capsule_assign_get_item(PyObject *self, PyObject *item, PyObject* value) {
    PyErr_Format(PyExc_TypeError,
                     "You cannot assign to ports directly. Please use the .set method.",
                     Py_TYPE(item)->tp_name);
    return -1;
}

/**
 * A function that allows the invocation of len() on a port.
 * @param self A port of type LinguaFranca.port_capsule
 */ 
Py_ssize_t py_port_length(PyObject *self) {
    generic_port_capsule_struct* port = (generic_port_capsule_struct*)self;
    LF_PRINT_DEBUG("Getting the length, which is %d.", port->width);
    return (Py_ssize_t)port->width;
}

/**
 * Methods that convert a LinguaFranca.port_capsule into a mapping,
 * which allows it to be subscriptble.
 */ 
PyMappingMethods py_port_as_mapping = {    
    (lenfunc) py_port_length,
    (binaryfunc) py_port_capsule_get_item,
    (objobjargproc) py_port_capsule_assign_get_item
};

/**
 * Initialize the port capsule self with the given optional values for
 * port, value, is_present, and num_destinations. If any of these arguments
 * are missing, the default values are assigned
 * @see port_intance_new 
 * @param self The port_instance PyObject that follows
 *              the generic_port_instance_struct* internal structure
 * @param args The optional arguments that are:
 *      - port: A capsule that holds a void* to the underlying C port
 *      - value: value of the port
 *      - is_present: An indication of whether or not the value of the port
 *                       is present at the current logical time.
 *      - current_index: Used to reference multiports in the iterator
 *      - width: Used to indicate the width of a multiport. If the port
 *                   is not a multiport, this field will be -2.
 */
int py_port_capsule_init(generic_port_capsule_struct *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = { "port", "value", "is_present", "width", "current_index", NULL};
    PyObject *value = NULL, *tmp, *port = NULL;

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|OOp", kwlist,
                                     &port, &value, &self->is_present, &self->width, &self->current_index))
    {
        return -1;
    }

    if (value){
        tmp = self->value;
        Py_INCREF(value);
        self->value = value;
        Py_XDECREF(tmp);
    }

    if (port){
        tmp = self->port;
        Py_INCREF(port);
        self->port = port;
        Py_XDECREF(tmp);
    }
    return 0;
}

////// Ports //////
/*
 * The members of a port_capsule, used to define
 * a native Python type.
 * port contains the port capsule, which holds a void* pointer to the underlying C port.
 * value contains the copied value of the C port. The type is always PyObject*.
 * is_present contains the copied value of the is_present field of the C port.
 * width indicates the width of a multiport or -2 if not a multiport.
 */
PyMemberDef py_port_capsule_members[] = {
    {"port", T_OBJECT, offsetof(generic_port_capsule_struct, port), READONLY, ""},
    {"value", T_OBJECT, offsetof(generic_port_capsule_struct, value), READONLY, "Value of the port"},
    {"is_present", T_BOOL, offsetof(generic_port_capsule_struct, is_present), READONLY, "Check if value is present at current logical time"},
    {"width", T_INT, offsetof(generic_port_capsule_struct, width), READONLY, "Width of the multiport"},    
    {NULL}  /* Sentinel */
};

/*
 * The function members of port_capsule
 * __getitem__ is used to reference a multiport with an index (e.g., foo[2])
 * set is used to set a port value and its is_present field.
 */
PyMethodDef py_port_capsule_methods[] = {
    {"__getitem__", (PyCFunction)py_port_capsule_get_item, METH_O|METH_COEXIST, "x.__getitem__(y) <==> x[y]"},
    {"set", (PyCFunction)py_port_set, METH_VARARGS, "Set value of the port as well as the is_present field"},
    {NULL}  /* Sentinel */
};


/*
 * The definition of port_capsule type object, which is
 * used to describe how port_capsule behaves.
 */
PyTypeObject py_port_capsule_t = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "LinguaFranca.port_capsule",
    .tp_doc = "port_capsule objects",
    .tp_basicsize = sizeof(generic_port_capsule_struct),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_as_mapping = &py_port_as_mapping,
    .tp_iter = py_port_iter,
    .tp_iternext = py_port_iter_next,
    .tp_new = py_port_capsule_new,
    .tp_init = (initproc) py_port_capsule_init,
    .tp_dealloc = (destructor) py_port_capsule_dealloc,
    .tp_members = py_port_capsule_members,
    .tp_methods = py_port_capsule_methods,
};