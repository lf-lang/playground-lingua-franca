/**
 * @file
 * @autohr Hou Seng Wong (housengw@berkeley.edu)
 *
 * @section LICENSE
Copyright (c) 2022, The University of California at Berkeley.

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

#include "python_tag.h"

PyTypeObject PyTagType;

/** 
 * Return the current tag object.
 */
static PyObject* py_lf_tag(PyObject *self, PyObject *args) {
    py_tag_t *t = (py_tag_t *) PyType_GenericNew(&PyTagType, NULL, NULL);
    if (t == NULL) {
        return NULL;
    }
    t->tag = lf_tag();
    return (PyObject *) t;
}

/** 
 * Return the current tag object.
 * @deprecated
 */
static PyObject* py_get_current_tag(PyObject *self, PyObject *args) {
    PyErr_WarnEx(PyExc_DeprecationWarning, "get_current_tag() is deprecated. Use lf.tag() instead", 1);
    return py_lf_tag(self, args);
}

/**
 * Compare two tags. Return -1 if the first is less than
 * the second, 0 if they are equal, and +1 if the first is
 * greater than the second. A tag is greater than another if
 * its time is greater or if its time is equal and its microstep
 * is greater.
 * @param tag1
 * @param tag2
 * @return -1, 0, or 1 depending on the relation.
 */
PyObject* py_tag_compare(PyObject *self, PyObject *args) {
    PyObject *tag1;
    PyObject *tag2;
    if (!PyArg_UnpackTuple(args, "args", 2, 2, &tag1, &tag2)) {
        return NULL;
    } 
    if (!PyObject_IsInstance(tag1, (PyObject *) &PyTagType) 
     || !PyObject_IsInstance(tag2, (PyObject *) &PyTagType)) {
        PyErr_SetString(PyExc_TypeError, "Arguments must be Tag type.");
        return NULL;
    }
    tag_t tag1_v = ((py_tag_t *) tag1)->tag;
    tag_t tag2_v = ((py_tag_t *) tag2)->tag;
    return PyLong_FromLong(lf_tag_compare(tag1_v, tag2_v));
}


/**
 * @deprecated version of "py_tag_compare"
 */
PyObject* py_compare_tags(PyObject *self, PyObject *args) {
    PyErr_WarnEx(PyExc_DeprecationWarning, "compare_tags() is deprecated. Use lf.tag_compare() instead", 1);
    return py_tag_compare(self, args);
}


/**
 * Initialize the Tag object with the given values for "time" and "microstep", 
 * both of which are required.
 * @param self A py_tag_t object.
 * @param args The arguments are:
 *      - time: A logical time.
 *      - microstep: A microstep within the logical time "time".
 */
int Tag_init(py_tag_t *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"time", "microstep", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "Lk", kwlist, &(self->tag.time), &(self->tag.microstep))) {
        return -1;
    }
    return 0;
}

/**
 * Rich compare function for Tag objects. Used in .tp_richcompare.
 * 
 * @param self A py_tag_t object on the left side of the operator.
 * @param other A py_tag_t object on the right side of the operator.
 * @param op the comparison operator
 */
PyObject *Tag_richcompare(py_tag_t *self, PyObject *other, int op) {
    if (!PyObject_IsInstance(other, (PyObject *) &PyTagType)) {
        PyErr_SetString(PyExc_TypeError, "Cannot compare a Tag with a non-Tag type.");
        return NULL;
    }

    tag_t other_tag = ((py_tag_t *) other)->tag;
    int c = -1;
    if (op == Py_LT) {
        c = (lf_tag_compare(self->tag, other_tag) < 0);
    } else if (op == Py_LE) {
        c = (lf_tag_compare(self->tag, other_tag) <= 0);
    } else if (op == Py_EQ) {
        c = (lf_tag_compare(self->tag, other_tag) == 0);
    } else if (op == Py_NE) {
        c = (lf_tag_compare(self->tag, other_tag) != 0);
    } else if (op == Py_GT) {
        c = (lf_tag_compare(self->tag, other_tag) > 0);
    } else if (op == Py_GE) {
        c = (lf_tag_compare(self->tag, other_tag) >= 0);
    }
    if (c < 0) {
        PyErr_SetString(PyExc_RuntimeError, "Invalid comparator (This statement should never be reached). ");
        return NULL;
    } else if (c) {
        Py_RETURN_TRUE;
    } else {
        Py_RETURN_FALSE;
    }
}

/**
 * Tag getter for the "time" attribute
 **/
PyObject* Tag_get_time(py_tag_t *self, void *closure) {
    return PyLong_FromLongLong(self->tag.time);
}

/**
 * Tag getter for the "microstep" attribute
 **/
PyObject* Tag_get_microstep(py_tag_t *self, void *closure) {
    return PyLong_FromUnsignedLong(self->tag.microstep);
}

/**
 * Link names to getter functions.
 * Getters are used when the variable name specified are referenced with a ".".
 * For example:
 * >>> t = Tag(time=1, microstep=2)
 * >>> t.time   # calls Tag_get_time.
 * >>> t.microstep  # calls Tag_get_microstep.
 * >>> t.time = 1  # illegal since setters are omitted.
 **/
PyGetSetDef Tag_getsetters[] = {
    {"time", (getter) Tag_get_time},
    {"microstep", (getter) Tag_get_microstep},
    {NULL}  /* Sentinel */
};

/**
 * Definition of the PyTagType Object. 
 **/
PyTypeObject PyTagType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "LinguaFranca.Tag",
    .tp_doc = "Tag object",
    .tp_basicsize = sizeof(py_tag_t),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_new = PyType_GenericNew,
    .tp_init = (initproc) Tag_init,
    .tp_richcompare = (richcmpfunc) Tag_richcompare,
    .tp_getset = Tag_getsetters,
};
