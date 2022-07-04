

import numpy as np

import os
import sys
import subprocess
import shutil
from functools import partial
import statistics



char_to_replace= {
    '$TIMEOUT$':'10 sec',
    '$NUM_WORKERS$':'8',
    '$STARTOUTPUT$' : '',
    '$STARTREACTION$': '',
    '$TASKCONFIG$': '',
}

