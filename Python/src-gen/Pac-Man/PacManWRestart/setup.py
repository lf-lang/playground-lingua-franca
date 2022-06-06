from setuptools import setup, Extension

linguafrancaPacManWRestartmodule = Extension("LinguaFrancaPacManWRestart",
                                            sources = ["ctarget/schedule.c", "ctarget/util.c", "core/mixed_radix.c", "core/platform/lf_linux_support.c", "PacManWRestart.c"],
                                            define_macros=[("MODULE_NAME", "LinguaFrancaPacManWRestart"), ("LOG_LEVEL", "2")])

setup(name="LinguaFrancaPacManWRestart", version="1.0",
        ext_modules = [linguafrancaPacManWRestartmodule],
        install_requires=["LinguaFrancaBase"])