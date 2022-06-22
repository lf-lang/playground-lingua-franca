from setuptools import setup, Extension

linguafrancaPacManmodule = Extension("LinguaFrancaPacMan",
                                            sources = ["ctarget/schedule.c", "ctarget/util.c", "core/mixed_radix.c", "core/platform/lf_linux_support.c", "PacMan.c"],
                                            define_macros=[("MODULE_NAME", "LinguaFrancaPacMan"), ("LOG_LEVEL", "2")])

setup(name="LinguaFrancaPacMan", version="1.0",
        ext_modules = [linguafrancaPacManmodule],
        install_requires=["LinguaFrancaBase"])