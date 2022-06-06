from setuptools import setup, Extension

linguafrancaPacManWithBankmodule = Extension("LinguaFrancaPacManWithBank",
                                            sources = ["ctarget/schedule.c", "ctarget/util.c", "core/mixed_radix.c", "core/platform/lf_linux_support.c", "PacManWithBank.c"],
                                            define_macros=[("MODULE_NAME", "LinguaFrancaPacManWithBank"), ("LOG_LEVEL", "2")])

setup(name="LinguaFrancaPacManWithBank", version="1.0",
        ext_modules = [linguafrancaPacManWithBankmodule],
        install_requires=["LinguaFrancaBase"])