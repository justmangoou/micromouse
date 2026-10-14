# The ststm32 builder does not pass the float ABI to the linker; give it to compiler and linker alike.
Import("env")
flags = ["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
env.Append(CCFLAGS=flags, LINKFLAGS=flags)
