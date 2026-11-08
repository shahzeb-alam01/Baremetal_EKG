Import("env")

# Keep the compiler and linker on the STM32G4 hard-float ABI.
env.Append(
    ASFLAGS=["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"],
    CCFLAGS=["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"],
    LINKFLAGS=["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"],
    CFLAGS=["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"],
    CXXFLAGS=["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
)

print("=== Hard-Float FPU Flags Added ===")
