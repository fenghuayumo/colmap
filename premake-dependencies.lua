-- VULKAN_SDK = os.getenv("VULKAN_SDK")
PACKAGE_PATH = os.getenv("PACKAGE_PATH")
CUDA_PATH = os.getenv("CUDA_PATH")

IncludeDir = {}
IncludeDir["Package"] = "%{PACKAGE_PATH}/include/"
-- IncludeDir["Eigen"] = "%{PACKAGE_PATH}/include/"
IncludeDir["CUDA_PATH"] = "%{CUDA_PATH}/include/"

LibraryDir = {}
LibraryDir["Package"] = "%{PACKAGE_PATH}/lib/"
LibraryDir["CUDA_PATH"] = "%{CUDA_PATH}/lib/x64"