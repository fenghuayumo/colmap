require 'Scripts/premake-utilities/premake-defines'
require 'Scripts/premake-utilities/premake-common'
require 'Scripts/premake-utilities/premake-triggers'
require 'Scripts/premake-utilities/premake-settings'
require 'Scripts/premake-utilities/android_studio'

if os.target() == "windows" or os.taget == "linux" then
require 'Scripts/premake-utilities/premake5-cuda/premake5-cuda'
end

include "premake-dependencies.lua"
--require 'Scripts/premake-utilities/premake-vscode/vscode'

function getCudaVersion()
	local nvccOutput = io.popen("nvcc --version"):read("*a")
	local cudaVersion = nvccOutput:match("Cuda compilation tools, release (%d+%.%d+)")
	if cudaVersion then
	print("Detected CUDA Version: " .. cudaVersion)
	else
	print("CUDA version could not be detected.")
	end
	return cudaVersion
end

cudaBuildCustomizations = "BuildCustomizations/CUDA " .. getCudaVersion()

-- print(cudaBuildCustomizations)

root_dir = os.getcwd()

Arch = ""

if _OPTIONS["arch"] then
	Arch = _OPTIONS["arch"]
else
	if _OPTIONS["os"] then
		_OPTIONS["arch"] = "arm"
		Arch = "arm"
	else
		_OPTIONS["arch"] = "x64"
		Arch = "x64"
	end
end

workspace( settings.workspace_name )
	startproject "util"
	flags 'MultiProcessorCompile'
	outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"
	targetdir ("bin/%{outputdir}/")
	objdir ("bin-int/%{outputdir}/obj/")

	gradleversion "com.android.tools.build:gradle:7.0.0"

	if Arch == "arm" then 
		architecture "ARM"
	elseif Arch == "x64" then 
		architecture "x86_64"
	elseif Arch == "x86" then
		architecture "x86"
	end

	print("Arch = ", Arch)

	configurations
	{
		"Debug",
		"Release",
		"Production"
	}

	assetpacks
	{
		["pack"] = "install-time",
	}
	
	defines
	{
		"COLMAP_GPU_ENABLED"
	}
	include "src/colmap/util/premake5"
	include "src/colmap/controllers/premake5"
	include "src/colmap/estimators/premake5"
	include "src/colmap/feature/premake5"
	include "src/colmap/image/premake5"
	include "src/colmap/geometry/premake5"
	include "src/colmap/math/premake5"
	include "src/colmap/scene/premake5"
	include "src/colmap/sensor/premake5"
	include "src/colmap/sfm/premake5"
	include "src/colmap/mvs/premake5"
	include "src/colmap/optim/premake5"
	include "src/colmap/retrieval/premake5"
	include "src/thirdparty/LSD/premake5"
	include "src/thirdparty/PoissonRecon/premake5"
	include "src/thirdparty/SiftGPU/premake5"
	include "src/thirdparty/VLFeat/premake5"
	include "src/thirdparty/PoseLib/premake5"
	include "src/colmap/dll/premake5"
	include "src/colmap/colmap_exe/premake5"
	include "src/glomap/premake5"